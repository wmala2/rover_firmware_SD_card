#include "control_server.h"
#include "motor_driver.h"
#include "encoder.h"
#include "json_helpers.h"
#include "http_helpers.h"
#include "sd_logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <math.h>

/*==============================================================================
  Network addressing
==============================================================================*/
// IPAddress WS_local_IP (192,168,50,223); // BadBunny
// Override per-build from platformio.ini: -DROVER_IP=192,168,50,210
#ifndef ROVER_IP
  #define ROVER_IP 192,168,50,204    // default (BaleNet)
#endif
#ifndef ROVER_GW
  #define ROVER_GW 192,168,50,1      // default gateway (BaleNet)
#endif
IPAddress WS_local_IP (ROVER_IP);
// IPAddress WS_gateway  (192,168,8,1); // BadBunny
IPAddress WS_gateway  (ROVER_GW);
IPAddress WS_subnet   (255,255,255,0);
IPAddress WS_dns1     (8,8,8,8);
IPAddress WS_dns2     (8,8,4,4);

/*==============================================================================
  Feature switches (runtime)
==============================================================================*/
static volatile bool g_http_enabled    = (WS_ENABLE_HTTP != 0);
static volatile bool g_logging_enabled = (WS_ENABLE_LOG  != 0);

bool ws_http_enabled()    { return g_http_enabled; }
bool ws_logging_enabled() { return g_logging_enabled; }
void ws_set_http_enabled(bool on)    { g_http_enabled = on; }
void ws_set_logging_enabled(bool on) { g_logging_enabled = on; }

/*==============================================================================
  UDP + constants
==============================================================================*/
static WiFiUDP Udp;
static const size_t UDP_MAX = 512;

// Hold-last window for 'm' playback before auto-stop + FLUSH
static const uint8_t M_QUEUE_GRACE_TICKS = 3;

/*==============================================================================
  Safety watchdog
==============================================================================*/
volatile uint32_t lastMotorCommandMs = 0;

/*==============================================================================
  Kinematics
==============================================================================*/
static constexpr float WHEEL_CIRCUMFERENCE_M =
  3.14159265358979323846f * WHEEL_DIAMETER_M;

/*==============================================================================
  Runtime TRIM (defaults from macros), with lock for concurrency safety
==============================================================================*/
static volatile float g_trim_left  = TRIM_LEFT;
static volatile float g_trim_right = TRIM_RIGHT;
static portMUX_TYPE g_trimMux = portMUX_INITIALIZER_UNLOCKED;

void  ws_set_trim(float left, float right) {
  portENTER_CRITICAL(&g_trimMux);
  g_trim_left  = left;
  g_trim_right = right;
  portEXIT_CRITICAL(&g_trimMux);
}
float ws_get_trim_left()  { float v; portENTER_CRITICAL(&g_trimMux); v = g_trim_left;  portEXIT_CRITICAL(&g_trimMux); return v; }
float ws_get_trim_right() { float v; portENTER_CRITICAL(&g_trimMux); v = g_trim_right; portEXIT_CRITICAL(&g_trimMux); return v; }

/*==============================================================================
  Command queue and playback timing
==============================================================================*/
struct MotorCmd {
  int16_t  l;    // counts-per-PID-interval
  int16_t  r;
  uint32_t seq;
};

static QueueHandle_t g_cmd_q             = nullptr;
volatile uint16_t g_cmd_rate_hz   = DEFAULT_CMD_RATE_HZ;    // made non-static for echo
static volatile uint32_t g_cmd_period_us = 1000000UL / DEFAULT_CMD_RATE_HZ;
static volatile uint32_t g_lastAppliedSeq = 0;

void set_command_rate_hz(uint16_t hz) {
  if (!hz) return;
  g_cmd_rate_hz   = hz;
  g_cmd_period_us = 1000000UL / hz;
}

/*==============================================================================
  Open-loop state & /log pause
==============================================================================*/
static volatile bool g_open_loop = false;
static volatile int  g_open_pwm_l = 0;
static volatile int  g_open_pwm_r = 0;

bool ws_open_loop_active() { return g_open_loop; }

static volatile bool g_pause_motion = false;
bool ws_motion_paused() { return g_pause_motion; }

/*==============================================================================
  m/s → counts-per-PID-interval  (uses PID_RATE, not command rate)
==============================================================================*/
static int16_t mps_to_counts_per_pid_tick(float v_mps) {
  const float cpt = (v_mps / WHEEL_CIRCUMFERENCE_M) * ENCODER_CPR_WHEEL * (1.0f / (float)PID_RATE);
  long ci = lroundf(cpt);
  if (ci >  32767) ci =  32767;
  if (ci < -32768) ci = -32768;
  return (int16_t)ci;
}

/*==============================================================================
  Logging ring buffer + helpers + HTTP server
==============================================================================*/
static WiFiServer g_http(HTTP_PORT);
static bool g_http_begun = false;

struct LineRing {
  char lines[LOG_LINES_CAP][LOG_LINE_BYTES];   // fixed storage: allocated once, no heap churn
  size_t head = 0, count = 0;
  SemaphoreHandle_t mtx = nullptr;
  void init() { mtx = xSemaphoreCreateMutex(); }
  void clear() { if (mtx) xSemaphoreTake(mtx, portMAX_DELAY); head = count = 0; if (mtx) xSemaphoreGive(mtx); }
  void add(const char* s) {
    if (mtx) xSemaphoreTake(mtx, portMAX_DELAY);
    strlcpy(lines[head], s, LOG_LINE_BYTES);    // copy into existing buffer; never allocates
    head = (head + 1) % LOG_LINES_CAP;
    if (count < LOG_LINES_CAP) count++;
    if (mtx) xSemaphoreGive(mtx);
  }
  void dump(Print& out) {
    if (mtx) xSemaphoreTake(mtx, portMAX_DELAY);
    size_t n = count;
    size_t idx = (count == LOG_LINES_CAP) ? head : 0;
    for (size_t i = 0; i < n; ++i) {
      const char* line = lines[(idx + i) % LOG_LINES_CAP];
      out.print(line);
      size_t L = strlen(line);
      if (L == 0 || line[L - 1] != '\n') out.print('\n');
    }
    if (mtx) xSemaphoreGive(mtx);
  }
} g_log;

void ws_logf(const char* fmt, ...) {
  if (!g_logging_enabled) return;
  char buf[LOG_MAX_LINE];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  char line[LOG_MAX_LINE+32];
  snprintf(line, sizeof(line), "[%lu ms] %s", (unsigned long)millis(), buf);
  if (WS_SERIAL_MIRROR) Serial.print(line);
  g_log.add(line);        // RAM ring (always)
  sd_logger_line(line);   // SD card (only when recording; no-ops otherwise)
}

/*==============================================================================
  HTTP task
==============================================================================*/
static void http_task(void* arg) {
  for (;;) {
#if DEBUG_DIAG
    { static uint32_t _t = 0;
      if (millis() - _t >= 1000) { _t = millis();
        Serial.printf("[stack] http free=%u bytes\n",
                      (unsigned)(uxTaskGetStackHighWaterMark(NULL) * 4)); } }
#endif
    if (!g_http_enabled) { vTaskDelay(50); continue; }

    WiFiClient client = g_http.available();
    if (!client) { vTaskDelay(1); continue; }
    client.setTimeout(1500);
    client.setNoDelay(true);

    String method, path, query;
    if (!http_read_request(client, method, path, query)) {
      http_send_404(client);
      client.stop();
      continue;
    }

    normalize_path(path);
    ws_logf("HTTP %s %s%s%s\n",
            method.c_str(),
            path.c_str(),
            (query.length() ? "?" : ""),
            (query.length() ? query.c_str() : ""));

    if (route_match(path, "/")) {
      static const char* body =
        "ESP32 UDP+PID server\n"
        "Endpoints:\n"
        "  /status                    - single-line status\n"
        "  /log                       - download log (text/plain; attachment)\n"
        "  /clear                     - clear log\n"
        "  /set?trimL=1.00&trimR=1.00&cmdRate=5  - set trims and command playback rate (Hz)\n";
      http_send_text(client, body);

    } else if (route_match(path, "/favicon.ico")) {
      http_send_404(client);

    } else if (route_match(path, "/status")) {
      long lc = readEncoder(LEFT), rc = readEncoder(RIGHT);
      uint32_t qdepth = g_cmd_q ? uxQueueMessagesWaiting(g_cmd_q) : 0;
      char body[320];
      int n = snprintf(body, sizeof(body),
        "ip=%s q=%lu cmdRate=%uHz pidRate=%uHz open=%d pause=%d http=%d log=%d "
        "trimL=%.3f trimR=%.3f seq=%lu "
        "L:tgt=%ld pwm=%d d=%ld enc=%ld  "
        "R:tgt=%ld pwm=%d d=%ld enc=%ld\n",
        WiFi.localIP().toString().c_str(),
        (unsigned long)qdepth, (unsigned)g_cmd_rate_hz, (unsigned)PID_RATE,
        (int)g_open_loop, (int)g_pause_motion, (int)g_http_enabled, (int)g_logging_enabled,
        (double)ws_get_trim_left(), (double)ws_get_trim_right(), (unsigned long)g_lastAppliedSeq,
        (long)leftPID.TargetTicksPerFrame,  leftPID.PWM, leftPID.LastDelta, lc,
        (long)rightPID.TargetTicksPerFrame, rightPID.PWM, rightPID.LastDelta, rc
      );
      if (n < 0) n = 0; if ((size_t)n >= sizeof(body)) n = sizeof(body)-1; body[n] = '\0';
      http_send_text(client, body);

    } else if (route_match(path, "/clear")) {
      g_log.clear();
      resetEncoders();
      http_send_text(client, "log cleared, encoders reset\n");

    } else if (route_match(path, "/log")) {
      g_pause_motion = true;
      const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Disposition: attachment; filename=\"pid_log.txt\"\r\n"
        "Connection: close\r\n\r\n";
      client.write((const uint8_t*)hdr, strlen(hdr));
      g_log.dump(client);
      g_pause_motion = false;

    } else if (route_match(path, "/set")) {
      float tl = ws_get_trim_left(), tr = ws_get_trim_right();
      bool any = false, ok = true;
      float v;

      // NEW: parse cmdRate
      uint16_t newRate = g_cmd_rate_hz;
      bool rate_any = false;

      if (query.length()) {
        if (query.indexOf("trimL=") >= 0 && query_get_float_kv(query, "trimL", v)) { tl = v; any = true; }
        if (query.indexOf("trimR=") >= 0 && query_get_float_kv(query, "trimR", v)) { tr = v; any = true; }

        uint16_t tmpRate = 0;
        if (query.indexOf("cmdRate=") >= 0 && query_get_uint16_kv(query, "cmdRate", tmpRate)) {
          rate_any = true;
          newRate = tmpRate;
          any = true;
        }
      }

      if (any) {
        if ((rate_any && (newRate < 1 || newRate > 50)) ||
            !isfinite(tl) || !isfinite(tr) || tl <= 0.0f || tr <= 0.0f || tl > 3.0f || tr > 3.0f) {
          ok = false;
        } else {
          ws_set_trim(tl, tr);
          if (rate_any) {
            set_command_rate_hz(newRate);
          }
        }
      }

      char body[160];
      if (ok) {
        int n = snprintf(body, sizeof(body),
          "ok trimL=%.6f trimR=%.6f cmdRate=%uHz\n",
          (double)ws_get_trim_left(), (double)ws_get_trim_right(), (unsigned)g_cmd_rate_hz);
        if (n < 0) n = 0; if ((size_t)n >= sizeof(body)) n = sizeof(body)-1; body[n] = '\0';
        http_send_text(client, body);

        ws_logf("HTTP set: trimL=%.6f trimR=%.6f cmdRate=%uHz\n",
                (double)ws_get_trim_left(), (double)ws_get_trim_right(), (unsigned)g_cmd_rate_hz);
      } else {
        http_send_text(client, "error: bad values (trim 0<..<=3, cmdRate 1..50Hz)\n");
      }

    } else {
      http_send_404(client);
    }

    client.flush();
    client.stop();
  }
}

/*==============================================================================
  UDP RX task
==============================================================================*/
static void udp_rx_task(void* arg) {
  char buf[UDP_MAX];
  uint32_t local_seq = 0;

  for (;;) {
#if DEBUG_DIAG
    { static uint32_t _t = 0;
      if (millis() - _t >= 1000) { _t = millis();
        Serial.printf("[stack] udp_rx free=%u bytes\n",
                      (unsigned)(uxTaskGetStackHighWaterMark(NULL) * 4)); } }
#endif
    int ps = Udp.parsePacket();
    if (ps > 0) {
      IPAddress senderIP = Udp.remoteIP();
      int len = Udp.read(buf, sizeof(buf) - 1); if (len < 0) len = 0; buf[len] = '\0';

      char cmd[8] = {0};
      parseJsonString(buf, "command", cmd, sizeof(cmd));
      if (cmd[0] == '\0') strcpy(cmd, "m");

      if (strcmp(cmd, "o") == 0) {                        // OPEN-LOOP
        long l_pwm = 0, r_pwm = 0;
        bool okL = parseJsonInt(buf, "left_pwm",  l_pwm) || parseJsonInt(buf, "left_speed",  l_pwm);
        bool okR = parseJsonInt(buf, "right_pwm", r_pwm) || parseJsonInt(buf, "right_speed", r_pwm);
        if (okL && okR) {
          if (l_pwm >  PWM_MAX) l_pwm =  PWM_MAX;
          if (l_pwm < -PWM_MAX) l_pwm = -PWM_MAX;
          if (r_pwm >  PWM_MAX) r_pwm =  PWM_MAX;
          if (r_pwm < -PWM_MAX) r_pwm = -PWM_MAX;

          g_open_pwm_l = (int)l_pwm;
          g_open_pwm_r = (int)r_pwm;
          g_open_loop  = (g_open_pwm_l != 0 || g_open_pwm_r != 0);

          leftPID.TargetTicksPerFrame  = 0;
          rightPID.TargetTicksPerFrame = 0;
          resetPID();

          setMotorSpeeds(g_open_pwm_l, g_open_pwm_r);

          moving = g_open_loop ? 1 : 0;
          lastMotorCommandMs = millis();
          ws_logf("UDP o: L=%ld R=%ld open=%d from %s\n", l_pwm, r_pwm, (int)g_open_loop, senderIP.toString().c_str());
        }

      } else if (strcmp(cmd, "m") == 0) {                 // CLOSED-LOOP (m/s)
        g_open_loop = false;

        float l_mps = 0.0f, r_mps = 0.0f;
        bool okL = parseJsonFloat(buf, "left_mps",  l_mps) || parseJsonFloat(buf, "left_speed",  l_mps);
        bool okR = parseJsonFloat(buf, "right_mps", r_mps) || parseJsonFloat(buf, "right_speed",  r_mps);
        if (okL && okR) {
          const float l_cmd = l_mps * ws_get_trim_left();
          const float r_cmd = r_mps * ws_get_trim_right();

          int16_t l_counts = (int16_t)mps_to_counts_per_pid_tick(l_cmd);
          int16_t r_counts = (int16_t)mps_to_counts_per_pid_tick(r_cmd);

          long seqVal = 0; uint32_t seq = parseJsonInt(buf, "index", seqVal) ? (uint32_t)seqVal : (++local_seq);

          struct MotorCmd mc { l_counts, r_counts, seq };
          if (g_cmd_q) {
            if (xQueueSend(g_cmd_q, &mc, 0) != pdTRUE) {
              MotorCmd trash; xQueueReceive(g_cmd_q, &trash, 0);
              xQueueSend(g_cmd_q, &mc, 0);
            }
          }
          lastMotorCommandMs = millis();
          ws_logf("UDP m: in L=%.4f R=%.4f | trimL=%.4f trimR=%.4f | CPT/PID L=%d R=%d seq=%lu from %s\n",
                  l_mps, r_mps, (double)ws_get_trim_left(), (double)ws_get_trim_right(),
                  (int)l_counts, (int)r_counts, (unsigned long)seq, senderIP.toString().c_str());
        }

      } else if (strcmp(cmd, "e") == 0) {                 // ENCODER QUERY
        long lft = readEncoder(LEFT);
        long rgt = readEncoder(RIGHT);
        char rbuf[96];
        int n = snprintf(rbuf, sizeof(rbuf), "{\"left_encoder\":%ld,\"right_encoder\":%ld}", lft, rgt);
        if (n < 0) n = 0; if ((size_t)n > sizeof(rbuf)) n = sizeof(rbuf);
        if (Udp.beginPacket(senderIP, ENCODER_REPLY_PORT)) { Udp.write((const uint8_t*)rbuf, (size_t)n); Udp.endPacket(); }
        ws_logf("UDP e: reply L=%ld R=%ld to %s\n", lft, rgt, senderIP.toString().c_str());

      } else if (strcmp(cmd, "r") == 0) {                 // RESET ENCODERS
        resetEncoders();
        ws_logf("UDP r: encoders reset by %s\n", senderIP.toString().c_str());

      } else if (strcmp(cmd, "rec") == 0) {               // SD RECORD on/off
        long on = 0; parseJsonInt(buf, "on", on);
        bool present = sd_logger_set_record(on != 0);
        char rbuf[80];
        int n = snprintf(rbuf, sizeof(rbuf), "{\"recording\":%d,\"card\":%d}",
                         (int)((on != 0) && present), (int)present);
        if (n < 0) n = 0; if ((size_t)n > sizeof(rbuf)) n = sizeof(rbuf);
        if (Udp.beginPacket(senderIP, ENCODER_REPLY_PORT)) {
          Udp.write((const uint8_t*)rbuf, (size_t)n); Udp.endPacket();
        }
        ws_logf("UDP rec: on=%ld card=%d by %s\n", on, (int)present, senderIP.toString().c_str());
      }
    }
    vTaskDelay(1);
  }
}

/*==============================================================================
  Public API
==============================================================================*/

// Prints link-layer transitions on their own, independent of incoming commands.
// reason codes: 200=BEACON_TIMEOUT, 8=ASSOC_LEAVE, 4=INACTIVITY,
//               3=AUTH_EXPIRE, 15=4WAY_HANDSHAKE_TIMEOUT.
static void onWiFiEvent(arduino_event_id_t e, arduino_event_info_t info) {
  if (e == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    Serial.printf("[wifi %lus] DISCONNECTED reason=%u\n",
                  (unsigned long)(millis()/1000), info.wifi_sta_disconnected.reason);
  else if (e == ARDUINO_EVENT_WIFI_STA_CONNECTED)
    Serial.printf("[wifi %lus] CONNECTED\n", (unsigned long)(millis()/1000));
  else if (e == ARDUINO_EVENT_WIFI_STA_GOT_IP)
    Serial.printf("[wifi %lus] GOT_IP %s\n", (unsigned long)(millis()/1000),
                  WiFi.localIP().toString().c_str());
}

void init_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.onEvent(onWiFiEvent);   // register before begin() so first connect is logged too
// possibly out of order call causing disconnect
// #if defined(ARDUINO_ARCH_ESP32)
//   WiFi.setSleep(false);
// #endif

  if (USE_STATIC_IP) {
    WiFi.config(WS_local_IP, WS_gateway, WS_subnet, WS_dns1, WS_dns2);
  }

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());

#if defined(ARDUINO_ARCH_ESP32)
  WiFi.setSleep(false);          // AFTER association so it isn't reset on connect
  WiFi.setAutoReconnect(true);   // belt-and-suspenders: re-join if AP ever deauths
#endif

  if (!Udp.begin(UDP_PORT)) Serial.println("ERROR: Udp.begin() failed!");
  else Serial.printf("UDP listening on %u\n", UDP_PORT);

  if (!g_cmd_q) {
    const uint16_t depth = (DEFAULT_CMD_RATE_HZ * QUEUE_SECONDS);
    g_cmd_q = xQueueCreate(depth, sizeof(MotorCmd));
  }

  g_log.init();
  if (g_http_enabled && !g_http_begun) {
    g_http.begin(); g_http_begun = true;
    g_http.setNoDelay(true);
    Serial.printf("HTTP server on %u\n", (unsigned)HTTP_PORT);
  }
  ws_logf("Server ready at http://%s:%u/  (http=%d log=%d)  trims L=%.3f R=%.3f  cmdRate=%uHz PIDRate=%uHz kp=%.3f ki=%.3f kd=%.3f kV=%.3f kA=%.3f CPR=%.3f\n",
          WiFi.localIP().toString().c_str(), (unsigned)HTTP_PORT,
          (int)g_http_enabled, (int)g_logging_enabled,
          (double)ws_get_trim_left(), (double)ws_get_trim_right(),
          (unsigned)g_cmd_rate_hz,
          (unsigned)PID_RATE,
          (double)leftPID.Kp,         
          (double)leftPID.Ki,         
          (double)leftPID.Kd,         
          (double)leftPID.kV,         
          (double)leftPID.kA,
          (double)ENCODER_CPR_WHEEL
        );

  xTaskCreatePinnedToCore(udp_rx_task, "udp_rx", 8192, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(http_task,   "http",   8192, nullptr, 2, nullptr, 1);

  lastMotorCommandMs = millis();
}

void process_udp() { /* background RX task */ }

void apply_next_from_queue_loop_clocked(bool hold_last) {
  if (g_pause_motion) { leftPID.TargetTicksPerFrame = 0; rightPID.TargetTicksPerFrame = 0; moving = 0; setMotorSpeeds(0,0); return; }
  if (g_open_loop) return;

  static uint32_t lastUs = 0;
  static int16_t  lastL  = 0, lastR = 0;
  static uint8_t  empty_ticks = 0;

  const uint32_t now = micros();
  if ((now - lastUs) < g_cmd_period_us) return;
  lastUs = now;

  MotorCmd mc;
  if (g_cmd_q && xQueueReceive(g_cmd_q, &mc, 0) == pdTRUE) {
    leftPID.TargetTicksPerFrame  = mc.l;
    rightPID.TargetTicksPerFrame = mc.r;
    moving = (mc.l != 0 || mc.r != 0) ? 1 : 0;
    g_lastAppliedSeq = mc.seq;
    lastL = mc.l; lastR = mc.r; empty_ticks = 0;
  } else {
    if (!hold_last) {
      leftPID.TargetTicksPerFrame  = 0; rightPID.TargetTicksPerFrame = 0; moving = 0; setMotorSpeeds(0,0);
      empty_ticks = 0; if (g_cmd_q) xQueueReset(g_cmd_q); return;
    }
    if (empty_ticks < M_QUEUE_GRACE_TICKS) {
      ++empty_ticks;
      leftPID.TargetTicksPerFrame  = lastL; rightPID.TargetTicksPerFrame = lastR;
      moving = (lastL != 0 || lastR != 0) ? 1 : 0;
    } else {
      lastL = 0; lastR = 0;
      leftPID.TargetTicksPerFrame  = 0; rightPID.TargetTicksPerFrame = 0; moving = 0; setMotorSpeeds(0,0);
      if (g_cmd_q) xQueueReset(g_cmd_q);
    }
  }
}

void auto_stop_if_inactive() {
  if (ws_open_loop_active() && (g_open_pwm_l != 0 || g_open_pwm_r != 0)) return;
  const uint32_t now = millis();
  if (now - lastMotorCommandMs > AUTO_STOP_INTERVAL_MS) {
    leftPID.TargetTicksPerFrame  = 0; rightPID.TargetTicksPerFrame = 0; moving = 0; setMotorSpeeds(0, 0);
    lastMotorCommandMs = now;
  }
}
