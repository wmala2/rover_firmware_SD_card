#include <Arduino.h>
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "hardware_config.h"
#include "pid.h"
#include "control_server.h"
#include "encoder.h"
#include "motor_driver.h"
#include "sd_logger.h"

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n== Rover main starting ==");

  ws_set_http_enabled(START_HTTP_ENABLED != 0);
  ws_set_logging_enabled(START_LOGGING_ENABLED != 0);

  robot_setup();
  init_wifi();
  init_encoders();
  resetPID();
  sd_logger_begin();    // MicroSD on custom SPI bus; no-ops if no card

  set_command_rate_hz(DEFAULT_CMD_RATE_HZ);
  nextPID = millis() + PID_INTERVAL;

  Serial.printf("HTTP: %s  | Logging: %s\n", ws_http_enabled()?"ON":"OFF", ws_logging_enabled()?"ON":"OFF");
  Serial.printf("Command playback: %u Hz\n", (unsigned)DEFAULT_CMD_RATE_HZ);
  Serial.printf("PID loop: %d Hz (dt=%ums)\n", PID_RATE, PID_INTERVAL);
}

void loop() {
#if DEBUG_DIAG
  // --- 1 Hz health print: fires regardless of network input ---
  // wifi=3 means WL_CONNECTED. If this keeps printing the loop is alive;
  // watch maxblk (largest contiguous free block) for fragmentation.
  static uint32_t lastHealth = 0;
  if (millis() - lastHealth >= 1000) {
    lastHealth = millis();
    Serial.printf("[health %lus] wifi=%d rssi=%ld heap=%u min=%u maxblk=%u\n",
                  (unsigned long)(millis() / 1000), (int)WiFi.status(),
                  (long)WiFi.RSSI(), ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  }
#endif

  apply_next_from_queue_loop_clocked(true);

  if (!ws_open_loop_active() && !ws_motion_paused()) {
    const long now = (long)millis();
    if ((now - (long)nextPID) >= 0) {
      updatePID();
      nextPID += PID_INTERVAL;
    }
  }

  auto_stop_if_inactive();

  static uint32_t lastBeat = 0;
  if (HEARTBEAT_MS && (millis() - lastBeat) >= HEARTBEAT_MS) {
    lastBeat = millis();
    // Serial.println("[hb] main alive");
  }

  delay(1);
}
