#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>
#include "hardware_config.h"
#include "pid.h"
#include "wifi_config.h"

#ifndef WS_ENABLE_HTTP
  #define WS_ENABLE_HTTP 1
#endif
#ifndef WS_ENABLE_LOG
  #define WS_ENABLE_LOG  1
#endif
#ifndef WS_SERIAL_MIRROR
  #define WS_SERIAL_MIRROR 1
#endif
// Diagnostic serial prints ([stack] per-task headroom + [health] heap/wifi).
// 0 = off (clean normal operation). Set to 1 here, or override per-build with
// -D DEBUG_DIAG=1 in platformio.ini build_flags, when chasing a problem.
#ifndef DEBUG_DIAG
  #define DEBUG_DIAG 0
#endif

// ---- MicroSD logging (custom SPI bus) --------------------------------------
// Defaults chosen to avoid motor (12/13/26/27) and encoder (19/21/22/23) pins.
// On a WROVER module GPIO16/17 are used by PSRAM -- pick other free pins there.
#ifndef SD_LOG_ENABLED
  #define SD_LOG_ENABLED 1
#endif
#ifndef SD_SCK
  #define SD_SCK 18
#endif
#ifndef SD_MISO
  #define SD_MISO 16
#endif
#ifndef SD_MOSI
  #define SD_MOSI 17
#endif
#ifndef SD_CS
  #define SD_CS 5
#endif
#ifndef SD_FREQ_HZ
  #define SD_FREQ_HZ 20000000UL   // 20 MHz; drop to 4000000 if SD wiring is flaky
#endif
#ifndef SD_FLUSH_BYTES
  #define SD_FLUSH_BYTES 4096     // flush in flash-sector blocks to minimize wear
#endif
#ifndef SD_QUEUE_DEPTH
  #define SD_QUEUE_DEPTH 64       // log lines buffered between control path and SD task
#endif
#ifndef LOG_LINES_CAP
  #define LOG_LINES_CAP 300      // was 2000: 2000 lines never fit in RAM (~300KB).
                                 // Fixed char ring below = LOG_LINES_CAP*LOG_LINE_BYTES static.
#endif
#ifndef LOG_LINE_BYTES
  #define LOG_LINE_BYTES 192     // per-slot storage; longest real line ~150 chars
#endif
#ifndef LOG_MAX_LINE
  #define LOG_MAX_LINE 256       // scratch width for formatting in ws_logf
#endif
#ifndef AUTO_STOP_INTERVAL_MS
  #define AUTO_STOP_INTERVAL_MS 10000UL
#endif

void init_wifi();
void process_udp();
void auto_stop_if_inactive();

void set_command_rate_hz(uint16_t hz);
void apply_next_from_queue_loop_clocked(bool hold_last = false);

// Open-loop & pause state
bool ws_open_loop_active();
bool ws_motion_paused();

// Logging / HTTP runtime switches
void ws_set_logging_enabled(bool on);
void ws_set_http_enabled(bool on);
bool ws_logging_enabled();
bool ws_http_enabled();

// TRIM runtime control
void  ws_set_trim(float left, float right);
float ws_get_trim_left();
float ws_get_trim_right();

// Log helper
void ws_logf(const char* fmt, ...);
