#pragma once
#include <Arduino.h>

// =============================================================================
// sd_logger.h  -  Optional MicroSD logging on a custom SPI bus.
//
// Design: log lines are pushed onto a FreeRTOS queue (non-blocking) and a
// dedicated writer task drains them into a 4 KB accumulator, flushing one
// flash-sector-sized block at a time. This keeps slow SD writes (tens of ms)
// entirely off the control/networking path -- ws_logf never blocks on the card.
//
// When no card is present, all of this no-ops and the in-RAM ring buffer
// (g_log) remains the sole log, so the firmware behaves exactly as before.
//
// Pins, bus frequency, flush size, and queue depth are configured in
// control_server.h (SD_* defines) and overridable via build_flags.
// =============================================================================

// Init SPI bus + SD card + queue + writer task. Call once in setup().
void sd_logger_begin();

// Enqueue one already-formatted log line (called from ws_logf). Non-blocking;
// drops the line if the queue is momentarily full. Copies the string, so the
// caller's buffer can be reused immediately.
void sd_logger_line(const char* s);

// Request recording start/stop. Returns true if a card is present (i.e. the
// request can be honored). Safe to call when SD logging is disabled (returns false).
bool sd_logger_set_record(bool on);

bool sd_logger_present();     // card detected at boot?
bool sd_logger_recording();   // actively writing a file right now?

// Fills 'out' with a short human-readable status string.
void sd_logger_status(char* out, size_t n);
