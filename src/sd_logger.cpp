#include "sd_logger.h"
#include "control_server.h"     // SD_* config + LOG_LINE_BYTES
#include <SPI.h>
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ---- Config fallbacks (normally defined in control_server.h) ----------------
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
  #define SD_FREQ_HZ 20000000UL    // 20 MHz; lower to 4 MHz if wiring is long/noisy
#endif
#ifndef SD_FLUSH_BYTES
  #define SD_FLUSH_BYTES 4096      // one flash sector: batch writes to minimize wear
#endif
#ifndef SD_QUEUE_DEPTH
  #define SD_QUEUE_DEPTH 64        // lines buffered between control path and SD task
#endif
#ifndef LOG_LINE_BYTES
  #define LOG_LINE_BYTES 192
#endif

#if SD_LOG_ENABLED

static SPIClass      s_spi(VSPI);
static QueueHandle_t s_q       = nullptr;
static volatile bool s_present = false;   // card detected at boot
static volatile bool s_recReq  = false;   // desired record state (set by command)
static volatile bool s_recOn   = false;   // actual record state (owned by writer task)
static File          s_file;
static char          s_fname[24] = "";
static char          s_acc[SD_FLUSH_BYTES];
static size_t        s_accLen  = 0;

// Sequential file naming. No RTC on a bare ESP32, so we can't stamp wall-clock
// time; instead we scan the card and pick the next LOGNNNN.CSV index. Each run
// gets its own file. (If you add NTP later, swap in a real timestamp here.)
static uint32_t next_file_index() {
  uint32_t maxIdx = 0;
  File root = SD.open("/");
  if (root) {
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      const char* nm = f.name();
      const char* slash = strrchr(nm, '/');
      const char* base = slash ? slash + 1 : nm;
      unsigned idx = 0;
      if (sscanf(base, "LOG%u.CSV", &idx) == 1 || sscanf(base, "LOG%u.csv", &idx) == 1) {
        if (idx > maxIdx) maxIdx = idx;
      }
      f.close();
    }
    root.close();
  }
  return maxIdx + 1;
}

static void flush_acc() {
  if (s_file && s_accLen) {
    s_file.write((const uint8_t*)s_acc, s_accLen);
    s_file.flush();                 // push FAT update so a yanked card keeps data
  }
  s_accLen = 0;
}

static void append_line(const char* item) {
  size_t L = strnlen(item, LOG_LINE_BYTES);
  if (s_accLen + L > SD_FLUSH_BYTES) flush_acc();   // flush a full block first
  if (L > SD_FLUSH_BYTES) L = SD_FLUSH_BYTES;       // guard (won't happen in practice)
  memcpy(s_acc + s_accLen, item, L);
  s_accLen += L;
}

static void open_file() {
  uint32_t idx = next_file_index();
  snprintf(s_fname, sizeof(s_fname), "/LOG%04u.CSV", (unsigned)idx);
  s_file = SD.open(s_fname, FILE_WRITE);
  if (s_file) {
    char hdr[96];
    int n = snprintf(hdr, sizeof(hdr), "# rover log  file=%s  boot_ms=%lu\n",
                     s_fname, (unsigned long)millis());
    if (n > 0) s_file.write((const uint8_t*)hdr, (size_t)n);
    s_accLen = 0;
    s_recOn = true;
    Serial.printf("[sd] recording -> %s\n", s_fname);
  } else {
    s_recOn = false;
    s_fname[0] = '\0';
    Serial.println("[sd] ERROR: could not open file");
  }
}

static void close_file() {
  // Drain whatever is still queued so stopping doesn't lose the tail.
  char item[LOG_LINE_BYTES];
  while (s_q && xQueueReceive(s_q, item, 0) == pdTRUE) append_line(item);
  flush_acc();
  if (s_file) s_file.close();
  Serial.printf("[sd] stopped (%s)\n", s_fname[0] ? s_fname : "-");
  s_recOn = false;
  s_fname[0] = '\0';
}

static void writer_task(void*) {
  char item[LOG_LINE_BYTES];
  uint32_t lastSafety = 0;
  for (;;) {
    // Apply record-state transitions here so ALL file ops live on one task.
    if (s_recReq && !s_recOn && s_present) open_file();
    if (!s_recReq && s_recOn)              close_file();

    // Drain one line (block up to 200 ms so the loop stays responsive to stops).
    if (s_q && xQueueReceive(s_q, item, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (s_recOn) append_line(item);
    }

    // Safety flush: if lines trickle in slowly, don't wait forever for a full
    // block -- get them onto the card within a few seconds.
    if (s_recOn && (millis() - lastSafety > 3000)) { lastSafety = millis(); flush_acc(); }
  }
}

#endif // SD_LOG_ENABLED

// ---- Public API -------------------------------------------------------------

void sd_logger_begin() {
#if SD_LOG_ENABLED
  s_q = xQueueCreate(SD_QUEUE_DEPTH, LOG_LINE_BYTES);
  s_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  s_present = SD.begin(SD_CS, s_spi, SD_FREQ_HZ);
  if (s_present && SD.cardType() == CARD_NONE) s_present = false;
  if (s_present) {
    uint64_t mb = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("[sd] card detected (%llu MB) on SCK=%d MISO=%d MOSI=%d CS=%d\n",
                  (unsigned long long)mb, SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  } else {
    Serial.println("[sd] no card -- logging stays in RAM ring only");
  }
  xTaskCreatePinnedToCore(writer_task, "sd_log", 6144, nullptr, 1, nullptr, 1);
#endif
}

void sd_logger_line(const char* s) {
#if SD_LOG_ENABLED
  if (s_q && s_recOn) xQueueSend(s_q, s, 0);   // non-blocking; drop if full
#else
  (void)s;
#endif
}

bool sd_logger_set_record(bool on) {
#if SD_LOG_ENABLED
  if (on && !s_present) return false;
  s_recReq = on;
  return s_present;
#else
  (void)on; return false;
#endif
}

bool sd_logger_present()   {
#if SD_LOG_ENABLED
  return s_present;
#else
  return false;
#endif
}

bool sd_logger_recording() {
#if SD_LOG_ENABLED
  return s_recOn;
#else
  return false;
#endif
}

void sd_logger_status(char* out, size_t n) {
#if SD_LOG_ENABLED
  snprintf(out, n, "present=%d recording=%d file=%s",
           (int)s_present, (int)s_recOn, s_fname[0] ? s_fname : "-");
#else
  snprintf(out, n, "sd_logging=disabled");
#endif
}
