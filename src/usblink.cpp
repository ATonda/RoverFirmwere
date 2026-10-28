#include "usblink.h"

#include "console.h"
#include "gnss.h"

Console Con;

namespace usblink {

static bool on = false;
static uint32_t fwdMs = 0;
static CmdHandler cmdHandler = nullptr;
static constexpr uint32_t GUARD_MS = 5000;
static uint32_t guardUntil = 0;

// Řádek JSON z S3 je krátký (usbhost, get info); rámec UBX až 2040 B dat
static uint8_t pbuf[2048], prep[2048];
static RxParser parser(pbuf, prep, sizeof(pbuf), 2040);

static void onFrame(const uint8_t *f, size_t n, bool rtcm) {
  gnss::writeRaw(f, n);  // celý rámec najednou – nepromíchá se s rámci z BT
  if (rtcm) fwdMs = millis();
}

static void onCmd(const char *line, size_t len) {
  if (cmdHandler) cmdHandler(line, len);
}

void setCmdHandler(CmdHandler h) { cmdHandler = h; }

void setActive(bool a) {
  if (a == on) return;
  on = a;
  parser.reset();
  parser.setHandlers(onCmd, onFrame);
  Con.mute(a);
  if (!a) guardUntil = millis() + GUARD_MS;
}

bool consoleGuard() { return (int32_t)(guardUntil - millis()) > 0; }
void noteBinary() { guardUntil = millis() + GUARD_MS; }

bool active() { return on; }

void poll() {
  if (!on) return;
  uint8_t b[256];
  int rounds = 0;  // nejvýš 8 kB za jedno volání, ať smyčka nestojí
  int n;
  while (on && rounds++ < 32 && (n = Serial.available()) > 0) {
    n = Serial.readBytes(b, n > (int)sizeof(b) ? sizeof(b) : n);
    if (n <= 0) break;
    for (int i = 0; i < n && on; i++) parser.put(b[i]);
  }
}

void send(const char *s, size_t n) {
  if (on) Serial.write((const uint8_t *)s, n);
}

const RxParser::Stats &stats() { return parser.stats(); }
uint32_t rtcmFwdMs() { return fwdMs; }

}  // namespace usblink
