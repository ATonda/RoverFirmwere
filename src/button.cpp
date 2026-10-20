#include "button.h"

#include "config.h"

namespace button {

static bool raw = false;
static bool state = false;
static uint32_t rawChangeMs = 0;
static uint32_t pressMs = 0;
static bool longFired = false;
static bool ignoreUntilRelease = false;

void begin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  raw = state = digitalRead(PIN_BUTTON) == LOW;
  // stisk držený už při startu se nepočítá
  ignoreUntilRelease = state;
}

Event poll() {
  uint32_t now = millis();
  bool r = digitalRead(PIN_BUTTON) == LOW;
  if (r != raw) {
    raw = r;
    rawChangeMs = now;
  }
  Event ev = Event::None;
  if (raw != state && now - rawChangeMs >= BTN_DEBOUNCE_MS) {
    state = raw;
    if (state) {
      pressMs = now;
      longFired = false;
    } else {
      if (!ignoreUntilRelease && !longFired) ev = Event::Short;
      ignoreUntilRelease = false;
    }
  }
  if (state && !ignoreUntilRelease && !longFired && now - pressMs >= BTN_LONG_MS) {
    longFired = true;
    ev = Event::Long;
  }
  return ev;
}

bool pressed() { return state; }

void resetEvents() { ignoreUntilRelease = state; }

}  // namespace button
