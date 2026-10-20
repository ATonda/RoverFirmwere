#include "power.h"

#include <Preferences.h>
#include <esp_random.h>

#include "config.h"

namespace power {

static float factor = VBAT_DIVIDER_DEFAULT;

// Na GPIO35 je kromě napětí děliče obdélník ~100 kHz (0 ↔ 2,9 V). Pravidelné vzorkování po 1 ms padá
// vždy do stejné fáze (1 ms = 100 period), proto vzorky v náhodných intervalech 0,5–1,5 ms: rozloží se
// po celé periodě a průměr odpovídá voltmetru. Průměr za 10 s (~10 000 vzorků), nad ním exponenciální
// průměr s časovou konstantou ~5 min. Napětí baterie se mění za hodiny, rychlá odezva není potřeba.
static const uint32_t FIRST_MS = 2000;   // první hodnota po startu
static const uint32_t WINDOW_MS = 10000; // okno průměru
static const float EMA_N = 30.0f;        // 30 oken × 10 s = 5 min
static const uint32_t GAP_MIN_US = 500, GAP_SPAN_US = 1000;  // náhodný interval 0,5–1,5 ms
static const uint32_t MIN_SAMPLES = 200; // okno s méně vzorky (blokace smyčky) se zahodí
static const float V_MIN = 2.5f, V_MAX = 4.4f;

static float ema = -1;                   // vyhlazené napětí [V], < 0 = ještě nezměřeno
static int pctShown = -1;
static uint32_t winStartMs = 0;
static uint32_t nextUs = 0;
static uint64_t accSum = 0;
static uint32_t accN = 0, accMin = UINT32_MAX, accMax = 0;

static uint32_t lastMeanMv = 0;          // průměr na pinu za poslední okno
static uint32_t spreadMin = 0, spreadMax = 0;

static void scheduleNext(uint32_t nowUs) { nextUs = nowUs + GAP_MIN_US + esp_random() % GAP_SPAN_US; }

// Bez hotového okna: krátké měření s náhodnými rozestupy (kalibrace hned po startu)
static uint32_t pinMvNow() {
  uint32_t s = 0;
  for (int i = 0; i < 64; i++) {
    s += analogReadMilliVolts(PIN_VBAT);
    delayMicroseconds(3 + esp_random() % 17);
  }
  return s / 64;
}

void begin() {
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  Preferences p;
  p.begin("power", true);
  factor = p.getFloat("vcal", VBAT_DIVIDER_DEFAULT);
  p.end();
  // nesmyslný uložený koeficient (stará kalibrace) – zpět na dělič 1:1
  if (factor < 1.6f || factor > 2.4f) factor = VBAT_DIVIDER_DEFAULT;
  winStartMs = millis();
  scheduleNext(micros());
}

void update() {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextUs) >= 0) {
    scheduleNext(nowUs);
    uint32_t v = analogReadMilliVolts(PIN_VBAT);
    accSum += v;
    accN++;
    if (v < accMin) accMin = v;
    if (v > accMax) accMax = v;
  }
  uint32_t now = millis();
  if (now - winStartMs < (ema < 0 ? FIRST_MS : WINDOW_MS)) return;
  winStartMs = now;
  bool ok = accN >= MIN_SAMPLES;
  if (ok) {
    lastMeanMv = (uint32_t)(accSum / accN);
    spreadMin = accMin;
    spreadMax = accMax;
  }
  accSum = 0;
  accN = 0;
  accMin = UINT32_MAX;
  accMax = 0;
  if (!ok) return;
  float v = lastMeanMv * factor / 1000.0f;
  if (ema < 0) {
    ema = v;
    pctShown = batteryPercent(v);
    return;
  }
  ema += (v - ema) / EMA_N;
  int p = batteryPercent(ema);
  // bez nabíjení procenta jen klesají; nárůst o 3 % a víc = nabíjení
  if (pctShown < 0 || p < pctShown || p >= pctShown + 3) pctShown = p;
}

float batteryVolts() { return ema < 0 ? instantVolts() : ema; }
float instantVolts() { return pinMillivolts() * factor / 1000.0f; }
uint32_t pinMillivolts() { return lastMeanMv ? lastMeanMv : pinMvNow(); }

void lastSpread(uint32_t &minMv, uint32_t &maxMv) {
  minMv = spreadMin;
  maxMv = spreadMax;
}

int batteryPercent(float v) {
  // Orientační křivka LiPo v klidu (při zátěži napětí mírně klesá)
  static const float V[] = {3.30f, 3.50f, 3.60f, 3.70f, 3.75f, 3.80f, 3.87f, 3.95f, 4.05f, 4.20f};
  static const int P[] = {0, 5, 10, 20, 30, 40, 50, 65, 80, 100};
  if (v <= V[0]) return 0;
  for (int i = 1; i < 10; i++) {
    if (v <= V[i]) return P[i - 1] + (int)((v - V[i - 1]) / (V[i] - V[i - 1]) * (P[i] - P[i - 1]));
  }
  return 100;
}

int batteryPercentStable() { return pctShown >= 0 ? pctShown : batteryPercent(batteryVolts()); }

bool valid() {
  float v = batteryVolts();
  return v >= V_MIN && v <= V_MAX;
}

bool calibrate(float measured) {
  int measuredMv = (int)lroundf(measured < 10.0f ? measured * 1000.0f : measured);
  if (measuredMv < 2500 || measuredMv > 4500) return false;
  uint32_t pin = pinMillivolts();  // průměr posledního okna 10 s
  if (pin < 500) return false;
  float f = (float)measuredMv / pin;
  if (f < 1.6f || f > 2.4f) return false;
  factor = f;
  Preferences p;
  p.begin("power", false);
  p.putFloat("vcal", factor);
  p.end();
  ema = pin * factor / 1000.0f;
  pctShown = batteryPercent(ema);
  return true;
}

float dividerFactor() { return factor; }

}  // namespace power
