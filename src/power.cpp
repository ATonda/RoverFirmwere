#include "power.h"

#include <Preferences.h>

#include "config.h"

namespace power {

static float factor = VBAT_DIVIDER_DEFAULT;

// vzorky ADC rozložené v čase (1× za ms, průměr za 1 s – potlačí rychlé rušení na vstupu, např. spínání
// měniče), pak exponenciální průměr s časovou konstantou ~60 s
static const uint32_t SAMPLE_MS = 1000;
static const float EMA_N = 60.0f;
static const float JUMP_V = 0.3f;       // trvalá změna o víc (připojení nabíječky) se převezme hned
static const int JUMP_SAMPLES = 5;      // … když trvá 5 vzorků za sebou
static const float V_MIN = 2.5f, V_MAX = 4.4f;

static float ema = -1;                  // vyhlazené napětí [V], < 0 = ještě nezměřeno
static int jumpCount = 0;
static int pctShown = -1;
static uint32_t lastSampleMs = 0;
static uint32_t spreadMin = 0, spreadMax = 0;
static uint32_t accSum = 0, accN = 0, accMin = UINT32_MAX, accMax = 0, lastSubUs = 0;

// posledních 5 průměrů na pinu (kalibrace)
static const int RING = 5;
static uint32_t ring[RING];
static int ringN = 0, ringPos = 0;

static uint32_t lastMeanMv = 0;  // průměr na pinu za poslední sekundu

static uint32_t pinMv() {
  uint32_t s = 0;
  for (int i = 0; i < 16; i++) s += analogReadMilliVolts(PIN_VBAT);
  return s / 16;
}

static void resetSmoothing(float v) {
  ema = v;
  jumpCount = 0;
  pctShown = batteryPercent(v);
}

void begin() {
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  Preferences p;
  p.begin("power", true);
  factor = p.getFloat("vcal", VBAT_DIVIDER_DEFAULT);
  p.end();
  // nesmyslný uložený koeficient (stará kalibrace) – zpět na dělič 1:1
  if (factor < 1.6f || factor > 2.4f) factor = VBAT_DIVIDER_DEFAULT;
  update();
}

void update() {
  uint32_t nowUs = micros();
  if (nowUs - lastSubUs >= 1000) {
    lastSubUs = nowUs;
    uint32_t v = analogReadMilliVolts(PIN_VBAT);
    accSum += v;
    accN++;
    if (v < accMin) accMin = v;
    if (v > accMax) accMax = v;
  }
  uint32_t now = millis();
  if (ema >= 0 && now - lastSampleMs < SAMPLE_MS) return;
  if (ema >= 0 && accN < 20) return;  // málo vzorků (dlouhá blokace smyčky) – počkat
  lastSampleMs = now;
  uint32_t mv;
  if (accN) {
    mv = accSum / accN;
    spreadMin = accMin;
    spreadMax = accMax;
  } else {
    mv = pinMv();  // první měření po startu
  }
  accSum = accN = 0;
  accMin = UINT32_MAX;
  accMax = 0;
  lastMeanMv = mv;
  ring[ringPos] = mv;
  ringPos = (ringPos + 1) % RING;
  if (ringN < RING) ringN++;
  float v = mv * factor / 1000.0f;
  if (ema < 0) {
    resetSmoothing(v);
    return;
  }
  if (fabsf(v - ema) > JUMP_V) {
    if (++jumpCount >= JUMP_SAMPLES) resetSmoothing(v);
    return;  // jednotlivá špička se do průměru nepočítá
  }
  jumpCount = 0;
  ema += (v - ema) / EMA_N;
  int p = batteryPercent(ema);
  // bez nabíjení procenta jen klesají; nárůst o 3 % a víc = nabíjení
  if (pctShown < 0 || p < pctShown || p >= pctShown + 3) pctShown = p;
}

float batteryVolts() { return ema < 0 ? instantVolts() : ema; }
float instantVolts() { return pinMv() * factor / 1000.0f; }
uint32_t pinMillivolts() { return lastMeanMv ? lastMeanMv : pinMv(); }

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
  // průměr posledních 5 s (bez čekání), jinak jedno měření hned
  uint32_t pin = 0;
  if (ringN > 0) {
    for (int i = 0; i < ringN; i++) pin += ring[i];
    pin /= ringN;
  } else {
    pin = pinMv();
  }
  if (pin < 500) return false;
  float f = (float)measuredMv / pin;
  if (f < 1.6f || f > 2.4f) return false;
  factor = f;
  Preferences p;
  p.begin("power", false);
  p.putFloat("vcal", factor);
  p.end();
  resetSmoothing(pin * factor / 1000.0f);
  return true;
}

float dividerFactor() { return factor; }

}  // namespace power
