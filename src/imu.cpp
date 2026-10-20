#include "imu.h"

#include <Preferences.h>
#include <Wire.h>
#include <esp_timer.h>

#include "config.h"

namespace imu {

// Registry LSM6DSV16X
static constexpr uint8_t REG_FIFO_CTRL3   = 0x09;  // BDR gyro [7:4], akcel. [3:0]
static constexpr uint8_t REG_FIFO_CTRL4   = 0x0A;  // ODR_TS_BATCH [7:6], FIFO_MODE [2:0]
static constexpr uint8_t REG_INT1_CTRL    = 0x0D;
static constexpr uint8_t REG_WHO_AM_I     = 0x0F;
static constexpr uint8_t REG_CTRL1        = 0x10;  // ODR + režim akcelerometru
static constexpr uint8_t REG_CTRL2        = 0x11;  // ODR + režim gyra
static constexpr uint8_t REG_CTRL3        = 0x12;  // BDU, IF_INC, SW_RESET
static constexpr uint8_t REG_CTRL6        = 0x15;  // rozsah gyra
static constexpr uint8_t REG_CTRL8        = 0x17;  // rozsah akcelerometru
static constexpr uint8_t REG_FIFO_STATUS1 = 0x1B;  // DIFF_FIFO[7:0]
static constexpr uint8_t REG_OUT_TEMP_L   = 0x20;
static constexpr uint8_t REG_TIMESTAMP0   = 0x40;  // 32bit časová značka
static constexpr uint8_t REG_FUNC_ENABLE  = 0x50;  // TIMESTAMP_EN bit 6
static constexpr uint8_t REG_FIFO_TAG     = 0x78;  // tag + 6 B dat
static constexpr uint8_t WHO_AM_I_VAL     = 0x70;

static constexpr uint8_t TAG_GYRO  = 0x01;
static constexpr uint8_t TAG_ACCEL = 0x02;
static constexpr uint8_t TAG_TS    = 0x04;

static constexpr float ACC_SENS  = 0.122e-3f;  // g/LSB při ±4 g
static constexpr float GYRO_SENS = 17.5e-3f;   // °/s/LSB při ±500 °/s
static constexpr double TS_US_NOMINAL = 21.75;  // µs na LSB časové značky (typ.)
static constexpr double ODR_NOMINAL = 120.0;
static constexpr uint8_t TS_DECIMATION = 8;     // časová značka každý 8. vzorek

static uint8_t addr = 0;
static Stats st;
static Sample last;
static bool haveLast = false;
static float biasX = 0, biasY = 0, biasZ = 0;
static float tempC = 25;
static uint32_t tempReadMs = 0;

// fronta vzorků pro spotřebitele (EKF)
static constexpr int QSIZE = 64;
static Sample queue[QSIZE];
static int qHead = 0, qTail = 0;

// skládání vzorku z FIFO slov
static bool hasG = false, hasA = false;
static int16_t gRaw[3], aRaw[3];

// časování: časová značka čipu -> esp_timer
static bool tsValid = false;
static uint32_t tsLast = 0;        // poslední značka z FIFO
static uint32_t samplesSinceTs = 0;
static double ticksPerSample = 1e6 / ODR_NOMINAL / TS_US_NOMINAL;
static bool syncValid = false;
static uint32_t syncChip0 = 0;
static int64_t syncUs0 = 0;
static double usPerTick = TS_US_NOMINAL;
static uint32_t syncReadMs = 0;

// ---------------------------------------------------------------- I2C
static bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool rd(uint8_t reg, uint8_t *dst, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) dst[i] = Wire.read();
  return true;
}

static void loadBias() {
  Preferences p;
  p.begin("imu", true);
  biasX = p.getFloat("gbx", 0);
  biasY = p.getFloat("gby", 0);
  biasZ = p.getFloat("gbz", 0);
  p.end();
}

// ---------------------------------------------------------------- časová synchronizace
// Čte časovou značku čipu a esp_timer zároveň; z dvojic odhaduje měřítko.
static void syncClock() {
  uint8_t b[4];
  int64_t t1 = esp_timer_get_time();
  if (!rd(REG_TIMESTAMP0, b, 4)) return;
  int64_t t2 = esp_timer_get_time();
  uint32_t chip = b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t)b[3] << 24);
  int64_t us = (t1 + t2) / 2;
  if (!syncValid) {
    syncChip0 = chip;
    syncUs0 = us;
    syncValid = true;
    return;
  }
  uint32_t dChip = chip - syncChip0;
  int64_t dUs = us - syncUs0;
  if (dChip > 200000) {  // po > ~4 s zpřesnit měřítko (dolní propust)
    double s = (double)dUs / dChip;
    if (s > TS_US_NOMINAL * 0.9 && s < TS_US_NOMINAL * 1.1) usPerTick = 0.9 * usPerTick + 0.1 * s;
    // posunout referenci, aby nerostla chyba z přetečení a zaokrouhlení
    syncChip0 = chip;
    syncUs0 = us;
  }
}

static int64_t chipToUs(double chip) {
  // chip může být mírně před i za referencí – rozdíl jako znaménkový
  double d = chip - (double)syncChip0;
  if (d > 2147483648.0) d -= 4294967296.0;
  if (d < -2147483648.0) d += 4294967296.0;
  return syncUs0 + (int64_t)(d * usPerTick);
}

// ---------------------------------------------------------------- vzorky
static void pushSample() {
  Sample s;
  double chip = (double)tsLast + samplesSinceTs * ticksPerSample;
  s.tUs = (tsValid && syncValid) ? chipToUs(chip) : esp_timer_get_time();
  s.gx = gRaw[0] * GYRO_SENS - biasX;
  s.gy = gRaw[1] * GYRO_SENS - biasY;
  s.gz = gRaw[2] * GYRO_SENS - biasZ;
  s.ax = aRaw[0] * ACC_SENS;
  s.ay = aRaw[1] * ACC_SENS;
  s.az = aRaw[2] * ACC_SENS;
  s.tempC = tempC;
  samplesSinceTs++;
  last = s;
  haveLast = true;
  st.samples++;
  int next = (qHead + 1) % QSIZE;
  if (next == qTail) {  // fronta plná – zahodit nejstarší
    qTail = (qTail + 1) % QSIZE;
    st.queueDrops++;
  }
  queue[qHead] = s;
  qHead = next;
}

static void handleWord(const uint8_t *w) {
  uint8_t tag = w[0] >> 3;
  const uint8_t *d = w + 1;
  if (tag == TAG_GYRO) {
    for (int i = 0; i < 3; i++) gRaw[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
    hasG = true;
  } else if (tag == TAG_ACCEL) {
    for (int i = 0; i < 3; i++) aRaw[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
    hasA = true;
  } else if (tag == TAG_TS) {
    uint32_t ts = d[0] | (d[1] << 8) | (d[2] << 16) | ((uint32_t)d[3] << 24);
    if (tsValid && samplesSinceTs > 0) {
      double tps = (double)(uint32_t)(ts - tsLast) / samplesSinceTs;
      // přijmout jen rozumné hodnoty (±10 % jmenovité ODR)
      double nom = 1e6 / ODR_NOMINAL / TS_US_NOMINAL;
      if (tps > nom * 0.9 && tps < nom * 1.1) ticksPerSample = 0.95 * ticksPerSample + 0.05 * tps;
    }
    tsLast = ts;
    tsValid = true;
    samplesSinceTs = 0;
  }
  if (hasG && hasA) {
    hasG = hasA = false;
    pushSample();
  }
}

// ---------------------------------------------------------------- veřejné API
// Nastavení registrů s ověřením zpětným čtením
struct RegVal {
  uint8_t reg, val;
};
static const RegVal REGS[] = {
    {REG_CTRL3, 0x44},        // BDU + IF_INC
    {REG_CTRL8, 0x01},        // akcelerometr ±4 g
    {REG_CTRL6, 0x02},        // gyro ±500 °/s
    {REG_INT1_CTRL, 0x00},    // přerušení se nepoužívá, FIFO se vyčítá dotazováním
    {REG_FUNC_ENABLE, 0x40},  // TIMESTAMP_EN
    {REG_FIFO_CTRL3, 0x66},   // BDR gyro i akcelerometr 120 Hz
    // časová značka do FIFO každý 8. vzorek (ODR_TS_BATCH = 10), režim continuous (110)
    {REG_FIFO_CTRL4, (0x02 << 6) | 0x06},
    {REG_CTRL1, 0x06},  // akcelerometr 120 Hz, high-performance
    {REG_CTRL2, 0x06},  // gyro 120 Hz, high-performance
};

static uint32_t initMs = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastSampleCount = 0;

static void resetState() {
  hasG = hasA = false;
  tsValid = false;
  samplesSinceTs = 0;
  syncValid = false;
  qHead = qTail = 0;
}

// Reset čipu a nastavení; true = všechny registry ověřeny
static bool configure() {
  wr(REG_CTRL3, 0x01);  // SW reset
  // počkat na dokončení resetu (bit SW_RESET se sám vynuluje), max. 50 ms
  uint32_t t0 = millis();
  uint8_t c3 = 0x01;
  delay(1);
  while (millis() - t0 < 50) {
    if (rd(REG_CTRL3, &c3, 1) && !(c3 & 0x01)) break;
    delay(1);
  }
  delay(5);

  for (int attempt = 0; attempt < 3; attempt++) {
    for (const RegVal &r : REGS) wr(r.reg, r.val);
    bool ok = true;
    for (const RegVal &r : REGS) {
      uint8_t v = 0;
      if (!rd(r.reg, &v, 1) || v != r.val) {
        ok = false;
        st.configErr++;
      }
    }
    if (ok) {
      resetState();
      syncClock();
      initMs = millis();
      lastSampleMs = initMs;
      lastSampleCount = st.samples;
      return true;
    }
    delay(5);
  }
  return false;
}

// Uvolnění sběrnice I2C: když se ESP restartuje uprostřed čtení, IMU zůstane v půlce bajtu
// a drží SDA v nule – do vypnutí napájení pak neodpovídá. 9 hodinových pulzů na SCL
// a podmínka STOP přenos dokončí a IMU sběrnici pustí.
static bool busRecover() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(10);
  bool stuck = digitalRead(PIN_SDA) == LOW;
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(10);
  }
  // STOP: SDA z 0 do 1 při SCL = 1
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_SDA, HIGH);
  delayMicroseconds(10);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  return stuck;
}

bool begin() {
  if (busRecover()) st.busRecovered = true;
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  addr = 0;
  for (uint8_t a : {IMU_ADDR_B, IMU_ADDR_A}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      addr = a;
      break;
    }
  }
  if (!addr) return false;

  uint8_t who = 0;
  if (!rd(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VAL) {
    addr = 0;
    return false;
  }

  loadBias();
  return configure();
}

void flush() {
  if (!addr) return;
  wr(REG_FIFO_CTRL4, 0x02 << 6);           // bypass – vyprázdní FIFO
  wr(REG_FIFO_CTRL4, (0x02 << 6) | 0x06);  // zpět continuous
  uint8_t s[2];
  rd(REG_FIFO_STATUS1, s, 2);              // čtení nuluje příznak přetečení
  resetState();
  syncClock();
  st.fifoMax = 0;
  lastSampleMs = millis();
  lastSampleCount = st.samples;
}

bool present() { return addr != 0; }
uint8_t address() { return addr; }

void poll() {
  if (!addr) return;
  uint32_t now = millis();

  if (now - syncReadMs >= 1000) {
    syncReadMs = now;
    syncClock();
  }
  if (now - tempReadMs >= 1000) {
    tempReadMs = now;
    uint8_t t[2];
    if (rd(REG_OUT_TEMP_L, t, 2)) tempC = 25.0f + (int16_t)(t[0] | (t[1] << 8)) / 256.0f;
  }

  uint8_t s[2];
  if (!rd(REG_FIFO_STATUS1, s, 2)) {
    st.readErr++;
    return;
  }
  uint16_t words = s[0] | ((s[1] & 0x01) << 8);

  // Pojistka: FIFO je prázdné a 1 s nepřišel žádný vzorek -> IMU znovu nastavit
  if (st.samples != lastSampleCount) {
    lastSampleCount = st.samples;
    lastSampleMs = now;
  } else if (words == 0 && now - lastSampleMs > 1000) {
    st.reinits++;
    configure();
    return;
  }
  if (s[1] & 0x08) st.overflows++;  // FIFO_OVR_LATCHED (nuluje se čtením)
  if (words > st.fifoMax) st.fifoMax = words;

  // Vyčíst nejvýš 32 slov na jedno volání, ať se neblokuje smyčka
  if (words > 32) words = 32;
  uint8_t w[7];
  for (uint16_t i = 0; i < words; i++) {
    if (!rd(REG_FIFO_TAG, w, 7)) {
      st.readErr++;
      return;
    }
    handleWord(w);
  }
}

bool latest(Sample &s) {
  if (!haveLast) return false;
  s = last;
  return true;
}

bool pop(Sample &s) {
  if (qTail == qHead) return false;
  s = queue[qTail];
  qTail = (qTail + 1) % QSIZE;
  return true;
}

Stats stats() {
  Stats s = st;
  s.lastSampleAgeMs = millis() - lastSampleMs;
  s.odrHz = 1e6 / (ticksPerSample * usPerTick);
  return s;
}

bool calibrateGyro(uint32_t durationMs, void (*idle)()) {
  if (!addr) return false;
  // dočasně vynulovat bias, aby se měřila surová hodnota
  float ox = biasX, oy = biasY, oz = biasZ;
  biasX = biasY = biasZ = 0;
  double sx = 0, sy = 0, sz = 0, qx = 0, qy = 0, qz = 0;
  uint32_t n = 0;
  uint32_t lastCount = st.samples;
  uint32_t t0 = millis();
  while (millis() - t0 < durationMs) {
    poll();
    if (st.samples != lastCount) {
      lastCount = st.samples;
      sx += last.gx;
      sy += last.gy;
      sz += last.gz;
      qx += last.gx * last.gx;
      qy += last.gy * last.gy;
      qz += last.gz * last.gz;
      n++;
    }
    if (idle) idle();
    delay(1);
  }
  bool ok = n >= 50;
  if (ok) {
    double mx = sx / n, my = sy / n, mz = sz / n;
    // směrodatná odchylka > 0,5 °/s = přístroj se hýbal
    double vx = qx / n - mx * mx, vy = qy / n - my * my, vz = qz / n - mz * mz;
    if (vx > 0.25 || vy > 0.25 || vz > 0.25) ok = false;
    if (ok) {
      biasX = mx;
      biasY = my;
      biasZ = mz;
    }
  }
  if (!ok) {
    biasX = ox;
    biasY = oy;
    biasZ = oz;
    return false;
  }
  Preferences p;
  p.begin("imu", false);
  p.putFloat("gbx", biasX);
  p.putFloat("gby", biasY);
  p.putFloat("gbz", biasZ);
  p.end();
  return true;
}

void gyroBias(float &bx, float &by, float &bz) {
  bx = biasX;
  by = biasY;
  bz = biasZ;
}

}  // namespace imu
