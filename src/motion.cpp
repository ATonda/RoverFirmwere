#include "motion.h"

#include "gnss.h"

namespace motion {

// ---------------------------------------------------------------- meze (první odhad z testů 20. 10.)
static constexpr int WIN = 240;                 // okno 2 s při 120 Hz
static constexpr int STEP = 60;                 // vyhodnocení po 0,5 s
static constexpr float STILL_GYRO = 3.0f;       // klid: |ω| < 3 °/s
static constexpr float STILL_DEV = 0.05f;       //       ||a| − 1 g| < 0,05 g
static constexpr float STILL_STD = 0.015f;      //       rozptyl |a| < 0,015 g
static constexpr float STEP_SCORE = 0.35f;      // kroky: autokorelace > 0,35
static constexpr float STEP_STD_MIN = 0.05f;    //        a rozptyl |a| 0,05–0,6 g
static constexpr float STEP_STD_MAX = 0.6f;
static constexpr float SHARP_GYRO = 30.0f;      // manipulace: |ω| > 30 °/s
static constexpr float SHARP_DEV = 0.3f;        //             nebo náraz > 0,3 g (bez kroků)
static constexpr float V_STOP = 0.2f;           // [m/s] stání
static constexpr float V_HANDLE = 0.3f;         // [m/s] manipulace = na místě (pod touto rychlostí)
static constexpr float V_WALK_MIN = 0.4f;       // [m/s] pohyb chůzí podle GNSS (i bez zřetelných kroků v ruce)
static constexpr float WALK_STD_MIN = 0.06f;    // [g] chůze podle rychlosti jen s otřesy krabičky v ruce (auto 20–45 mg)
static constexpr float V_CAR = 2.0f;            // [m/s] jízda: nad touto rychlostí hladký průběh bez kroků
static constexpr float CAR_STD_MAX = 0.06f;     // [g]   rozptyl |a| pod 60 mg
static constexpr float V_WALK = 3.0f;           // [m/s] nad = ne chůze
static constexpr float V_DRIVE = 6.0f;          // [m/s] jistě jízda
static constexpr int N_MEASURE = 20;            // 10 s klidu -> Měřím
static constexpr int N_DRIVE_FAST = 10;         // 5 s nad 6 m/s -> Jedu
static constexpr int N_DRIVE_MED = 20;          // 10 s nad 3 m/s bez kroků -> Jedu
static constexpr int N_DRIVE_SMOOTH = 6;        // 3 s nad 2 m/s, hladce, bez kroků -> Jedu (rozjezd)
static constexpr int N_WALK_FROM_DRIVE = 20;    // 10 s kroků -> z jízdy Chodím
static constexpr int N_WALK = 6;                // 3 s kroků -> Chodím
static constexpr int N_HANDLE = 2;              // 1 s prudkých pohybů na místě -> Manipulace (ze stání, z jízdy)
static constexpr int N_HANDLE_WALK = 6;         // 3 s prudkých pohybů na místě -> z chůze Manipulace
static constexpr uint32_t DRIVE_RESUME_MS = 120000;  // stání v autě do 2 min: rozjezd = Jedu
static constexpr uint32_t MODEL_MIN_MS = 2000;  // přepnutí modelu nejvýš 1× za 2 s

static float bufA[WIN], bufG[WIN];  // |a| [g], |ω| [°/s]
static int head = 0, filled = 0, sinceEval = 0;
static Features f;
static State cur = HANDLE;
static State lastMoving = HANDLE;    // poslední stav s pohybem (rozjezd po stání v autě)
static uint32_t measureSince = 0;
static int nStill = 0, nSteps = 0, nSharpSlow = 0, nWalkV = 0, nFast = 0, nMed = 0, nSmooth = 0;
static bool autoMode = false;
static uint8_t sentModel = 0xFF;
static uint32_t sentMs = 0;
static uint32_t nChanges = 0;
static bool evalPending = false;

void onImu(const imu::Sample &s) {
  bufA[head] = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  bufG[head] = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  head = (head + 1) % WIN;
  if (filled < WIN) filled++;
  if (++sinceEval >= STEP) {
    sinceEval = 0;
    if (filled == WIN) evalPending = true;
  }
}

void onPvt() {
  const gnss::Pvt &p = gnss::pvt();
  f.speed = (p.valid && p.gnssFixOk && p.fixType >= 2 && p.sAcc < 1.0f) ? p.gSpeed : -1.0f;
}

// Znaky okna: rozptyl |a|, odchylka od 1 g, max |ω|, autokorelace kroků
static void computeFeatures() {
  float mean = 0, gmax = 0, dev = 0;
  for (int i = 0; i < WIN; i++) {
    mean += bufA[i];
    if (bufG[i] > gmax) gmax = bufG[i];
    float d = fabsf(bufA[i] - 1.0f);
    if (d > dev) dev = d;
  }
  mean /= WIN;
  static float x[WIN];
  float var = 0;
  for (int i = 0; i < WIN; i++) {
    x[i] = bufA[(head + i) % WIN] - mean;  // časově seřazené
    var += x[i] * x[i];
  }
  f.accStd = sqrtf(var / WIN);
  f.accDev = dev;
  f.gyroMax = gmax;
  // autokorelace pro zpoždění 0,33–0,8 s (kroky 1,25–3 Hz)
  float best = 0;
  int bestLag = 0;
  if (var > 1e-6f) {
    for (int lag = 40; lag <= 96; lag++) {
      float s = 0;
      for (int i = 0; i + lag < WIN; i++) s += x[i] * x[i + lag];
      float r = s / var * (float)WIN / (float)(WIN - lag);
      if (r > best) {
        best = r;
        bestLag = lag;
      }
    }
  }
  f.stepScore = best;
  f.steps = best > STEP_SCORE && f.accStd > STEP_STD_MIN && f.accStd < STEP_STD_MAX;
  f.stepHz = f.steps && bestLag ? 120.0f / bestLag : 0;
  f.still = gmax < STILL_GYRO && dev < STILL_DEV && f.accStd < STILL_STD;
  f.sharp = !f.steps && (gmax > SHARP_GYRO || dev > SHARP_DEV);
}

static void setState(State s) {
  if (s == cur) return;
  State old = cur;
  if (s == MEASURE) measureSince = millis();
  if (old != MEASURE) lastMoving = old;
  cur = s;
  nChanges++;
  Serial.printf("[pohyb] %s -> %s (v %.1f m/s, |a| σ %.0f mg, ω max %.0f °/s, kroky %.2f)\n", stateName(old),
                stateName(s), f.speed, f.accStd * 1000, f.gyroMax, f.stepScore);
}

static void evaluate() {
  computeFeatures();
  float v = f.speed;  // -1 = bez fixu
  nStill = f.still ? nStill + 1 : 0;
  nSteps = f.steps ? nSteps + 1 : 0;
  nSharpSlow = (f.sharp && v < V_HANDLE) ? nSharpSlow + 1 : 0;  // i bez fixu (v = -1)
  nWalkV = (v > V_WALK_MIN && v <= V_WALK && f.accStd > WALK_STD_MIN) ? nWalkV + 1 : 0;
  nSmooth = (v > V_CAR && f.accStd < CAR_STD_MAX && !f.steps) ? nSmooth + 1 : 0;
  nFast = v > V_DRIVE ? nFast + 1 : 0;
  nMed = (v > V_WALK && !f.steps) ? nMed + 1 : 0;

  if (cur == MEASURE) {
    if (f.still) return;
    // pohyb: hned ven ze stacionárního modelu
    if (f.steps) setState(WALK);
    else if (f.sharp) setState(HANDLE);
    else if (v > V_WALK || (lastMoving == DRIVE && millis() - measureSince < DRIVE_RESUME_MS)) setState(DRIVE);
    else setState(HANDLE);
    return;
  }
  if (nStill >= N_MEASURE && v < V_STOP) setState(MEASURE);
  else if (cur != DRIVE && (nFast >= N_DRIVE_FAST || nMed >= N_DRIVE_MED || nSmooth >= N_DRIVE_SMOOTH)) setState(DRIVE);
  else if (cur == DRIVE && nSteps >= N_WALK_FROM_DRIVE && v < V_WALK) setState(WALK);
  else if (cur == HANDLE && (nSteps >= N_WALK || nWalkV >= N_WALK) && v < V_WALK) setState(WALK);
  else if (cur == WALK && nSharpSlow >= N_HANDLE_WALK) setState(HANDLE);
  else if (cur == DRIVE && nSharpSlow >= N_HANDLE) setState(HANDLE);
}

uint8_t model() {
  switch (cur) {
    case MEASURE: return 2;   // stationary
    case DRIVE: return 4;     // automotive
    default: return 3;        // pedestrian
  }
}

void poll() {
  if (evalPending) {
    evalPending = false;
    evaluate();
  }
  if (!autoMode || gnss::measStatic()) return;
  uint8_t m = model();
  if (m != sentModel && millis() - sentMs >= MODEL_MIN_MS) {
    sentMs = millis();
    if (gnss::setDynAuto(m)) sentModel = m;
  }
}

void setAuto(bool on) {
  autoMode = on;
  sentModel = 0xFF;  // profil poslal svůj model – po startu znovu podle stavu
}

bool autoOn() { return autoMode; }
State state() { return cur; }

const char *stateName(State s) {
  switch (s) {
    case MEASURE: return "Měřím";
    case HANDLE: return "Manipulace";
    case WALK: return "Chodím";
    case DRIVE: return "Jedu";
  }
  return "?";
}

const Features &features() { return f; }
uint32_t changes() { return nChanges; }

}  // namespace motion
