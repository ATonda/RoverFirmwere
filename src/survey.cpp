#include "survey.h"

#include <LittleFS.h>
#include <math.h>

#include <algorithm>

#include "buzzer.h"
#include "config.h"
#include "geo.h"
#include "gnss.h"
#include "imu.h"
#include "settings.h"
#include "testctl.h"

namespace survey {

static const char *MEAS_FILE = "/meas.json";

// ---------------------------------------------------------------- vzorky (1 Hz)
enum SFlags : uint8_t { SF_DIFF = 1, SF_RTCM = 0x20, SF_MOT = 0x40, SF_REJ = 0x80 };  // bity 1–2: carrSoln; RTCM = korekce z mobilu; MOT = pohyb
struct Sample {
  uint32_t t;            // UTC [s od 1970]
  int32_t dlat, dlon;    // [1e-9 °] od prvního vzorku
  int32_t he, hm;        // výška elipsoidická / nad geoidem [mm]
  uint16_t hAcc, vAcc;   // [mm]
  uint8_t sv, ft, fl;
};

static Sample *buf = nullptr;
static int cap = 0, nS = 0;
static int every = 1;          // ukládá se každá n-tá sekunda (při nedostatku RAM)
static uint32_t secCount = 0;
static double lat0 = 0, lon0 = 0;

// ---------------------------------------------------------------- souhrny
struct Meas {
  uint16_t id, no;
  char pt[25];               // bod ("" = bez bodu)
  char t[20];                // začátek sběru UTC
  uint32_t dur;              // doba sběru [s]
  uint16_t n, rej, nf;       // epoch použito / vyřazeno (odlehlé) / bez fixu
  uint16_t mr, mot;          // epoch vyřazeno kvůli pohybu / počet pohybů
  uint8_t end;               // konec: 0 doba sběru, 1 ručně, 2 přístroj odnesen
  uint16_t settle;           // doba ustálení [s]
  bool sto;                  // ustálení skončilo limitem
  double lat, lon;           // průměr v systému bodů (ETRS89 nebo ITRF podle nastavení)
  float he, hm;              // průměrná výška elipsoidická / nad geoidem (fázové centrum antény)
  float sN, sE, sU, c50, c95, ha, va, sv;
  uint16_t fx, fl, df;       // epoch RTK fix / float (s RTCM) / s korekcemi bez fází
  uint16_t hs;               // epoch HAS/PPP (fáze bez RTCM)
  bool tr;                   // poloha přepočtena ITRF -> ETRS89
  float ant;                 // výška antény + PCO
  bool hasD;                 // porovnání s bodem
  float dn, de, du, d2;
  uint8_t dflags;
};

static Meas meas[MAX_MEAS];
static int nMeas = 0;
static uint16_t nextId = 1;
static uint32_t listVer = 1;

// ---------------------------------------------------------------- stav
enum class St { Idle, Countdown, Settle, Collect, Done, Failed };
static St st = St::Idle;
static Cfg cfg;
static uint32_t phaseMs = 0;      // začátek aktuální fáze
static uint32_t settleS = 0;
static bool settleTo = false;
static uint32_t nNoFix = 0;
static uint8_t endReason = 0;

// ---------------------------------------------------------------- pohyb přístroje (IMU)
// Krátký otřes (pták, vítr) jen vyřadí epochy během něj; souvislý pohyb > 3 s = odneseno -> konec.
static constexpr float MOT_GYRO_DPS = 15.0f;   // rotace
static constexpr float MOT_ACC_G = 0.20f;      // odchylka |a| od 1 g
static constexpr uint32_t MOT_GAP_MS = 400;    // mezera, která ještě nepřeruší souvislý pohyb
static constexpr uint32_t MOT_CARRY_MS = 3000; // souvislý pohyb = odneseno
static constexpr uint32_t MOT_GUARD_MS = 1500; // epochy do 1,5 s po pohybu se vyřadí (zpoždění GNSS)
static uint32_t motLastMs = 0;     // poslední vzorek IMU s pohybem
static uint32_t motStartMs = 0;    // začátek souvislého pohybu
static int motStartN = 0;          // počet uložených epoch na začátku pohybu
static uint16_t motCount = 0;      // počet pohybů během sběru
static bool motCarry = false;      // zjištěno odnesení (vyhodnotí poll)
static uint8_t gnssJump = 0;       // po sobě jdoucí epochy daleko od průměru
static uint8_t lastSec = 255;
// Ustálení podle stability polohy: průměr posledních 30 s proti průměru 30 s o minutu dřív
static constexpr int ST_WIN = 30, ST_SPAN = 90;   // okno a celková délka [s]
// Meze z dat Kraví hora 12. 10.: zkonvergovaný HAS bloudí 1–15 cm/min, konvergence 20–50 cm/min
static constexpr float ST_MAX_H = 0.05f;          // [m] posun vodorovně za 60 s
static constexpr float ST_MAX_U = 0.08f;          // [m] posun svisle za 60 s
static float stN[ST_SPAN], stE[ST_SPAN], stU[ST_SPAN];
static int nHist = 0;                              // počet vzorků v ringu (od začátku / od pohybu)
static double stLat0 = 0, stLon0 = 0, stH0 = 0;
static float stDriftH = -1, stDriftU = -1;         // poslední vypočtený posun (pro stránku)
static char msgText[64] = "";
static char startTime[20] = "";
static uint16_t ey = 0, em = 0, ed = 0;  // datum začátku (epocha pro přepočet)
static int lastMeasId = 0;

// ---------------------------------------------------------------- pomocné
static uint32_t unixTime(const gnss::Pvt &p) {
  // dny od 1970-01-01 (H. Hinnant, days_from_civil)
  int y = p.year - (p.month <= 2);
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (p.month + (p.month > 2 ? -3 : 9)) + 2) / 5 + p.day - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int32_t days = era * 146097 + (int32_t)doe - 719468;
  return (uint32_t)days * 86400UL + p.hour * 3600UL + p.min * 60UL + p.sec;
}

// bez gmtime/strftime – ty by do firmwaru přitáhly ~20 kB knihoven
static void fmtTime(uint32_t t, char *out, size_t len) {
  int32_t z = t / 86400 + 719468;
  uint32_t sod = t % 86400;
  int32_t era = z / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int y = (int)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  unsigned d = doy - (153 * mp + 2) / 5 + 1;
  unsigned m = mp < 10 ? mp + 3 : mp - 9;
  if (m <= 2) y++;
  snprintf(out, len, "%04d-%02u-%02u %02lu:%02lu:%02lu", y, m, d, (unsigned long)(sod / 3600),
           (unsigned long)(sod / 60 % 60), (unsigned long)(sod % 60));
}

static bool sound() { return settings::profile(settings::active()).buzzer; }

static void freeBuf() {
  if (buf) free(buf);
  buf = nullptr;
  cap = nS = 0;
}

// metry na 1e-9 ° v místě prvního vzorku
static void scales(double &kN, double &kE) {
  double la = lat0 * M_PI / 180.0;
  double s2 = sin(la) * sin(la);
  const double a = 6378137.0, e2 = 0.00669438002290;
  double rn = a / sqrt(1 - e2 * s2);
  double rm = rn * (1 - e2) / (1 - e2 * s2);
  kN = rm * M_PI / 180.0 * 1e-9;
  kE = rn * cos(la) * M_PI / 180.0 * 1e-9;
}

struct Stats {
  int n = 0, rej = 0, mot = 0;
  double mN = 0, mE = 0, mHe = 0, mHm = 0;  // průměr: N/E [m] od prvního vzorku, výšky [m]
  double sN = 0, sE = 0, sU = 0;
  float c50 = 0, c95 = 0;
  double ha = 0, va = 0, sv = 0;
  int fx = 0, fl = 0, df = 0, hs = 0, rtcm = 0;
};

static void accumulate(Stats &r, double kN, double kE) {
  double sn = 0, se = 0, su = 0, sm = 0, qn = 0, qe = 0, qu = 0;
  r = Stats();
  int rej = 0;
  for (int i = 0; i < nS; i++) {
    const Sample &s = buf[i];
    if (s.fl & SF_MOT) {
      r.mot++;
      continue;
    }
    if (s.fl & SF_REJ) {
      rej++;
      continue;
    }
    double n = s.dlat * kN, e = s.dlon * kE, u = s.he / 1000.0;
    r.n++;
    sn += n;
    se += e;
    su += u;
    sm += s.hm / 1000.0;
    qn += n * n;
    qe += e * e;
    qu += u * u;
    r.ha += s.hAcc / 1000.0;
    r.va += s.vAcc / 1000.0;
    r.sv += s.sv;
    uint8_t cs = (s.fl >> 1) & 3;
    if (s.fl & SF_RTCM) r.rtcm++;
    if (cs && !(s.fl & SF_RTCM)) r.hs++;
    else if (cs == 2) r.fx++;
    else if (cs == 1) r.fl++;
    else if (s.fl & SF_DIFF) r.df++;
  }
  r.rej = rej;
  if (!r.n) return;
  double k = r.n;
  r.mN = sn / k;
  r.mE = se / k;
  r.mHe = su / k;
  r.mHm = sm / k;
  r.sN = sqrt(fmax(0, qn / k - r.mN * r.mN));
  r.sE = sqrt(fmax(0, qe / k - r.mE * r.mE));
  r.sU = sqrt(fmax(0, qu / k - r.mHe * r.mHe));
  r.ha /= k;
  r.va /= k;
  r.sv /= k;
}

// Úplný výpočet: vyřazení odlehlých (> 3σ, aspoň 2 cm vodorovně / 3 cm svisle) a CEP
static void compute(Stats &r) {
  double kN, kE;
  scales(kN, kE);
  for (int i = 0; i < nS; i++) buf[i].fl &= ~SF_REJ;
  accumulate(r, kN, kE);
  if (r.n >= 10) {
    double t2 = fmax(3 * sqrt(r.sN * r.sN + r.sE * r.sE), 0.02);
    double tu = fmax(3 * r.sU, 0.03);
    for (int i = 0; i < nS; i++) {
      Sample &s = buf[i];
      if (s.fl & SF_MOT) continue;
      double dn = s.dlat * kN - r.mN, de = s.dlon * kE - r.mE, du = s.he / 1000.0 - r.mHe;
      if (sqrt(dn * dn + de * de) > t2 || fabs(du) > tu) s.fl |= SF_REJ;
    }
    accumulate(r, kN, kE);
  }
  if (!r.n) return;
  float *d = (float *)malloc(sizeof(float) * r.n);
  if (!d) return;
  int k = 0;
  for (int i = 0; i < nS; i++) {
    const Sample &s = buf[i];
    if (s.fl & (SF_REJ | SF_MOT)) continue;
    double dn = s.dlat * kN - r.mN, de = s.dlon * kE - r.mE;
    d[k++] = sqrt(dn * dn + de * de);
  }
  std::sort(d, d + k);
  r.c50 = d[(k - 1) / 2];
  r.c95 = d[(int)ceil(0.95 * k) - 1];
  free(d);
}

// ---------------------------------------------------------------- soubor souhrnů
static void measToJson(const Meas &m, JsonObject o) {
  o["id"] = m.id;
  o["no"] = m.no;
  o["pt"] = m.pt;
  o["t"] = m.t;
  o["dur"] = m.dur;
  o["n"] = m.n;
  o["rej"] = m.rej;
  o["nf"] = m.nf;
  o["mr"] = m.mr;
  o["mot"] = m.mot;
  o["end"] = m.end;
  o["st"] = m.settle;
  o["sto"] = m.sto;
  o["lat"] = serialized(String(m.lat, 9));
  o["lon"] = serialized(String(m.lon, 9));
  o["he"] = serialized(String(m.he, 3));
  o["hm"] = serialized(String(m.hm, 3));
  o["sN"] = serialized(String(m.sN, 3));
  o["sE"] = serialized(String(m.sE, 3));
  o["sU"] = serialized(String(m.sU, 3));
  o["c50"] = serialized(String(m.c50, 3));
  o["c95"] = serialized(String(m.c95, 3));
  o["ha"] = serialized(String(m.ha, 3));
  o["va"] = serialized(String(m.va, 3));
  o["sv"] = serialized(String(m.sv, 1));
  o["fx"] = m.fx;
  o["fl"] = m.fl;
  o["df"] = m.df;
  o["hs"] = m.hs;
  o["tr"] = m.tr;
  o["ant"] = serialized(String(m.ant, 3));
  if (m.hasD) {
    JsonObject d = o["d"].to<JsonObject>();
    d["n"] = serialized(String(m.dn, 3));
    d["e"] = serialized(String(m.de, 3));
    d["u"] = serialized(String(m.du, 3));
    d["d2"] = serialized(String(m.d2, 3));
    d["f"] = m.dflags;
  }
}

static void measFromJson(JsonObjectConst o, Meas &m) {
  memset(&m, 0, sizeof(m));
  m.id = o["id"] | 0;
  m.no = o["no"] | 0;
  strlcpy(m.pt, o["pt"] | "", sizeof(m.pt));
  strlcpy(m.t, o["t"] | "", sizeof(m.t));
  m.dur = o["dur"] | 0;
  m.n = o["n"] | 0;
  m.rej = o["rej"] | 0;
  m.nf = o["nf"] | 0;
  m.mr = o["mr"] | 0;
  m.mot = o["mot"] | 0;
  m.end = o["end"] | 0;
  m.settle = o["st"] | 0;
  m.sto = o["sto"] | false;
  m.lat = o["lat"] | 0.0;
  m.lon = o["lon"] | 0.0;
  m.he = o["he"] | 0.0f;
  m.hm = o["hm"] | 0.0f;
  m.sN = o["sN"] | 0.0f;
  m.sE = o["sE"] | 0.0f;
  m.sU = o["sU"] | 0.0f;
  m.c50 = o["c50"] | 0.0f;
  m.c95 = o["c95"] | 0.0f;
  m.ha = o["ha"] | 0.0f;
  m.va = o["va"] | 0.0f;
  m.sv = o["sv"] | 0.0f;
  m.fx = o["fx"] | 0;
  m.fl = o["fl"] | 0;
  m.df = o["df"] | 0;
  m.hs = o["hs"] | 0;
  m.tr = o["tr"] | false;
  m.ant = o["ant"] | 0.0f;
  JsonObjectConst d = o["d"];
  m.hasD = !d.isNull();
  if (m.hasD) {
    m.dn = d["n"] | 0.0f;
    m.de = d["e"] | 0.0f;
    m.du = d["u"] | 0.0f;
    m.d2 = d["d2"] | 0.0f;
    m.dflags = d["f"] | 0;
  }
}

static bool save() {
  JsonDocument doc;
  doc["next"] = nextId;
  JsonArray a = doc["meas"].to<JsonArray>();
  for (int i = 0; i < nMeas; i++) measToJson(meas[i], a.add<JsonObject>());
  String s;
  serializeJson(doc, s);
  String tmp = String(MEAS_FILE) + ".tmp";
  File f = LittleFS.open(tmp, "w");
  if (!f) return false;
  size_t n = f.print(s);
  f.close();
  if (n != s.length()) return false;
  LittleFS.remove(MEAS_FILE);
  return LittleFS.rename(tmp, MEAS_FILE);
}

bool begin() {
  File f = LittleFS.open(MEAS_FILE, "r");
  if (!f) return true;
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, f);
  f.close();
  if (e) return false;
  nextId = doc["next"] | 1;
  nMeas = 0;
  for (JsonObjectConst o : doc["meas"].as<JsonArrayConst>()) {
    if (nMeas >= MAX_MEAS) break;
    measFromJson(o, meas[nMeas++]);
  }
  return true;
}

// ---------------------------------------------------------------- průběh
static void fail(const char *why) {
  st = St::Failed;
  strlcpy(msgText, why, sizeof(msgText));
  if (sound()) buzzer::beep(800, 200, 150, 3);
}

static void startCollect() {
  st = St::Collect;
  phaseMs = millis();
  nS = 0;
  secCount = 0;
  nNoFix = 0;
  motCount = 0;
  motCarry = false;
  gnssJump = 0;
  endReason = 0;
  startTime[0] = 0;
  if (sound()) buzzer::beep(2700, 80, 120, 2);
}

static void finishCollect() {
  if (nS < 5) return fail("málo epoch s fixem (méně než 5)");
  Stats r;
  compute(r);
  if (!r.n) return fail("výpočet se nepovedl");

  double kN, kE;
  scales(kN, kE);
  // průměr fázového centra antény
  double lat = lat0 + r.mN / kN * 1e-9;
  double lon = lon0 + r.mE / kE * 1e-9;
  bool rtk = r.rtcm * 2 > r.n;  // většina epoch s RTCM ze sítě = ETRS89, jinak ITRF
  double ep = ey ? geo::epochYear(ey, em, ed) : 2026.0;
  const settings::Profile &pr = settings::profile(settings::active());

  Meas m;
  memset(&m, 0, sizeof(m));
  m.id = nextId++;
  strlcpy(m.t, startTime, sizeof(m.t));
  m.dur = (millis() - phaseMs) / 1000;
  m.n = r.n;
  m.rej = r.rej;
  m.nf = nNoFix;
  m.mr = r.mot;
  m.mot = motCount;
  m.end = endReason;
  m.settle = settleS;
  m.sto = settleTo;
  m.he = r.mHe;
  m.hm = r.mHm;
  m.sN = r.sN;
  m.sE = r.sE;
  m.sU = r.sU;
  m.c50 = r.c50;
  m.c95 = r.c95;
  m.ha = r.ha;
  m.va = r.va;
  m.sv = r.sv;
  m.fx = r.fx;
  m.fl = r.fl;
  m.df = r.df;
  m.hs = r.hs;
  m.ant = pr.antH + pr.pco;
  m.tr = testctl::needsTransform(rtk);
  double latT = lat, lonT = lon;
  testctl::toPointFrame(latT, lonT, r.mHe, rtk, ep);
  m.lat = latT;
  m.lon = lonT;

  String err;
  if (cfg.newName[0]) {
    // nový bod = značka pod anténou
    char note[49];
    snprintf(note, sizeof(note), "změřeno %.10s, %lu s, σ2D %.1f cm", startTime, (unsigned long)m.dur,
             sqrt(r.sN * r.sN + r.sE * r.sE) * 100);
    if (testctl::addPoint(cfg.newName, latT, lonT, r.mHe - m.ant, note, err) < 0) {
      snprintf(msgText, sizeof(msgText), "bod neuložen: %s", err.c_str());
    }
    strlcpy(m.pt, cfg.newName, sizeof(m.pt));
  } else if (cfg.point[0]) {
    strlcpy(m.pt, cfg.point, sizeof(m.pt));
    for (int i = 0; i < testctl::pointCount(); i++) {
      if (strcmp(testctl::pointName(i), cfg.point) != 0) continue;
      testctl::Diff d;
      if (testctl::compare(i, lat, lon, r.mHe, r.mHm, rtk, ep, d)) {
        m.hasD = true;
        m.dn = d.n;
        m.de = d.e;
        m.du = d.u;
        m.d2 = d.d2;
        m.dflags = d.flags;
      }
      break;
    }
  }
  uint16_t no = 0;
  for (int i = 0; i < nMeas; i++)
    if (strcmp(meas[i].pt, m.pt) == 0 && meas[i].no > no) no = meas[i].no;
  m.no = no + 1;

  if (nMeas == MAX_MEAS) {
    memmove(meas, meas + 1, sizeof(Meas) * (MAX_MEAS - 1));
    nMeas--;
  }
  meas[nMeas++] = m;
  lastMeasId = m.id;
  listVer++;
  if (!save() && !msgText[0]) strlcpy(msgText, "zápis do paměti selhal", sizeof(msgText));
  st = St::Done;
  if (sound()) buzzer::tones2(1500, 2500);
}

bool start(const Cfg &c, String &err) {
  if (running()) {
    err = "měření už běží";
    return false;
  }
  if (c.durS < 10 || c.durS > 7200) {
    err = "doba sběru 10–7200 s";
    return false;
  }
  if (c.countdownS > 300 || c.settleMaxS > 3600 || c.settleAcc < 0 || c.settleAcc > 100) {
    err = "neplatné nastavení odpočtu nebo ustálení";
    return false;
  }
  if (c.newName[0]) {
    if (testctl::pointCount() >= testctl::MAX_POINTS) {
      err = "seznam bodů je plný";
      return false;
    }
    for (int i = 0; i < testctl::pointCount(); i++) {
      if (strcmp(testctl::pointName(i), c.newName) == 0) {
        err = "bod s tímto názvem už existuje";
        return false;
      }
    }
  }
  freeBuf();
  int need = c.durS + 2;
  int avail = (int)ESP.getMaxAllocHeap() - 40000;  // rezerva pro web, BT a JSON
  int fit = avail / (int)sizeof(Sample);
  if (fit < 60) {
    err = "málo volné paměti";
    return false;
  }
  every = (need + fit - 1) / fit;
  if (every < 1) every = 1;
  cap = need / every + 2;
  buf = (Sample *)malloc(sizeof(Sample) * cap);
  if (!buf) {
    cap = 0;
    err = "málo volné paměti";
    return false;
  }
  cfg = c;
  msgText[0] = 0;
  settleS = 0;
  settleTo = false;
  nHist = 0;
  nS = 0;
  lastSec = 255;
  motLastMs = 0;
  motCarry = false;
  st = St::Countdown;
  phaseMs = millis();
  return true;
}

void finish() {
  if (st == St::Collect) {
    endReason = 1;
    finishCollect();
  } else if (running()) cancel();
}

// Odnesení přístroje: sběr ukončit a uložit epochy před začátkem pohybu
static void carried() {
  motCarry = false;
  if (st == St::Collect) {
    endReason = 2;
    nS = motStartN > 2 ? motStartN - 2 : 0;  // epochy těsně před pohybem jsou nejisté
    finishCollect();
    if (st == St::Done) strlcpy(msgText, "přístroj odnesen – uložena část před pohybem", sizeof(msgText));
  } else if (st == St::Settle) {
    fail("přístroj odnesen během ustálení");
    freeBuf();
  }
}

void onImu(const imu::Sample &s) {
  if (st != St::Settle && st != St::Collect) return;  // při odpočtu se s přístrojem manipuluje
  float g = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  float a = fabsf(sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az) - 1.0f);
  if (g < MOT_GYRO_DPS && a < MOT_ACC_G) return;
  uint32_t now = millis();
  if (now - motLastMs > MOT_GAP_MS) {  // nový pohyb
    motStartMs = now;
    motStartN = nS;
    if (st == St::Collect) motCount++;
    if (st == St::Settle) nHist = 0;   // ustálení začne znovu
  }
  motLastMs = now;
  if (now - motStartMs >= MOT_CARRY_MS) motCarry = true;
}

void cancel() {
  if (!running()) return;
  st = St::Idle;
  freeBuf();
}

bool running() { return st == St::Countdown || st == St::Settle || st == St::Collect; }

int phase() { return (int)st; }

uint32_t phaseSeconds() { return st == St::Idle ? 0 : (millis() - phaseMs) / 1000; }

void poll() {
  uint32_t now = millis();
  if (motCarry) carried();
  switch (st) {
    case St::Countdown:
      if (now - phaseMs >= cfg.countdownS * 1000UL) {
        st = St::Settle;
        phaseMs = now;
        nHist = 0;
        if (sound()) buzzer::beep(2000, 80, 0, 1);
      }
      break;
    case St::Settle:
      if (cfg.settleMaxS && now - phaseMs >= cfg.settleMaxS * 1000UL) {
        settleS = (now - phaseMs) / 1000;
        settleTo = true;
        startCollect();
      }
      break;
    case St::Collect:
      if (now - phaseMs >= cfg.durS * 1000UL) finishCollect();
      break;
    default: break;
  }
}

void onPvt() {
  if (st != St::Settle && st != St::Collect) return;
  const gnss::Pvt &p = gnss::pvt();
  if (!p.timeValid || p.sec == lastSec) return;
  lastSec = p.sec;
  bool fixOk = p.gnssFixOk && p.fixType >= 3;

  if (st == St::Settle) {
    if (!fixOk) {
      nHist = 0;
      return;
    }
    if (cfg.settleMaxS == 0) {
      settleS = (millis() - phaseMs) / 1000;
      startCollect();
      return;
    }
    // Poloha stabilní: průměr posledních 30 s se proti průměru 30 s o minutu dřív posunul
    // nejvýš o 5 cm vodorovně a 8 cm svisle (konvergence HAS se pozná podle driftu, ne podle hAcc).
    if (nHist == 0) {
      stLat0 = p.lat;
      stLon0 = p.lon;
      stH0 = p.hEll;
      stDriftH = stDriftU = -1;
    }
    {
      double la = stLat0 * M_PI / 180.0;
      int i = nHist % ST_SPAN;
      stN[i] = (float)((p.lat - stLat0) * (111132.954 - 559.822 * cos(2 * la)));
      stE[i] = (float)((p.lon - stLon0) * 111412.84 * cos(la));
      stU[i] = (float)(p.hEll - stH0);
      nHist++;
    }
    if (nHist >= ST_SPAN) {
      double aN = 0, aE = 0, aU = 0, bN = 0, bE = 0, bU = 0;
      for (int k = 0; k < ST_WIN; k++) {
        int ia = (nHist - 1 - k) % ST_SPAN;           // posledních 30 s
        int ib = (nHist - ST_SPAN + k) % ST_SPAN;     // nejstarších 30 s v okně
        aN += stN[ia];
        aE += stE[ia];
        aU += stU[ia];
        bN += stN[ib];
        bE += stE[ib];
        bU += stU[ib];
      }
      stDriftH = (float)(sqrt((aN - bN) * (aN - bN) + (aE - bE) * (aE - bE)) / ST_WIN);
      stDriftU = (float)(fabs(aU - bU) / ST_WIN);
      bool accOk = cfg.settleAcc <= 0 || p.hAcc <= cfg.settleAcc;
      if (accOk && stDriftH < ST_MAX_H && stDriftU < ST_MAX_U) {
        settleS = (millis() - phaseMs) / 1000;
        startCollect();
      }
    }
    return;
  }

  // sběr
  if (!fixOk) {
    nNoFix++;
    return;
  }
  if (!startTime[0]) {
    char t[24];
    fmtTime(unixTime(p), t, sizeof(t));
    strlcpy(startTime, t, sizeof(startTime));
    ey = p.year;
    em = p.month;
    ed = p.day;
  }
  if (secCount++ % every) return;
  if (nS >= cap) return;
  bool moving = motLastMs && millis() - motLastMs < MOT_GUARD_MS;
  if (nS == 0) {
    lat0 = p.lat;
    lon0 = p.lon;
  }
  Sample &s = buf[nS++];
  s.t = unixTime(p);
  s.dlat = (int32_t)lround((p.lat - lat0) * 1e9);
  s.dlon = (int32_t)lround((p.lon - lon0) * 1e9);
  s.he = (int32_t)lround(p.hEll * 1000);
  s.hm = (int32_t)lround(p.hMSL * 1000);
  s.hAcc = (uint16_t)fminf(p.hAcc * 1000, 65535);
  s.vAcc = (uint16_t)fminf(p.vAcc * 1000, 65535);
  s.sv = p.numSV;
  s.ft = p.fixType;
  s.fl = (uint8_t)((p.diffSoln ? SF_DIFF : 0) | ((p.carrSoln & 3) << 1) | (moving ? SF_MOT : 0) |
                   (gnss::rtcmActive() ? SF_RTCM : 0));

  // Záloha k IMU: 5 epoch po sobě daleko od průměru (> 2 m a > 10σ) = přístroj přenesen
  if (nS >= 30 && !moving) {
    double kN, kE;
    scales(kN, kE);
    Stats r;
    accumulate(r, kN, kE);
    double dn = s.dlat * kN - r.mN, de = s.dlon * kE - r.mE;
    double lim = fmax(2.0, 10 * sqrt(r.sN * r.sN + r.sE * r.sE));
    if (r.n >= 20 && sqrt(dn * dn + de * de) > lim) {
      if (++gnssJump >= 5) {
        motStartN = nS - 5;
        motCarry = true;
      }
    } else {
      gnssJump = 0;
    }
  }
}

// ---------------------------------------------------------------- JSON a export
void statusJson(JsonDocument &doc) {
  static const char *const ST[] = {"idle", "countdown", "settle", "collect", "done", "failed"};
  uint32_t now = millis();
  doc["st"] = ST[(int)st];
  doc["ver"] = listVer;
  doc["last"] = lastMeasId;
  doc["raw"] = nS;
  if (msgText[0]) doc["msg"] = msgText;
  if (st == St::Idle) return;
  doc["pt"] = cfg.point;
  doc["new"] = cfg.newName;
  doc["dur"] = cfg.durS;
  doc["every"] = every;
  uint32_t el = (now - phaseMs) / 1000;
  doc["el"] = el;
  if (st == St::Countdown) doc["left"] = cfg.countdownS > el ? cfg.countdownS - el : 0;
  if (st == St::Settle) {
    doc["max"] = cfg.settleMaxS;
    doc["hAcc"] = serialized(String(gnss::pvt().hAcc, 3));
    doc["need"] = nHist < ST_SPAN ? ST_SPAN - nHist : 0;  // s do prvního vyhodnocení stability
    if (stDriftH >= 0) {
      doc["drH"] = serialized(String(stDriftH, 3));  // posun polohy za 60 s
      doc["drU"] = serialized(String(stDriftU, 3));
    }
  }
  if (st == St::Collect || st == St::Done) {
    doc["settle"] = settleS;
    doc["sto"] = settleTo;
    doc["nf"] = nNoFix;
    doc["mot"] = motCount;
  }
  if ((st == St::Collect || st == St::Done) && nS) {
    double kN, kE;
    scales(kN, kE);
    Stats r;
    if (st == St::Collect) {
      for (int i = 0; i < nS; i++) buf[i].fl &= ~SF_REJ;
    }
    accumulate(r, kN, kE);  // po skončení zůstávají vyřazené epochy označené
    doc["n"] = r.n;
    doc["rej"] = r.rej;
    doc["mr"] = r.mot;
    doc["sN"] = serialized(String(r.sN, 3));
    doc["sE"] = serialized(String(r.sE, 3));
    doc["sU"] = serialized(String(r.sU, 3));
    doc["fx"] = r.fx;
    doc["fl"] = r.fl;
    doc["df"] = r.df;
    doc["hs"] = r.hs;
    doc["ha"] = serialized(String(r.ha, 3));
  }
}

void listJson(JsonDocument &doc) {
  doc["ver"] = listVer;
  doc["max"] = MAX_MEAS;
  JsonArray a = doc["meas"].to<JsonArray>();
  for (int i = 0; i < nMeas; i++) measToJson(meas[i], a.add<JsonObject>());
}

uint32_t listVersion() { return listVer; }

bool remove(int id, String &err) {
  for (int i = 0; i < nMeas; i++) {
    if (meas[i].id != id) continue;
    memmove(meas + i, meas + i + 1, sizeof(Meas) * (nMeas - i - 1));
    nMeas--;
    listVer++;
    if (!save()) {
      err = "zápis do paměti selhal";
      return false;
    }
    return true;
  }
  err = "měření nenalezeno";
  return false;
}

bool removePoint(const char *point, String &err) {
  int k = 0;
  for (int i = 0; i < nMeas; i++)
    if (strcmp(meas[i].pt, point) != 0) meas[k++] = meas[i];
  if (k == nMeas) return true;
  nMeas = k;
  listVer++;
  if (!save()) {
    err = "zápis do paměti selhal";
    return false;
  }
  return true;
}

int rawCount() { return nS; }

size_t rawLine(int i, char *out, size_t len) {
  if (i < 0) return snprintf(out, len, "cas_utc;lat;lon;h_ell;h_msl;fix;rtk;korekce;hAcc;vAcc;sv;vyrazeno(1=odlehle,2=pohyb)\r\n");
  if (i >= nS) return 0;
  const Sample &s = buf[i];
  char t[24];
  fmtTime(s.t, t, sizeof(t));
  static const char *const FT[] = {"-", "DR", "2D", "3D", "GNSS+DR", "cas"};
  return snprintf(out, len, "%s;%.9f;%.9f;%.3f;%.3f;%s;%u;%u;%.3f;%.3f;%u;%u\r\n", t, lat0 + s.dlat * 1e-9,
                  lon0 + s.dlon * 1e-9, s.he / 1000.0, s.hm / 1000.0, s.ft < 6 ? FT[s.ft] : "?", (s.fl >> 1) & 3,
                  s.fl & SF_DIFF, s.hAcc / 1000.0, s.vAcc / 1000.0, s.sv, (s.fl & SF_MOT) ? 2 : (s.fl & SF_REJ) ? 1 : 0);
}

}  // namespace survey
