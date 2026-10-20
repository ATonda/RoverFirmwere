#include "testctl.h"

#include <LittleFS.h>
#include <math.h>

#include <memory>
#include <new>

#include "geo.h"
#include "settings.h"

namespace testctl {

static const char *POINTS_FILE = "/points.json";

struct Point {
  char name[25];
  double lat, lon;   // ETRS89 [°]
  double h;          // elipsoidická výška ETRS89 [m], NAN = neznámá
  double bpv;        // normální výška Bpv [m], NAN = neznámá
  double y, x;       // S-JTSK [m], 0 = nezadáno (jen informace, přepočet dělá stránka)
  uint8_t pq;        // PosQ
  char typ[13];      // druh bodu (TB, ZhB, nivelační…)
  char note[49];     // poznámka
};

static Point points[MAX_POINTS];
static int nPoints = 0;
static int sel = -1;
static Frame frame = Frame::Auto;

// ---------------------------------------------------------------- odchylka od bodu (živě)
struct Delta {
  bool ok = false;        // je vybraný bod a platná poloha
  bool fixOk = false;
  Diff d;
  float hAcc = 0, vAcc = 0;
};
static Delta cur;
static double shiftN = 0, shiftE = 0;  // posun ITRF -> ETRS89 v aktuální epoše (informace)

// ---------------------------------------------------------------- test doby do fixu
enum class TState { Idle, Run, Pause, Done };
struct Run {
  uint8_t kind;
  uint32_t tFix, tCorr, tFloat, tRtk, tAcc;  // ms od restartu, 0 = nedosaženo
  bool timeout;
};
static TState tstate = TState::Idle;
static TtffCfg tcfg;
static Run runs[MAX_RUNS];
static int nRuns = 0;
static Run curRun;
static uint32_t runStartMs = 0, pauseUntilMs = 0;

// ---------------------------------------------------------------- pomocné
static const char *frameName(Frame f) {
  return f == Frame::Etrs ? "etrs" : f == Frame::Itrf ? "itrf" : "auto";
}

static const char *goalName(Goal g) {
  switch (g) {
    case Goal::Corr: return "corr";
    case Goal::Float: return "float";
    case Goal::RtkFix: return "rtk";
    case Goal::Acc: return "acc";
    default: return "fix";
  }
}

static const char *kindName(uint8_t k) {
  static const char *const K[] = {"hot", "warm", "cold"};
  return k < 3 ? K[k] : "?";
}


static void putNum(JsonObject o, const char *k, double v, int dec) {
  if (isnan(v)) o[k] = nullptr;
  else o[k] = serialized(String(v, dec));
}

static double getNum(JsonObjectConst o, const char *k) {
  JsonVariantConst v = o[k];
  if (v.isNull()) return NAN;
  if (v.is<const char *>()) {
    String s = v.as<const char *>();
    s.trim();
    if (!s.length()) return NAN;
    s.replace(',', '.');
    return s.toDouble();
  }
  return v.as<double>();
}

static void pointsToJson(JsonDocument &doc) {
  doc["sel"] = sel;
  doc["frame"] = frameName(frame);
  JsonArray a = doc["points"].to<JsonArray>();
  for (int i = 0; i < nPoints; i++) {
    const Point &p = points[i];
    JsonObject o = a.add<JsonObject>();
    o["name"] = p.name;
    putNum(o, "lat", p.lat, 9);
    putNum(o, "lon", p.lon, 9);
    putNum(o, "h", p.h, 3);
    putNum(o, "bpv", p.bpv, 3);
    putNum(o, "y", p.y ? p.y : NAN, 2);
    putNum(o, "x", p.x ? p.x : NAN, 2);
    o["pq"] = p.pq;
    o["typ"] = p.typ;
    o["pozn"] = p.note;
  }
}

static bool writePoints() {
  JsonDocument doc;
  pointsToJson(doc);
  String s;
  serializeJson(doc, s);
  String tmp = String(POINTS_FILE) + ".tmp";
  File f = LittleFS.open(tmp, "w");
  if (!f) return false;
  size_t n = f.print(s);
  f.close();
  if (n != s.length()) return false;
  LittleFS.remove(POINTS_FILE);
  return LittleFS.rename(tmp, POINTS_FILE);
}

static bool parsePoints(const char *json, String &err) {
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, json);
  if (e) {
    err = String("chybný JSON: ") + e.c_str();
    return false;
  }
  JsonArrayConst a = doc["points"];
  if ((int)a.size() > MAX_POINTS) {
    err = String("nejvýš ") + MAX_POINTS + " bodů";
    return false;
  }
  // pomocná kopie jen po dobu načítání (statická by trvale brala 4 kB RAM)
  std::unique_ptr<Point[]> tmp(new (std::nothrow) Point[MAX_POINTS]);
  if (!tmp) {
    err = "málo paměti";
    return false;
  }
  int n = 0;
  for (JsonObjectConst o : a) {
    Point &p = tmp[n];
    memset(&p, 0, sizeof(p));
    const char *name = o["name"] | "";
    size_t l = strlen(name);
    if (l == 0 || l >= sizeof(p.name)) {
      err = "název bodu musí mít 1–24 znaků";
      return false;
    }
    strlcpy(p.name, name, sizeof(p.name));
    p.lat = getNum(o, "lat");
    p.lon = getNum(o, "lon");
    p.h = getNum(o, "h");
    p.bpv = getNum(o, "bpv");
    double y = getNum(o, "y"), x = getNum(o, "x");
    p.y = isnan(y) ? 0 : y;
    p.x = isnan(x) ? 0 : x;
    p.pq = o["pq"] | 0;
    if (p.pq > PQ_NONE) p.pq = PQ_NONE;
    strlcpy(p.typ, o["typ"] | "", sizeof(p.typ));
    strlcpy(p.note, o["pozn"] | "", sizeof(p.note));
    if (isnan(p.lat) || isnan(p.lon) || p.lat < -90 || p.lat > 90 || p.lon < -180 || p.lon > 180) {
      err = String("bod ") + name + ": chybí nebo neplatná šířka/délka";
      return false;
    }
    if ((!isnan(p.h) && (p.h < -500 || p.h > 9000)) || (!isnan(p.bpv) && (p.bpv < -500 || p.bpv > 9000))) {
      err = String("bod ") + name + ": neplatná výška";
      return false;
    }
    for (int j = 0; j < n; j++) {
      if (strcmp(tmp[j].name, p.name) == 0) {
        err = String("název bodu ") + name + " je dvakrát";
        return false;
      }
    }
    n++;
  }
  int s = doc["sel"] | -1;
  if (s < -1 || s >= n) s = n ? 0 : -1;
  const char *fr = doc["frame"] | "auto";
  for (int i = 0; i < n; i++) points[i] = tmp[i];
  nPoints = n;
  sel = s;
  frame = strcmp(fr, "etrs") == 0 ? Frame::Etrs : strcmp(fr, "itrf") == 0 ? Frame::Itrf : Frame::Auto;
  return true;
}

// ---------------------------------------------------------------- veřejné API – body
bool begin() {
  File f = LittleFS.open(POINTS_FILE, "r");
  if (!f) return true;
  String s = f.readString();
  f.close();
  String err;
  return parsePoints(s.c_str(), err);
}

String pointsJson() {
  JsonDocument doc;
  pointsToJson(doc);
  doc["max"] = MAX_POINTS;
  String s;
  serializeJson(doc, s);
  return s;
}

bool setPointsJson(const char *json, String &err) {
  if (!parsePoints(json, err)) {
    begin();  // vrátit uložený stav
    return false;
  }
  if (!writePoints()) {
    err = "zápis do paměti selhal";
    return false;
  }
  return true;
}

int selected() { return sel; }
int pointCount() { return nPoints; }
const char *pointName(int i) { return (i >= 0 && i < nPoints) ? points[i].name : "-"; }

bool needsTransform(bool rtk) { return frame == Frame::Itrf || (frame == Frame::Auto && !rtk); }

void toPointFrame(double &lat, double &lon, double hEll, bool rtk, double epoch) {
  if (!needsTransform(rtk)) return;
  double h;
  geo::fromEcef(geo::itrfToEtrs(geo::toEcef(lat, lon, hEll), epoch), lat, lon, h);
}

bool compare(int i, double lat, double lon, double hEll, double hMSL, bool rtk, double epoch, Diff &d) {
  d = Diff();
  if (i < 0 || i >= nPoints) return false;
  const Point &pt = points[i];
  const settings::Profile &pr = settings::profile(settings::active());
  double ant = pr.antH + pr.pco;
  geo::Ecef rx = geo::toEcef(lat, lon, hEll);
  if (needsTransform(rtk)) {
    rx = geo::itrfToEtrs(rx, epoch);
    d.flags |= DF_TR;
  }
  bool hasH = !isnan(pt.h);
  // očekávané fázové centrum antény = bod + výška antény + PCO; bez výšky bodu jen vodorovně
  geo::Ecef expect = geo::toEcef(pt.lat, pt.lon, hasH ? pt.h + ant : hEll);
  double e, n, u;
  geo::enu(expect, rx, pt.lat, pt.lon, e, n, u);
  if (pt.pq != PQ_NONE) {
    d.e = e;
    d.n = n;
    d.d2 = sqrt(e * e + n * n);
    d.flags |= DF_HOR;
    if (pt.pq == PQ_APPROX) d.flags |= DF_APPROX;
  }
  if (hasH) {
    d.u = u;
    d.flags |= DF_U_ELL;
  } else if (!isnan(pt.bpv)) {
    d.u = hMSL - (pt.bpv + ant);
    d.flags |= DF_U_BPV;
  }
  return true;
}

int addPoint(const char *name, double lat, double lon, double hEll, const char *note, String &err) {
  if (nPoints >= MAX_POINTS) {
    err = String("nejvýš ") + MAX_POINTS + " bodů";
    return -1;
  }
  size_t l = strlen(name);
  if (l == 0 || l >= sizeof(points[0].name)) {
    err = "název bodu musí mít 1–24 znaků";
    return -1;
  }
  for (int j = 0; j < nPoints; j++) {
    if (strcmp(points[j].name, name) == 0) {
      err = String("bod ") + name + " už existuje";
      return -1;
    }
  }
  Point &p = points[nPoints];
  memset(&p, 0, sizeof(p));
  strlcpy(p.name, name, sizeof(p.name));
  p.lat = lat;
  p.lon = lon;
  p.h = hEll;
  p.bpv = NAN;
  p.pq = PQ_EXACT;
  strlcpy(p.typ, "změřený", sizeof(p.typ));
  strlcpy(p.note, note, sizeof(p.note));
  nPoints++;
  sel = nPoints - 1;
  if (!writePoints()) {
    err = "zápis do paměti selhal";
    return -1;
  }
  return sel;
}

// ---------------------------------------------------------------- živá odchylka
static double epochOf(const gnss::Pvt &p) {
  return p.timeValid ? geo::epochYear(p.year, p.month, p.day) : 2026.0;
}

static void updateDelta(const gnss::Pvt &p) {
  cur = Delta();
  cur.hAcc = p.hAcc;
  cur.vAcc = p.vAcc;
  cur.fixOk = p.gnssFixOk && p.fixType >= 3;
  double ep = epochOf(p);

  // informativní posun ITRF -> ETRS89 v místě přijímače
  if (p.valid) {
    geo::Ecef a = geo::toEcef(p.lat, p.lon, p.hEll);
    geo::Ecef b = geo::itrfToEtrs(a, ep);
    double e, n, u;
    geo::enu(a, b, p.lat, p.lon, e, n, u);
    shiftN = n;
    shiftE = e;
  }
  if (sel < 0 || !p.valid) return;
  cur.ok = compare(sel, p.lat, p.lon, p.hEll, p.hMSL, gnss::rtcmActive(), ep, cur.d);
}

// ---------------------------------------------------------------- test doby do fixu
static void finishRun(bool timeout) {
  curRun.timeout = timeout;
  if (nRuns < MAX_RUNS) runs[nRuns++] = curRun;
  if (nRuns >= tcfg.count || nRuns >= MAX_RUNS) {
    tstate = TState::Done;
  } else {
    tstate = TState::Pause;
    pauseUntilMs = millis() + tcfg.pauseS * 1000UL;
  }
}

static void startRun() {
  curRun = Run();
  curRun.kind = (uint8_t)tcfg.kind;
  gnss::resetReceiver(tcfg.kind);
  runStartMs = millis();
  tstate = TState::Run;
}


void onPvt() {
  const gnss::Pvt &p = gnss::pvt();
  updateDelta(p);

  if (tstate == TState::Run) {
    uint32_t t = millis() - runStartMs;
    if (t < 500) return;  // první epochy po restartu mohou být ještě ze starého řešení
    if (cur.fixOk && !curRun.tFix) curRun.tFix = t;
    if (cur.fixOk && p.diffSoln && !curRun.tCorr) curRun.tCorr = t;
    if (p.carrSoln >= 1 && !curRun.tFloat) curRun.tFloat = t;
    if (p.carrSoln == 2 && !curRun.tRtk) curRun.tRtk = t;
    if (cur.fixOk && !curRun.tAcc) {
      bool hor = cur.ok && (cur.d.flags & DF_HOR);
      bool accOk = hor ? cur.d.d2 < tcfg.accM : p.hAcc < tcfg.accM;
      if (accOk) curRun.tAcc = t;
    }
    bool done = false;
    switch (tcfg.goal) {
      case Goal::Fix3D: done = curRun.tFix; break;
      case Goal::Corr: done = curRun.tCorr; break;
      case Goal::Float: done = curRun.tFloat; break;
      case Goal::RtkFix: done = curRun.tRtk; break;
      case Goal::Acc: done = curRun.tAcc; break;
    }
    if (done) finishRun(false);
  }
}

void poll() {
  uint32_t now = millis();
  if (tstate == TState::Run && now - runStartMs > (uint32_t)tcfg.timeoutS * 1000UL) finishRun(true);
  else if (tstate == TState::Pause && (int32_t)(now - pauseUntilMs) >= 0) startRun();
}

bool ttffStart(const TtffCfg &c, String &err) {
  if (c.count < 1 || c.count > MAX_RUNS) {
    err = String("počet opakování 1–") + MAX_RUNS;
    return false;
  }
  if (c.timeoutS < 10 || c.timeoutS > 3600) {
    err = "limit pokusu 10–3600 s";
    return false;
  }
  if (c.pauseS < 0 || c.pauseS > 600) {
    err = "pauza 0–600 s";
    return false;
  }
  if (c.goal == Goal::Acc && (c.accM <= 0 || c.accM > 100)) {
    err = "přesnost 0–100 m";
    return false;
  }
  tcfg = c;
  nRuns = 0;
  startRun();
  return true;
}

void ttffStop() {
  if (tstate == TState::Run || tstate == TState::Pause) tstate = TState::Done;
}

void ttffClear() {
  if (tstate == TState::Run || tstate == TState::Pause) return;
  nRuns = 0;
  tstate = TState::Idle;
}

bool ttffRunning() { return tstate == TState::Run || tstate == TState::Pause; }


void statusJson(JsonDocument &doc) {
  const gnss::Pvt &p = gnss::pvt();
  const settings::Profile &pr = settings::profile(settings::active());
  doc["frame"] = frameName(frame);
  doc["antH"] = serialized(String(pr.antH, 3));
  doc["pco"] = serialized(String(pr.pco, 3));
  JsonObject sh = doc["shift"].to<JsonObject>();
  sh["n"] = serialized(String(shiftN, 3));
  sh["e"] = serialized(String(shiftE, 3));
  if (sel >= 0) doc["point"] = points[sel].name;

  JsonObject d = doc["d"].to<JsonObject>();
  d["ok"] = cur.ok;
  d["fixOk"] = cur.fixOk;
  d["fix"] = gnss::fixText(p);
  d["cs"] = p.carrSoln;
  d["sv"] = p.numSV;
  d["hAcc"] = serialized(String(cur.hAcc, 3));
  d["vAcc"] = serialized(String(cur.vAcc, 3));
  if (cur.ok) {
    d["f"] = cur.d.flags;
    d["e"] = serialized(String(cur.d.e, 3));
    d["n"] = serialized(String(cur.d.n, 3));
    d["u"] = serialized(String(cur.d.u, 3));
    d["d2"] = serialized(String(cur.d.d2, 3));
  }
  if (p.timeValid) {
    char t[24];
    snprintf(t, sizeof(t), "%04u-%02u-%02u %02u:%02u:%02u", p.year, p.month, p.day, p.hour, p.min, p.sec);
    d["t"] = t;
  }

  JsonObject t = doc["ttff"].to<JsonObject>();
  static const char *const ST[] = {"idle", "run", "pause", "done"};
  t["state"] = ST[(int)tstate];
  t["kind"] = kindName((uint8_t)tcfg.kind);
  t["count"] = tcfg.count;
  t["timeout"] = tcfg.timeoutS;
  t["pause"] = tcfg.pauseS;
  t["goal"] = goalName(tcfg.goal);
  t["acc"] = serialized(String(tcfg.accM, 2));
  if (tstate == TState::Run) t["elapsed"] = millis() - runStartMs;
  if (tstate == TState::Pause) t["next"] = (int32_t)(pauseUntilMs - millis());
  JsonArray ra = t["runs"].to<JsonArray>();
  for (int i = 0; i < nRuns; i++) {
    JsonArray r = ra.add<JsonArray>();
    r.add(kindName(runs[i].kind));
    r.add(runs[i].tFix);
    r.add(runs[i].tCorr);
    r.add(runs[i].tFloat);
    r.add(runs[i].tRtk);
    r.add(runs[i].tAcc);
    r.add(runs[i].timeout ? 1 : 0);
  }
}

}  // namespace testctl
