#include "proto.h"

#include <ArduinoJson.h>

#include "btlink.h"
#include "config.h"
#include "epochbuf.h"
#include "gnss.h"
#include "power.h"
#include "settings.h"
#include "survey.h"

namespace proto {

// ---------------------------------------------------------------- pohyb (IMU)
// Stejné meze jako při měření bodu: otáčení > 15 °/s nebo zrychlení mimo 1 g o víc než 0,2 g.
// Pohyb trvá ještě 1,5 s po posledním vzorku nad mezí.
static constexpr float MOT_GYRO_DPS = 15.0f;
static constexpr float MOT_ACC_G = 0.2f;
static constexpr uint32_t MOT_HOLD_MS = 1500;
static uint32_t motLastMs = 0;

void onImu(const imu::Sample &s) {
  float g = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  float a = fabsf(sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az) - 1.0f);
  if (g >= MOT_GYRO_DPS || a >= MOT_ACC_G) motLastMs = millis();
}

bool moving() { return motLastMs && millis() - motLastMs < MOT_HOLD_MS; }

// ---------------------------------------------------------------- věty
static void (*profileHook)(int) = nullptr;
void setProfileHook(void (*fn)(int)) { profileHook = fn; }

static bool app = false;

bool appMode() {
  if (!btlink::connected()) app = false;
  return app;
}

// Doplní *CS\r\n a pošle do dávky BT. b musí mít místo na 5 znaků za n.
static void emit(char *b, size_t size, int n) {
  if (n <= 0 || (size_t)n + 5 >= size) return;
  uint8_t cs = 0;
  for (int i = 1; i < n; i++) cs ^= (uint8_t)b[i];
  n += snprintf(b + n, size - n, "*%02X\r\n", cs);
  btlink::send(b, n);
}

uint8_t quality() {
  const gnss::Pvt &p = gnss::pvt();
  if (!p.valid) return Q_NONE;
  if (p.fixType == 1) return Q_DR;
  if (!p.gnssFixOk) return Q_NONE;
  if (p.carrSoln) {
    if (!gnss::rtcmActive()) return Q_HAS;
    return p.carrSoln == 2 ? Q_FIX : Q_FLOAT;
  }
  return p.diffSoln ? Q_DGNSS : Q_GNSS;
}

// $PRVR,POS,čas,datum,lat,lon,hEll,hMSL,σN,σE,σU,hAcc,vAcc,kvalita,carrSoln,rtcm,stáříKor,SV,PDOP,rychlost,kurz,pohyb,hp
// $PRVR,BUF,… stejná pole – epocha ze zásobníku (výpadek BT)
bool capturePos(PosRec &r) {
  const gnss::Pvt &p = gnss::pvt();
  if (!p.valid) return false;
  const gnss::Cov &cv = gnss::cov();
  memset(&r, 0, sizeof(r));
  r.q = quality();
  if (p.timeValid) {
    r.flags |= F_TIME;
    r.year = p.year;
    r.month = p.month;
    r.day = p.day;
    r.hour = p.hour;
    r.min = p.min;
    r.sec = p.sec;
    r.ms = p.iTOW % 1000;
  }
  r.lat = p.lat;
  r.lon = p.lon;
  r.hEll = p.hEll;
  r.hMSL = p.hMSL;
  if (cv.valid && millis() - cv.rxMs < 3000) {
    r.flags |= F_COV;
    r.sN = cv.sN;
    r.sE = cv.sE;
    r.sD = cv.sD;
  }
  r.hAcc = p.hAcc;
  r.vAcc = p.vAcc;
  r.carrSoln = p.carrSoln;
  if (gnss::rtcmActive()) r.flags |= F_RTCM;
  if (p.corrAge) {
    r.flags |= F_CORR;
    r.corrAgeS = gnss::corrAgeSeconds(p.corrAge);
  }
  r.numSV = p.numSV;
  r.pDOP100 = (uint16_t)(p.pDOP * 100 + 0.5f);
  r.gSpeed = p.gSpeed;
  r.headMot = p.headMot;
  if (moving()) r.flags |= F_MOVING;
  if (p.hp) r.flags |= F_HP;
  return true;
}

void sendRec(const PosRec &r, bool buffered) {
  char b[256];
  int n = snprintf(b, sizeof(b), buffered ? "$PRVR,BUF," : "$PRVR,POS,");
  if (r.flags & F_TIME) {
    n += snprintf(b + n, sizeof(b) - n, "%02u%02u%02u.%03u,%02u%02u%02u,", r.hour, r.min, r.sec, r.ms, r.day, r.month,
                  r.year % 100);
  } else {
    n += snprintf(b + n, sizeof(b) - n, ",,");
  }
  if (r.q != Q_NONE) {
    n += snprintf(b + n, sizeof(b) - n, "%.9f,%.9f,%.4f,%.4f,", r.lat, r.lon, r.hEll, r.hMSL);
  } else {
    n += snprintf(b + n, sizeof(b) - n, ",,,,");
  }
  if (r.flags & F_COV) n += snprintf(b + n, sizeof(b) - n, "%.4f,%.4f,%.4f,", r.sN, r.sE, r.sD);
  else n += snprintf(b + n, sizeof(b) - n, ",,,");
  n += snprintf(b + n, sizeof(b) - n, "%.4f,%.4f,%u,%u,%u,", r.hAcc, r.vAcc, r.q, r.carrSoln, (r.flags & F_RTCM) ? 1 : 0);
  if (r.flags & F_CORR) n += snprintf(b + n, sizeof(b) - n, "%.1f,", r.corrAgeS);
  else n += snprintf(b + n, sizeof(b) - n, ",");
  n += snprintf(b + n, sizeof(b) - n, "%u,%.2f,%.3f,%.1f,%u,%u", r.numSV, r.pDOP100 / 100.0f, r.gSpeed, r.headMot,
                (r.flags & F_MOVING) ? 1 : 0, (r.flags & F_HP) ? 1 : 0);
  emit(b, sizeof(b), n);
}

void sendBufEnd(uint32_t n) {
  char b[48];
  int k = snprintf(b, sizeof(b), "$PRVR,BUFEND,%lu", (unsigned long)n);
  emit(b, sizeof(b), k);
  btlink::flush();
}

void sendPos() {
  PosRec r;
  if (!capturePos(r)) return;
  bool c = btlink::connected();
  ebuf::onEpoch(r, c);
  if (c) sendRec(r, false);
}

// Text do věty: čárka, hvězdička a řídicí znaky se nahradí mezerou
static void safeCopy(char *dst, size_t size, const char *src) {
  size_t i = 0;
  for (; src[i] && i + 1 < size; i++) {
    char c = src[i];
    dst[i] = (c == ',' || c == '*' || c == '$' || (uint8_t)c < 32) ? ' ' : c;
  }
  dst[i] = 0;
}

// $PRVR,STA,FW,baterieV,baterie%,profil,početProfilů,názevProfilu,rtcmOk,rtcmChybné,stáříRtcm,typRtcm,
//           ubxChyby,volnáRAM,měřeníFáze,měřeníČas,měřeníEpochy,btZahozeno,
//           btFronta[B],režimAplikace,eoe(1 = X20 posílá konec epochy),hpEpoch,
//           ubxOk,ubxVadné,mgaAck,mgaNak,mgaDbd,ackAiding (0.11.0),frekvence[Hz] (0.12.0)
void sendSta() {
  if (!btlink::connected()) return;
  btlink::Stats bs = btlink::stats();
  gnss::Stats gs = gnss::stats();
  float v = power::batteryVolts();
  char pname[25];
  safeCopy(pname, sizeof(pname), settings::profile(settings::active()).name);
  char b[320];
  int n = snprintf(b, sizeof(b), "$PRVR,STA,%s,%.2f,%d,%d,%d,%s,%lu,%lu,", FW_VERSION, v, power::batteryPercentStable(),
                   settings::active(), settings::count(), pname, (unsigned long)bs.rtcmOk, (unsigned long)bs.rtcmBad);
  if (bs.rtcmOk) n += snprintf(b + n, sizeof(b) - n, "%.1f,%u,", (millis() - bs.rtcmLastMs) / 1000.0f, bs.rtcmLastType);
  else n += snprintf(b + n, sizeof(b) - n, ",,");
  n += snprintf(b + n, sizeof(b) - n, "%lu,%lu,%d,%lu,%d,%lu,%lu,%d,%d,%lu", (unsigned long)gs.ubxErr,
                (unsigned long)ESP.getFreeHeap(), survey::phase(), (unsigned long)survey::phaseSeconds(),
                survey::rawCount(), (unsigned long)bs.txDropped, (unsigned long)btlink::backlog(), appMode() ? 1 : 0,
                gs.eoeCount ? 1 : 0, (unsigned long)gs.hpCount);
  // od FW 0.11.0: rámce UBX z mobilu (OK, vadné), MGA-ACK (přijato, odmítnuto), vyslané MGA-DBD, ACKAIDING
  n += snprintf(b + n, sizeof(b) - n, ",%lu,%lu,%lu,%lu,%lu,%d", (unsigned long)bs.ubxOk, (unsigned long)bs.ubxBad,
                (unsigned long)gs.mgaAccepted, (unsigned long)gs.mgaRejected, (unsigned long)gs.mgaDbd, gs.ackAiding);
  // od FW 0.12.0: frekvence navigace aktivního profilu [Hz]
  n += snprintf(b + n, sizeof(b) - n, ",%u", (unsigned)settings::profile(settings::active()).rateHz);
  emit(b, sizeof(b), n);
  btlink::flush();
}

// $PRVR,SAT,část,částí,celkem,{gnssId,svId,C/N0,elevace,azimut,použit}×(max 12)
// Jen sledované družice (C/N0 > 0).
void sendSat() {
  if (!btlink::connected()) return;
  const gnss::SatSummary &s = gnss::sats();
  if (!s.valid) return;
  static uint8_t skip = 0;
  if (appMode() && (++skip & 1)) return;  // v režimu aplikace každé 2 s
  static constexpr int PER = 12;
  int total = 0;
  for (int i = 0; i < s.count; i++)
    if (s.list[i].cno) total++;
  int parts = total ? (total + PER - 1) / PER : 1;
  int idx = 0;
  for (int part = 1; part <= parts; part++) {
    char b[320];
    int n = snprintf(b, sizeof(b), "$PRVR,SAT,%d,%d,%d", part, parts, total);
    int k = 0;
    while (idx < s.count && k < PER) {
      const gnss::SatInfo &si = s.list[idx++];
      if (!si.cno) continue;
      n += snprintf(b + n, sizeof(b) - n, ",%u,%u,%u,%d,%d,%u", si.gnssId, si.svId, si.cno, si.elev, si.azim,
                    si.used ? 1 : 0);
      k++;
    }
    emit(b, sizeof(b), n);
    btlink::flush();
  }
}

// ---------------------------------------------------------------- příkazy JSON
// Odpověď na jednom řádku; delší než dávka BT se posílá po částech
static void reply(JsonDocument &d) {
  String out;
  serializeJson(d, out);
  out += "\r\n";
  btlink::flush();  // rozpracovaná dávka NMEA nejdřív
  for (size_t i = 0; i < out.length(); i += 1024) {
    size_t n = out.length() - i < 1024 ? out.length() - i : 1024;
    btlink::send(out.c_str() + i, n);
    btlink::flush();
  }
}

static void infoJson(JsonDocument &d) {
  d["fw"] = FW_VERSION;
  d["proto"] = VERSION;
  d["name"] = btlink::name();
  const gnss::Info &gi = gnss::info();
  JsonObject x = d["x20"].to<JsonObject>();
  x["ok"] = gi.valid;
  if (gi.valid) {
    x["mod"] = gi.mod;
    x["fw"] = gi.fwver;
    x["baud"] = gnss::baud();
  }
  d["imu"] = imu::present();
  JsonObject p = d["prof"].to<JsonObject>();
  p["i"] = settings::active();
  p["n"] = settings::count();
  p["def"] = settings::defaultIndex();
  p["name"] = settings::profile(settings::active()).name;
  p["hz"] = settings::profile(settings::active()).rateHz;
  float v = power::batteryVolts();
  d["bat"] = serialized(String(v, 2));
  d["heap"] = ESP.getFreeHeap();
  d["up"] = millis() / 1000;
}

void handleCmd(const char *line, size_t len) {
  JsonDocument in;
  JsonDocument out;
  DeserializationError e = deserializeJson(in, line, len);
  if (e) {
    out["ok"] = false;
    out["err"] = "chybný JSON";
    reply(out);
    return;
  }
  if (!in["id"].isNull()) out["id"] = in["id"];
  app = true;  // příkaz poslala aplikace RoverGnss
  const char *cmd = in["cmd"] | "";
  const char *what = in["what"] | "";
  out["ok"] = true;

  if (!strcmp(cmd, "get") && !strcmp(what, "info")) {
    infoJson(out);
  } else if (!strcmp(cmd, "get") && !strcmp(what, "profiles")) {
    out["data"] = serialized(settings::profilesJson());
  } else if (!strcmp(cmd, "profile")) {
    int i = in["i"] | -1;
    if (i < 0 || i >= settings::count() || !profileHook) {
      out["ok"] = false;
      out["err"] = "neplatný profil";
    } else {
      profileHook(i);
      out["i"] = settings::active();
      out["name"] = settings::profile(settings::active()).name;
      out["hz"] = settings::profile(settings::active()).rateHz;
    }
  } else if (!strcmp(cmd, "meas")) {
    const char *op = in["op"] | "";
    bool st = false, raw = false;
    if (!strcmp(op, "start")) {
      st = in["static"] | true;
      raw = in["raw"] | false;
    } else if (strcmp(op, "stop")) {
      out["ok"] = false;
      out["err"] = "op: start|stop";
    }
    if (out["ok"]) {
      bool ack = gnss::setMeasMode(st, raw);
      out["static"] = gnss::measStatic();
      out["raw"] = gnss::measRaw();
      out["ack"] = ack;  // X20 potvrdila konfiguraci
      Serial.printf("[app] měření bodu %s%s%s (X20 %s)\n", st || raw ? "ZAP" : "VYP", st ? ", statický model" : "",
                    raw ? ", RAWX" : "", ack ? "OK" : "bez potvrzení");
    }
  } else if (!strcmp(cmd, "rec")) {
    const char *op = in["op"] | "";
    if (!strcmp(op, "start")) ebuf::setRecording(true);
    else if (!strcmp(op, "stop")) ebuf::setRecording(false);
    else {
      out["ok"] = false;
      out["err"] = "op: start|stop";
    }
    out["rec"] = ebuf::recording();
    out["n"] = ebuf::count();
    out["cap"] = ebuf::capacity();
  } else if (!strcmp(cmd, "buf")) {
    const char *op = in["op"] | "";
    if (!strcmp(op, "get")) {
      out["n"] = ebuf::count();
      out["start"] = ebuf::startReplay();  // epochy jdou po této odpovědi, konec $PRVR,BUFEND
    } else if (!strcmp(op, "clear")) {
      ebuf::clear();
    } else {
      out["ok"] = false;
      out["err"] = "op: get|clear";
    }
  } else if (!strcmp(cmd, "x20") && !strcmp(in["op"] | "", "reset")) {
    const char *k = in["kind"] | "";
    if (!strcmp(k, "hot")) gnss::resetReceiver(gnss::ResetKind::Hot);
    else if (!strcmp(k, "warm")) gnss::resetReceiver(gnss::ResetKind::Warm);
    else if (!strcmp(k, "cold")) gnss::resetReceiver(gnss::ResetKind::Cold);
    else {
      out["ok"] = false;
      out["err"] = "kind: hot|warm|cold";
    }
    if (out["ok"]) Serial.printf("[app] X20 %s start\n", gnss::resetKindText());
  } else {
    out["ok"] = false;
    out["err"] = "neznámý příkaz";
  }
  reply(out);
}

}  // namespace proto
