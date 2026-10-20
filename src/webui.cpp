#include "webui.h"

#include <ArduinoJson.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>

#include "btlink.h"
#include "config.h"
#include "gnss.h"
#include "imu.h"
#include "power.h"
#include "settings.h"
#include "survey.h"
#include "testctl.h"
#include "webpage.h"

namespace webui {

static WebServer server(80);
static DNSServer dns;
static bool running = false;
static bool rescueMode = false;
static bool testMode = false;
static bool bridgeOn = false;
static uint32_t lastActivityMs = 0;
static uint32_t restartAtMs = 0;
static uint32_t apOffAtMs = 0;
static bool devChanged = false;  // změněno heslo Wi-Fi nebo název BT – projeví se až restartem
static bool routesDone = false;
static Event pending = Event::None;
static char apSsid[25] = "";
static char apPass[33] = "";

static const IPAddress AP_IP(192, 168, 4, 1);

static void touch() { lastActivityMs = millis(); }

static void sendJson(int code, const String &s) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", s);
}

static void sendOk() { sendJson(200, "{\"ok\":true}"); }

static void sendErr(const String &msg) {
  JsonDocument d;
  d["ok"] = false;
  d["err"] = msg;
  String s;
  serializeJson(d, s);
  sendJson(400, s);
}

static void scheduleRestart(uint32_t delayMs) { restartAtMs = millis() + delayMs; }
static void scheduleApOff(uint32_t delayMs) { apOffAtMs = millis() + delayMs; }

// ---------------------------------------------------------------- statistika
static void buildStats(String &out) {
  JsonDocument doc;
  doc["fw"] = FW_VERSION;
  doc["up"] = millis();
  doc["rescue"] = rescueMode;
  doc["test"] = testMode;
  doc["bridge"] = bridgeOn;
  doc["heap"] = ESP.getFreeHeap();
  const esp_partition_t *run = esp_ota_get_running_partition();
  doc["part"] = run ? run->label : "?";

  float v = power::batteryVolts();
  JsonObject bat = doc["bat"].to<JsonObject>();
  bat["v"] = serialized(String(v, 3));
  bat["p"] = power::batteryPercentStable();
  bat["ok"] = power::valid();

  int ai = settings::active();
  JsonObject pr = doc["prof"].to<JsonObject>();
  pr["i"] = ai;
  pr["n"] = settings::count();
  pr["name"] = settings::profile(ai).name;

  const gnss::Info &gi = gnss::info();
  JsonObject x = doc["x20"].to<JsonObject>();
  x["ok"] = gi.valid;
  x["mod"] = gi.mod;
  x["fw"] = gi.fwver;
  x["baud"] = gnss::baud();
  x["ubxAge"] = gnss::lastUbxAgeMs() == UINT32_MAX ? -1 : (int32_t)gnss::lastUbxAgeMs();
  if (gnss::sinceResetMs()) {
    x["rst"] = gnss::resetKindText();
    x["rstAge"] = gnss::sinceResetMs();
    x["ttff"] = gnss::ttff();
  }

  gnss::Stats gs = gnss::stats();
  JsonObject gn = doc["gn"].to<JsonObject>();
  gn["bytes"] = gs.bytes;
  gn["ubxOk"] = gs.ubxOk;
  gn["ubxErr"] = gs.ubxErr;
  gn["pvt"] = gs.pvtCount;
  gn["pps"] = gs.ppsCount;
  gn["ppsPer"] = gs.ppsPeriodUs;
  gn["ppsAge"] = gs.ppsCount ? (int32_t)((esp_timer_get_time() - gs.ppsLastUs) / 1000) : -1;
  gn["lat"] = gs.pvtLatencyUs;
  gn["reinit"] = gs.reinits;

  const gnss::Pvt &p = gnss::pvt();
  JsonObject pv = doc["pvt"].to<JsonObject>();
  pv["ok"] = p.valid;
  if (p.valid) {
    pv["age"] = millis() - p.rxMs;
    pv["fix"] = gnss::fixText(p);
    pv["ft"] = p.fixType;
    pv["fixOk"] = p.gnssFixOk;
    pv["cs"] = p.carrSoln;
    pv["diff"] = p.diffSoln;
    pv["sv"] = p.numSV;
    pv["lat"] = serialized(String(p.lat, 9));
    pv["lon"] = serialized(String(p.lon, 9));
    pv["h"] = serialized(String(p.hMSL, 3));
    pv["he"] = serialized(String(p.hEll, 3));
    pv["hAcc"] = serialized(String(p.hAcc, 3));
    pv["vAcc"] = serialized(String(p.vAcc, 3));
    pv["gs"] = serialized(String(p.gSpeed, 2));
    pv["hd"] = serialized(String(p.headMot, 1));
    pv["pdop"] = serialized(String(p.pDOP, 2));
    pv["corr"] = gnss::corrAgeSeconds(p.corrAge);
    if (p.timeValid) {
      char t[24];
      snprintf(t, sizeof(t), "%04u-%02u-%02u %02u:%02u:%02u", p.year, p.month, p.day, p.hour, p.min, p.sec);
      pv["t"] = t;
    }
  }
  const gnss::Dop &dp = gnss::dop();
  if (dp.valid) {
    JsonObject d = doc["dop"].to<JsonObject>();
    d["h"] = serialized(String(dp.hDOP, 2));
    d["v"] = serialized(String(dp.vDOP, 2));
    d["p"] = serialized(String(dp.pDOP, 2));
  }
  const gnss::Cov &cv = gnss::cov();
  if (cv.rxMs) {
    JsonObject c = doc["cov"].to<JsonObject>();
    c["ok"] = cv.valid;
    c["n"] = serialized(String(cv.sN, 3));
    c["e"] = serialized(String(cv.sE, 3));
    c["d"] = serialized(String(cv.sD, 3));
  }

  const gnss::SatSummary &ss = gnss::sats();
  JsonObject sa = doc["sat"].to<JsonObject>();
  sa["tr"] = ss.tracked;
  sa["us"] = ss.used;
  sa["cnoM"] = serialized(String(ss.cnoMeanUsed, 1));
  JsonArray l = sa["l"].to<JsonArray>();
  for (int i = 0; i < ss.count; i++) {
    const gnss::SatInfo &s = ss.list[i];
    JsonArray e = l.add<JsonArray>();
    e.add(s.gnssId);
    e.add(s.svId);
    e.add(s.cno);
    e.add(s.elev);
    e.add(s.azim);
    e.add(s.used ? 1 : 0);
  }

  JsonObject im = doc["imu"].to<JsonObject>();
  im["ok"] = imu::present();
  if (imu::present()) {
    imu::Stats is = imu::stats();
    imu::Sample a;
    im["n"] = is.samples;
    im["odr"] = serialized(String(is.odrHz, 1));
    im["ovf"] = is.overflows;
    im["re"] = is.reinits;
    im["err"] = is.readErr;
    im["age"] = is.lastSampleAgeMs;
    im["rec"] = is.busRecovered;
    if (imu::latest(a)) im["t"] = serialized(String(a.tempC, 1));
  }

  btlink::Stats bs = btlink::stats();
  JsonObject bt = doc["bt"].to<JsonObject>();
  bt["name"] = btlink::name();
  bt["con"] = bs.connects;
  bt["tx"] = bs.txBytes;
  bt["drop"] = bs.txDropped;
  bt["rx"] = bs.rxBytes;
  bt["rtcmOk"] = bs.rtcmOk;
  bt["rtcmBad"] = bs.rtcmBad;
  bt["rtcmType"] = bs.rtcmLastType;
  bt["rtcmAge"] = bs.rtcmOk ? (int32_t)(millis() - bs.rtcmLastMs) : -1;
  bt["pairOk"] = bs.pairOk;
  bt["pairFail"] = bs.pairFail;

  serializeJson(doc, out);
}

// ---------------------------------------------------------------- obsluha požadavků
static void handleRoot() {
  touch();
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html; charset=utf-8", (const char *)PAGE_GZ, PAGE_GZ_LEN);
}

static void handleStats() {
  touch();
  String s;
  buildStats(s);
  sendJson(200, s);
}

static void handleGetProfiles() {
  touch();
  sendJson(200, settings::profilesJson());
}

static void handleSetProfiles() {
  touch();
  if (rescueMode || bridgeOn) return sendErr("teď nelze");
  String err;
  if (!settings::setProfilesJson(server.arg("plain").c_str(), err)) return sendErr(err);
  pending = Event::ProfilesChanged;
  sendJson(200, settings::profilesJson());
}

static void handleGetDevice() {
  touch();
  JsonDocument doc;
  doc["apPass"] = settings::device().apPass;
  doc["btName"] = settings::device().btName;
  doc["btDefault"] = btlink::defaultName();
  doc["vcal"] = serialized(String(power::dividerFactor(), 4));
  float bx, by, bz;
  imu::gyroBias(bx, by, bz);
  JsonArray g = doc["gyroBias"].to<JsonArray>();
  g.add(serialized(String(bx, 3)));
  g.add(serialized(String(by, 3)));
  g.add(serialized(String(bz, 3)));
  String s;
  serializeJson(doc, s);
  sendJson(200, s);
}

static void handleSetDevice() {
  touch();
  String err;
  if (!settings::setDeviceJson(server.arg("plain").c_str(), err)) return sendErr(err);
  devChanged = true;
  sendOk();
}

static void handleCalBattery() {
  touch();
  // napětí ve V (3,86) nebo mV (3860), desetinná čárka i tečka
  String a = server.arg("mv");
  a.replace(',', '.');
  if (!power::calibrate(a.toFloat())) return sendErr("neplatná hodnota (2,5–4,5 V; koeficient musí vyjít 1,6–2,4)");
  JsonDocument d;
  d["ok"] = true;
  d["vcal"] = serialized(String(power::dividerFactor(), 4));
  d["v"] = serialized(String(power::batteryVolts(), 3));
  String s;
  serializeJson(d, s);
  sendJson(200, s);
}

static void handleCalGyro() {
  touch();
  if (!imu::present()) return sendErr("IMU nenalezeno");
  if (!imu::calibrateGyro(3000)) return sendErr("přístroj se hýbal nebo málo vzorků – bias beze změny");
  float bx, by, bz;
  imu::gyroBias(bx, by, bz);
  JsonDocument d;
  d["ok"] = true;
  JsonArray g = d["gyroBias"].to<JsonArray>();
  g.add(serialized(String(bx, 3)));
  g.add(serialized(String(by, 3)));
  g.add(serialized(String(bz, 3)));
  String s;
  serializeJson(d, s);
  sendJson(200, s);
}

static void handleBridge() {
  touch();
  if (rescueMode) return sendErr("v záchranném režimu nelze");
  String a = server.arg("on");
  pending = (a == "1") ? Event::BridgeStart : Event::BridgeStop;
  sendOk();
}

static void handleX20Reset() {
  touch();
  if (rescueMode || bridgeOn) return sendErr("teď nelze");
  String k = server.arg("k");
  if (k == "hot") gnss::resetReceiver(gnss::ResetKind::Hot);
  else if (k == "warm") gnss::resetReceiver(gnss::ResetKind::Warm);
  else if (k == "cold") gnss::resetReceiver(gnss::ResetKind::Cold);
  else return sendErr("neznámý typ startu");
  sendOk();
}

static void handleGetPoints() {
  touch();
  sendJson(200, testctl::pointsJson());
}

static void handleSetPoints() {
  touch();
  String err;
  if (!testctl::setPointsJson(server.arg("plain").c_str(), err)) return sendErr(err);
  sendJson(200, testctl::pointsJson());
}

static void handleTest() {
  touch();
  JsonDocument doc;
  testctl::statusJson(doc);
  String s;
  serializeJson(doc, s);
  sendJson(200, s);
}

// ---------------------------------------------------------------- měření bodu
static void handleMeas() {
  touch();
  JsonDocument doc;
  survey::statusJson(doc);
  String s;
  serializeJson(doc, s);
  sendJson(200, s);
}

static void handleMeasList() {
  touch();
  JsonDocument doc;
  survey::listJson(doc);
  String s;
  serializeJson(doc, s);
  sendJson(200, s);
}

static void handleMeasStart() {
  touch();
  if (rescueMode || bridgeOn) return sendErr("teď nelze");
  survey::Cfg c;
  c.countdownS = server.arg("cd").toInt();
  c.settleMaxS = server.arg("smax").toInt();
  c.settleAcc = server.arg("sacc").toFloat();
  c.durS = server.arg("dur").toInt();
  strlcpy(c.point, server.arg("pt").c_str(), sizeof(c.point));
  strlcpy(c.newName, server.arg("new").c_str(), sizeof(c.newName));
  String err;
  if (!survey::start(c, err)) return sendErr(err);
  sendOk();
}

static void handleMeasStop() {
  touch();
  String c = server.arg("cmd");
  if (c == "finish") survey::finish();
  else if (c == "cancel") survey::cancel();
  else return sendErr("neznámý příkaz");
  sendOk();
}

static void handleMeasDel() {
  touch();
  String err;
  bool ok = server.hasArg("pt") ? survey::removePoint(server.arg("pt").c_str(), err)
                                : survey::remove(server.arg("id").toInt(), err);
  if (!ok) return sendErr(err);
  sendOk();
}

// Vzorky posledního měření jako CSV (po částech)
static void handleMeasRaw() {
  touch();
  int n = survey::rawCount();
  if (!n) return sendErr("žádné vzorky");
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Disposition", "attachment; filename=\"rover-mereni.csv\"");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  char line[160];
  String chunk;
  chunk.reserve(2200);
  survey::rawLine(-1, line, sizeof(line));
  chunk += line;
  for (int i = 0; i < n; i++) {
    survey::rawLine(i, line, sizeof(line));
    chunk += line;
    if (chunk.length() > 2000) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("");
}

static void handleTestTtff() {
  touch();
  if (rescueMode || bridgeOn) return sendErr("teď nelze");
  String c = server.arg("cmd");
  if (c == "stop") {
    testctl::ttffStop();
    return sendOk();
  }
  if (c == "clear") {
    testctl::ttffClear();
    return sendOk();
  }
  if (c != "start") return sendErr("neznámý příkaz");
  testctl::TtffCfg cfg;
  String k = server.arg("kind");
  cfg.kind = k == "hot" ? gnss::ResetKind::Hot : k == "warm" ? gnss::ResetKind::Warm : gnss::ResetKind::Cold;
  cfg.count = server.arg("n").toInt();
  cfg.timeoutS = server.arg("timeout").toInt();
  cfg.pauseS = server.arg("pause").toInt();
  String g = server.arg("goal");
  cfg.goal = g == "corr"    ? testctl::Goal::Corr
             : g == "float" ? testctl::Goal::Float
             : g == "rtk"   ? testctl::Goal::RtkFix
             : g == "acc"   ? testctl::Goal::Acc
                            : testctl::Goal::Fix3D;
  cfg.accM = server.arg("acc").toFloat();
  String err;
  if (!testctl::ttffStart(cfg, err)) return sendErr(err);
  sendOk();
}

static void handleTestMode() {
  touch();
  if (rescueMode) return sendErr("v záchranném režimu nelze");
  pending = Event::TestMode;
  sendOk();
}

// „Uložit a vypnout Wi-Fi“: AP se vypne bez restartu (měření běží dál), BT zůstává.
// Po změně hesla Wi-Fi / názvu BT restart – jen když neběží měření bodu.
static void handleRestart() {
  touch();
  bool restart = rescueMode || (devChanged && !survey::running());
  JsonDocument d;
  d["ok"] = true;
  d["restart"] = restart;
  String s;
  serializeJson(d, s);
  sendJson(200, s);
  if (restart) scheduleRestart(500);
  else scheduleApOff(500);
}

// Captive portal: vše ostatní přesměrovat na menu (Android, iOS, Windows si stránku otevřou samy)
static void handleNotFound() {
  touch();
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

// ---------------------------------------------------------------- veřejné API
bool begin(bool rescue) {
  if (running) return true;
  apOffAtMs = 0;
  rescueMode = rescue;
  const settings::Device &d = settings::device();
  strlcpy(apSsid, d.btName[0] ? d.btName : btlink::defaultName(), sizeof(apSsid));
  strlcpy(apPass, d.apPass, sizeof(apPass));

  WiFi.persistent(false);
  if (!WiFi.mode(WIFI_AP)) {
    WiFi.mode(WIFI_OFF);
    return false;
  }
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(apSsid, apPass, 6, 0, 4)) {
    WiFi.mode(WIFI_OFF);
    return false;
  }
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", AP_IP);

  if (routesDone) {
    server.begin();
    running = true;
    touch();
    return true;
  }
  routesDone = true;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/stats", HTTP_GET, handleStats);
  server.on("/api/profiles", HTTP_GET, handleGetProfiles);
  server.on("/api/profiles", HTTP_POST, handleSetProfiles);
  server.on("/api/device", HTTP_GET, handleGetDevice);
  server.on("/api/device", HTTP_POST, handleSetDevice);
  server.on("/api/cal/battery", HTTP_POST, handleCalBattery);
  server.on("/api/cal/gyro", HTTP_POST, handleCalGyro);
  server.on("/api/bridge", HTTP_POST, handleBridge);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/x20reset", HTTP_POST, handleX20Reset);
  server.on("/api/points", HTTP_GET, handleGetPoints);
  server.on("/api/points", HTTP_POST, handleSetPoints);
  server.on("/api/test", HTTP_GET, handleTest);
  server.on("/api/meas", HTTP_GET, handleMeas);
  server.on("/api/meas/list", HTTP_GET, handleMeasList);
  server.on("/api/meas/start", HTTP_POST, handleMeasStart);
  server.on("/api/meas/stop", HTTP_POST, handleMeasStop);
  server.on("/api/meas/del", HTTP_POST, handleMeasDel);
  server.on("/api/meas/raw", HTTP_GET, handleMeasRaw);
  server.on("/api/test/ttff", HTTP_POST, handleTestTtff);
  server.on("/api/testmode", HTTP_POST, handleTestMode);
  server.onNotFound(handleNotFound);
  server.begin();

  running = true;
  touch();
  return true;
}

bool active() { return running; }

void end() {
  if (!running) return;
  server.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  running = false;
  apOffAtMs = 0;
}

bool needsRestart() { return devChanged; }

Event poll() {
  if (!running) return Event::None;
  dns.processNextRequest();
  server.handleClient();
  if (apOffAtMs && (int32_t)(millis() - apOffAtMs) >= 0) {
    apOffAtMs = 0;
    return Event::ApOff;
  }
  if (restartAtMs && (int32_t)(millis() - restartAtMs) >= 0) {
    restartAtMs = 0;
    return Event::Restart;
  }
  Event e = pending;
  pending = Event::None;
  return e;
}

void setBridge(bool a) {
  bridgeOn = a;
  touch();
}

void setTestMode(bool on) { testMode = on; }

const char *ssid() { return apSsid; }
const char *password() { return apPass; }

}  // namespace webui
