#include "proto.h"
#include "console.h"

#include <ArduinoJson.h>

#include "btlink.h"
#include "config.h"
#include "epochbuf.h"
#include "gnss.h"
#include "motion.h"
#include "power.h"
#include "settings.h"
#include "usblink.h"

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
static Hooks hooks;
void setHooks(const Hooks &h) { hooks = h; }

static bool app = false;

// měření rychlosti (FW 0.16.0) – stav, popis u speedStart()
static constexpr uint32_t SPEED_ALIVE_MS = 30000;
static constexpr uint8_t SPEED_HZ = 10;
static bool speedOn = false;
static uint32_t speedAliveMs = 0;

// režim USB host (FW 0.17.0) – stav, popis u usbStart()
static constexpr uint32_t USB_ALIVE_MS = 3000;
static bool usbOn = false;
static uint8_t usbHz = 5;
static settings::Dyn usbDyn = settings::Dyn::Pedestrian;
static uint32_t usbAliveMs = 0;

bool appMode() {
  if (!btlink::connected()) app = false;
  return app;
}

// Doplní *CS\r\n; vrací novou délku, 0 = nevejde se. b musí mít místo na 5 znaků za n.
static int seal(char *b, size_t size, int n) {
  if (n <= 0 || (size_t)n + 5 >= size) return 0;
  uint8_t cs = 0;
  for (int i = 1; i < n; i++) cs ^= (uint8_t)b[i];
  return n + snprintf(b + n, size - n, "*%02X\r\n", cs);
}

// Doplní *CS\r\n a pošle do dávky BT. b musí mít místo na 5 znaků za n.
static void emit(char *b, size_t size, int n) {
  n = seal(b, size, n);
  if (n) btlink::send(b, n);
}

uint8_t quality() {
  const gnss::Pvt &p = gnss::pvt();
  if (!p.valid) return Q_NONE;
  if (p.fixType == 1) return Q_DR;
  if (!p.gnssFixOk) return Q_NONE;
  bool rtcm = gnss::rtcmActive();
  if (p.carrSoln) {
    if (!rtcm) return Q_HAS;
    return p.carrSoln == 2 ? Q_FIX : Q_FLOAT;
  }
  if (!p.diffSoln) return Q_GNSS;
  // Korekce bez fáze: SBAS, nebo HAS před konvergencí (zdroj z NAV-SIG; bez NAV-SIG jako dřív DGNSS)
  const gnss::SigSummary &g = gnss::sig();
  if (!rtcm && g.valid && millis() - g.rxMs < 3000 && g.dominant > 1) return Q_HAS;
  return Q_DGNSS;
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

// Věta $PRVR,POS / $PRVR,BUF i s *CS\r\n; vrací délku (0 = chyba)
static int fmtRec(char *b, size_t size, const PosRec &r, bool buffered) {
  int n = snprintf(b, size, buffered ? "$PRVR,BUF," : "$PRVR,POS,");
  if (r.flags & F_TIME) {
    n += snprintf(b + n, size - n, "%02u%02u%02u.%03u,%02u%02u%02u,", r.hour, r.min, r.sec, r.ms, r.day, r.month,
                  r.year % 100);
  } else {
    n += snprintf(b + n, size - n, ",,");
  }
  if (r.q != Q_NONE) {
    n += snprintf(b + n, size - n, "%.9f,%.9f,%.4f,%.4f,", r.lat, r.lon, r.hEll, r.hMSL);
  } else {
    n += snprintf(b + n, size - n, ",,,,");
  }
  if (r.flags & F_COV) n += snprintf(b + n, size - n, "%.4f,%.4f,%.4f,", r.sN, r.sE, r.sD);
  else n += snprintf(b + n, size - n, ",,,");
  n += snprintf(b + n, size - n, "%.4f,%.4f,%u,%u,%u,", r.hAcc, r.vAcc, r.q, r.carrSoln, (r.flags & F_RTCM) ? 1 : 0);
  if (r.flags & F_CORR) n += snprintf(b + n, size - n, "%.1f,", r.corrAgeS);
  else n += snprintf(b + n, size - n, ",");
  n += snprintf(b + n, size - n, "%u,%.2f,%.3f,%.1f,%u,%u", r.numSV, r.pDOP100 / 100.0f, r.gSpeed, r.headMot,
                (r.flags & F_MOVING) ? 1 : 0, (r.flags & F_HP) ? 1 : 0);
  return seal(b, size, n);
}

void sendRec(const PosRec &r, bool buffered) {
  char b[256];
  int n = fmtRec(b, sizeof(b), r, buffered);
  if (n) btlink::send(b, n);
}

void sendBufEnd(uint32_t n) {
  char b[48];
  int k = snprintf(b, sizeof(b), "$PRVR,BUFEND,%lu", (unsigned long)n);
  emit(b, sizeof(b), k);
  btlink::flush();
}

// Doba od zapnutí do prvního 3D fixu, korekcí, RTK/HAS float a fix [ms] (test zálohy X20 po vypnutí)
static uint32_t bootFix = 0, bootCorr = 0, bootFloat = 0, bootRtk = 0;

static void bootTimes(const PosRec &r) {
  uint32_t now = millis();
  if (!bootFix && r.q != Q_NONE && r.q != Q_DR) bootFix = now;
  if (!bootCorr && (r.q == Q_DGNSS || r.q == Q_HAS || r.q == Q_FLOAT || r.q == Q_FIX)) bootCorr = now;
  if (!bootFloat && r.carrSoln >= 1) bootFloat = now;
  if (!bootRtk && r.carrSoln == 2) bootRtk = now;
}

void sendPos() {
  PosRec r;
  if (!capturePos(r)) return;
  bootTimes(r);
  bool c = btlink::connected();
  ebuf::onEpoch(r, c);
  if (c) sendRec(r, false);
  if (usbOn) {  // režim USB host: každá epocha i na Serial, nezávisle na BT
    char b[256];
    int n = fmtRec(b, sizeof(b), r, false);
    if (n) usblink::send(b, n);
  }
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
//           ubxOk,ubxVadné,mgaAck,mgaNak,mgaDbd,ackAiding (0.11.0),frekvence[Hz] (0.12.0),
//           zdrojKorekcí(NAV-SIG, -1 žádný),signálůSKorekcí,korekceCfg(1 OK/0 NAK/-1),průchozí,navZáznam (0.13.0),
//           stav(0 Měřím,1 Manipulace,2 Chodím,3 Jedu),automaticky(0/1),modelX20,|a|σ[mg],|a|odchylka[mg],
//           ωmax[°/s],kroky(0–100),krokyHz,rychlost[m/s](-1 bez fixu),změnStavu (0.14.0),
//           teplotaIMU[°C],klid(0/1, okno 1 s),krokůCelkem,aktualizacíBiasu,biasX,biasY,biasZ[°/s],kalibraceBěží(0/1) (0.15.0)
//           btPřijemZahozeno[B],btPřijemMaxBuffer[B],jsonZrušeno (0.15.1)
//           měřeníRychlosti(0/1),biasZeStáníVAutě(počet),stáníVAutě[s] (0.16.0)
//           usbhost(0/1),zdrojRtcm(0 žádný/1 BT/2 USB),rtcmUsbOk,rtcmUsbChybné (0.17.0)
// rtcmOk, rtcmChybné, stáříRtcm, typRtcm: zdroj, který jde do X20 (od 0.17.0 v režimu USB host z USB).
// V režimu USB host jde STA i na Serial (nezávisle na BT).
// Od 0.13.0 nemá rover měření bodu – pole měřeníFáze/Čas/Epochy jsou 0.
void pollCal();  // níže (výsledek kalibrace gyra na pozadí)

// Statistika RTCM zdroje, který jde do X20 (režim USB host: USB, jinak BT)
struct RtcmStat {
  uint32_t ok, bad, lastMs;
  uint16_t lastType;
};

static RtcmStat rtcmStat() {
  RtcmStat r;
  if (rtcmSource() == 2) {
    const RxParser::Stats &u = usblink::stats();
    r = {u.rtcmOk, u.rtcmBad, u.rtcmLastMs, u.rtcmLastType};
  } else {
    btlink::Stats bs = btlink::stats();
    r = {bs.rtcmOk, bs.rtcmBad, bs.rtcmLastMs, bs.rtcmLastType};
  }
  return r;
}

void sendSta() {
  pollCal();
  bool bt = btlink::connected();
  if (!bt && !usbOn) return;
  btlink::Stats bs = btlink::stats();
  RtcmStat rs = rtcmStat();
  gnss::Stats gs = gnss::stats();
  float v = power::batteryVolts();
  char pname[25];
  safeCopy(pname, sizeof(pname), settings::profile(settings::active()).name);
  char b[640];
  int n = snprintf(b, sizeof(b), "$PRVR,STA,%s,%.2f,%d,%d,%d,%s,%lu,%lu,", FW_VERSION, v, power::batteryPercentStable(),
                   settings::active(), settings::count(), pname, (unsigned long)rs.ok, (unsigned long)rs.bad);
  if (rs.ok) n += snprintf(b + n, sizeof(b) - n, "%.1f,%u,", (millis() - rs.lastMs) / 1000.0f, rs.lastType);
  else n += snprintf(b + n, sizeof(b) - n, ",,");
  n += snprintf(b + n, sizeof(b) - n, "%lu,%lu,0,0,0,%lu,%lu,%d,%d,%lu", (unsigned long)gs.ubxErr,
                (unsigned long)ESP.getFreeHeap(), (unsigned long)bs.txDropped, (unsigned long)btlink::backlog(),
                appMode() ? 1 : 0, gs.eoeCount ? 1 : 0, (unsigned long)gs.hpCount);
  // od FW 0.11.0: rámce UBX z mobilu (OK, vadné), MGA-ACK (přijato, odmítnuto), vyslané MGA-DBD, ACKAIDING
  n += snprintf(b + n, sizeof(b) - n, ",%lu,%lu,%lu,%lu,%lu,%d", (unsigned long)bs.ubxOk, (unsigned long)bs.ubxBad,
                (unsigned long)gs.mgaAccepted, (unsigned long)gs.mgaRejected, (unsigned long)gs.mgaDbd, gs.ackAiding);
  // od FW 0.12.0: frekvence navigace aktivního profilu [Hz]
  n += snprintf(b + n, sizeof(b) - n, ",%u", (unsigned)navRateHz());
  // od FW 0.13.0: zdroj korekcí, signály s korekcí, výsledek nastavení korekcí, průchozí režim, rámce NAV při záznamu
  const gnss::SigSummary &g = gnss::sig();
  bool sigOk = g.valid && millis() - g.rxMs < 3000;
  n += snprintf(b + n, sizeof(b) - n, ",%d,%u,%d,%d,%d", sigOk ? g.dominant : -1, sigOk ? g.corrUsed : 0, gs.corrCfg,
                hooks.bridgeActive && hooks.bridgeActive() ? 1 : 0, navRecording() ? 1 : 0);
  // od FW 0.14.0: stav přístroje a automatický model
  const motion::Features &mf = motion::features();
  n += snprintf(b + n, sizeof(b) - n, ",%u,%d,%u,%d,%d,%.1f,%d,%.1f,%.2f,%lu", (unsigned)motion::state(),
                motion::autoOn() ? 1 : 0, gnss::dynEffective(), (int)lroundf(mf.accStd * 1000),
                (int)lroundf(mf.accDev * 1000), mf.gyroMax, (int)lroundf(mf.stepScore * 100), mf.stepHz, mf.speed,
                (unsigned long)motion::changes());
  // od FW 0.15.0: teplota IMU, klid, kroky, průběžná kalibrace gyra
  imu::BiasInfo bi = imu::biasInfo();
  n += snprintf(b + n, sizeof(b) - n, ",%.2f,%d,%lu,%lu,%.4f,%.4f,%.4f,%d", imu::temperature(), bi.still ? 1 : 0,
                (unsigned long)motion::steps(), (unsigned long)bi.updates, bi.bx, bi.by, bi.bz,
                imu::calRunning() ? 1 : 0);
  // od FW 0.15.1: příjem z mobilu – zahozené bajty (přetečení bufferu), největší zaplnění bufferu, zrušené „JSON“
  n += snprintf(b + n, sizeof(b) - n, ",%lu,%lu,%lu", (unsigned long)bs.rxDropped, (unsigned long)bs.rxMaxFill,
                (unsigned long)bs.cmdAborted);
  // od FW 0.16.0: měření rychlosti (0/1), aktualizace biasu ze stání v autě, probíhající stání v autě [s]
  n += snprintf(b + n, sizeof(b) - n, ",%d,%lu,%lu", speedOn ? 1 : 0, (unsigned long)bi.carUpdates,
                (unsigned long)bi.carStopSec);
  // od FW 0.17.0: režim USB host (0/1), zdroj RTCM do X20 (0 žádný, 1 BT, 2 USB), RTCM z USB správné / chybné
  const RxParser::Stats &us = usblink::stats();
  n += snprintf(b + n, sizeof(b) - n, ",%d,%u,%lu,%lu", usbOn ? 1 : 0, rtcmSource(), (unsigned long)us.rtcmOk,
                (unsigned long)us.rtcmBad);
  n = seal(b, sizeof(b), n);
  if (!n) return;
  if (bt) {
    btlink::send(b, n);
    btlink::flush();
  }
  if (usbOn) usblink::send(b, n);
}

// $PRVR,SAT,část,částí,celkem,{gnssId,svId,C/N0,elevace,azimut,použit}×(max 12)
// Jen sledované družice (C/N0 > 0).
void sendSat() {
  if (!btlink::connected()) return;
  const gnss::SatSummary &s = gnss::sats();
  if (!s.valid) return;
  static uint8_t skip = 0;
  ++skip;
  if (speedOn) {
    if (skip % 5) return;  // měření rychlosti: každých 5 s (šetří BT)
  } else if (appMode() && (skip & 1)) {
    return;  // v režimu aplikace každé 2 s
  }
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
// Odpověď na jednom řádku; do BT po částech (dávka BT má 2 kB), do konzole USB přímo
static bool toConsole = false;
static bool fromUsb = false;       // příkaz z ESP32-S3 (režim USB host)
static bool calToConsole = false;  // kam poslat výsledek kalibrace gyra na pozadí
static void calJson(JsonDocument &d);
static void reply(JsonDocument &d);

// Výsledek kalibrace gyra na pozadí: {"ev":"cal_gyro","ok":true|false,"gyro":[…],…}
void pollCal() {
  bool ok = false;
  if (!imu::calResult(ok)) return;
  JsonDocument d;
  d["ev"] = "cal_gyro";
  d["ok"] = ok;
  if (!ok) d["err"] = "pohyb nebo málo vzorků – bias beze změny";
  calJson(d);
  bool prev = toConsole;
  toConsole = calToConsole;
  if (toConsole || btlink::connected()) reply(d);
  toConsole = prev;
}

static void reply(JsonDocument &d) {
  String out;
  serializeJson(d, out);
  out += "\r\n";
  // režim USB host: odpověď na Serial i při umlčené konzoli (i na příkaz usbhost, který režim zapnul)
  if (fromUsb || (toConsole && usbOn)) {
    usblink::send(out.c_str(), out.length());
    return;
  }
  if (toConsole) {
    Con.print(out);
    return;
  }
  btlink::flush();  // rozpracovaná dávka NMEA nejdřív
  for (size_t i = 0; i < out.length(); i += 1024) {
    size_t n = out.length() - i < 1024 ? out.length() - i : 1024;
    btlink::send(out.c_str() + i, n);
    btlink::flush();
  }
}

bool navRecording() { return ebuf::recording() && btlink::connected() && !speedMode(); }

// ---------------------------------------------------------------- měření rychlosti (FW 0.16.0)
bool speedMode() { return speedOn; }

uint8_t navRateHz() {
  if (usbOn) return usbHz;
  return speedOn ? SPEED_HZ : settings::profile(settings::active()).rateHz;
}

static void speedStart() {
  const settings::Profile &p = settings::profile(settings::active());
  speedOn = true;
  speedAliveMs = millis();
  motion::setAuto(false);  // model napevno automotive
  gnss::setParams(1000 / SPEED_HZ, settings::dynCode(settings::Dyn::Automotive), p.elevMask);
  gnss::configure(true);
  Con.println("[rychlost] ZAP – X20 10 Hz, automotive, IMU 120 Hz do BT");
}

void speedCancel() {
  if (!speedOn) return;
  speedOn = false;
  Con.println("[rychlost] VYP – přepnutí profilu");
}

static void speedStop(const char *why) {
  if (!speedOn) return;
  speedOn = false;
  Con.printf("[rychlost] VYP – %s, návrat na profil %s\n", why, settings::profile(settings::active()).name);
  if (hooks.applyProfile) hooks.applyProfile(settings::active());
}

void speedPoll() {
  if (speedOn && millis() - speedAliveMs > SPEED_ALIVE_MS) speedStop("aplikace 30 s neozvala");
}

// ---------------------------------------------------------------- režim USB host (FW 0.17.0)
// ESP32-S3 připojená kabelem USB-C pošle ze Serial {"cmd":"usbhost","hz":5,"model":"pedestrian"} a opakuje ho 1×/s.
// Režim je jen v RAM (nic se neukládá), mimo seznam profilů. Frekvence a model z příkazu, ostatní (maska, SBAS, HAS,
// výška antény, PCO) z aktivního profilu. 3 s bez příkazu -> návrat na aktivní profil a obnovení konzole.
bool usbMode() { return usbOn; }

static void usbApply() {
  const settings::Profile &p = settings::profile(settings::active());
  motion::setAuto(usbDyn == settings::Dyn::Auto);  // pevný model z S3 – automatické přepínání vypnout
  gnss::setParams(1000 / usbHz, settings::dynCode(usbDyn), p.elevMask);
  gnss::configure(false);
}

static void usbStart(uint8_t hz, settings::Dyn dyn) {
  Con.printf("[usb] režim USB host ZAP – %u Hz, %s; konzole umlčena\n", hz, settings::dynName(dyn));
  speedOn = false;  // měření rychlosti končí, parametry X20 nastaví usbApply()
  if (gnss::measStatic() || gnss::measRaw()) gnss::setMeasMode(false, false);  // statický model / RAWX vypnout
  usbOn = true;
  usbHz = hz;
  usbDyn = dyn;
  usbAliveMs = millis();
  usblink::setActive(true);
  btlink::setRtcmBlocked(true);  // RTCM z USB má přednost
  usbApply();
}

static void usbStop(const char *why) {
  if (!usbOn) return;
  usbOn = false;
  usblink::setActive(false);
  btlink::setRtcmBlocked(false);
  Con.printf("\n[usb] režim USB host VYP – %s, návrat na profil %s\n", why,
             settings::profile(settings::active()).name);
  if (hooks.applyProfile) hooks.applyProfile(settings::active());
}

void usbPoll() {
  if (usbOn && millis() - usbAliveMs > USB_ALIVE_MS) usbStop("3 s bez příkazu z S3");
}

uint8_t rtcmSource() {
  uint32_t now = millis();
  uint32_t b = btlink::rtcmFwdMs(), u = usblink::rtcmFwdMs();
  bool bOk = b && now - b < 30000, uOk = u && now - u < 30000;
  if (uOk && (!bOk || (int32_t)(u - b) >= 0)) return 2;
  return bOk ? 1 : 0;
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
  p["hz"] = navRateHz();
  p["speed"] = speedOn;
  p["usbhost"] = usbOn;
  float v = power::batteryVolts();
  d["bat"] = serialized(String(v, 2));
  d["heap"] = ESP.getFreeHeap();
  d["up"] = millis() / 1000;
  // od 0.13.0: rover umí nastavení z aplikace (profily, přístroj, kalibrace, průchozí režim)
  d["caps"] = "set,cal,pass,navrec,sig,boot,motion,speed,usbhost";
  // doba od zapnutí ESP (= napájení X20) do prvního fixu atd. [ms]; -1 = zatím ne
  JsonObject bt = d["boot"].to<JsonObject>();
  bt["fix"] = bootFix ? (int32_t)bootFix : -1;
  bt["corr"] = bootCorr ? (int32_t)bootCorr : -1;
  bt["float"] = bootFloat ? (int32_t)bootFloat : -1;
  bt["fixRtk"] = bootRtk ? (int32_t)bootRtk : -1;
  d["upMs"] = millis();
  d["reset"] = (int)esp_reset_reason();  // 1 zapnutí napájení, 3 SW restart, …
}

static void calJson(JsonDocument &d) {
  float bx, by, bz;
  imu::gyroBias(bx, by, bz);
  JsonArray g = d["gyro"].to<JsonArray>();
  g.add(serialized(String(bx, 3)));
  g.add(serialized(String(by, 3)));
  g.add(serialized(String(bz, 3)));
  d["div"] = serialized(String(power::dividerFactor(), 4));
  d["bat"] = serialized(String(power::batteryVolts(), 3));
  d["batOk"] = power::valid();
}

static void handle(const char *line, size_t len) {
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
  bool fromBt = !toConsole && !fromUsb;
  if (fromBt) app = true;  // příkaz poslala aplikace RoverGnss
  const char *cmd = in["cmd"] | "";
  const char *what = in["what"] | "";
  out["ok"] = true;
  bool restartAfter = false;

  // Režim USB host: frekvenci a model určuje S3 – změny profilu, měření rychlosti a bodu a průchozí režim odmítnout
  if (usbOn) {
    const char *op = in["op"] | "";
    if (!strcmp(cmd, "profile") || (!strcmp(cmd, "speed") && !strcmp(op, "start")) ||
        (!strcmp(cmd, "meas") && !strcmp(op, "start")) || !strcmp(cmd, "passthrough")) {
      out["ok"] = false;
      out["err"] = "režim USB host";
      out["usbhost"] = true;
      out["hz"] = navRateHz();
      reply(out);
      return;
    }
  }

  if (!strcmp(cmd, "get") && !strcmp(what, "info")) {
    infoJson(out);
  } else if (!strcmp(cmd, "get") && !strcmp(what, "profiles")) {
    out["data"] = serialized(settings::profilesJson());
  } else if (!strcmp(cmd, "set") && !strcmp(what, "profiles")) {
    String data;
    serializeJson(in["data"], data);
    String err;
    if (in["data"].isNull() || !settings::setProfilesJson(data.c_str(), err)) {
      out["ok"] = false;
      out["err"] = in["data"].isNull() ? String("chybí data") : err;
    } else {
      // v režimu USB host jen uložit – aktivní profil se aplikuje po konci režimu
      if (hooks.applyProfile && !usbOn) hooks.applyProfile(settings::active());
      out["data"] = serialized(settings::profilesJson());
      Con.printf("[app] profily uloženy (%d), aktivní %d\n", settings::count(), settings::active() + 1);
    }
  } else if (!strcmp(cmd, "get") && !strcmp(what, "device")) {
    out["data"] = serialized(settings::deviceJson());
    out["default"] = btlink::defaultName();
    out["name"] = btlink::name();
  } else if (!strcmp(cmd, "set") && !strcmp(what, "device")) {
    String data;
    serializeJson(in["data"], data);
    String err;
    if (in["data"].isNull() || !settings::setDeviceJson(data.c_str(), err)) {
      out["ok"] = false;
      out["err"] = in["data"].isNull() ? String("chybí data") : err;
    } else {
      out["data"] = serialized(settings::deviceJson());
      out["restart"] = true;  // název BT platí po restartu
    }
  } else if (!strcmp(cmd, "get") && !strcmp(what, "cal")) {
    calJson(out);
  } else if (!strcmp(cmd, "cal")) {
    if (!strcmp(what, "battery")) {
      float v = in["v"] | 0.0f;
      if (!power::calibrate(v)) {
        out["ok"] = false;
        out["err"] = "neplatné napětí (koeficient musí vyjít 1,6–2,4)";
      }
    } else if (!strcmp(what, "gyro")) {
      if (!imu::present()) {
        out["ok"] = false;
        out["err"] = "IMU nenalezeno";
      } else {
        // FW 0.15.0: 30 s na pozadí (smyčka a BT běží dál), výsledek přijde jako událost "cal_gyro"
        uint32_t ms = in["ms"] | 30000;
        if (ms < 3000) ms = 3000;
        if (ms > 120000) ms = 120000;
        calToConsole = toConsole || fromUsb;
        imu::startCalibration(ms);
        out["pending"] = true;
        out["ms"] = ms;
      }
    } else {
      out["ok"] = false;
      out["err"] = "what: battery|gyro";
    }
    calJson(out);
  } else if (!strcmp(cmd, "passthrough")) {
    const char *op = in["op"] | "";
    bool on = !strcmp(op, "start");
    if (!on && strcmp(op, "stop")) {
      out["ok"] = false;
      out["err"] = "op: start|stop";
    } else if (hooks.bridge) {
      reply(out);  // odpověď ještě před přepnutím USB na X20
      hooks.bridge(on);
      return;
    }
  } else if (!strcmp(cmd, "restart")) {
    restartAfter = true;
  } else if (!strcmp(cmd, "profile")) {
    int i = in["i"] | -1;
    if (i < 0 || i >= settings::count() || !hooks.applyProfile) {
      out["ok"] = false;
      out["err"] = "neplatný profil";
    } else {
      hooks.applyProfile(i);
      out["i"] = settings::active();
      out["name"] = settings::profile(settings::active()).name;
      out["hz"] = navRateHz();
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
      Con.printf("[app] měření bodu %s%s%s (X20 %s)\n", st || raw ? "ZAP" : "VYP", st ? ", statický model" : "",
                    raw ? ", RAWX" : "", ack ? "OK" : "bez potvrzení");
    }
  } else if (!strcmp(cmd, "speed")) {
    const char *op = in["op"] | "";
    if (!strcmp(op, "start")) {
      if (!speedOn) speedStart();
      else speedAliveMs = millis();
    } else if (!strcmp(op, "alive")) {
      if (speedOn) speedAliveMs = millis();
    } else if (!strcmp(op, "stop")) {
      speedStop("příkaz z aplikace");
    } else {
      out["ok"] = false;
      out["err"] = "op: start|alive|stop";
    }
    out["speed"] = speedOn;
    out["hz"] = navRateHz();
  } else if (!strcmp(cmd, "usbhost")) {
    // {"cmd":"usbhost","hz":5,"model":"pedestrian"} – zapne režim USB host, opakováním 1×/s ho drží
    out["cmd"] = "usbhost";
    uint8_t hz = in["hz"] | 5;
    const char *m = in["model"] | "pedestrian";
    settings::Dyn d;
    if (fromBt) {
      out["ok"] = false;
      out["err"] = "jen ze Serial (USB)";
    } else if (hz != 1 && hz != 5 && hz != 10) {
      out["ok"] = false;
      out["err"] = "hz: 1|5|10";
    } else if (!settings::dynFromName(m, d)) {
      out["ok"] = false;
      out["err"] = "model: auto|pedestrian|automotive|stationary";
    } else if (!usbOn) {
      usbStart(hz, d);
    } else {
      usbAliveMs = millis();
      if (hz != usbHz || d != usbDyn) {  // heartbeat se změnou parametrů – přenastavit X20
        usbHz = hz;
        usbDyn = d;
        usbApply();
      }
    }
    out["usbhost"] = usbOn;
    out["hz"] = navRateHz();
    out["model"] = settings::dynName(usbOn ? usbDyn : settings::profile(settings::active()).dyn);
    out["profile"] = settings::active();
    out["name"] = settings::profile(settings::active()).name;
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
    if (out["ok"]) Con.printf("[app] X20 %s start\n", gnss::resetKindText());
  } else {
    out["ok"] = false;
    out["err"] = "neznámý příkaz";
  }
  reply(out);
  if (restartAfter && hooks.restart) {
    if (!toConsole && !fromUsb) btlink::flush();
    delay(300);  // odpověď musí odejít
    hooks.restart();
  }
}

void handleCmd(const char *line, size_t len) {
  toConsole = false;
  handle(line, len);
}

void handleConsoleCmd(const char *line, size_t len) {
  toConsole = true;
  handle(line, len);
  toConsole = false;
}

void handleUsbCmd(const char *line, size_t len) {
  fromUsb = true;
  handle(line, len);
  fromUsb = false;
}

}  // namespace proto
