#include "gnss.h"
#include "console.h"

#include <esp_timer.h>

#include "config.h"
#include "ubx.h"

namespace gnss {

static HardwareSerial &UART = Serial2;

static uint32_t curBaud = 0;
static Pvt pvtData;
static SatSummary satData;
static Dop dopData;
static Cov covData;
static SigSummary sigData;
static uint32_t sigCnt = 0;
static int8_t corrCfgRes = -1;
static void (*pvtCb)() = nullptr;
static void (*satCb)() = nullptr;
static Info infoData;
static uint32_t bytesRx = 0;
static uint32_t pvtCnt = 0, satCnt = 0;
// Konec epochy: NAV-PVT čeká na NAV-EOE, aby měla epocha i HPPOSLLH a COV téže epochy
static bool pvtWaiting = false;
static uint32_t pvtWaitMs = 0;
static uint32_t eoeLastMs = 0, eoeCnt = 0, hpCnt = 0;
static constexpr uint32_t EOE_TIMEOUT_MS = 150;  // EOE nepřišlo – epocha se pošle i tak

static void firePvt() {
  pvtWaiting = false;
  if (pvtData.hp) hpCnt++;
  if (pvtCb) pvtCb();
}
static int32_t pvtLatency = 0;
static volatile uint32_t lastUbxMs = 0;  // poslední platná zpráva UBX (hlídání spojení)
static uint32_t resetMs = 0;             // restart příjmu GNSS (CFG-RST)
static uint32_t ttffMs = 0;              // doba do prvního fixu po restartu
static uint8_t resetKind = 0;            // 0 žádný, 1 horký, 2 teplý, 3 studený

// odpovědi na dotazy
static volatile bool gotVer = false;
static volatile int8_t ackState = -1;  // 1 ACK, 0 NAK, -1 nic
static uint8_t ackCls = 0, ackId = 0;

// PPS
static volatile uint32_t ppsCnt = 0;
static volatile int64_t ppsLast = 0;
static volatile uint32_t ppsPeriod = 0;

static void handleUbx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len);
static ubx::Parser parser(handleUbx);

// ---------------------------------------------------------------- PPS
static void IRAM_ATTR ppsIsr() {
  int64_t now = esp_timer_get_time();
  // GPIO36: errata ESP32 – falešné hrany při ADC/Wi-Fi. PPS 1 Hz, kratší interval zahodit.
  int64_t d = now - ppsLast;
  if (ppsCnt != 0 && d < 500000) return;
  if (ppsCnt != 0) ppsPeriod = (uint32_t)d;
  ppsLast = now;
  ppsCnt = ppsCnt + 1;
}

// ---------------------------------------------------------------- UBX zpracování
static void copyStr(char *dst, const uint8_t *src, size_t n) {
  memcpy(dst, src, n);
  dst[n] = 0;
}

// NAV-HPPOSLLH: přesná poloha (0,1 mm). Může přijít před i po NAV-PVT téže epochy.
struct Hp {
  bool valid = false;
  uint32_t iTOW = 0;
  double lat = 0, lon = 0, hEll = 0, hMSL = 0;
};
static Hp hpData;
static bool rtcmOn = false;

void setRtcmActive(bool on) { rtcmOn = on; }
bool rtcmActive() { return rtcmOn; }

static void applyHp(Pvt &d) {
  if (!hpData.valid || hpData.iTOW != d.iTOW || d.hp) return;
  d.lat = hpData.lat;
  d.lon = hpData.lon;
  d.hEll = hpData.hEll;
  d.hMSL = hpData.hMSL;
  d.hp = true;
}

static void (*rawFrameCb)(const uint8_t *, uint16_t) = nullptr;
static uint32_t mgaAcc = 0, mgaRej = 0, mgaDbdCnt = 0;
static int8_t ackAidingRes = -1;
void onRawFrame(void (*cb)(const uint8_t *, uint16_t)) { rawFrameCb = cb; }
static void (*navFrameCb)(const uint8_t *, uint16_t) = nullptr;
void onNavFrame(void (*cb)(const uint8_t *, uint16_t)) { navFrameCb = cb; }

// Celý rámec NAV-PVT / COV / HPPOSLLH dál (jen když ho main chce – záznam v aplikaci)
static void navFrame(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len) {
  if (!navFrameCb || len + 8 > 160) return;
  uint8_t fr[160];
  uint16_t n = ubx::frame(cls, id, p, len, fr);
  navFrameCb(fr, n);
}

static void handleUbx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len) {
  lastUbxMs = millis();
  if (cls == ubx::CLS_RXM && (id == ubx::ID_RXM_RAWX || id == ubx::ID_RXM_SFRBX)) {
    if (!rawFrameCb || len + 8 > 2056) return;
    static uint8_t fr[2056];
    uint16_t n = ubx::frame(cls, id, p, len, fr);
    rawFrameCb(fr, n);
    return;
  }
  // Asistence: potvrzení a výpis databáze jdou celé do BT (aplikace je ukládá a vyhodnocuje)
  if (cls == ubx::CLS_MGA && (id == ubx::ID_MGA_ACK || id == ubx::ID_MGA_DBD)) {
    if (id == ubx::ID_MGA_ACK && len >= 1) {
      if (p[0] == 1) mgaAcc++;
      else mgaRej++;
    } else if (id == ubx::ID_MGA_DBD) {
      mgaDbdCnt++;
    }
    if (!rawFrameCb || len + 8 > 2056) return;
    static uint8_t fm[2056];
    uint16_t n = ubx::frame(cls, id, p, len, fm);
    rawFrameCb(fm, n);
    return;
  }
  if (cls == ubx::CLS_NAV && (id == ubx::ID_NAV_PVT || id == ubx::ID_NAV_COV || id == ubx::ID_NAV_HPPOSLLH))
    navFrame(cls, id, p, len);
  if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_HPPOSLLH && len >= 36) {
    if (p[3] & 0x01) return;  // invalidLlh
    hpData.iTOW = ubx::rdU4(p + 4);
    hpData.lon = ubx::rdI4(p + 8) * 1e-7 + (int8_t)p[24] * 1e-9;
    hpData.lat = ubx::rdI4(p + 12) * 1e-7 + (int8_t)p[25] * 1e-9;
    hpData.hEll = ubx::rdI4(p + 16) * 1e-3 + (int8_t)p[26] * 1e-4;
    hpData.hMSL = ubx::rdI4(p + 20) * 1e-3 + (int8_t)p[27] * 1e-4;
    hpData.valid = true;
    applyHp(pvtData);  // NAV-PVT téže epochy už mohl přijít
    return;
  }
  if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_PVT && len >= 92) {
    if (pvtWaiting) firePvt();  // EOE předchozí epochy se ztratilo – odeslat ji dřív, než se přepíše
    Pvt &d = pvtData;
    d.rxUs = esp_timer_get_time();
    d.rxMs = millis();
    d.iTOW = ubx::rdU4(p + 0);
    d.year = ubx::rdU2(p + 4);
    d.month = p[6];
    d.day = p[7];
    d.hour = p[8];
    d.min = p[9];
    d.sec = p[10];
    d.timeValid = (p[11] & 0x03) == 0x03;
    d.fixType = p[20];
    d.gnssFixOk = p[21] & 0x01;
    d.diffSoln = p[21] & 0x02;
    d.carrSoln = (p[21] >> 6) & 0x03;
    d.numSV = p[23];
    d.lon = ubx::rdI4(p + 24) * 1e-7;
    d.lat = ubx::rdI4(p + 28) * 1e-7;
    d.hEll = ubx::rdI4(p + 32) * 1e-3;
    d.hMSL = ubx::rdI4(p + 36) * 1e-3;
    d.hAcc = ubx::rdU4(p + 40) * 1e-3f;
    d.vAcc = ubx::rdU4(p + 44) * 1e-3f;
    d.gSpeed = ubx::rdI4(p + 60) * 1e-3f;
    d.headMot = ubx::rdI4(p + 64) * 1e-5f;
    d.sAcc = ubx::rdU4(p + 68) * 1e-3f;
    d.pDOP = ubx::rdU2(p + 76) * 0.01f;
    d.corrAge = (ubx::rdU2(p + 78) >> 1) & 0x0F;
    d.hp = false;
    applyHp(d);
    d.valid = true;
    pvtCnt++;
    if (resetMs && !ttffMs && d.gnssFixOk && d.fixType >= 2) ttffMs = d.rxMs - resetMs;
    // Zpoždění zprávy za PPS: jen epochy na celé sekundě a PPS z poslední sekundy
    if (d.iTOW % 1000 == 0 && ppsCnt > 0) {
      int64_t dt = d.rxUs - ppsLast;
      if (dt >= 0 && dt < 1000000) pvtLatency = (int32_t)dt;
    }
    // Když X20 posílá EOE (poslední 3 s), počkat na konec epochy; jinak hned
    if (eoeLastMs && millis() - eoeLastMs < 3000) {
      pvtWaiting = true;
      pvtWaitMs = millis();
    } else {
      firePvt();
    }
  } else if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_EOE && len >= 4) {
    eoeLastMs = millis();
    eoeCnt++;
    if (pvtWaiting && ubx::rdU4(p) == pvtData.iTOW) firePvt();
  } else if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_DOP && len >= 18) {
    dopData.pDOP = ubx::rdU2(p + 6) * 0.01f;
    dopData.vDOP = ubx::rdU2(p + 10) * 0.01f;
    dopData.hDOP = ubx::rdU2(p + 12) * 0.01f;
    dopData.rxMs = millis();
    dopData.valid = true;
  } else if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_COV && len >= 40) {
    float nn, ee, dd;
    memcpy(&nn, p + 16, 4);
    memcpy(&ee, p + 28, 4);
    memcpy(&dd, p + 36, 4);
    covData.valid = p[5] != 0;
    covData.sN = nn > 0 ? sqrtf(nn) : 0;
    covData.sE = ee > 0 ? sqrtf(ee) : 0;
    covData.sD = dd > 0 ? sqrtf(dd) : 0;
    covData.rxMs = millis();
  } else if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_SIG && len >= 8) {
    // blok 16 B: gnssId, svId, sigId, freqId, prRes, cno, qualityInd, corrSource, ionoModel, sigFlags (X2), rez.
    SigSummary g;
    uint8_t n = p[5];
    for (uint8_t i = 0; i < n; i++) {
      uint16_t o = 8 + i * 16;
      if (o + 16 > len) break;
      g.n++;
      uint8_t src = p[o + 8];
      uint16_t fl = ubx::rdU2(p + o + 10);
      if (fl & 0x0008) g.used++;                 // prUsed
      if (fl & 0x00C0) {                         // prCorrUsed | crCorrUsed
        g.corrUsed++;
        if (src < 16) g.bySrc[src]++;
      }
    }
    uint8_t best = 0;
    for (int k = 1; k < 16; k++) {
      if (g.bySrc[k] > best) {
        best = g.bySrc[k];
        g.dominant = k;
      }
    }
    g.valid = true;
    g.rxMs = millis();
    sigData = g;
    sigCnt++;
  } else if (cls == ubx::CLS_NAV && id == ubx::ID_NAV_SAT && len >= 8) {
    static SatSummary s;  // statická – obsahuje seznam satelitů (velká na zásobník)
    s = SatSummary();
    uint8_t n = p[5];
    uint32_t cnoSum = 0;
    for (uint8_t i = 0; i < n; i++) {
      uint16_t o = 8 + i * 12;
      if (o + 12 > len) break;
      uint8_t gnssId = p[o];
      uint8_t cno = p[o + 2];
      uint32_t flags = ubx::rdU4(p + o + 8);
      bool used = flags & 0x08;
      if (cno > 0) s.tracked++;
      if (cno > s.cnoMax) s.cnoMax = cno;
      if (s.count < MAX_SATS) {
        SatInfo &si = s.list[s.count++];
        si.gnssId = gnssId;
        si.svId = p[o + 1];
        si.cno = cno;
        si.elev = (int8_t)p[o + 3];
        si.azim = (int16_t)ubx::rdU2(p + o + 4);
        si.used = used;
      }
      if (used) {
        s.used++;
        cnoSum += cno;
        if (gnssId < 8) s.usedBy[gnssId]++;
      }
    }
    s.cnoMeanUsed = s.used ? (float)cnoSum / s.used : 0;
    s.valid = true;
    s.rxMs = millis();
    satData = s;
    satCnt++;
    if (satCb) satCb();
  } else if (cls == ubx::CLS_MON && id == ubx::ID_MON_VER && len >= 40) {
    Info &i = infoData;
    copyStr(i.sw, p, 30);
    copyStr(i.hw, p + 30, 10);
    i.fwver[0] = 0;
    i.mod[0] = 0;
    for (uint16_t o = 40; o + 30 <= len; o += 30) {
      char ext[31];
      copyStr(ext, p + o, 30);
      if (strncmp(ext, "FWVER=", 6) == 0) strlcpy(i.fwver, ext + 6, sizeof(i.fwver));
      if (strncmp(ext, "MOD=", 4) == 0) strlcpy(i.mod, ext + 4, sizeof(i.mod));
    }
    i.valid = true;
    gotVer = true;
  } else if (cls == ubx::CLS_ACK && len >= 2) {
    ackCls = p[0];
    ackId = p[1];
    ackState = (id == ubx::ID_ACK_ACK) ? 1 : 0;
  }
}

// ---------------------------------------------------------------- odesílání
static void send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t frameBuf[300];
  if (len + 8 > sizeof(frameBuf)) return;
  uint16_t n = ubx::frame(cls, id, payload, len, frameBuf);
  UART.write(frameBuf, n);
  UART.flush();
}

// Pošle CFG-VALSET a čeká na ACK/NAK. Vrací 1 ACK, 0 NAK, -1 bez odpovědi.
static int8_t sendValSet(const ubx::ValSet &v, uint32_t timeoutMs) {
  ackState = -1;
  send(ubx::CLS_CFG, ubx::ID_CFG_VALSET, v.data(), v.size());
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    poll();
    if (ackState >= 0 && ackCls == ubx::CLS_CFG && ackId == ubx::ID_CFG_VALSET) return ackState;
    delay(1);
  }
  return -1;
}

static void uartBegin(uint32_t b) {
  UART.end();
  UART.setRxBufferSize(8192);
  UART.begin(b, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  curBaud = b;
}

// ---------------------------------------------------------------- veřejné API
void begin() {
  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), ppsIsr, RISING);
  uartBegin(GNSS_BAUD_TARGET);
}

bool pollVersion(uint32_t timeoutMs) {
  gotVer = false;
  send(ubx::CLS_MON, ubx::ID_MON_VER, nullptr, 0);
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    poll();
    if (gotVer) return true;
    delay(1);
  }
  return false;
}

bool detect() {
  for (uint32_t b : GNSS_BAUDS) {
    uartBegin(b);
    delay(20);
    while (UART.available()) UART.read();
    if (pollVersion(500)) return true;
  }
  uartBegin(GNSS_BAUD_TARGET);
  return false;
}

static uint16_t measRateMs = GNSS_MEAS_RATE_MS;
static uint8_t dynModel = GNSS_DYNMODEL;
static int8_t minElev = 10;
static bool corrSbas = true, corrHas = true;
static bool mStatic = false;   // měření bodu: statický model
static bool mRaw = false;      // měření bodu: RAWX/SFRBX
static constexpr uint8_t DYN_STATIONARY = 2;

static uint8_t effDyn() { return mStatic ? DYN_STATIONARY : dynModel; }
static uint8_t rawxRate() { return mRaw ? (uint8_t)(1000 / measRateMs) : 0; }  // 1× za sekundu

void setCorr(bool sbas, bool has) {
  corrSbas = sbas;
  corrHas = has;
}

void setParams(uint16_t rateMs, uint8_t dyn, int8_t elev) {
  measRateMs = rateMs;
  dynModel = dyn;
  minElev = elev;
}

bool configure(bool verbose) {
  struct Item {
    uint32_t key;
    uint8_t size;
    uint32_t val;
    const char *name;
  };
  const Item items[] = {
      {ubx::KEY_UART1INPROT_UBX, 1, 1, "UART1INPROT-UBX"},
      {ubx::KEY_UART1INPROT_RTCM3X, 1, 1, "UART1INPROT-RTCM3X"},
      {ubx::KEY_UART1OUTPROT_UBX, 1, 1, "UART1OUTPROT-UBX"},
      {ubx::KEY_UART1OUTPROT_NMEA, 1, 0, "UART1OUTPROT-NMEA"},
      {ubx::KEY_MSGOUT_NAV_PVT_UART1, 1, 1, "MSGOUT-NAV_PVT_UART1"},
      {ubx::KEY_MSGOUT_NAV_SAT_UART1, 1, (uint32_t)(1000 / measRateMs), "MSGOUT-NAV_SAT_UART1"},
      {ubx::KEY_MSGOUT_NAV_DOP_UART1, 1, 1, "MSGOUT-NAV_DOP_UART1"},
      {ubx::KEY_MSGOUT_NAV_COV_UART1, 1, 1, "MSGOUT-NAV_COV_UART1"},
      {ubx::KEY_MSGOUT_NAV_HPPOSLLH_UART1, 1, 1, "MSGOUT-NAV_HPPOSLLH_UART1"},
      {ubx::KEY_MSGOUT_NAV_EOE_UART1, 1, 1, "MSGOUT-NAV_EOE_UART1"},
      {ubx::KEY_MSGOUT_NAV_SIG_UART1, 1, (uint32_t)(1000 / measRateMs), "MSGOUT-NAV_SIG_UART1"},
      {ubx::KEY_RATE_MEAS, 2, measRateMs, "RATE-MEAS"},
      {ubx::KEY_RATE_NAV, 2, 1, "RATE-NAV"},
      {ubx::KEY_NAVSPG_DYNMODEL, 1, effDyn(), "NAVSPG-DYNMODEL"},
      {ubx::KEY_MSGOUT_RXM_RAWX_UART1, 1, rawxRate(), "MSGOUT-RXM_RAWX_UART1"},
      {ubx::KEY_MSGOUT_RXM_SFRBX_UART1, 1, (uint32_t)(mRaw ? 1 : 0), "MSGOUT-RXM_SFRBX_UART1"},
      {ubx::KEY_NAVSPG_INFIL_MINELEV, 1, (uint32_t)(uint8_t)minElev, "NAVSPG-INFIL_MINELEV"},
  };
  auto add = [](ubx::ValSet &v, const Item &it) {
    if (it.size == 1) v.addU1(it.key, (uint8_t)it.val);
    else if (it.size == 2) v.addU2(it.key, (uint16_t)it.val);
    else v.addU4(it.key, it.val);
  };

  // Potvrzování asistence (MGA-ACK) zvlášť: neznámý klíč nesmí shodit hlavní konfiguraci
  {
    ubx::ValSet a(ubx::LAYER_RAM);
    a.addL(ubx::KEY_NAVSPG_ACKAIDING, true);
    ackAidingRes = sendValSet(a, 500);
    if (verbose) Con.printf("  NAVSPG-ACKAIDING: %s\n", ackAidingRes == 1 ? "OK" : ackAidingRes == 0 ? "NAK" : "bez odpovědi");
  }
  // Korekce zvlášť (klíče CFG-NAVCOR jsou jen v HPG 2.1x): SBAS, HAS, korekce od hostitele (RTCM) vždy zap.
  {
    ubx::ValSet c(ubx::LAYER_RAM);
    c.addL(ubx::KEY_SBAS_USE_DIFFCORR, corrSbas);
    c.addL(ubx::KEY_NAVCOR_ENABLE_HOST, true);
    c.addL(ubx::KEY_NAVCOR_ENABLE_GAL_HAS, corrHas);
    corrCfgRes = sendValSet(c, 500);
    if (verbose)
      Con.printf("  korekce SBAS %s, HAS %s, RTCM zap: %s\n", corrSbas ? "zap" : "vyp", corrHas ? "zap" : "vyp",
                    corrCfgRes == 1 ? "OK" : corrCfgRes == 0 ? "NAK" : "bez odpovědi");
  }

  ubx::ValSet all(ubx::LAYER_RAM);
  for (const Item &it : items) add(all, it);
  int8_t r = sendValSet(all, 1000);
  if (r == 1) {
    if (verbose) Con.println("  konfigurace X20 (RAM): OK");
    return true;
  }
  // Při NAK nebo bez odpovědi poslat klíče jednotlivě a zjistit, který vadí
  if (verbose) Con.printf("  konfigurace X20 (RAM): %s, zkouším klíče jednotlivě\n", r == 0 ? "NAK" : "bez odpovědi");
  bool allOk = true;
  for (const Item &it : items) {
    ubx::ValSet one(ubx::LAYER_RAM);
    add(one, it);
    int8_t ri = sendValSet(one, 500);
    if (ri != 1) allOk = false;
    if (verbose) Con.printf("    %-24s %s\n", it.name, ri == 1 ? "OK" : ri == 0 ? "NAK" : "bez odpovědi");
  }
  return allOk;
}

bool setMeasMode(bool staticModel, bool raw) {
  mStatic = staticModel;
  mRaw = raw;
  ubx::ValSet v(ubx::LAYER_RAM);
  v.addU1(ubx::KEY_NAVSPG_DYNMODEL, effDyn());
  v.addU1(ubx::KEY_MSGOUT_RXM_RAWX_UART1, rawxRate());
  v.addU1(ubx::KEY_MSGOUT_RXM_SFRBX_UART1, mRaw ? 1 : 0);
  return sendValSet(v, 1000) == 1;
}

bool setDynAuto(uint8_t dyn) {
  dynModel = dyn;
  if (mStatic) return true;
  ubx::ValSet v(ubx::LAYER_RAM);
  v.addU1(ubx::KEY_NAVSPG_DYNMODEL, dyn);
  return sendValSet(v, 300) == 1;
}

uint8_t dynEffective() { return effDyn(); }

bool measStatic() { return mStatic; }
bool measRaw() { return mRaw; }

static uint32_t initCalls = 0;

InitResult init(uint32_t searchMs) {
  initCalls++;
  // Po zapnutí napájení X20 startuje déle než ESP – hledat opakovaně až searchMs
  uint32_t t0 = millis();
  bool found = detect();
  while (!found && millis() - t0 < searchMs) found = detect();
  if (!found) return InitResult::NotFound;
  Con.printf("  X20 nalezena na %lu Bd\n", (unsigned long)curBaud);

  if (curBaud != GNSS_BAUD_TARGET) {
    // Rychlost jen do RAM; po vypnutí X20 se vrátí hodnota z Flash a ESP ji při startu znovu přepne
    ubx::ValSet v(ubx::LAYER_RAM);
    v.addU4(ubx::KEY_UART1_BAUDRATE, GNSS_BAUD_TARGET);
    send(ubx::CLS_CFG, ubx::ID_CFG_VALSET, v.data(), v.size());
    delay(100);  // ACK přijde ještě na staré rychlosti nebo se ztratí, ověří se dotazem
    uartBegin(GNSS_BAUD_TARGET);
    delay(50);
    while (UART.available()) UART.read();
    if (!pollVersion(1000)) {
      Con.printf("  přepnutí na %lu Bd selhalo, hledám znovu\n", (unsigned long)GNSS_BAUD_TARGET);
      detect();
      return InitResult::BaudFailed;
    }
    Con.printf("  X20 přepnuta na %lu Bd\n", (unsigned long)GNSS_BAUD_TARGET);
  }

  return configure(true) ? InitResult::Ok : InitResult::ConfigPartial;
}

void poll() {
  int n = UART.available();
  while (n-- > 0) {
    uint8_t c = UART.read();
    bytesRx++;
    parser.feed(c);
  }
  if (pvtWaiting && millis() - pvtWaitMs > EOE_TIMEOUT_MS) firePvt();
}

uint32_t lastUbxAgeMs() { return lastUbxMs ? millis() - lastUbxMs : UINT32_MAX; }

// Restart jen příjmu GNSS (resetMode 0x02) – konfigurace v RAM i rychlost UART zůstanou
void resetReceiver(ResetKind k) {
  uint16_t mask = 0x0000;  // horký: zachovat vše
  if (k == ResetKind::Warm) mask = 0x0001;   // teplý: smazat efemeridy
  if (k == ResetKind::Cold) mask = 0xFFFF;   // studený: smazat vše
  uint8_t pl[4] = {(uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), 0x02, 0x00};
  send(ubx::CLS_CFG, ubx::ID_CFG_RST, pl, sizeof(pl));  // CFG-RST nemá ACK
  resetMs = millis();
  ttffMs = 0;
  resetKind = (uint8_t)k + 1;
}

uint32_t sinceResetMs() { return resetMs ? millis() - resetMs : 0; }
uint32_t ttff() { return ttffMs; }
const char *resetKindText() {
  static const char *const T[] = {"", "horký", "teplý", "studený"};
  return T[resetKind];
}

void writeRaw(const uint8_t *data, size_t len) { UART.write(data, len); }

void onPvt(void (*cb)()) { pvtCb = cb; }
void onSat(void (*cb)()) { satCb = cb; }

// Průchozí režim po krocích – smyčka mezitím obsluhuje web a tlačítko
static bool bridging = false;

void bridgeBegin() {
  Con.flush();
  delay(50);
  Con.updateBaudRate(curBaud);
  while (Con.available()) Con.read();
  bridging = true;
}

void bridgeStep() {
  if (!bridging) return;
  uint8_t buf[512];
  int n = Con.available();
  if (n > 0) {
    n = Con.readBytes(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
    UART.write(buf, n);
  }
  n = UART.available();
  if (n > 0) {
    n = UART.readBytes(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
    Con.write(buf, n);
  }
}

void bridgeEnd() {
  if (!bridging) return;
  bridging = false;
  Con.flush();
  Con.updateBaudRate(115200);
  delay(50);
  while (UART.available()) UART.read();
}

bool bridgeActive() { return bridging; }

void bridge(bool (*exitRequested)()) {
  bridgeBegin();
  while (!exitRequested()) {
    bridgeStep();
    yield();
  }
  bridgeEnd();
}

uint32_t baud() { return curBaud; }
const Pvt &pvt() { return pvtData; }
const SatSummary &sats() { return satData; }
const Dop &dop() { return dopData; }
const Cov &cov() { return covData; }
const SigSummary &sig() { return sigData; }
const Info &info() { return infoData; }

Stats stats() {
  Stats s;
  s.bytes = bytesRx;
  s.ubxOk = parser.okCount();
  s.ubxErr = parser.errCount();
  s.pvtCount = pvtCnt;
  s.satCount = satCnt;
  noInterrupts();
  s.ppsCount = ppsCnt;
  s.ppsLastUs = ppsLast;
  s.ppsPeriodUs = ppsPeriod;
  interrupts();
  s.pvtLatencyUs = pvtLatency;
  s.mgaAccepted = mgaAcc;
  s.mgaRejected = mgaRej;
  s.mgaDbd = mgaDbdCnt;
  s.ackAiding = ackAidingRes;
  s.corrCfg = corrCfgRes;
  s.sigCount = sigCnt;
  s.reinits = initCalls > 1 ? initCalls - 1 : 0;
  s.eoeCount = eoeCnt;
  s.hpCount = hpCnt;
  return s;
}

float corrAgeSeconds(uint8_t code) {
  static const float maxAge[] = {0, 1, 2, 5, 10, 15, 20, 30, 45, 60, 90, 120, 999};
  return code < 13 ? maxAge[code] : 999;
}

const char *initResultText(InitResult r) {
  switch (r) {
    case InitResult::Ok: return "OK";
    case InitResult::NotFound: return "X20 neodpovídá";
    case InitResult::BaudFailed: return "přepnutí rychlosti selhalo";
    case InitResult::ConfigPartial: return "konfigurace jen částečně";
  }
  return "?";
}

const char *fixText(const Pvt &p) {
  if (!p.valid) return "-";
  if (p.carrSoln == 2) return rtcmOn ? "RTK fix" : "PPP fix";
  if (p.carrSoln == 1) return rtcmOn ? "RTK float" : "HAS";
  if (!p.gnssFixOk) return "žádný";
  switch (p.fixType) {
    case 1: return "DR";
    case 2: return p.diffSoln ? "2D+kor" : "2D";
    case 3: return p.diffSoln ? "3D+kor" : "3D";
    case 4: return "GNSS+DR";
    case 5: return "čas";
    default: return "žádný";
  }
}

}  // namespace gnss
