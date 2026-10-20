// X20 (ZED-X20P) – komunikace, konfigurace po startu, data, PPS, průchozí režim
#pragma once
#include <Arduino.h>

namespace gnss {

struct Pvt {
  bool valid = false;        // přišla aspoň jedna zpráva
  uint32_t rxMs = 0;         // millis() příjmu
  int64_t rxUs = 0;          // esp_timer příjmu
  uint32_t iTOW = 0;         // ms týdne GPS
  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, min = 0, sec = 0;
  bool timeValid = false;
  uint8_t fixType = 0;       // 0 žádný, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 čas
  bool gnssFixOk = false;
  uint8_t carrSoln = 0;      // 0 žádné, 1 RTK float, 2 RTK fix
  bool diffSoln = false;     // použité korekce
  uint8_t numSV = 0;
  double lat = 0, lon = 0;   // [°]
  double hMSL = 0;           // [m] nad geoidem
  double hEll = 0;           // [m] nad elipsoidem
  float hAcc = 0, vAcc = 0;  // [m]
  float gSpeed = 0;          // [m/s]
  float headMot = 0;         // [°]
  float sAcc = 0;            // [m/s]
  float pDOP = 0;
  uint8_t corrAge = 0;       // stáří korekcí – kód u-blox: 0 nedostupné, 1 <1 s, 2 1–2 s, 3 2–5 s, 4 5–10 s, … 12 ≥120 s
  bool hp = false;           // lat/lon/výšky zpřesněné z NAV-HPPOSLLH (0,1 mm) pro tuto epochu
};

// Korekce RTCM z mobilu přicházejí (nastavuje main podle btlink). Bez nich je carrSoln > 0 řešení HAS (PPP),
// ne RTK ze sítě – jiný souřadnicový systém (ITRF) a jiné označení.
void setRtcmActive(bool on);
bool rtcmActive();

// Horní mez stáří korekcí [s] podle kódu z NAV-PVT (0 = nedostupné)
float corrAgeSeconds(uint8_t code);

struct Dop {
  bool valid = false;
  uint32_t rxMs = 0;
  float hDOP = 99.99f, vDOP = 99.99f, pDOP = 99.99f;
};

struct Cov {
  bool valid = false;        // X20 označila kovarianci polohy za platnou
  uint32_t rxMs = 0;
  float sN = 0, sE = 0, sD = 0;  // směrodatné odchylky [m]
};

struct SatInfo {
  uint8_t gnssId;            // 0 GPS, 1 SBAS, 2 GAL, 3 BDS, 5 QZSS, 6 GLO, 7 NavIC
  uint8_t svId;
  uint8_t cno;               // [dBHz]
  int8_t elev;               // [°]
  int16_t azim;              // [°]
  bool used;
};

static constexpr int MAX_SATS = 96;

struct SatSummary {
  bool valid = false;
  uint32_t rxMs = 0;
  uint8_t tracked = 0;       // CN0 > 0
  uint8_t used = 0;          // použité v řešení
  uint8_t usedBy[8] = {0};   // podle gnssId: 0 GPS, 1 SBAS, 2 GAL, 3 BDS, 5 QZSS, 6 GLO, 7 NavIC
  uint8_t cnoMax = 0;
  float cnoMeanUsed = 0;
  uint8_t count = 0;         // počet položek v list
  SatInfo list[MAX_SATS];
};

struct Info {
  bool valid = false;
  char sw[31] = {0};
  char hw[11] = {0};
  char fwver[31] = {0};
  char mod[31] = {0};
};

struct Stats {
  uint32_t bytes = 0;
  uint32_t ubxOk = 0;
  uint32_t ubxErr = 0;
  uint32_t pvtCount = 0;
  uint32_t satCount = 0;
  uint32_t ppsCount = 0;
  int64_t ppsLastUs = 0;
  uint32_t ppsPeriodUs = 0;
  int32_t pvtLatencyUs = 0;  // zpoždění NAV-PVT za PPS (epocha na celé sekundě)
  uint32_t reinits = 0;      // opakované hledání a nastavení X20 (po startu)
  uint32_t eoeCount = 0;     // UBX-NAV-EOE (konec epochy)
  uint32_t hpCount = 0;      // epochy odeslané s polohou z NAV-HPPOSLLH
  uint32_t mgaAccepted = 0;  // UBX-MGA-ACK: asistenční zpráva přijata
  uint32_t mgaRejected = 0;  // UBX-MGA-ACK: asistenční zpráva odmítnuta
  uint32_t mgaDbd = 0;       // UBX-MGA-DBD vyslané X20 (výpis databáze)
  int8_t ackAiding = -1;     // CFG-NAVSPG-ACKAIDING: 1 OK, 0 NAK, -1 bez odpovědi
};

// Výsledek konfigurace po startu
enum class InitResult { Ok, NotFound, BaudFailed, ConfigPartial };

void begin();                    // UART, PPS přerušení
// detekce rychlosti (opakovaně až searchMs), přepnutí na 460800, konfigurace RAM
InitResult init(uint32_t searchMs = 0);
bool detect();                   // najde aktuální rychlost X20, nastaví UART
// Parametry z profilu: perioda měření [ms], dyn. model (kód u-blox), elevační maska [°].
// Projeví se při dalším configure().
void setParams(uint16_t measRateMs, uint8_t dynModel, int8_t minElev);
bool configure(bool verbose);    // pošle konfiguraci do RAM X20
// Měření bodu z aplikace: statický model X20 (místo modelu z profilu) a surová data RXM-RAWX (1 Hz)
// a RXM-SFRBX. Pošle hned do RAM X20, platí i při dalším configure(). Vrací true = X20 potvrdila.
bool setMeasMode(bool staticModel, bool raw);
bool measStatic();
bool measRaw();
// Rámce RXM-RAWX / RXM-SFRBX (kompletní UBX) – předávají se do BT
void onRawFrame(void (*cb)(const uint8_t *frame, uint16_t len));
bool pollVersion(uint32_t timeoutMs);
void poll();                     // zpracuje přijatá data (volat z loop)
void bridge(bool (*exitRequested)());  // průchozí režim USB <-> X20 (blokující, konzole)
// Průchozí režim po krocích (konfigurační režim – web běží dál)
void bridgeBegin();
void bridgeStep();
void bridgeEnd();
bool bridgeActive();

void writeRaw(const uint8_t *data, size_t len);  // např. RTCM

uint32_t lastUbxAgeMs();         // doba od poslední platné zprávy UBX (UINT32_MAX = žádná)

// Restart příjmu (horký / teplý / studený start), měření doby do fixu
enum class ResetKind { Hot, Warm, Cold };
void resetReceiver(ResetKind k);
uint32_t sinceResetMs();         // 0 = restart nebyl
uint32_t ttff();                 // doba do fixu po restartu [ms], 0 = zatím bez fixu
const char *resetKindText();

// Volání po skončení epochy (NAV-EOE; když X20 EOE neposílá, hned po NAV-PVT) a po NAV-SAT (1 Hz)
void onPvt(void (*cb)());
void onSat(void (*cb)());

uint32_t baud();
const Pvt &pvt();
const SatSummary &sats();
const Dop &dop();
const Cov &cov();
const Info &info();
Stats stats();
const char *initResultText(InitResult r);
const char *fixText(const Pvt &p);

}  // namespace gnss
