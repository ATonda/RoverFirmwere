// Protokol rover <-> aplikace RoverGnss přes BT SPP (verze 1)
// Popis: docs/rover-app-specifikace.md, kap. 5
//
// rover -> app: $PRVR,POS (každá epocha), $PRVR,STA (1 Hz), $PRVR,SAT (1 Hz, po 12 družicích),
//               {json} odpovědi na příkazy; vedle toho NMEA beze změny
// app -> rover: RTCM3 (0xD3…) do X20, {json} příkaz na jednom řádku (ukončený \n)
// Stejné příkazy JSON přijímá i konzole USB (řádek začínající '{'), odpověď jde zpět do konzole.
#pragma once
#include <Arduino.h>

#include "imu.h"

namespace proto {

static constexpr int VERSION = 1;

// Kvalita polohy v $PRVR,POS
enum Quality : uint8_t {
  Q_NONE = 0,   // bez fixu
  Q_GNSS = 1,   // 2D/3D bez korekcí
  Q_DGNSS = 2,  // SBAS / DGNSS (podle NAV-SIG zdroj korekcí SBAS nebo neznámý)
  Q_HAS = 3,    // HAS (PPP) – bez RTCM: carrSoln > 0, nebo korekce z jiného zdroje než SBAS; souřadnice ITRF2020
  Q_FIX = 4,    // RTK fix (RTCM z mobilu)
  Q_FLOAT = 5,  // RTK float
  Q_DR = 6      // jen dopočet
};
uint8_t quality();

// Jedna epocha $PRVR,POS binárně (zásobník při výpadku BT, epochbuf)
struct PosRec {
  double lat, lon;            // [°]
  float hEll, hMSL;           // [m]
  float sN, sE, sD;           // [m] (platné s F_COV)
  float hAcc, vAcc;           // [m]
  float gSpeed, headMot;      // [m/s], [°]
  float corrAgeS;             // [s] (platné s F_CORR)
  uint16_t year, ms;
  uint16_t pDOP100;
  uint8_t month, day, hour, min, sec;
  uint8_t q, carrSoln, numSV;
  uint8_t flags;
};
static constexpr uint8_t F_TIME = 1, F_COV = 2, F_RTCM = 4, F_MOVING = 8, F_HP = 16, F_CORR = 32;

bool capturePos(PosRec &r);                  // false = žádná data z X20
void sendRec(const PosRec &r, bool buffered); // $PRVR,POS nebo $PRVR,BUF (ze zásobníku)
void sendBufEnd(uint32_t n);                  // $PRVR,BUFEND,počet

void sendPos();   // po každém NAV-PVT (před btlink::flush); při záznamu i do zásobníku
void sendSta();   // 1 Hz, sám odešle
void sendSat();   // po NAV-SAT, sám odesílá po větách

void onImu(const imu::Sample &s);  // detekce pohybu pro POS
bool moving();

// Příkaz JSON z mobilu (jeden řádek bez \n); odpověď jde hned do BT
void handleCmd(const char *line, size_t len);
// Totéž z konzole USB – odpověď do Serial
void handleConsoleCmd(const char *line, size_t len);

// Akce, které provede main
struct Hooks {
  void (*applyProfile)(int index) = nullptr;  // přepnout profil a poslat do X20
  void (*bridge)(bool on) = nullptr;          // průchozí režim USB <-> X20
  bool (*bridgeActive)() = nullptr;
  void (*restart)() = nullptr;                // restart ESP (po odeslání odpovědi)
  void (*idle)() = nullptr;                   // obsluha X20 během blokující akce (kalibrace gyra)
};
void setHooks(const Hooks &h);

// Nastavení z aplikace (FW 0.13.0, náhrada webového menu):
//   {"cmd":"set","what":"profiles","data":{"active":0,"default":0,"profiles":[…]}}  – stejný formát jako get profiles
//   {"cmd":"get","what":"device"} / {"cmd":"set","what":"device","data":{"btName":"…"}}  – název BT platí po restartu
//   {"cmd":"get","what":"cal"}  – bias gyra, koeficient děliče baterie
//   {"cmd":"cal","what":"battery","v":3.86}  – změřené napětí baterie [V]
//   {"cmd":"cal","what":"gyro","ms":30000}  – přístroj v klidu, na pozadí 30 s (FW 0.15.0); odpověď
//       {"ok":true,"pending":true,"ms":…}, výsledek událostí {"ev":"cal_gyro","ok":…,"gyro":[…]}
//   {"cmd":"passthrough","op":"start|stop"}  – průchozí režim USB <-> X20 (konec i krátkým stiskem tlačítka)
//   {"cmd":"restart"}

// Měření bodu z aplikace: {"cmd":"meas","op":"start","static":true,"raw":false} / {"cmd":"meas","op":"stop"}
// Statický model X20 a surová data RAWX/SFRBX do BT po dobu měření; odpojení BT měření ukončí.

// Záznam v aplikaci (zásobník epoch při výpadku BT): {"cmd":"rec","op":"start|stop"}
// Zásobník: {"cmd":"buf","op":"get"} (pošle $PRVR,BUF… a $PRVR,BUFEND), {"cmd":"buf","op":"clear"}

// Režim aplikace: zapne ho první příkaz z RoverGnss, vypne odpojení BT.
// V režimu aplikace jde z NMEA jen GGA a RMC a $PRVR,SAT každé 2 s (méně dat přes BT).
bool appMode();

// Záznam v aplikaci a připojený BT: rover posílá navíc celé rámce NAV-PVT, NAV-COV a NAV-HPPOSLLH (EKF offline)
// (v režimu měření rychlosti ne – šetří přenos BT)
bool navRecording();

// Měření rychlosti z aplikace (FW 0.16.0) – vnitřní profil „Rychlost“, není v seznamu profilů ani v cyklu tlačítka:
//   {"cmd":"speed","op":"start"}  – X20 10 Hz, model automotive (maska z aktivního profilu), surové IMU 120 Hz do BT
//                                   (rámce 0xF5, decimace 1) i bez záznamu, $PRVR,SAT každých 5 s, bez rámců NAV
//   {"cmd":"speed","op":"alive"}  – aplikace žije (posílat každých 5 s)
//   {"cmd":"speed","op":"stop"}   – návrat na aktivní profil
// Bez „alive“ 30 s (i po odpojení BT) se rover sám vrátí na aktivní profil. Přepnutí profilu (tlačítko, příkaz
// profile, set profiles) režim ukončí. Odpověď {"ok":true,"speed":true|false,"hz":…}.
bool speedMode();
void speedPoll();      // z loop – hlídání „alive“
void speedCancel();    // volá main při přepnutí profilu (profil se aplikuje sám)
uint8_t navRateHz();   // skutečná frekvence navigace (profil, nebo 10 Hz v režimu rychlosti)

}  // namespace proto
