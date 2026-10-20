// Protokol rover <-> aplikace RoverGnss přes BT SPP (verze 1)
// Popis: docs/rover-app-specifikace.md, kap. 5
//
// rover -> app: $PRVR,POS (každá epocha), $PRVR,STA (1 Hz), $PRVR,SAT (1 Hz, po 12 družicích),
//               {json} odpovědi na příkazy; vedle toho NMEA beze změny
// app -> rover: RTCM3 (0xD3…) do X20, {json} příkaz na jednom řádku (ukončený \n)
#pragma once
#include <Arduino.h>

#include "imu.h"

namespace proto {

static constexpr int VERSION = 1;

// Kvalita polohy v $PRVR,POS
enum Quality : uint8_t {
  Q_NONE = 0,   // bez fixu
  Q_GNSS = 1,   // 2D/3D bez korekcí
  Q_DGNSS = 2,  // SBAS / DGNSS
  Q_HAS = 3,    // HAS (PPP) – carrSoln > 0 bez RTCM; souřadnice ITRF2020
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

// Přepnutí profilu provede main (applyProfile)
void setProfileHook(void (*fn)(int index));

// Měření bodu z aplikace: {"cmd":"meas","op":"start","static":true,"raw":false} / {"cmd":"meas","op":"stop"}
// Statický model X20 a surová data RAWX/SFRBX do BT po dobu měření; odpojení BT měření ukončí.

// Záznam v aplikaci (zásobník epoch při výpadku BT): {"cmd":"rec","op":"start|stop"}
// Zásobník: {"cmd":"buf","op":"get"} (pošle $PRVR,BUF… a $PRVR,BUFEND), {"cmd":"buf","op":"clear"}

// Režim aplikace: zapne ho první příkaz z RoverGnss, vypne odpojení BT.
// V režimu aplikace jde z NMEA jen GGA a RMC a $PRVR,SAT každé 2 s (méně dat přes BT).
bool appMode();

}  // namespace proto
