// Generování NMEA 0183 (4.10) z dat X20
// Zatím přímo z NAV-PVT / NAV-SAT; v kroku 6 se poloha převezme z EKF.
#pragma once
#include <Arduino.h>

namespace nmea {

// Výstup jedné věty (včetně $…*CS\r\n)
using Sink = void (*)(const char *sentence, size_t len);

void setSink(Sink s);

// Výběr vět (z profilu)
static constexpr uint8_t GGA = 0x01, RMC = 0x02, GST = 0x04, GSA = 0x08, GSV = 0x10;
void setMask(uint8_t mask);

void sendEpoch();  // GGA, RMC, GST – volat po každém NAV-PVT
void sendSats();   // GSA, GSV – volat po NAV-SAT (1 Hz)

// Kvalita v GGA podle typu řešení (0 žádná, 1 GNSS, 2 DGNSS/HAS, 4 RTK fix, 5 RTK float, 6 DR)
int ggaQuality();

}  // namespace nmea
