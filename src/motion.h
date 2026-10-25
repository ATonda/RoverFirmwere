// Rozpoznání stavu přístroje z IMU a rychlosti GNSS a automatický dynamický model X20
//
// Stavy: Měřím (klid), Manipulace (prudké pohyby bez kroků), Chodím (kroky), Jedu (auto).
// Vyhodnocení každých 0,5 s z okna 2 s (240 vzorků IMU 120 Hz). Při profilu s modelem „automaticky“
// se podle stavu přepíná CFG-NAVSPG-DYNMODEL (jen RAM): Měřím = stationary, Manipulace a Chodím = pedestrian,
// Jedu = automotive. Popis a meze: docs/rover-firmware-specifikace.md (FW 0.14.0, klid FW 0.15.0, Jedu FW 0.16.0).
#pragma once
#include <Arduino.h>

#include "imu.h"

namespace motion {

enum State : uint8_t { MEASURE = 0, HANDLE = 1, WALK = 2, DRIVE = 3 };

struct Features {
  float accStd = 0;     // směrodatná odchylka |a| v okně [g]
  float accDev = 0;     // největší odchylka |a| od 1 g v okně [g]
  float gyroMax = 0;    // největší |ω| v okně [°/s]
  float stepScore = 0;  // autokorelace |a| v rozsahu kroků 1,25–2,5 Hz (0–1)
  float stepHz = 0;     // frekvence kroků [Hz] (0 = bez kroků)
  float speed = 0;      // rychlost GNSS [m/s] (-1 = bez fixu)
  bool still = false;   // klid v tomto okně (2 s)
  float unrest = 0;     // doba nad mezí klidu za posledních 1,5 s [s] (odchod z Měřím nad 0,5 s)
  bool steps = false;   // kroky v tomto okně
  bool sharp = false;   // prudký pohyb bez kroků
};

void onImu(const imu::Sample &s);  // každý vzorek IMU
void onPvt();                      // po každé epoše (rychlost)
void poll();                       // z loop: vyhodnocení po 0,5 s, přepnutí modelu

void setAuto(bool on);             // profil s modelem „automaticky“
bool autoOn();
State state();
const char *stateName(State s);
uint8_t model();                   // model, který stav požaduje (2 stationary, 3 pedestrian, 4 automotive)
const Features &features();
uint32_t changes();                // počet změn stavu od startu
uint32_t steps();                  // odhad počtu kroků od startu (kadence × doba s kroky)

}  // namespace motion
