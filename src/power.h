// Měření baterie (LiPo 1S přes dělič na GPIO35)
#pragma once
#include <Arduino.h>

namespace power {

void begin();
void update();                 // volat z loop – vzorek 1× za sekundu, vyhlazení
float batteryVolts();          // vyhlazené napětí (exponenciální průměr ~60 s) – stav baterie
float instantVolts();          // okamžité napětí (16 vzorků) – diagnostika
uint32_t pinMillivolts();      // napětí na pinu děliče [mV], průměr poslední sekundy (diagnostika)
void lastSpread(uint32_t &minMv, uint32_t &maxMv);  // rozptyl vzorků poslední sekundy na pinu [mV]
int batteryPercent(float v);   // orientační odhad podle napětí
int batteryPercentStable();    // procenta z vyhlazeného napětí s hysterezí (bez nabíjení jen klesají)
bool valid();                  // vyhlazené napětí je věrohodné pro článek LiPo (2,5–4,4 V)
// Kalibrace: změřené napětí baterie multimetrem ve V (3.86) nebo mV (3860) → koeficient do NVS.
// Bere průměr posledních 5 s, koeficient musí vyjít 1,6–2,4 (dělič 1:1).
bool calibrate(float measured);
float dividerFactor();

}  // namespace power
