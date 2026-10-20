// IMU LSM6DSV16X – gyroskop + akcelerometr, čtení přes FIFO s časovými značkami čipu
#pragma once
#include <Arduino.h>

namespace imu {

struct Sample {
  int64_t tUs = 0;               // čas vzorku v esp_timer [µs] (z časové značky čipu)
  float ax = 0, ay = 0, az = 0;  // [g]
  float gx = 0, gy = 0, gz = 0;  // [°/s], po odečtení biasu
  float tempC = 0;               // poslední změřená teplota
};

struct Stats {
  uint32_t samples = 0;    // vzorky (dvojice gyro + akcelerometr)
  uint32_t overflows = 0;  // přetečení FIFO (ztráta dat)
  uint32_t queueDrops = 0; // vzorky zahozené z fronty, kterou nikdo nečetl
  uint32_t readErr = 0;    // chyby I2C
  uint16_t fifoMax = 0;    // nejvyšší zaplnění FIFO [slov]
  float odrHz = 0;         // skutečná frekvence vzorků podle časových značek
  uint32_t reinits = 0;    // nové nastavení IMU, protože nepřicházely vzorky
  uint32_t configErr = 0;  // registry, které se napoprvé nezapsaly (ověřeno zpětným čtením)
  uint32_t lastSampleAgeMs = 0;  // doba od posledního vzorku
  bool busRecovered = false;     // při startu byla sběrnice I2C zablokovaná (SDA v nule)
};

bool begin();                 // detekce, WHO_AM_I, reset a ověřené nastavení FIFO 120 Hz; true = OK
void flush();                 // vyprázdní FIFO (po dlouhé inicializaci ostatních částí)
bool present();
uint8_t address();
void poll();                  // vyčte FIFO (volat často z loop)
bool latest(Sample &s);       // poslední vzorek
bool pop(Sample &s);          // další vzorek z fronty (pro EKF), false = prázdná
Stats stats();

// Kalibrace biasu gyra – přístroj musí ležet v klidu. Uloží do NVS.
// idle (volitelně) se volá během čekání – obsluha X20 a BT, aby nepřetekly buffery.
bool calibrateGyro(uint32_t durationMs, void (*idle)() = nullptr);
void gyroBias(float &bx, float &by, float &bz);

}  // namespace imu
