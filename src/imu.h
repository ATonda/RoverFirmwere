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

// Kalibrace gyra na pozadí (příkaz z aplikace): průměr surového gyra za durationMs, smyčka běží dál.
// Výsledek se zjistí přes calResult() – vrací true jednou po dokončení (ok = bias změněn a uložen).
void startCalibration(uint32_t durationMs);
bool calRunning();
bool calResult(bool &ok);

// Průběžná kalibrace v klidu (FW 0.15.0): okno 1 s je klid, když σ|a| < 5 mg a max |ω| < 1,5 °/s.
// Po 30 s nepřerušeného klidu se bias nastaví na průměr surového gyra za těchto 30 s (pak každých dalších 30 s).
// Do NVS se uloží nejvýš 1× za 10 min a jen při změně > 0,002 °/s.
struct BiasInfo {
  float bx = 0, by = 0, bz = 0;  // aktuální bias [°/s]
  float tempC = 0;               // teplota IMU při poslední změně biasu
  uint32_t updates = 0;          // počet průběžných aktualizací od startu
  uint32_t stillSec = 0;         // délka nepřerušeného klidu [s]
  bool still = false;            // poslední okno 1 s je klid
};
BiasInfo biasInfo();
float temperature();           // poslední teplota IMU [°C]

}  // namespace imu
