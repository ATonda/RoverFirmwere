// Rover GNSS – firmware ESP32
// Specifikace: docs/rover-firmware-specifikace.md
//
// Sériová konzole 115200 Bd, příkaz + Enter, "h" = nápověda.

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>

#include "epochbuf.h"
#include "imustream.h"
#include "btlink.h"
#include "buzzer.h"
#include "button.h"
#include <Wire.h>
#include "config.h"
#include "gnss.h"
#include "ubx.h"
#include "imu.h"
#include "nmea.h"
#include "power.h"
#include "proto.h"
#include "settings.h"
#include "survey.h"
#include "testctl.h"
#include "webui.h"

// Záchranný režim: počet pádů po sobě přežije restart (ne vypnutí napájení)
static constexpr uint32_t CRASH_MAGIC = 0x52565231;
RTC_NOINIT_ATTR static uint32_t crashMagic;
RTC_NOINIT_ATTR static uint32_t crashCount;
// Požadavek na start v testovacím režimu (nastaví web nebo konzole, platí jen pro jeden start)
static constexpr uint32_t TEST_MAGIC = 0x54455354;
RTC_NOINIT_ATTR static uint32_t testBootMagic;

enum class Mode { Run, Config, Test, Rescue };
static Mode mode = Mode::Run;

static bool liveOn = false;
static bool nmeaEcho = false;            // NMEA i do konzole
static volatile bool pvtPending = false; // nová epocha NAV-PVT
static volatile bool satPending = false; // nový NAV-SAT

static void nmeaSink(const char *s, size_t n) {
  btlink::send(s, n);
  if (nmeaEcho) Serial.write((const uint8_t *)s, n);
}

// ---------------------------------------------------------------- výpisy
static void printHelp() {
  Serial.println();
  Serial.printf("=== %s %s – konzole ===\n", FW_NAME, FW_VERSION);
  Serial.println("h       nápověda");
  Serial.println("s       živý výpis 1 Hz zap/vyp");
  Serial.println("i       informace o přístroji a X20");
  Serial.println("c       znovu nakonfigurovat X20 (detekce, 460800, RAM)");
  Serial.println("m       10 vzorků IMU");
  Serial.println("g       kalibrace gyra (přístroj v klidu, 3 s)");
  Serial.println("k3860   kalibrace baterie – změřené napětí v mV");
  Serial.println("n       výpis NMEA do konzole zap/vyp");
  Serial.println("b       průchozí režim USB <-> X20 (konec tlačítkem)");
  Serial.println("p       seznam profilů, p2 = přepnout na profil 2");
  Serial.println("w       Wi-Fi AP zap (webové menu, BT běží dál); vypnutí podržením tlačítka 2 s nebo z webu");
  Serial.println("wt      testovací režim po restartu (Wi-Fi AP + BT současně, záložka Test)");
  Serial.println("xh xw xc  restart příjmu X20: horký / teplý / studený start");
  Serial.println("t1      test tlačítek 20 s: Wi-Fi/profil (GPIO13) a BOOT (GPIO0) – stav a události, konec klávesou");
  Serial.println("t2      test pieza (GPIO25): tóny 500–4000 Hz, pak ukázky signálů (fix, varování, profil)");
  Serial.println("t3      test LED WS2812 (GPIO26): červená, zelená, modrá, bílá po 1 s");
  Serial.println("t4      souhrn: sken I2C, X20, PPS, baterie (pin i přepočet), tlačítka, paměť, důvod startu");
  Serial.println("r       restart");
  Serial.println();
}

static void printInfo() {
  const esp_partition_t *run = esp_ota_get_running_partition();
  Serial.printf("Firmware: %s %s, oddíl %s\n", FW_NAME, FW_VERSION, run ? run->label : "?");
  const gnss::Info &gi = gnss::info();
  if (gi.valid) {
    Serial.printf("X20: %s, FW %s (%s), HW %s, UART1 %lu Bd\n", gi.mod, gi.fwver, gi.sw, gi.hw,
                  (unsigned long)gnss::baud());
  } else {
    Serial.println("X20: neodpovídá");
  }
  if (imu::present()) {
    float bx, by, bz;
    imu::gyroBias(bx, by, bz);
    imu::Stats is = imu::stats();
    Serial.printf("IMU: LSM6DSV16X na 0x%02X, bias gyra %.3f %.3f %.3f °/s\n", imu::address(), bx, by, bz);
    Serial.printf("     vzorků %lu, ODR %.1f Hz, poslední před %lu ms, přetečení %lu, nové nastavení %lu×, chyb zápisu %lu\n",
                  (unsigned long)is.samples, is.odrHz, (unsigned long)is.lastSampleAgeMs, (unsigned long)is.overflows,
                  (unsigned long)is.reinits, (unsigned long)is.configErr);
    if (is.busRecovered) Serial.println("     při startu byla sběrnice I2C zablokovaná – uvolněna");
  } else {
    Serial.println("IMU: nenalezeno");
  }
  float v = power::batteryVolts();
  Serial.printf("Baterie: %.3f V (%d %%)%s, koeficient děliče %.4f\n", v, power::batteryPercentStable(),
                power::valid() ? "" : " – CHYBA MĚŘENÍ", power::dividerFactor());
  Serial.printf("Bluetooth SPP: %s\n", btlink::name());
  const settings::Profile &pr = settings::profile(settings::active());
  Serial.printf("Profil: %d/%d %s (%u Hz, %s, maska %d°)\n", settings::active() + 1, settings::count(), pr.name,
                pr.rateHz, settings::dynName(pr.dyn), pr.elevMask);
}

static void printStatus() {
  static uint32_t lastMs = 0, lastBytes = 0, lastPvt = 0, lastImu = 0, lastBtTx = 0, lastBtRx = 0;
  uint32_t now = millis();
  gnss::Stats gs = gnss::stats();
  imu::Stats is = imu::stats();
  btlink::Stats bs = btlink::stats();
  if (lastMs == 0) {  // první volání jen nastaví výchozí stav, rychlosti by počítaly od startu
    lastMs = now;
    lastBytes = gs.bytes;
    lastPvt = gs.pvtCount;
    lastImu = is.samples;
    lastBtTx = bs.txBytes;
    lastBtRx = bs.rxBytes;
    return;
  }
  float dt = (now - lastMs) / 1000.0f;
  float bps = (gs.bytes - lastBytes) / dt;
  float pvtHz = (gs.pvtCount - lastPvt) / dt;
  float imuHz = (is.samples - lastImu) / dt;
  lastMs = now;
  lastBytes = gs.bytes;
  lastPvt = gs.pvtCount;
  lastImu = is.samples;
  float btTx = (bs.txBytes - lastBtTx) / dt;
  float btRx = (bs.rxBytes - lastBtRx) / dt;
  lastBtTx = bs.txBytes;
  lastBtRx = bs.rxBytes;

  const gnss::Pvt &p = gnss::pvt();
  const gnss::SatSummary &s = gnss::sats();
  int64_t ppsAgeMs = gs.ppsCount ? (esp_timer_get_time() - gs.ppsLastUs) / 1000 : -1;

  Serial.printf("GNSS %lu Bd %5.0f B/s PVT %.1f Hz UBX %lu/%lu | fix %-8s SV %u/%u (sled. %u) CN0 %.0f/%u | hAcc %.2f vAcc %.2f m pDOP %.2f\n",
                (unsigned long)gnss::baud(), bps, pvtHz, (unsigned long)gs.ubxOk, (unsigned long)gs.ubxErr,
                gnss::fixText(p), p.numSV, s.used, s.tracked, s.cnoMeanUsed, s.cnoMax, p.hAcc, p.vAcc, p.pDOP);
  if (p.valid && p.timeValid) {
    Serial.printf("     %04u-%02u-%02u %02u:%02u:%02u UTC lat %.7f lon %.7f h %.2f m v %.2f m/s kurz %.1f°\n",
                  p.year, p.month, p.day, p.hour, p.min, p.sec, p.lat, p.lon, p.hMSL, p.gSpeed, p.headMot);
  }
  Serial.printf("     PPS %lu (perioda %.6f s, před %lld ms, PVT za PPS %.1f ms)",
                (unsigned long)gs.ppsCount, gs.ppsPeriodUs / 1e6, (long long)ppsAgeMs, gs.pvtLatencyUs / 1000.0f);

  imu::Sample a;
  if (imu::latest(a)) {
    float n = sqrtf(a.ax * a.ax + a.ay * a.ay + a.az * a.az);
    Serial.printf(" | IMU %.0f Hz (ODR %.1f) FIFO max %u přetečení %lu |a| %.3f g gyro %.2f %.2f %.2f °/s %.1f °C",
                  imuHz, is.odrHz, is.fifoMax, (unsigned long)is.overflows, n, a.gx, a.gy, a.gz, a.tempC);
  } else {
    Serial.print(" | IMU -");
  }
  float v = power::batteryVolts();
  Serial.printf(" | BAT %.2f V %d %%%s\n", v, power::batteryPercentStable(), power::valid() ? "" : " CHYBA");

  const gnss::Dop &dp = gnss::dop();
  const gnss::Cov &cv = gnss::cov();
  Serial.printf("     BT %s (%lu×, pár. %lu/%lu) NMEA %.0f B/s zahoz. %lu | z mobilu %.0f B/s RTCM %lu/%lu",
                bs.connected ? "PŘIPOJENO" : "nepřipojeno", (unsigned long)bs.connects, (unsigned long)bs.pairOk,
                (unsigned long)bs.pairFail, btTx,
                (unsigned long)bs.txDropped, btRx, (unsigned long)bs.rtcmOk, (unsigned long)bs.rtcmBad);
  if (bs.rtcmOk) Serial.printf(" posl. %u před %.1f s", bs.rtcmLastType, (millis() - bs.rtcmLastMs) / 1000.0f);
  Serial.printf(" | GGA kval. %d kor. %s | HDOP %.2f σN %.2f σE %.2f σD %.2f m%s\n", nmea::ggaQuality(),
                p.corrAge ? "ano" : "-", dp.hDOP, cv.sN, cv.sE, cv.sD, cv.valid ? "" : " (kov. neplatná)");
}

static void printImuSamples() {
  if (!imu::present()) {
    Serial.println("IMU nenalezeno.");
    return;
  }
  Serial.println("  T[°C]   ax[g]   ay[g]   az[g]  |a|[g]   gx[°/s]  gy[°/s]  gz[°/s]");
  uint32_t lastCount = imu::stats().samples;
  int shown = 0;
  uint32_t t0 = millis();
  while (shown < 10 && millis() - t0 < 3000) {
    gnss::poll();
    imu::poll();
    if (imu::stats().samples - lastCount >= 12) {  // ~10 Hz
      lastCount = imu::stats().samples;
      imu::Sample a;
      imu::latest(a);
      float n = sqrtf(a.ax * a.ax + a.ay * a.ay + a.az * a.az);
      Serial.printf("  %5.1f  %6.3f  %6.3f  %6.3f  %6.3f  %8.2f %8.2f %8.2f\n", a.tempC, a.ax, a.ay, a.az, n, a.gx,
                    a.gy, a.gz);
      shown++;
    }
    delay(1);
  }
}

// ---------------------------------------------------------------- X20
static void initGnss(uint32_t searchMs = 0) {
  Serial.println("X20: inicializace...");
  gnss::InitResult r = gnss::init(searchMs);
  Serial.printf("X20: %s\n", gnss::initResultText(r));
}

static bool bridgeExit() {
  // konec průchozího režimu stiskem tlačítka (odrušeno 50 ms)
  static uint32_t pressMs = 0;
  if (digitalRead(PIN_BUTTON) == LOW) {
    if (pressMs == 0) pressMs = millis();
    if (millis() - pressMs > 50) {
      pressMs = 0;
      return true;
    }
  } else {
    pressMs = 0;
  }
  return false;
}

static void bridgeMode() {
  Serial.printf("\nPrůchozí režim USB <-> X20 na %lu Bd.\n", (unsigned long)gnss::baud());
  Serial.printf("Zavři monitor, v u-center 2 otevři COM port na %lu Bd.\n", (unsigned long)gnss::baud());
  Serial.println("Ukončení: stisk tlačítka.");
  while (digitalRead(PIN_BUTTON) == LOW) delay(10);
  gnss::bridge(bridgeExit);
  while (digitalRead(PIN_BUTTON) == LOW) delay(10);
  button::resetEvents();
  Serial.println("\nPrůchozí režim ukončen. Konfigurace X20 se znovu posílá do RAM.");
  // u-center mohl rychlost nebo výstupy změnit – obnovit provozní stav
  initGnss();
}

// ---------------------------------------------------------------- profily
// Nastaví parametry z profilu; reconfigure = hned poslat do X20 (jinak až při init)
static void applyProfile(int i, bool reconfigure) {
  settings::setActive(i);
  const settings::Profile &p = settings::profile(settings::active());
  gnss::setParams(1000 / p.rateHz, settings::dynCode(p.dyn), p.elevMask);
  nmea::setMask(proto::appMode() ? (uint8_t)(p.nmea & (nmea::GGA | nmea::RMC)) : p.nmea);
  buzzer::setEnabled(p.buzzer);
  Serial.printf("[profil] %d/%d %s – %u Hz, %s, maska %d°\n", settings::active() + 1, settings::count(), p.name,
                p.rateHz, settings::dynName(p.dyn), p.elevMask);
  if (reconfigure) gnss::configure(true);
}

static void listProfiles() {
  for (int i = 0; i < settings::count(); i++) {
    const settings::Profile &p = settings::profile(i);
    Serial.printf("%c%c %d  %-24s %2u Hz  %-10s maska %2d°\n", i == settings::active() ? '>' : ' ',
                  i == settings::defaultIndex() ? '*' : ' ', i + 1, p.name, p.rateHz, settings::dynName(p.dyn),
                  p.elevMask);
  }
  Serial.println("> aktivní, * výchozí po startu");
}

// ---------------------------------------------------------------- konfigurační režim
static bool btStoppedForAp = false;  // BT vypnuto, protože se vedle něj Wi-Fi nevešla do RAM

static bool startWeb(bool rescue) {
  Serial.printf("[Wi-Fi] start AP, volná RAM %lu B (největší blok %lu B)\n", (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMaxAllocHeap());
  // Wi-Fi AP + web potřebuje víc RAM, než zbývá vedle BT (~56 kB volné). Rozhodnout PŘED startem Wi-Fi –
  // nepovedený start nechá ovladač Wi-Fi napůl inicializovaný a vypnutí BT pak zamrzne.
  if (btlink::active() && (ESP.getFreeHeap() < WIFI_MIN_HEAP || ESP.getMaxAllocHeap() < WIFI_MIN_BLOCK)) {
    Serial.println("[Wi-Fi] málo RAM vedle Bluetooth – BT se po dobu zapnuté Wi-Fi vypne");
    btlink::end();
    btStoppedForAp = true;
    delay(100);
    Serial.printf("[Wi-Fi] volná RAM bez BT %lu B (největší blok %lu B)\n", (unsigned long)ESP.getFreeHeap(),
                  (unsigned long)ESP.getMaxAllocHeap());
  }
  if (!webui::begin(rescue)) {
    Serial.println("[Wi-Fi] start AP selhal");
    return false;
  }
  Serial.printf("[Wi-Fi] AP %s, heslo %s, http://192.168.4.1/\n", webui::ssid(), webui::password());
  if (!rescue) Serial.println("[Wi-Fi] konec: tlačítko „Uložit a vypnout Wi-Fi“ nebo podržení tlačítka přístroje 2 s");
  Serial.printf("[Wi-Fi] Bluetooth %s, volná RAM %lu B\n", btlink::active() ? "běží" : "vypnuto",
                (unsigned long)ESP.getFreeHeap());
  return true;
}

static void enterConfig() {
  if (mode != Mode::Run) return;
  Serial.println("[režim] konfigurace – Wi-Fi AP");
  if (!startWeb(false)) {
    buzzer::beep(800, 200, 150, 3);
    return;
  }
  mode = Mode::Config;
  button::resetEvents();  // stisk, kterým se režim zapnul, nesmí režim hned zase ukončit
  buzzer::beep(2000, 100, 80, 2);
}

static void bridgeStart() {
  if (gnss::bridgeActive()) return;
  Serial.printf("[průchozí] USB <-> X20 na %lu Bd. Zavři monitor, v u-center otevři COM na této rychlosti.\n",
                (unsigned long)gnss::baud());
  Serial.println("[průchozí] konec: tlačítko na webu, podržení tlačítka přístroje nebo vypnutí.");
  liveOn = false;
  gnss::bridgeBegin();
  webui::setBridge(true);
}

static void bridgeStop();

static void restartNow(const char *why);

// Vypnout AP bez restartu – BT, výpočet i měření bodu běží dál
static void exitConfig() {
  if (mode != Mode::Config && mode != Mode::Test) return;
  if (webui::needsRestart() && !survey::running()) {
    restartNow("změněné nastavení Wi-Fi/BT");
    return;
  }
  bridgeStop();
  webui::end();
  mode = Mode::Run;
  if (btStoppedForAp) {
    btStoppedForAp = false;
    if (btlink::begin(settings::device().btName)) Serial.printf("Bluetooth SPP: %s (znovu zapnuto)\n", btlink::name());
    else Serial.println("Bluetooth: nový start selhal");
  }
  Serial.println("[režim] Wi-Fi AP vypnuto, provoz s Bluetooth");
  buzzer::stop();
  buzzer::tones2(2500, 1500);
}

static void bridgeStop() {
  if (!gnss::bridgeActive()) return;
  gnss::bridgeEnd();
  webui::setBridge(false);
  Serial.println("\n[průchozí] ukončen, konfigurace X20 se znovu posílá do RAM.");
  initGnss();
}

static void restartNow(const char *why) {
  Serial.printf("[režim] %s – restart\n", why);
  Serial.flush();
  delay(100);
  ESP.restart();
}

// ---------------------------------------------------------------- testy hardwaru
// Čekání s obsluhou X20, IMU a BT (aby test nezpůsobil přetečení). true = stisk klávesy.
static bool serviceWait(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (!gnss::bridgeActive()) gnss::poll();
    imu::poll();
    btlink::poll();
    imu::Sample smp;
    while (imu::pop(smp)) {
    }
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      return true;
    }
    delay(1);
  }
  return false;
}

static void flushConsoleInput() {
  delay(30);
  while (Serial.available()) Serial.read();
}

static void testButtons() {
  flushConsoleInput();
  pinMode(PIN_BOOT, INPUT_PULLUP);
  Serial.printf("Test tlačítek 20 s (konec libovolnou klávesou). Wi-Fi/profil = GPIO%d, BOOT = GPIO%d.\n", PIN_BUTTON,
                PIN_BOOT);
  button::resetEvents();
  button::poll();
  bool btn = button::pressed();
  bool boot = digitalRead(PIN_BOOT) == LOW, bootRaw = boot;
  uint32_t bootChangeMs = 0, btnDownMs = millis(), bootDownMs = millis();
  Serial.printf("  výchozí stav: Wi-Fi/profil %s, BOOT %s\n", btn ? "STISKNUTO" : "uvolněno", boot ? "STISKNUTO" : "uvolněno");
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    uint32_t now = millis();
    // tlačítko Wi-Fi/profil – odrušené v modulu button
    button::Event e = button::poll();
    bool b = button::pressed();
    if (b != btn) {
      btn = b;
      if (b) {
        btnDownMs = now;
        Serial.printf("  %5.1f s  Wi-Fi/profil: stisk\n", (now - t0) / 1000.0f);
      } else {
        Serial.printf("  %5.1f s  Wi-Fi/profil: uvolnění po %lu ms\n", (now - t0) / 1000.0f,
                      (unsigned long)(now - btnDownMs));
      }
    }
    if (e == button::Event::Short) Serial.println("           -> událost KRÁTKÝ STISK (v provozu: další profil)");
    if (e == button::Event::Long) Serial.println("           -> událost PODRŽENÍ 2 s (v provozu: Wi-Fi zap, v konfiguraci/testu: Wi-Fi vyp)");
    if (e == button::Event::Exit) Serial.println("           -> událost PODRŽENÍ 5 s (bez akce)");
    // BOOT – odrušení 30 ms
    bool r = digitalRead(PIN_BOOT) == LOW;
    if (r != bootRaw) {
      bootRaw = r;
      bootChangeMs = now;
    }
    if (bootRaw != boot && now - bootChangeMs >= BTN_DEBOUNCE_MS) {
      boot = bootRaw;
      if (boot) {
        bootDownMs = now;
        Serial.printf("  %5.1f s  BOOT: stisk\n", (now - t0) / 1000.0f);
      } else {
        Serial.printf("  %5.1f s  BOOT: uvolnění po %lu ms\n", (now - t0) / 1000.0f, (unsigned long)(now - bootDownMs));
      }
    }
    if (serviceWait(2)) break;
  }
  button::resetEvents();
  Serial.println("Test tlačítek ukončen.");
}

static bool beep(uint32_t freq, uint32_t onMs, uint32_t offMs) {
  buzzer::start(freq);
  bool abort = serviceWait(onMs);
  buzzer::stop();
  return abort || serviceWait(offMs);
}

static void testBuzzer() {
  flushConsoleInput();
  Serial.printf("Test pieza na GPIO%d (konec libovolnou klávesou).\n", PIN_BUZZER);
  static const uint16_t F[] = {500, 1000, 2000, 2700, 3000, 4000};
  for (uint16_t f : F) {
    Serial.printf("  %u Hz\n", f);
    if (beep(f, 400, 300)) return;
  }
  Serial.println("  signál FIX (dva stoupající tóny)");
  if (beep(1500, 100, 60) || beep(2500, 150, 700)) return;
  Serial.println("  signál VAROVÁNÍ (3× hluboký)");
  for (int i = 0; i < 3; i++)
    if (beep(800, 200, 150)) return;
  if (serviceWait(500)) return;
  for (int n = 1; n <= 3; n++) {
    Serial.printf("  signál PROFIL %d (%d× krátce)\n", n, n);
    for (int i = 0; i < n; i++)
      if (beep(2700, 80, 120)) return;
    if (serviceWait(600)) return;
  }
  Serial.println("Test pieza ukončen.");
}

static void testLed() {
  flushConsoleInput();
  Serial.printf("Test LED WS2812 na GPIO%d (jas 25 %%, konec libovolnou klávesou).\n", PIN_LED);
  struct C {
    const char *name;
    uint8_t r, g, b;
  };
  static const C colors[] = {{"červená", 64, 0, 0}, {"zelená", 0, 64, 0}, {"modrá", 0, 0, 64}, {"bílá", 64, 64, 64}};
  for (const C &c : colors) {
    Serial.printf("  %s\n", c.name);
    rgbLedWrite(PIN_LED, c.r, c.g, c.b);
    if (serviceWait(1000)) break;
  }
  rgbLedWrite(PIN_LED, 0, 0, 0);
  Serial.println("Test LED ukončen (zhasnuto).");
}

static void testSummary() {
  Serial.println("=== Souhrn hardwaru ===");
  // I2C
  Serial.print("I2C (SDA 21, SCL 22):");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  Serial.println(found ? "" : " nic nenalezeno");
  if (imu::present()) {
    imu::Stats is = imu::stats();
    Serial.printf("IMU: 0x%02X, ODR %.1f Hz, vzorků %lu, přetečení %lu, chyb I2C %lu\n", imu::address(), is.odrHz,
                  (unsigned long)is.samples, (unsigned long)is.overflows, (unsigned long)is.readErr);
  } else {
    Serial.println("IMU: nenalezeno");
  }
  // X20
  gnss::Stats gs = gnss::stats();
  uint32_t ubxAge = gnss::lastUbxAgeMs();
  const gnss::Info &gi = gnss::info();
  Serial.printf("X20: %s, UART1 %lu Bd, UBX %lu/%lu, PVT %lu, ", gi.valid ? gi.mod : "neodpovídá",
                (unsigned long)gnss::baud(), (unsigned long)gs.ubxOk, (unsigned long)gs.ubxErr,
                (unsigned long)gs.pvtCount);
  if (ubxAge == UINT32_MAX) Serial.println("poslední UBX: nikdy");
  else Serial.printf("poslední UBX před %lu ms, nové hledání %lu×\n", (unsigned long)ubxAge, (unsigned long)gs.reinits);
  const gnss::Pvt &p = gnss::pvt();
  const gnss::SatSummary &ss = gnss::sats();
  Serial.printf("     fix %s, SV %u, sledováno %u, C/N0 max %u dBHz\n", gnss::fixText(p), p.numSV, ss.tracked,
                ss.cnoMax);
  Serial.printf("PPS: %lu pulzů, perioda %.6f s\n", (unsigned long)gs.ppsCount, gs.ppsPeriodUs / 1e6);
  // baterie
  uint32_t mv = power::pinMillivolts();
  uint32_t mvMin, mvMax;
  power::lastSpread(mvMin, mvMax);
  Serial.printf("Baterie: pin 35 %lu mV (průměr 1 s, vzorky %lu–%lu) × %.4f = %.3f V, vyhlazeně %.3f V (%d %%)%s\n",
                (unsigned long)mv, (unsigned long)mvMin, (unsigned long)mvMax, power::dividerFactor(),
                mv * power::dividerFactor() / 1000.0f, power::batteryVolts(), power::batteryPercentStable(),
                mv > 2300 ? " – nad 4,6 V: baterie odpojená nebo vadný dělič"
                          : (power::valid() ? "" : " – mimo rozsah článku: chyba měření"));
  Serial.println("         (s připojeným USB nabíječka nabíjí – napětí je nabíjecí, ne klidové)");
  // tlačítka
  pinMode(PIN_BOOT, INPUT_PULLUP);
  Serial.printf("Tlačítka: Wi-Fi/profil (GPIO%d) %s, BOOT (GPIO%d) %s\n", PIN_BUTTON,
                digitalRead(PIN_BUTTON) == LOW ? "STISKNUTO" : "uvolněno", PIN_BOOT,
                digitalRead(PIN_BOOT) == LOW ? "STISKNUTO" : "uvolněno");
  // systém
  const esp_partition_t *run = esp_ota_get_running_partition();
  Serial.printf("Systém: FW %s, oddíl %s, důvod startu %d, běh %lu s, volná RAM %lu B (min. %lu B)\n", FW_VERSION,
                run ? run->label : "?", (int)esp_reset_reason(), (unsigned long)(millis() / 1000),
                (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
  Serial.printf("Režim: %s, BT %s\n",
                mode == Mode::Run      ? "provoz"
                : mode == Mode::Config ? "konfigurace"
                : mode == Mode::Test   ? "test (Wi-Fi + BT)"
                                       : "záchranný",
                btlink::name());
}

// ---------------------------------------------------------------- konzole
static void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  char c = tolower(cmd[0]);
  switch (c) {
    case 'h': printHelp(); break;
    case 's':
      liveOn = !liveOn;
      Serial.printf("Živý výpis %s\n", liveOn ? "ZAP" : "VYP");
      break;
    case 'i': printInfo(); break;
    case 'c': initGnss(); break;
    case 'm': printImuSamples(); break;
    case 'g':
      Serial.println("Kalibrace gyra – nehýbej přístrojem 3 s...");
      if (imu::calibrateGyro(3000)) {
        float bx, by, bz;
        imu::gyroBias(bx, by, bz);
        Serial.printf("Bias gyra %.3f %.3f %.3f °/s (uloženo).\n", bx, by, bz);
      } else {
        Serial.println("Kalibrace se nepovedla (pohyb nebo málo vzorků), bias beze změny.");
      }
      break;
    case 'k':
    {
      String a = cmd.substring(1);
      a.replace(',', '.');
      if (power::calibrate(a.toFloat()))
        Serial.printf("Kalibrace baterie uložena, koeficient %.4f, nyní %.3f V\n", power::dividerFactor(),
                      power::batteryVolts());
      else
        Serial.println("Neplatná hodnota. Použití: k3860 nebo k3,86 (změřené napětí baterie; koeficient musí vyjít 1,6–2,4).");
    }
      break;
    case 'n':
      nmeaEcho = !nmeaEcho;
      Serial.printf("Výpis NMEA do konzole %s\n", nmeaEcho ? "ZAP" : "VYP");
      break;
    case 'b':
      if (mode == Mode::Run) bridgeMode();
      else bridgeStart();
      break;
    case 'p':
      if (cmd.length() > 1) {
        int i = cmd.substring(1).toInt();
        if (i >= 1 && i <= settings::count()) applyProfile(i - 1, true);
        else Serial.println("Neplatné číslo profilu.");
      } else {
        listProfiles();
      }
      break;
    case 'w':
      if (cmd.length() > 1 && tolower(cmd[1]) == 't') {
        testBootMagic = TEST_MAGIC;
        restartNow("testovací režim");
      } else {
        enterConfig();
      }
      break;
    case 't': {
      char k = cmd.length() > 1 ? cmd[1] : ' ';
      if (k == '1') testButtons();
      else if (k == '2') testBuzzer();
      else if (k == '3') testLed();
      else if (k == '4') testSummary();
      else Serial.println("Testy: t1 tlačítka, t2 piezo, t3 LED, t4 souhrn (popis v h).");
      break;
    }
    case 'x': {
      char k = cmd.length() > 1 ? tolower(cmd[1]) : ' ';
      if (k == 'h') gnss::resetReceiver(gnss::ResetKind::Hot);
      else if (k == 'w') gnss::resetReceiver(gnss::ResetKind::Warm);
      else if (k == 'c') gnss::resetReceiver(gnss::ResetKind::Cold);
      else {
        Serial.println("Použití: xh horký, xw teplý, xc studený start.");
        break;
      }
      Serial.printf("[X20] %s start – doba do fixu se vypíše\n", gnss::resetKindText());
      break;
    }
    case 'r':
      Serial.println("Restart...");
      Serial.flush();
      ESP.restart();
      break;
    default: Serial.println("Neznámý příkaz, h = nápověda."); break;
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.setTxBufferSize(4096);  // výpis nesmí blokovat smyčku
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n\n%s %s\n", FW_NAME, FW_VERSION);

  // Potvrdit běžící OTA oddíl (ochrana proti návratu na předchozí verzi)
  esp_ota_mark_app_valid_cancel_rollback();

  // Počítadlo pádů po sobě (panic, watchdog). Jiný důvod startu ho nuluje.
  esp_reset_reason_t rr = esp_reset_reason();
  if (crashMagic != CRASH_MAGIC) {
    crashMagic = CRASH_MAGIC;
    crashCount = 0;
  }
  if (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT) crashCount++;
  else crashCount = 0;
  bool testBoot = testBootMagic == TEST_MAGIC && rr == ESP_RST_SW;
  testBootMagic = 0;

  button::begin();
  power::begin();
  if (!settings::begin()) Serial.println("Paměť nastavení (LittleFS): chyba, použity výchozí profily");
  testctl::begin();
  if (!survey::begin()) Serial.println("Měření bodů (/meas.json): chyba čtení");
  ebuf::begin();

  if (crashCount >= RESCUE_CRASHES) {
    Serial.printf("ZÁCHRANNÝ REŽIM: %lu pádů po sobě – jen Wi-Fi AP s aktualizací firmwaru\n",
                  (unsigned long)crashCount);
    mode = Mode::Rescue;
    startWeb(true);
    return;
  }

  if (imu::begin()) Serial.printf("IMU: LSM6DSV16X na 0x%02X, 120 Hz\n", imu::address());
  else if (imu::present()) Serial.println("IMU: nalezeno, ale nastavení se nepodařilo ověřit");
  else Serial.println("IMU: nenalezeno");
  if (imu::stats().busRecovered) Serial.println("IMU: sběrnice I2C byla zablokovaná – uvolněna");

  if (btlink::begin(settings::device().btName)) Serial.printf("Bluetooth SPP: %s\n", btlink::name());
  else Serial.println("Bluetooth: start selhal");
  nmea::setSink(nmeaSink);
  btlink::setCmdHandler(proto::handleCmd);
  proto::setProfileHook([](int i) { applyProfile(i, true); });
  gnss::onPvt([]() { pvtPending = true; });
  // surová data měření bodu (RAWX/SFRBX) celým rámcem UBX do BT, mimo dávky vět
  gnss::onRawFrame([](const uint8_t *f, uint16_t n) {
    if (!btlink::connected()) return;
    // MGA-DBD (výpis databáze na žádost aplikace) se nesmí zahodit – jde mimo limit fronty
    if (n >= 4 && f[2] == ubx::CLS_MGA && f[3] == ubx::ID_MGA_DBD) {
      btlink::sendNow(f, n);
      return;
    }
    btlink::flush();
    btlink::send((const char *)f, n);
    btlink::flush();
  });
  gnss::onSat([]() { satPending = true; });

  applyProfile(settings::defaultIndex(), false);
  gnss::begin();
  initGnss(15000);  // po zapnutí napájení X20 nabíhá déle než ESP

  if (testBoot) {
    Serial.println("[režim] TEST – Wi-Fi AP a Bluetooth současně");
    if (startWeb(false)) {
      mode = Mode::Test;
      webui::setTestMode(true);
    }
  }

  printInfo();
  printHelp();

  // FIFO IMU se během inicializace X20 a BT přeplnilo – začít s prázdným
  imu::flush();
}

static void readConsole() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length()) {
        handleCommand(line);
        line = "";
      }
    } else if (line.length() < 32) {
      line += ch;
    }
  }
}

static void handleWebEvent(webui::Event e) {
  switch (e) {
    case webui::Event::Restart: restartNow("konec konfigurace"); break;
    case webui::Event::ApOff: exitConfig(); break;
    case webui::Event::ProfilesChanged: applyProfile(settings::active(), true); break;
    case webui::Event::BridgeStart: bridgeStart(); break;
    case webui::Event::BridgeStop: bridgeStop(); break;
    case webui::Event::TestMode:
      testBootMagic = TEST_MAGIC;
      restartNow("testovací režim");
      break;
    default: break;
  }
}

void loop() {
  static uint32_t lastPrint = 0;

  // Po minutě bez pádu vynulovat počítadlo pádů
  if (crashCount && millis() > 60000) crashCount = 0;
  power::update();

  if (mode == Mode::Rescue) {
    handleWebEvent(webui::poll());
    readConsole();
    return;
  }

  // Průchozí režim: USB patří X20, konzole i čtení X20 stojí, web běží
  if (gnss::bridgeActive()) {
    gnss::bridgeStep();
    imu::poll();
    handleWebEvent(webui::poll());
    if (button::poll() == button::Event::Long) bridgeStop();
    return;
  }

  gnss::poll();
  imu::poll();

  // Hlídání X20: 3 s bez platné zprávy UBX -> znovu najít a nastavit (nejdřív za 10 s od pokusu)
  static uint32_t lastGnssTry = 0;
  if (gnss::lastUbxAgeMs() > 3000 && millis() - lastGnssTry > 10000) {
    lastGnssTry = millis();
    Serial.println("[X20] 3 s bez dat – hledám znovu");
    initGnss();
  }
  static uint32_t ttffShown = 0;
  if (gnss::ttff() && gnss::ttff() != ttffShown) {
    ttffShown = gnss::ttff();
    Serial.printf("[X20] fix po %s startu za %.1f s\n", gnss::resetKindText(), ttffShown / 1000.0f);
  }

  bool newPvt = pvtPending, newSat = satPending;
  pvtPending = false;
  satPending = false;
  if (newPvt) {
    testctl::onPvt();
    survey::onPvt();
  }
  testctl::poll();
  survey::poll();
  buzzer::poll();

  {  // BT běží ve všech režimech kromě záchranného
    btlink::poll();
    ebuf::poll();
    {
      btlink::Stats bs = btlink::stats();
      gnss::setRtcmActive(bs.rtcmOk && millis() - bs.rtcmLastMs < 30000);  // RTCM z mobilu za posledních 30 s
    }
    // NMEA po každé epoše a satelity 1 Hz – mimo callback, aby odesílání nezdržovalo parser
    if (newPvt) {
      nmea::sendEpoch();
      proto::sendPos();
      btlink::flush();
    }
    if (newSat) {
      nmea::sendSats();
      btlink::flush();
      proto::sendSat();
    }
    // Režim aplikace: z NMEA jen GGA a RMC (zbytek je v $PRVR); po odpojení zase podle profilu
    static bool lastApp = false;
    bool appNow = proto::appMode();
    if (appNow != lastApp) {
      lastApp = appNow;
      uint8_t m = settings::profile(settings::active()).nmea;
      nmea::setMask(appNow ? (uint8_t)(m & (nmea::GGA | nmea::RMC)) : m);
      Serial.printf("[BT] režim aplikace %s\n", appNow ? "ZAP (NMEA jen GGA, RMC)" : "VYP");
      // aplikace odpojena uprostřed měření bodu – model z profilu, bez RAWX
      // (při záznamu v aplikaci zůstává – epochy jdou do zásobníku a měření po připojení pokračuje)
      if (!appNow && !ebuf::recording() && (gnss::measStatic() || gnss::measRaw())) {
        gnss::setMeasMode(false, false);
        Serial.println("[app] měření bodu ukončeno (odpojení BT)");
      }
    }
    static uint32_t lastSta = 0;
    if (millis() - lastSta >= 1000) {
      lastSta = millis();
      proto::sendSta();
    }
    switch (btlink::takeEvent()) {
      case btlink::Event::Connected: Serial.println("[BT] klient připojen"); break;
      case btlink::Event::Disconnected: Serial.println("[BT] klient odpojen"); break;
      case btlink::Event::PairOk: Serial.println("[BT] spárování úspěšné"); break;
      case btlink::Event::PairFail: Serial.println("[BT] spárování SELHALO"); break;
      default: break;
    }
  }
  if (mode == Mode::Config || mode == Mode::Test) handleWebEvent(webui::poll());

  // Spotřebitel vzorků IMU – zde bude EKF (krok 6); zatím se fronta jen vyprazdňuje
  imu::Sample smp;
  while (imu::pop(smp)) {
    survey::onImu(smp);
    proto::onImu(smp);
    imustream::onImu(smp);
  }

  static uint32_t imuReinits = 0;
  if (imu::stats().reinits != imuReinits) {
    imuReinits = imu::stats().reinits;
    Serial.printf("[IMU] 1 s bez vzorků – IMU znovu nastaveno (%lu×)\n", (unsigned long)imuReinits);
  }

  switch (button::poll()) {
    case button::Event::Short:
      if (mode == Mode::Run) applyProfile(settings::next(), true);
      break;
    case button::Event::Long:  // podržení 2 s: Wi-Fi zap / vyp
      if (mode == Mode::Run) enterConfig();
      else if (mode == Mode::Config || mode == Mode::Test) exitConfig();
      break;
    default: break;
  }

  readConsole();

  if (liveOn && millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printStatus();
  }
}
