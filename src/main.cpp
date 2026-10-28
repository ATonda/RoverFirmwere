// Rover GNSS – firmware ESP32
// Specifikace: docs/rover-firmware-specifikace.md
//
// Sériová konzole 115200 Bd, příkaz + Enter, "h" = nápověda. Řádek začínající '{' = příkaz JSON
// (stejný jako z aplikace RoverGnss), odpověď do konzole.

#include <Arduino.h>
#include <esp_timer.h>

#include "epochbuf.h"
#include "console.h"
#include "imustream.h"
#include "motion.h"
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
#include "usblink.h"

static bool liveOn = false;
static bool nmeaEcho = false;            // NMEA i do konzole
static volatile bool pvtPending = false; // nová epocha NAV-PVT
static volatile bool satPending = false; // nový NAV-SAT

static void nmeaSink(const char *s, size_t n) {
  btlink::send(s, n);
  if (nmeaEcho) Con.write((const uint8_t *)s, n);
}

// ---------------------------------------------------------------- výpisy
static void printHelp() {
  Con.println();
  Con.printf("=== %s %s – konzole ===\n", FW_NAME, FW_VERSION);
  Con.println("h       nápověda");
  Con.println("s       živý výpis 1 Hz zap/vyp");
  Con.println("i       informace o přístroji a X20");
  Con.println("c       znovu nakonfigurovat X20 (detekce, 460800, RAM)");
  Con.println("m       10 vzorků IMU");
  Con.println("g       kalibrace gyra (přístroj v klidu, 3 s)");
  Con.println("k3860   kalibrace baterie – změřené napětí v mV");
  Con.println("n       výpis NMEA do konzole zap/vyp");
  Con.println("b       průchozí režim USB <-> X20 (konec krátkým stiskem tlačítka nebo z aplikace)");
  Con.println("p       seznam profilů, p2 = přepnout na profil 2");
  Con.println("xh xw xc  restart příjmu X20: horký / teplý / studený start");
  Con.println("{...}   příkaz JSON jako z aplikace, např. {\"cmd\":\"get\",\"what\":\"profiles\"}");
  Con.println("        {\"cmd\":\"usbhost\",\"hz\":5,\"model\":\"pedestrian\"} = režim USB host (ESP32-S3), drží se opakováním 1×/s");
  Con.println("t1      test tlačítek 20 s: profil/restart (GPIO13) a BOOT (GPIO0) – stav a události, konec klávesou");
  Con.println("t2      test pieza (GPIO25): tóny 500–4000 Hz, pak ukázky signálů (fix, varování, profil)");
  Con.println("t3      test LED WS2812 (GPIO26): červená, zelená, modrá, bílá po 1 s");
  Con.println("t4      souhrn: sken I2C, X20, PPS, baterie (pin i přepočet), tlačítka, paměť, důvod startu");
  Con.println("r       restart");
  Con.println();
}

static void printInfo() {
  Con.printf("Firmware: %s %s\n", FW_NAME, FW_VERSION);
  const gnss::Info &gi = gnss::info();
  if (gi.valid) {
    Con.printf("X20: %s, FW %s (%s), HW %s, UART1 %lu Bd\n", gi.mod, gi.fwver, gi.sw, gi.hw,
                  (unsigned long)gnss::baud());
  } else {
    Con.println("X20: neodpovídá");
  }
  if (imu::present()) {
    float bx, by, bz;
    imu::gyroBias(bx, by, bz);
    imu::Stats is = imu::stats();
    Con.printf("IMU: LSM6DSV16X na 0x%02X, bias gyra %.3f %.3f %.3f °/s\n", imu::address(), bx, by, bz);
    Con.printf("     vzorků %lu, ODR %.1f Hz, poslední před %lu ms, přetečení %lu, nové nastavení %lu×, chyb zápisu %lu\n",
                  (unsigned long)is.samples, is.odrHz, (unsigned long)is.lastSampleAgeMs, (unsigned long)is.overflows,
                  (unsigned long)is.reinits, (unsigned long)is.configErr);
    if (is.busRecovered) Con.println("     při startu byla sběrnice I2C zablokovaná – uvolněna");
  } else {
    Con.println("IMU: nenalezeno");
  }
  float v = power::batteryVolts();
  Con.printf("Baterie: %.3f V (%d %%)%s, koeficient děliče %.4f\n", v, power::batteryPercentStable(),
                power::valid() ? "" : " – CHYBA MĚŘENÍ", power::dividerFactor());
  Con.printf("Bluetooth SPP: %s\n", btlink::name());
  const settings::Profile &pr = settings::profile(settings::active());
  Con.printf("Profil: %d/%d %s (%u Hz, %s, maska %d°, SBAS %s, HAS %s)\n", settings::active() + 1,
                settings::count(), pr.name, pr.rateHz, settings::dynName(pr.dyn), pr.elevMask, pr.sbas ? "zap" : "vyp",
                pr.has ? "zap" : "vyp");
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

  Con.printf("GNSS %lu Bd %5.0f B/s PVT %.1f Hz UBX %lu/%lu | fix %-8s SV %u/%u (sled. %u) CN0 %.0f/%u | hAcc %.2f vAcc %.2f m pDOP %.2f\n",
                (unsigned long)gnss::baud(), bps, pvtHz, (unsigned long)gs.ubxOk, (unsigned long)gs.ubxErr,
                gnss::fixText(p), p.numSV, s.used, s.tracked, s.cnoMeanUsed, s.cnoMax, p.hAcc, p.vAcc, p.pDOP);
  if (p.valid && p.timeValid) {
    Con.printf("     %04u-%02u-%02u %02u:%02u:%02u UTC lat %.7f lon %.7f h %.2f m v %.2f m/s kurz %.1f°\n",
                  p.year, p.month, p.day, p.hour, p.min, p.sec, p.lat, p.lon, p.hMSL, p.gSpeed, p.headMot);
  }
  Con.printf("     PPS %lu (perioda %.6f s, před %lld ms, PVT za PPS %.1f ms)",
                (unsigned long)gs.ppsCount, gs.ppsPeriodUs / 1e6, (long long)ppsAgeMs, gs.pvtLatencyUs / 1000.0f);

  imu::Sample a;
  if (imu::latest(a)) {
    float n = sqrtf(a.ax * a.ax + a.ay * a.ay + a.az * a.az);
    Con.printf(" | IMU %.0f Hz (ODR %.1f) FIFO max %u přetečení %lu |a| %.3f g gyro %.2f %.2f %.2f °/s %.1f °C",
                  imuHz, is.odrHz, is.fifoMax, (unsigned long)is.overflows, n, a.gx, a.gy, a.gz, a.tempC);
  } else {
    Con.print(" | IMU -");
  }
  float v = power::batteryVolts();
  Con.printf(" | BAT %.2f V %d %%%s\n", v, power::batteryPercentStable(), power::valid() ? "" : " CHYBA");

  const gnss::Dop &dp = gnss::dop();
  const gnss::Cov &cv = gnss::cov();
  Con.printf("     BT %s (%lu×, pár. %lu/%lu) NMEA %.0f B/s zahoz. %lu | z mobilu %.0f B/s RTCM %lu/%lu",
                bs.connected ? "PŘIPOJENO" : "nepřipojeno", (unsigned long)bs.connects, (unsigned long)bs.pairOk,
                (unsigned long)bs.pairFail, btTx,
                (unsigned long)bs.txDropped, btRx, (unsigned long)bs.rtcmOk, (unsigned long)bs.rtcmBad);
  if (bs.rtcmOk) Con.printf(" posl. %u před %.1f s", bs.rtcmLastType, (millis() - bs.rtcmLastMs) / 1000.0f);
  const gnss::SigSummary &sg = gnss::sig();
  Con.printf(" | GGA kval. %d kor. %s (zdroj %d, sig. %u/%u) | HDOP %.2f σN %.2f σE %.2f σD %.2f m%s\n",
                nmea::ggaQuality(), p.corrAge ? "ano" : "-", sg.dominant, sg.corrUsed, sg.n, dp.hDOP, cv.sN, cv.sE,
                cv.sD, cv.valid ? "" : " (kov. neplatná)");
  const motion::Features &mf = motion::features();
  Con.printf("     stav %s%s, model X20 %u | |a| σ %.0f mg, odchylka %.2f g, ω max %.0f °/s, kroky %.2f (%.1f Hz), v %.1f m/s\n",
                motion::stateName(motion::state()), motion::autoOn() ? " (automaticky)" : "", gnss::dynEffective(),
                mf.accStd * 1000, mf.accDev, mf.gyroMax, mf.stepScore, mf.stepHz, mf.speed);
}

static void printImuSamples() {
  if (!imu::present()) {
    Con.println("IMU nenalezeno.");
    return;
  }
  Con.println("  T[°C]   ax[g]   ay[g]   az[g]  |a|[g]   gx[°/s]  gy[°/s]  gz[°/s]");
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
      Con.printf("  %5.1f  %6.3f  %6.3f  %6.3f  %6.3f  %8.2f %8.2f %8.2f\n", a.tempC, a.ax, a.ay, a.az, n, a.gx,
                    a.gy, a.gz);
      shown++;
    }
    delay(1);
  }
}

// ---------------------------------------------------------------- X20
static void initGnss(uint32_t searchMs = 0) {
  Con.println("X20: inicializace...");
  gnss::InitResult r = gnss::init(searchMs);
  Con.printf("X20: %s\n", gnss::initResultText(r));
}

// Průchozí režim USB <-> X20 po krocích: BT běží dál (příkaz stop z aplikace), konec i krátkým stiskem tlačítka
static void bridgeStart() {
  if (gnss::bridgeActive()) return;
  Con.printf("[průchozí] USB <-> X20 na %lu Bd. Zavři monitor, v u-center otevři COM na této rychlosti.\n",
                (unsigned long)gnss::baud());
  Con.println("[průchozí] konec: krátký stisk tlačítka, příkaz z aplikace nebo vypnutí.");
  liveOn = false;
  gnss::bridgeBegin();
  button::resetEvents();
}

static void bridgeStop() {
  if (!gnss::bridgeActive()) return;
  gnss::bridgeEnd();
  button::resetEvents();
  Con.println("\n[průchozí] ukončen, konfigurace X20 se znovu posílá do RAM.");
  // u-center mohl rychlost nebo výstupy změnit – obnovit provozní stav
  initGnss();
}

// ---------------------------------------------------------------- profily
// Nastaví parametry z profilu; reconfigure = hned poslat do X20 (jinak až při init)
static void applyProfile(int i, bool reconfigure) {
  proto::speedCancel();  // přepnutí profilu ukončí měření rychlosti (FW 0.16.0)
  settings::setActive(i);
  const settings::Profile &p = settings::profile(settings::active());
  gnss::setParams(1000 / p.rateHz, settings::dynCode(p.dyn), p.elevMask);
  gnss::setCorr(p.sbas, p.has);
  motion::setAuto(p.dyn == settings::Dyn::Auto);
  nmea::setMask(proto::appMode() ? (uint8_t)(p.nmea & (nmea::GGA | nmea::RMC)) : p.nmea);
  buzzer::setEnabled(p.buzzer);
  Con.printf("[profil] %d/%d %s – %u Hz, %s, maska %d°, SBAS %s, HAS %s\n", settings::active() + 1,
                settings::count(), p.name, p.rateHz, settings::dynName(p.dyn), p.elevMask, p.sbas ? "zap" : "vyp",
                p.has ? "zap" : "vyp");
  if (reconfigure) gnss::configure(true);
}

static void listProfiles() {
  for (int i = 0; i < settings::count(); i++) {
    const settings::Profile &p = settings::profile(i);
    Con.printf("%c%c %d  %-24s %2u Hz  %-10s maska %2d°  SBAS %s HAS %s\n", i == settings::active() ? '>' : ' ',
                  i == settings::defaultIndex() ? '*' : ' ', i + 1, p.name, p.rateHz, settings::dynName(p.dyn),
                  p.elevMask, p.sbas ? "zap" : "vyp", p.has ? "zap" : "vyp");
  }
  Con.println("> aktivní, * výchozí po startu");
}

static void restartNow(const char *why) {
  Con.printf("[režim] %s – restart\n", why);
  Con.flush();
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
    if (Con.available()) {
      while (Con.available()) Con.read();
      return true;
    }
    delay(1);
  }
  return false;
}

static void flushConsoleInput() {
  delay(30);
  while (Con.available()) Con.read();
}

static void testButtons() {
  flushConsoleInput();
  pinMode(PIN_BOOT, INPUT_PULLUP);
  Con.printf("Test tlačítek 20 s (konec libovolnou klávesou). Profil/restart = GPIO%d, BOOT = GPIO%d.\n",
                PIN_BUTTON, PIN_BOOT);
  button::resetEvents();
  button::poll();
  bool btn = button::pressed();
  bool boot = digitalRead(PIN_BOOT) == LOW, bootRaw = boot;
  uint32_t bootChangeMs = 0, btnDownMs = millis(), bootDownMs = millis();
  Con.printf("  výchozí stav: profil/restart %s, BOOT %s\n", btn ? "STISKNUTO" : "uvolněno", boot ? "STISKNUTO" : "uvolněno");
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    uint32_t now = millis();
    // tlačítko profil/restart – odrušené v modulu button
    button::Event e = button::poll();
    bool b = button::pressed();
    if (b != btn) {
      btn = b;
      if (b) {
        btnDownMs = now;
        Con.printf("  %5.1f s  profil/restart: stisk\n", (now - t0) / 1000.0f);
      } else {
        Con.printf("  %5.1f s  profil/restart: uvolnění po %lu ms\n", (now - t0) / 1000.0f,
                      (unsigned long)(now - btnDownMs));
      }
    }
    if (e == button::Event::Short) Con.println("           -> událost KRÁTKÝ STISK (v provozu: další profil)");
    if (e == button::Event::Long) Con.println("           -> událost PODRŽENÍ 5 s (v provozu: restart ESP; v testu bez akce)");
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
        Con.printf("  %5.1f s  BOOT: stisk\n", (now - t0) / 1000.0f);
      } else {
        Con.printf("  %5.1f s  BOOT: uvolnění po %lu ms\n", (now - t0) / 1000.0f, (unsigned long)(now - bootDownMs));
      }
    }
    if (serviceWait(2)) break;
  }
  button::resetEvents();
  Con.println("Test tlačítek ukončen.");
}

static bool beep(uint32_t freq, uint32_t onMs, uint32_t offMs) {
  buzzer::start(freq);
  bool abort = serviceWait(onMs);
  buzzer::stop();
  return abort || serviceWait(offMs);
}

static void testBuzzer() {
  flushConsoleInput();
  Con.printf("Test pieza na GPIO%d (konec libovolnou klávesou).\n", PIN_BUZZER);
  static const uint16_t F[] = {500, 1000, 2000, 2700, 3000, 4000};
  for (uint16_t f : F) {
    Con.printf("  %u Hz\n", f);
    if (beep(f, 400, 300)) return;
  }
  Con.println("  signál FIX (dva stoupající tóny)");
  if (beep(1500, 100, 60) || beep(2500, 150, 700)) return;
  Con.println("  signál VAROVÁNÍ (3× hluboký)");
  for (int i = 0; i < 3; i++)
    if (beep(800, 200, 150)) return;
  if (serviceWait(500)) return;
  for (int n = 1; n <= 3; n++) {
    Con.printf("  signál PROFIL %d (%d× krátce)\n", n, n);
    for (int i = 0; i < n; i++)
      if (beep(2700, 80, 120)) return;
    if (serviceWait(600)) return;
  }
  Con.println("Test pieza ukončen.");
}

static void testLed() {
  flushConsoleInput();
  Con.printf("Test LED WS2812 na GPIO%d (jas 25 %%, konec libovolnou klávesou).\n", PIN_LED);
  struct C {
    const char *name;
    uint8_t r, g, b;
  };
  static const C colors[] = {{"červená", 64, 0, 0}, {"zelená", 0, 64, 0}, {"modrá", 0, 0, 64}, {"bílá", 64, 64, 64}};
  for (const C &c : colors) {
    Con.printf("  %s\n", c.name);
    rgbLedWrite(PIN_LED, c.r, c.g, c.b);
    if (serviceWait(1000)) break;
  }
  rgbLedWrite(PIN_LED, 0, 0, 0);
  Con.println("Test LED ukončen (zhasnuto).");
}

static void testSummary() {
  Con.println("=== Souhrn hardwaru ===");
  // I2C
  Con.print("I2C (SDA 21, SCL 22):");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Con.printf(" 0x%02X", a);
      found++;
    }
  }
  Con.println(found ? "" : " nic nenalezeno");
  if (imu::present()) {
    imu::Stats is = imu::stats();
    Con.printf("IMU: 0x%02X, ODR %.1f Hz, vzorků %lu, přetečení %lu, chyb I2C %lu\n", imu::address(), is.odrHz,
                  (unsigned long)is.samples, (unsigned long)is.overflows, (unsigned long)is.readErr);
  } else {
    Con.println("IMU: nenalezeno");
  }
  // X20
  gnss::Stats gs = gnss::stats();
  uint32_t ubxAge = gnss::lastUbxAgeMs();
  const gnss::Info &gi = gnss::info();
  Con.printf("X20: %s, UART1 %lu Bd, UBX %lu/%lu, PVT %lu, ", gi.valid ? gi.mod : "neodpovídá",
                (unsigned long)gnss::baud(), (unsigned long)gs.ubxOk, (unsigned long)gs.ubxErr,
                (unsigned long)gs.pvtCount);
  if (ubxAge == UINT32_MAX) Con.println("poslední UBX: nikdy");
  else Con.printf("poslední UBX před %lu ms, nové hledání %lu×\n", (unsigned long)ubxAge, (unsigned long)gs.reinits);
  const gnss::Pvt &p = gnss::pvt();
  const gnss::SatSummary &ss = gnss::sats();
  Con.printf("     fix %s, SV %u, sledováno %u, C/N0 max %u dBHz\n", gnss::fixText(p), p.numSV, ss.tracked,
                ss.cnoMax);
  Con.printf("PPS: %lu pulzů, perioda %.6f s\n", (unsigned long)gs.ppsCount, gs.ppsPeriodUs / 1e6);
  // baterie
  uint32_t mv = power::pinMillivolts();
  uint32_t mvMin, mvMax;
  power::lastSpread(mvMin, mvMax);
  Con.printf("Baterie: pin 35 %lu mV (průměr 10 s, vzorky %lu–%lu) × %.4f = %.3f V, vyhlazeně %.3f V (%d %%)%s\n",
                (unsigned long)mv, (unsigned long)mvMin, (unsigned long)mvMax, power::dividerFactor(),
                mv * power::dividerFactor() / 1000.0f, power::batteryVolts(), power::batteryPercentStable(),
                mv > 2300 ? " – nad 4,6 V: baterie odpojená nebo vadný dělič"
                          : (power::valid() ? "" : " – mimo rozsah článku: chyba měření"));
  Con.println("         (s připojeným USB nabíječka nabíjí – napětí je nabíjecí, ne klidové)");
  // tlačítka
  pinMode(PIN_BOOT, INPUT_PULLUP);
  Con.printf("Tlačítka: profil/restart (GPIO%d) %s, BOOT (GPIO%d) %s\n", PIN_BUTTON,
                digitalRead(PIN_BUTTON) == LOW ? "STISKNUTO" : "uvolněno", PIN_BOOT,
                digitalRead(PIN_BOOT) == LOW ? "STISKNUTO" : "uvolněno");
  // systém
  Con.printf("Systém: FW %s, důvod startu %d, běh %lu s, volná RAM %lu B (min. %lu B, největší blok %lu B)\n",
                FW_VERSION, (int)esp_reset_reason(), (unsigned long)(millis() / 1000), (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
  Con.printf("Bluetooth %s, záznam v aplikaci %s\n", btlink::name(), ebuf::recording() ? "ANO" : "ne");
}

// ---------------------------------------------------------------- konzole
static void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  if (cmd[0] == '{') {
    proto::handleConsoleCmd(cmd.c_str(), cmd.length());
    return;
  }
  char c = tolower(cmd[0]);
  switch (c) {
    case 'h': printHelp(); break;
    case 's':
      liveOn = !liveOn;
      Con.printf("Živý výpis %s\n", liveOn ? "ZAP" : "VYP");
      break;
    case 'i': printInfo(); break;
    case 'c': initGnss(); break;
    case 'm': printImuSamples(); break;
    case 'g':
      Con.println("Kalibrace gyra – nehýbej přístrojem 30 s...");
      if (imu::calibrateGyro(30000, gnss::poll)) {
        float bx, by, bz;
        imu::gyroBias(bx, by, bz);
        Con.printf("Bias gyra %.3f %.3f %.3f °/s (uloženo).\n", bx, by, bz);
      } else {
        Con.println("Kalibrace se nepovedla (pohyb nebo málo vzorků), bias beze změny.");
      }
      break;
    case 'k':
    {
      String a = cmd.substring(1);
      a.replace(',', '.');
      if (power::calibrate(a.toFloat()))
        Con.printf("Kalibrace baterie uložena, koeficient %.4f, nyní %.3f V\n", power::dividerFactor(),
                      power::batteryVolts());
      else
        Con.println("Neplatná hodnota. Použití: k3860 nebo k3,86 (změřené napětí baterie; koeficient musí vyjít 1,6–2,4).");
    }
      break;
    case 'n':
      nmeaEcho = !nmeaEcho;
      Con.printf("Výpis NMEA do konzole %s\n", nmeaEcho ? "ZAP" : "VYP");
      break;
    case 'b': bridgeStart(); break;
    case 'p':
      if (cmd.length() > 1) {
        int i = cmd.substring(1).toInt();
        if (i >= 1 && i <= settings::count()) applyProfile(i - 1, true);
        else Con.println("Neplatné číslo profilu.");
      } else {
        listProfiles();
      }
      break;
    case 't': {
      char k = cmd.length() > 1 ? cmd[1] : ' ';
      if (k == '1') testButtons();
      else if (k == '2') testBuzzer();
      else if (k == '3') testLed();
      else if (k == '4') testSummary();
      else Con.println("Testy: t1 tlačítka, t2 piezo, t3 LED, t4 souhrn (popis v h).");
      break;
    }
    case 'x': {
      char k = cmd.length() > 1 ? tolower(cmd[1]) : ' ';
      if (k == 'h') gnss::resetReceiver(gnss::ResetKind::Hot);
      else if (k == 'w') gnss::resetReceiver(gnss::ResetKind::Warm);
      else if (k == 'c') gnss::resetReceiver(gnss::ResetKind::Cold);
      else {
        Con.println("Použití: xh horký, xw teplý, xc studený start.");
        break;
      }
      Con.printf("[X20] %s start – doba do fixu se vypíše\n", gnss::resetKindText());
      break;
    }
    case 'r':
      Con.println("Restart...");
      Con.flush();
      ESP.restart();
      break;
    default: Con.println("Neznámý příkaz, h = nápověda."); break;
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Con.setTxBufferSize(4096);  // výpis nesmí blokovat smyčku
  Con.setRxBufferSize(4096);  // režim USB host: RTCM z S3 po dávkách 1–3 kB
  Con.begin(115200);
  delay(200);
  Con.printf("\n\n%s %s\n", FW_NAME, FW_VERSION);

  button::begin();
  power::begin();
  if (!settings::begin()) Con.println("Paměť nastavení (LittleFS): chyba, použity výchozí profily");
  ebuf::begin();

  if (imu::begin()) Con.printf("IMU: LSM6DSV16X na 0x%02X, 120 Hz\n", imu::address());
  else if (imu::present()) Con.println("IMU: nalezeno, ale nastavení se nepodařilo ověřit");
  else Con.println("IMU: nenalezeno");
  if (imu::stats().busRecovered) Con.println("IMU: sběrnice I2C byla zablokovaná – uvolněna");

  if (btlink::begin(settings::device().btName)) Con.printf("Bluetooth SPP: %s\n", btlink::name());
  else Con.println("Bluetooth: start selhal");
  nmea::setSink(nmeaSink);
  btlink::setCmdHandler(proto::handleCmd);
  usblink::setCmdHandler(proto::handleUsbCmd);
  {
    proto::Hooks h;
    h.applyProfile = [](int i) { applyProfile(i, true); };
    h.bridge = [](bool on) {
      if (on) bridgeStart();
      else bridgeStop();
    };
    h.bridgeActive = []() { return gnss::bridgeActive(); };
    h.restart = []() { restartNow("příkaz z aplikace"); };
    h.idle = gnss::poll;
    proto::setHooks(h);
  }
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
  // Záznam v aplikaci: celé NAV-PVT, NAV-COV, NAV-HPPOSLLH do dávky BT (před $PRVR,POS téže epochy)
  gnss::onNavFrame([](const uint8_t *f, uint16_t n) {
    if (proto::navRecording()) btlink::send((const char *)f, n);
  });

  applyProfile(settings::defaultIndex(), false);
  gnss::begin();
  initGnss(15000);  // po zapnutí napájení X20 nabíhá déle než ESP

  printInfo();
  printHelp();

  // FIFO IMU se během inicializace X20 a BT přeplnilo – začít s prázdným
  imu::flush();
}

static void readConsole() {
  static String line;
  static bool binary = false;  // v řádku byl řídicí bajt – data (RTCM z S3), ne příkaz
  while (Con.available()) {
    char ch = Con.read();
    if (ch == '\n' || ch == '\r') {
      // ochrana: po režimu USB host a po binárních datech 5 s jen řádky JSON (bajty RTCM by spustily příkaz písmenem)
      if (line.length() && !binary && (line[0] == '{' || !usblink::consoleGuard())) handleCommand(line);
      line = "";
      binary = false;
      if (usblink::active()) return;  // příkaz zapnul režim USB host – zbytek vstupu patří parseru USB
    } else if ((uint8_t)ch < 0x20 && ch != '\t') {
      binary = true;
      usblink::noteBinary();
    } else if (line.length() < 4096) {  // příkaz JSON (set profiles) může mít několik kB
      line += ch;
    }
  }
}

void loop() {
  static uint32_t lastPrint = 0;

  power::update();

  // Průchozí režim: USB patří X20, konzole i čtení X20 stojí; BT běží (příkaz stop z aplikace)
  if (gnss::bridgeActive()) {
    gnss::bridgeStep();
    imu::poll();
    imu::Sample drop;
    while (imu::pop(drop)) {
    }
    btlink::poll();
    if (button::poll() != button::Event::None) bridgeStop();
    return;
  }

  gnss::poll();
  imu::poll();

  // Hlídání X20: 3 s bez platné zprávy UBX -> znovu najít a nastavit (nejdřív za 10 s od pokusu)
  static uint32_t lastGnssTry = 0;
  if (gnss::lastUbxAgeMs() > 3000 && millis() - lastGnssTry > 10000) {
    lastGnssTry = millis();
    Con.println("[X20] 3 s bez dat – hledám znovu");
    initGnss();
  }
  static uint32_t ttffShown = 0;
  if (gnss::ttff() && gnss::ttff() != ttffShown) {
    ttffShown = gnss::ttff();
    Con.printf("[X20] fix po %s startu za %.1f s\n", gnss::resetKindText(), ttffShown / 1000.0f);
  }

  bool newPvt = pvtPending, newSat = satPending;
  pvtPending = false;
  satPending = false;
  buzzer::poll();
  if (newPvt) motion::onPvt();
  motion::poll();
  proto::speedPoll();

  {
    btlink::poll();
    ebuf::poll();
    usblink::poll();   // režim USB host: JSON a RTCM z S3
    proto::usbPoll();  // 3 s bez příkazu z S3 -> konec režimu
    gnss::setRtcmActive(proto::rtcmSource() != 0);  // RTCM do X20 (BT nebo USB) za posledních 30 s
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
      Con.printf("[BT] režim aplikace %s\n", appNow ? "ZAP (NMEA jen GGA, RMC)" : "VYP");
      // aplikace odpojena uprostřed měření bodu – model z profilu, bez RAWX
      // (při záznamu v aplikaci zůstává – epochy jdou do zásobníku a měření po připojení pokračuje)
      if (!appNow && !ebuf::recording() && (gnss::measStatic() || gnss::measRaw())) {
        gnss::setMeasMode(false, false);
        Con.println("[app] měření bodu ukončeno (odpojení BT)");
      }
    }
    static uint32_t lastSta = 0;
    if (millis() - lastSta >= 1000) {
      lastSta = millis();
      proto::sendSta();
    }
    switch (btlink::takeEvent()) {
      case btlink::Event::Connected: Con.println("[BT] klient připojen"); break;
      case btlink::Event::Disconnected: Con.println("[BT] klient odpojen"); break;
      case btlink::Event::PairOk: Con.println("[BT] spárování úspěšné"); break;
      case btlink::Event::PairFail: Con.println("[BT] spárování SELHALO"); break;
      default: break;
    }
  }

  // Spotřebitel vzorků IMU – zde bude EKF (krok 7); zatím pohyb pro POS a surová data do aplikace
  imu::Sample smp;
  while (imu::pop(smp)) {
    motion::onImu(smp);
    proto::onImu(smp);
    imustream::onImu(smp);
  }

  static uint32_t imuReinits = 0;
  if (imu::stats().reinits != imuReinits) {
    imuReinits = imu::stats().reinits;
    Con.printf("[IMU] 1 s bez vzorků – IMU znovu nastaveno (%lu×)\n", (unsigned long)imuReinits);
  }

  switch (button::poll()) {
    case button::Event::Short:
      if (proto::usbMode()) buzzer::beep(800, 80, 0, 1);  // režim USB host: profil určuje S3 – odmítnuto
      else applyProfile(settings::next(), true);
      break;
    case button::Event::Long: {  // podržení 5 s: pípnutí a restart ESP (X20 dostane po startu znovu konfiguraci)
      buzzer::stop();
      buzzer::start(2000);
      uint32_t t0 = millis();
      while (millis() - t0 < 300) delay(1);
      buzzer::stop();
      restartNow("podržení tlačítka 5 s");
      break;
    }
    default: break;
  }

  if (!usblink::active()) readConsole();

  if (liveOn && millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printStatus();
  }
}
