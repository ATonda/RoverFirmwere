// Bluetooth Classic SPP: výstup NMEA, příjem RTCM korekcí z mobilu
#pragma once
#include <Arduino.h>

namespace btlink {

struct Stats {
  bool connected = false;
  uint32_t connects = 0;      // počet připojení
  uint32_t txBytes = 0;       // odeslané NMEA
  uint32_t txDropped = 0;     // dávky zahozené (klient nestíhá)
  uint32_t rxBytes = 0;       // přijato z mobilu (předáno X20)
  uint32_t rtcmOk = 0;        // celé zprávy RTCM3 se správným CRC
  uint32_t rtcmBad = 0;       // chybné CRC
  uint16_t rtcmLastType = 0;  // typ poslední zprávy
  uint32_t rtcmLastMs = 0;    // millis() poslední správné zprávy
  uint32_t pairOk = 0;        // úspěšná spárování
  uint32_t pairFail = 0;      // neúspěšná spárování
  uint32_t cmds = 0;          // přijaté příkazy JSON
  uint32_t backlogMax = 0;    // největší nepotvrzená data v zásobníku BT [B]
  uint32_t ubxOk = 0;         // celé rámce UBX z mobilu (asistence MGA) se správným součtem
  uint32_t ubxBad = 0;        // rámce UBX z mobilu s chybným součtem nebo délkou
};

// Příkaz JSON z mobilu: řádek začínající '{' mimo rámec RTCM a UBX, bez \n
using CmdHandler = void (*)(const char *line, size_t len);
void setCmdHandler(CmdHandler h);
bool connected();                   // BT běží a klient je připojen
uint32_t backlog();                 // odeslaná, ale zásobníkem BT dosud nepotvrzená data [B]

// Událost pro výpis do konzole (vrací a nuluje poslední událost)
enum class Event { None, Connected, Disconnected, PairOk, PairFail };
Event takeEvent();

// Výchozí jméno ROVER-xxxx (z MAC) – stejné používá i Wi-Fi AP
const char *defaultName();
bool begin(const char *customName); // spustí SPP (jen BT Classic, paměť BLE uvolněna); prázdné jméno = výchozí
void end();                         // vypne BT (záloha, když se vedle BT nevejde Wi-Fi AP)
bool active();
const char *name();
void poll();                        // příjem z mobilu -> X20
void send(const char *s, size_t n); // přidá do dávky
void flush();                       // odešle dávku (jen při připojeném klientovi)
// Odešle hned, bez limitu fronty BT_MAX_BACKLOG (výpis databáze X20 – nesmí se zahodit část)
void sendNow(const uint8_t *d, size_t n);
Stats stats();

}  // namespace btlink
