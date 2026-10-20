// Konfigurační režim: Wi-Fi AP (WPA2), captive portal, webové menu. BT běží současně (pokud se vejde do RAM).
#pragma once
#include <Arduino.h>

namespace webui {

enum class Event {
  None,
  Restart,          // změna hesla Wi-Fi / názvu BT, záchranný režim
  ApOff,            // "Uložit a vypnout Wi-Fi" – AP vypnout bez restartu
  ProfilesChanged,  // uložené profily nebo jiný aktivní profil – znovu nastavit X20
  BridgeStart,      // průchozí režim USB <-> X20
  BridgeStop,
  TestMode,         // restart do testovacího režimu (Wi-Fi + BT)
};

// rescue = záchranný režim po opakovaném pádu (jen stav a restart; firmware se nahrává přes USB)
bool begin(bool rescue);
bool active();
void end();                   // vypne AP a web bez restartu (BT a měření běží dál)
bool needsRestart();          // změněno nastavení, které se projeví až restartem
Event poll();                 // volat často z loop
void setBridge(bool active);  // stav průchozího režimu pro stránku
void setTestMode(bool on);    // testovací režim: BT běží, bez časového limitu
const char *ssid();
const char *password();

}  // namespace webui
