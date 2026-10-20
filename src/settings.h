// Uživatelské profily a nastavení přístroje (LittleFS, JSON)
// Stejný formát JSON používá web a později i BLE konfigurace.
#pragma once
#include <Arduino.h>

namespace settings {

// Dynamický model X20
enum class Dyn : uint8_t { Auto, Pedestrian, Automotive, Stationary };

struct Profile {
  char name[25] = "Profil";
  uint8_t rateHz = 5;          // 1 / 5 / 10
  Dyn dyn = Dyn::Pedestrian;
  int8_t elevMask = 10;        // [°] 0–60
  uint8_t nmea = 0x1F;         // maska vět (nmea::GGA | …)
  bool ekf = false;            // krok 6
  bool buzzer = true;          // krok 4
  uint8_t volume = 50;         // [%] krok 4
  uint8_t led = 30;            // jas [%] krok 4
  float antH = 0;              // výška antény: značka -> ARP [m]
  float pco = 0;               // posun fázového centra: ARP -> fázové centrum [m]
};

struct Device {
  char apPass[33] = "";        // heslo Wi-Fi AP (WPA2, 8–32 znaků)
  char btName[25] = "";        // prázdné = ROVER-xxxx
};

bool begin();                  // připojí LittleFS (při chybě naformátuje), načte soubory

int count();
const Profile &profile(int i);
int active();                  // aktivní profil (po startu = výchozí)
int defaultIndex();
void setActive(int i);
int next();                    // další profil cyklicky, vrací nový index

uint8_t dynCode(Dyn d);        // kód u-blox pro CFG-NAVSPG-DYNMODEL
const char *dynName(Dyn d);

// Profily jako JSON: {"active":0,"default":0,"profiles":[{…}]}
String profilesJson();
// Uloží celý seznam profilů (z webu / importu). Při chybě nic nemění a vrátí popis do err.
bool setProfilesJson(const char *json, String &err);

const Device &device();
String deviceJson();
bool setDeviceJson(const char *json, String &err);

}  // namespace settings
