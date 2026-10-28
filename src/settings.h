// Uživatelské profily a nastavení přístroje (LittleFS, JSON)
// Stejný formát JSON používá aplikace RoverGnss (get/set profiles, get/set device) i konzole USB.
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
  bool sbas = true;            // korekce SBAS (CFG-SBAS-USE_DIFFCORR)
  bool has = true;             // Galileo HAS (CFG-NAVCOR-ENABLE_GAL_HAS)
};

struct Device {
  char btName[25] = "";        // prázdné = ROVER-xxxx (změna platí po restartu)
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
bool dynFromName(const char *s, Dyn &d);  // "auto", "pedestrian", "automotive", "stationary"

// Profily jako JSON: {"active":0,"default":0,"profiles":[{…}]}
String profilesJson();
// Uloží celý seznam profilů (z aplikace / konzole). Při chybě nic nemění a vrátí popis do err.
bool setProfilesJson(const char *json, String &err);

const Device &device();
String deviceJson();
bool setDeviceJson(const char *json, String &err);

}  // namespace settings
