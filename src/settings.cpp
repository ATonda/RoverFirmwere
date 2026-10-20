#include "settings.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "config.h"
#include "nmea.h"

namespace settings {

static const char *PROFILES_FILE = "/profiles.json";
static const char *DEVICE_FILE = "/device.json";

static Profile profiles[PROFILES_MAX];
static int nProfiles = 0;
static int activeIdx = 0;
static int defIdx = 0;
static Device dev;

// ---------------------------------------------------------------- převody
static const char *const DYN_NAMES[] = {"auto", "pedestrian", "automotive", "stationary"};

const char *dynName(Dyn d) { return DYN_NAMES[(int)d]; }

uint8_t dynCode(Dyn d) {
  switch (d) {
    case Dyn::Automotive: return 4;
    case Dyn::Stationary: return 2;
    case Dyn::Auto:  // start jako chodec, dál přepíná modul motion podle stavu
    case Dyn::Pedestrian:
    default: return 3;
  }
}

static bool dynFromName(const char *s, Dyn &d) {
  for (int i = 0; i < 4; i++) {
    if (s && strcmp(s, DYN_NAMES[i]) == 0) {
      d = (Dyn)i;
      return true;
    }
  }
  return false;
}

static void toJson(const Profile &p, JsonObject o) {
  o["name"] = p.name;
  o["rate"] = p.rateHz;
  o["dyn"] = dynName(p.dyn);
  o["elev"] = p.elevMask;
  JsonObject n = o["nmea"].to<JsonObject>();
  n["gga"] = (p.nmea & nmea::GGA) != 0;
  n["rmc"] = (p.nmea & nmea::RMC) != 0;
  n["gst"] = (p.nmea & nmea::GST) != 0;
  n["gsa"] = (p.nmea & nmea::GSA) != 0;
  n["gsv"] = (p.nmea & nmea::GSV) != 0;
  o["ekf"] = p.ekf;
  o["buzzer"] = p.buzzer;
  o["volume"] = p.volume;
  o["led"] = p.led;
  o["antH"] = serialized(String(p.antH, 3));
  o["pco"] = serialized(String(p.pco, 3));
  o["sbas"] = p.sbas;
  o["has"] = p.has;
}

// Kontrola a převod jednoho profilu; chybějící položky dostanou výchozí hodnotu
static bool fromJson(JsonObjectConst o, Profile &p, String &err) {
  p = Profile();
  const char *name = o["name"] | "";
  size_t nl = strlen(name);
  if (nl == 0 || nl >= sizeof(p.name)) {
    err = "jméno profilu musí mít 1–24 znaků";
    return false;
  }
  strlcpy(p.name, name, sizeof(p.name));
  int rate = o["rate"] | 5;
  if (rate != 1 && rate != 5 && rate != 10) {
    err = String("profil ") + name + ": frekvence musí být 1, 5 nebo 10 Hz";
    return false;
  }
  p.rateHz = rate;
  if (!dynFromName(o["dyn"] | "pedestrian", p.dyn)) {
    err = String("profil ") + name + ": neznámý dynamický model";
    return false;
  }
  int elev = o["elev"] | 10;
  if (elev < 0 || elev > 60) {
    err = String("profil ") + name + ": elevační maska 0–60°";
    return false;
  }
  p.elevMask = elev;
  JsonObjectConst n = o["nmea"];
  if (!n.isNull()) {
    p.nmea = 0;
    if (n["gga"] | false) p.nmea |= nmea::GGA;
    if (n["rmc"] | false) p.nmea |= nmea::RMC;
    if (n["gst"] | false) p.nmea |= nmea::GST;
    if (n["gsa"] | false) p.nmea |= nmea::GSA;
    if (n["gsv"] | false) p.nmea |= nmea::GSV;
  }
  p.ekf = o["ekf"] | false;
  p.buzzer = o["buzzer"] | true;
  p.volume = constrain((int)(o["volume"] | 50), 0, 100);
  p.led = constrain((int)(o["led"] | 30), 0, 100);
  float antH = o["antH"] | 0.0f;
  float pco = o["pco"] | 0.0f;
  if (antH < 0 || antH > 10) {
    err = String("profil ") + name + ": výška antény 0–10 m";
    return false;
  }
  if (pco < -0.2f || pco > 0.2f) {
    err = String("profil ") + name + ": fázové centrum ±0,2 m";
    return false;
  }
  p.antH = antH;
  p.pco = pco;
  p.sbas = o["sbas"] | true;
  p.has = o["has"] | true;
  return true;
}

// ---------------------------------------------------------------- soubory
static bool writeFile(const char *path, const String &s) {
  // zápis přes dočasný soubor, aby výpadek napájení nenechal rozepsaný JSON
  String tmp = String(path) + ".tmp";
  File f = LittleFS.open(tmp, "w");
  if (!f) return false;
  size_t n = f.print(s);
  f.close();
  if (n != s.length()) return false;
  LittleFS.remove(path);
  return LittleFS.rename(tmp, path);
}

static void defaultProfiles() {
  nProfiles = 4;
  profiles[0] = Profile();
  strlcpy(profiles[0].name, "Automaticky 5 Hz", sizeof(profiles[0].name));
  profiles[0].dyn = Dyn::Auto;
  profiles[1] = Profile();
  strlcpy(profiles[1].name, "Chodec 5 Hz", sizeof(profiles[1].name));
  profiles[2] = Profile();
  strlcpy(profiles[2].name, "Auto 5 Hz", sizeof(profiles[2].name));
  profiles[2].dyn = Dyn::Automotive;
  profiles[3] = Profile();
  strlcpy(profiles[3].name, "Statika 1 Hz", sizeof(profiles[3].name));
  profiles[3].rateHz = 1;
  profiles[3].dyn = Dyn::Stationary;
  defIdx = 0;
}

static String buildProfilesFile() {
  JsonDocument doc;
  doc["default"] = defIdx;
  JsonArray a = doc["profiles"].to<JsonArray>();
  for (int i = 0; i < nProfiles; i++) toJson(profiles[i], a.add<JsonObject>());
  String s;
  serializeJson(doc, s);
  return s;
}

// Načte seznam profilů z JSON; při chybě nic nezmění
static bool parseProfiles(const char *json, bool takeActive, String &err) {
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, json);
  if (e) {
    err = String("chybný JSON: ") + e.c_str();
    return false;
  }
  JsonArrayConst a = doc["profiles"];
  if (a.isNull() || a.size() == 0) {
    err = "seznam profilů je prázdný";
    return false;
  }
  if ((int)a.size() > PROFILES_MAX) {
    err = String("nejvýš ") + PROFILES_MAX + " profilů";
    return false;
  }
  static Profile tmp[PROFILES_MAX];
  int n = 0;
  for (JsonObjectConst o : a) {
    if (!fromJson(o, tmp[n], err)) return false;
    for (int k = 0; k < n; k++) {
      if (strcmp(tmp[k].name, tmp[n].name) == 0) {
        err = String("dva profily se jmenují ") + tmp[n].name;
        return false;
      }
    }
    n++;
  }
  int d = doc["default"] | 0;
  if (d < 0 || d >= n) d = 0;
  for (int i = 0; i < n; i++) profiles[i] = tmp[i];
  nProfiles = n;
  defIdx = d;
  if (takeActive && doc["active"].is<int>()) activeIdx = doc["active"];
  if (activeIdx < 0 || activeIdx >= nProfiles) activeIdx = defIdx;
  return true;
}

static void defaultDevice() { dev = Device(); }

static bool parseDevice(const char *json, String &err) {
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, json);
  if (e) {
    err = String("chybný JSON: ") + e.c_str();
    return false;
  }
  Device d = dev;
  if (doc["btName"].is<const char *>()) {
    const char *bn = doc["btName"];
    if (strlen(bn) >= sizeof(d.btName)) {
      err = "název Bluetooth nejvýš 24 znaků";
      return false;
    }
    for (const char *c = bn; *c; c++) {
      if ((uint8_t)*c < 32 || *c == ',' || *c == '*' || *c == '$') {
        err = "název Bluetooth: bez čárky, hvězdičky, $ a řídicích znaků";
        return false;
      }
    }
    strlcpy(d.btName, bn, sizeof(d.btName));
  }
  dev = d;
  return true;
}

static String buildDeviceFile() {
  JsonDocument doc;
  doc["btName"] = dev.btName;
  String s;
  serializeJson(doc, s);
  return s;
}

// ---------------------------------------------------------------- veřejné API
bool begin() {
  bool fsOk = LittleFS.begin(true);  // true = při chybě naformátovat
  defaultProfiles();
  defaultDevice();
  if (!fsOk) return false;

  String err;
  File f = LittleFS.open(PROFILES_FILE, "r");
  if (f) {
    String s = f.readString();
    f.close();
    if (!parseProfiles(s.c_str(), false, err)) defaultProfiles();
  } else {
    writeFile(PROFILES_FILE, buildProfilesFile());
  }
  f = LittleFS.open(DEVICE_FILE, "r");
  if (f) {
    String s = f.readString();
    f.close();
    parseDevice(s.c_str(), err);
  }
  activeIdx = defIdx;
  return true;
}

int count() { return nProfiles; }
const Profile &profile(int i) { return profiles[constrain(i, 0, nProfiles - 1)]; }
int active() { return activeIdx; }
int defaultIndex() { return defIdx; }
void setActive(int i) {
  if (i >= 0 && i < nProfiles) activeIdx = i;
}
int next() {
  activeIdx = (activeIdx + 1) % nProfiles;
  return activeIdx;
}

String profilesJson() {
  JsonDocument doc;
  doc["active"] = activeIdx;
  doc["default"] = defIdx;
  doc["max"] = PROFILES_MAX;
  JsonArray a = doc["profiles"].to<JsonArray>();
  for (int i = 0; i < nProfiles; i++) toJson(profiles[i], a.add<JsonObject>());
  String s;
  serializeJson(doc, s);
  return s;
}

bool setProfilesJson(const char *json, String &err) {
  if (!parseProfiles(json, true, err)) return false;
  if (!writeFile(PROFILES_FILE, buildProfilesFile())) {
    err = "zápis do paměti selhal";
    return false;
  }
  return true;
}

const Device &device() { return dev; }

String deviceJson() { return buildDeviceFile(); }

bool setDeviceJson(const char *json, String &err) {
  if (!parseDevice(json, err)) return false;
  if (!writeFile(DEVICE_FILE, buildDeviceFile())) {
    err = "zápis do paměti selhal";
    return false;
  }
  return true;
}

}  // namespace settings
