// Test přesnosti: kontrolní body, odchylka od bodu, automatický test doby do fixu
// Výpočty běží v ESP – test pokračuje i bez otevřené stránky.
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

#include "gnss.h"

namespace testctl {

static constexpr int MAX_POINTS = 30;
static constexpr int MAX_RUNS = 30;

enum class Frame { Auto, Etrs, Itrf };          // v jakém systému je poloha z přijímače
enum class Goal { Fix3D, Corr, Float, RtkFix, Acc };

// Kvalita polohy bodu
enum PosQ : uint8_t { PQ_EXACT = 0, PQ_APPROX = 1, PQ_NONE = 2 };  // přesná / přibližná / nevyhodnocovat

struct TtffCfg {
  gnss::ResetKind kind = gnss::ResetKind::Cold;
  int count = 5;          // počet opakování
  int timeoutS = 300;     // limit jednoho pokusu
  int pauseS = 10;        // pauza mezi pokusy
  Goal goal = Goal::Fix3D;
  float accM = 1.0f;      // přesnost pro cíl Acc (vůči bodu, bez bodu podle hAcc)
};

// Výsledek porovnání polohy s bodem
enum DiffFlags : uint8_t {
  DF_HOR = 1,      // vodorovná odchylka platná
  DF_U_ELL = 2,    // ΔU z elipsoidické výšky bodu
  DF_U_BPV = 4,    // ΔU z Bpv (h_msl přijímače – geoid u-blox)
  DF_TR = 8,       // poloha přijímače přepočtena ITRF -> ETRS89
  DF_APPROX = 16,  // poloha bodu přibližná
};
struct Diff {
  double e = 0, n = 0, u = 0, d2 = 0;
  uint8_t flags = 0;
};

bool begin();                                    // načte body z LittleFS
String pointsJson();                             // {"sel":0,"frame":"auto","points":[…]}
bool setPointsJson(const char *json, String &err);

int selected();                                  // vybraný bod, -1 = žádný
int pointCount();
const char *pointName(int i);
// Má se poloha přijímače přepočítat ITRF -> ETRS89? (rtcm = přicházejí korekce RTCM ze sítě v ČR = ETRS89;
// bez nich je i řešení s fázemi HAS/PPP v ITRF)
bool needsTransform(bool rtcm);
// Poloha přijímače v systému bodů (případně přepočtená do ETRS89), výšky beze změny
void toPointFrame(double &lat, double &lon, double hEll, bool rtk, double epoch);
// Porovnání polohy přijímače (fázové centrum antény) s bodem i; anténa z aktivního profilu
bool compare(int i, double lat, double lon, double hEll, double hMSL, bool rtk, double epoch, Diff &d);
// Nový bod (ze změřené polohy, už v systému bodů). Vrací index, -1 = chyba.
int addPoint(const char *name, double lat, double lon, double hEll, const char *note, String &err);

void onPvt();                                    // volat po každém NAV-PVT
void poll();                                     // volat z loop (časové limity testu)

bool ttffStart(const TtffCfg &c, String &err);
void ttffStop();
void ttffClear();
bool ttffRunning();

void statusJson(JsonDocument &doc);              // stav pro /api/test

}  // namespace testctl
