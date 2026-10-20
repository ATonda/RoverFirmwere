// Měření bodu: odpočet -> ustálení -> sběr 1 Hz -> souhrn (průměr, σ, CEP) -> uložení
// Běží v ESP i se zhasnutým mobilem. Souhrny v LittleFS (/meas.json), vzorky posledního
// měření v RAM do dalšího startu (export CSV).
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

#include "imu.h"

namespace survey {

static constexpr int MAX_MEAS = 30;  // uložených souhrnů (nejstarší se přepisuje)

struct Cfg {
  uint16_t countdownS = 15;   // odpočet na odstoupení 0–300 s
  uint16_t settleMaxS = 900;  // nejdelší čekání na ustálení 0–3600 s (0 = jen čekat na fix)
  float settleAcc = 0;        // [m] hAcc musí klesnout pod mez; 0 = jen stabilita polohy
  uint16_t durS = 300;        // doba sběru 10–7200 s
  char point[25] = "";        // existující bod (porovnání), prázdné = bez bodu
  char newName[25] = "";      // neprázdné = z výsledku se založí nový bod
};

bool begin();                 // načte uložené souhrny
bool start(const Cfg &c, String &err);
void finish();                // ukončit sběr předčasně a výsledek uložit
void cancel();                // zrušit bez uložení
void onPvt();                 // volat po každém NAV-PVT
void onImu(const imu::Sample &s);  // každý vzorek IMU – pohyb přístroje
void poll();                  // volat z loop
bool running();               // odpočet, ustálení nebo sběr
int phase();                  // 0 nic, 1 odpočet, 2 ustálení, 3 sběr, 4 hotovo, 5 selhalo
uint32_t phaseSeconds();      // doba v aktuální fázi [s] (0 = nic)

void statusJson(JsonDocument &doc);  // stav měření (bez seznamu)
void listJson(JsonDocument &doc);    // uložené souhrny
uint32_t listVersion();              // mění se při každé změně seznamu
bool remove(int id, String &err);
bool removePoint(const char *point, String &err);  // všechna měření bodu

int rawCount();               // vzorky posledního měření
size_t rawLine(int i, char *buf, size_t len);  // i = -1 hlavička CSV

}  // namespace survey
