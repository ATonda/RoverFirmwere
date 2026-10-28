// Režim USB host (ESP32-S3) – vstup a výstup na Serial (UART0, CH340K) – FW 0.17.0
// Stav režimu (frekvence, model, heartbeat) řídí proto; tady je jen přenos.
//   Serial -> rover: řádky JSON a rámce RTCM3/UBX (stejný parser jako BT, vlastní stav)
//   rover -> Serial: $PRVR,POS, $PRVR,STA a odpovědi JSON; ostatní výpisy jsou umlčené (Con)
#pragma once
#include <Arduino.h>

#include "rxparse.h"

namespace usblink {

using CmdHandler = void (*)(const char *line, size_t len);
void setCmdHandler(CmdHandler h);

void setActive(bool on);   // zap: konzole umlčená, příjem do parseru; vyp: konzole zpět
bool active();
void poll();               // příjem ze Serial (jen v režimu)
void send(const char *s, size_t n);  // přímo do Serial, i při umlčené konzoli (jen v režimu)

const RxParser::Stats &stats();
uint32_t rtcmFwdMs();      // millis() poslední RTCM předané do X20 (0 = nikdy)

// Ochrana konzole: po konci režimu (S3 může dál posílat RTCM) a po každém binárním bajtu na vstupu konzole
// se 5 s neprovádějí příkazy písmeny (bajty RTCM by je mohly náhodně spustit), řádky JSON ano.
bool consoleGuard();
void noteBinary();

}  // namespace usblink
