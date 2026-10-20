// Tlačítko (GPIO13 proti GND): krátký stisk = další profil, podržení 5 s = restart ESP
#pragma once
#include <Arduino.h>

namespace button {

enum class Event { None, Short, Long };  // Long = podržení BTN_LONG_MS (hlásí se hned při dosažení)

void begin();
Event poll();        // volat často z loop
bool pressed();      // aktuální stav po odrušení
void resetEvents();  // zapomenout rozpracovaný stisk (např. po průchozím režimu)

}  // namespace button
