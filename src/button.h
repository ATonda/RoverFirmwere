// Tlačítko (GPIO13 proti GND): krátký stisk, podržení 2 s, podržení 5 s
#pragma once
#include <Arduino.h>

namespace button {

enum class Event { None, Short, Long, Exit };  // Long = 2 s, Exit = 5 s (při stejném stisku po Long)

void begin();
Event poll();        // volat často z loop
bool pressed();      // aktuální stav po odrušení
void resetEvents();  // zapomenout rozpracovaný stisk (např. po průchozím režimu)

}  // namespace button
