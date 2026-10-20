// Piezo (pasivní) na GPIO25 – tón přes LEDC PWM
#pragma once
#include <Arduino.h>

namespace buzzer {

void begin();
void start(uint32_t freqHz);  // pípá, dokud se nezavolá stop()
void stop();

// Signály bez blokování smyčky (volat poll() z loop). Při vypnutém bzučáku v profilu se nehrají.
void setEnabled(bool on);
void beep(uint16_t freqHz, uint16_t onMs, uint16_t offMs, uint8_t count);  // count× stejný tón
void tones2(uint16_t f1, uint16_t f2);  // dva tóny po sobě (např. hotovo)
void poll();
bool busy();

}  // namespace buzzer
