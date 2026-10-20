// Zásobník epoch při výpadku BT během záznamu v aplikaci RoverGnss
//
// Aplikace při záznamu trasy / měření bodu pošle {"cmd":"rec","op":"start"} (platí i po restartu roveru).
// Když pak BT vypadne, epochy ($PRVR,POS binárně) se ukládají do LittleFS (/buf.bin, /buf.old – dvě
// poloviny kruhového zásobníku, při zaplnění se zahodí starší polovina). Posledních 32 epoch před výpadkem
// se uloží také (rover ztrátu spojení pozná se zpožděním). Po připojení aplikace pošle {"cmd":"buf","op":"get"},
// rover je pošle jako $PRVR,BUF,… (stejná pole jako POS) a ukončí $PRVR,BUFEND,počet; aplikace pak
// pošle {"cmd":"buf","op":"clear"}. Duplicity odfiltruje aplikace podle času epochy.
#pragma once
#include <Arduino.h>

#include "proto.h"

namespace ebuf {

bool begin();                        // po LittleFS.begin; načte příznak záznamu
void setRecording(bool on);          // vypnutí zásobník smaže
bool recording();
void onEpoch(const proto::PosRec &r, bool connected);  // každá epocha
void poll();                         // ukládání na flash, odesílání zásobníku
bool startReplay();                  // false = zásobník je prázdný
bool replaying();
void clear();
uint32_t count();                    // uložených epoch
uint32_t capacity();                 // nejvýš epoch (obě poloviny)
uint32_t dropped();                  // zahozených při zaplnění (od startu)

}  // namespace ebuf
