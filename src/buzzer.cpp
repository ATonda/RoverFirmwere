#include "buzzer.h"

#include "config.h"

namespace buzzer {

static bool attached = false;
static bool enabled = true;

struct Step {
  uint16_t freq;  // 0 = ticho
  uint16_t ms;
};
static Step seq[16];
static uint8_t nSeq = 0, iSeq = 0;
static uint32_t stepStartMs = 0;

void begin() {
  if (attached) return;
  attached = ledcAttach(PIN_BUZZER, 2000, 10);  // 10 bit, frekvence se mění při start()
  ledcWriteTone(PIN_BUZZER, 0);
}

void start(uint32_t freqHz) {
  begin();
  if (attached) ledcWriteTone(PIN_BUZZER, freqHz);  // střída 50 %
}

void stop() {
  if (attached) ledcWriteTone(PIN_BUZZER, 0);
}

void setEnabled(bool on) { enabled = on; }

static void play() {
  iSeq = 0;
  stepStartMs = millis();
  if (seq[0].freq) start(seq[0].freq);
  else stop();
}

void beep(uint16_t freqHz, uint16_t onMs, uint16_t offMs, uint8_t count) {
  if (!enabled) return;
  nSeq = 0;
  for (uint8_t i = 0; i < count && nSeq + 2 <= 16; i++) {
    seq[nSeq++] = {freqHz, onMs};
    seq[nSeq++] = {0, offMs};
  }
  if (nSeq) play();
}

void tones2(uint16_t f1, uint16_t f2) {
  if (!enabled) return;
  seq[0] = {f1, 100};
  seq[1] = {0, 60};
  seq[2] = {f2, 150};
  seq[3] = {0, 10};
  nSeq = 4;
  play();
}

void poll() {
  if (iSeq >= nSeq) return;
  uint32_t now = millis();
  if (now - stepStartMs < seq[iSeq].ms) return;
  iSeq++;
  stepStartMs = now;
  if (iSeq >= nSeq) {
    stop();
    nSeq = iSeq = 0;
    return;
  }
  if (seq[iSeq].freq) start(seq[iSeq].freq);
  else stop();
}

bool busy() { return iSeq < nSeq; }

}  // namespace buzzer
