#include "epochbuf.h"
#include "console.h"

#include <LittleFS.h>
#include <Preferences.h>

#include "btlink.h"
#include "config.h"

namespace ebuf {

static const char *F_NEW = "/buf.bin";
static const char *F_OLD = "/buf.old";
static constexpr uint32_t RESERVE = 128 * 1024;  // volné místo pro nastavení, body a měření
static constexpr int HIST = 32;                  // epochy před zjištěným výpadkem
static constexpr uint32_t FLUSH_MS = 2000;
static constexpr int REPLAY_BATCH = 6;           // epoch na jedno kolo smyčky

static const size_t REC = sizeof(proto::PosRec);

static bool rec = false;
static uint32_t half = 0;                        // velikost jedné poloviny [B]
static File wf;                                  // zápis (/buf.bin)
static uint32_t lastFlushMs = 0;
static bool lastConnected = true;
static uint32_t droppedN = 0;

static proto::PosRec hist[HIST];
static int histN = 0, histPos = 0;

static File rf;                                  // čtení při odesílání
static int rfStage = 0;                          // 0 = /buf.old, 1 = /buf.bin
static bool replayOn = false;
static uint32_t replayN = 0;

static uint32_t fileRecs(const char *path) {
  if (!LittleFS.exists(path)) return 0;
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  uint32_t n = f.size() / REC;
  f.close();
  return n;
}

bool begin() {
  Preferences p;
  if (p.begin("ebuf", true)) {
    rec = p.getBool("rec", false);
    p.end();
  }
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  // zásobník smí zabrat to, co je teď volné, mimo rezervu (už uložený zásobník se počítá jako volný)
  size_t own = fileRecs(F_NEW) * REC + fileRecs(F_OLD) * REC;
  size_t avail = total > used + RESERVE ? total - used - RESERVE + own : 0;
  half = (uint32_t)(avail / 2 / REC) * REC;
  if (half < 64 * REC) half = 64 * REC;
  Con.printf("[buf] zásobník epoch: %lu epoch (%lu kB), záznam %s, uloženo %lu\n", (unsigned long)capacity(),
                (unsigned long)(2 * half / 1024), rec ? "ZAP" : "VYP", (unsigned long)count());
  return true;
}

static void closeWriter() {
  if (wf) {
    wf.flush();
    wf.close();
  }
}

void clear() {
  closeWriter();
  if (rf) rf.close();
  replayOn = false;
  LittleFS.remove(F_NEW);
  LittleFS.remove(F_OLD);
}

void setRecording(bool on) {
  if (on != rec) {
    rec = on;
    Preferences p;
    if (p.begin("ebuf", false)) {
      p.putBool("rec", on);
      p.end();
    }
    Con.printf("[buf] záznam aplikace %s\n", on ? "ZAP – při výpadku BT se epochy ukládají" : "VYP");
  }
  if (!on) clear();
}

bool recording() { return rec; }

static void writeRec(const proto::PosRec &r) {
  if (!wf) {
    wf = LittleFS.open(F_NEW, "a");
    if (!wf) return;
  }
  if (wf.size() + REC > half) {
    // plná polovina: starší polovina se zahodí, aktuální se stane starší
    closeWriter();
    droppedN += fileRecs(F_OLD);
    LittleFS.remove(F_OLD);
    LittleFS.rename(F_NEW, F_OLD);
    wf = LittleFS.open(F_NEW, "w");
    if (!wf) return;
  }
  wf.write((const uint8_t *)&r, REC);
}

void onEpoch(const proto::PosRec &r, bool connected) {
  if (!rec) {
    lastConnected = connected;
    histN = 0;
    return;
  }
  if (connected) {
    hist[histPos] = r;
    histPos = (histPos + 1) % HIST;
    if (histN < HIST) histN++;
  } else {
    if (lastConnected) {
      // výpadek: i poslední epochy „odeslané“ před zjištěním odpojení
      for (int i = 0; i < histN; i++) writeRec(hist[(histPos - histN + i + HIST) % HIST]);
      histN = 0;
      Con.println("[buf] výpadek BT – ukládám epochy");
    }
    writeRec(r);
  }
  lastConnected = connected;
}

bool startReplay() {
  closeWriter();
  if (rf) rf.close();
  replayN = 0;
  rfStage = 0;
  if (!count()) return false;
  replayOn = true;
  return true;
}

bool replaying() { return replayOn; }

static bool openStage() {
  const char *path = rfStage == 0 ? F_OLD : F_NEW;
  if (!LittleFS.exists(path)) return false;
  rf = LittleFS.open(path, "r");
  return (bool)rf;
}

void poll() {
  if (wf && millis() - lastFlushMs >= FLUSH_MS) {
    lastFlushMs = millis();
    wf.flush();
  }
  if (!replayOn) return;
  if (!btlink::connected()) {
    if (rf) rf.close();
    replayOn = false;  // zásobník zůstává, aplikace si ho vyžádá znovu
    return;
  }
  if (btlink::backlog() > BT_MAX_BACKLOG / 2) return;
  for (int k = 0; k < REPLAY_BATCH; k++) {
    proto::PosRec r;
    if (!rf && !openStage()) {
      if (rfStage == 0) {
        rfStage = 1;
        if (openStage()) continue;
      }
      proto::sendBufEnd(replayN);
      Con.printf("[buf] odesláno %lu epoch ze zásobníku\n", (unsigned long)replayN);
      replayOn = false;
      break;
    }
    if (rf.read((uint8_t *)&r, REC) == REC) {
      proto::sendRec(r, true);
      replayN++;
      continue;
    }
    rf.close();
    if (rfStage == 0) {
      rfStage = 1;
      if (openStage()) continue;
    }
    rfStage = 2;  // konec obou souborů
    proto::sendBufEnd(replayN);
    Con.printf("[buf] odesláno %lu epoch ze zásobníku\n", (unsigned long)replayN);
    replayOn = false;
    break;
  }
  btlink::flush();
}

uint32_t count() {
  uint32_t n = fileRecs(F_OLD);
  if (wf) n += wf.size() / REC;
  else n += fileRecs(F_NEW);
  return n;
}

uint32_t capacity() { return 2 * half / REC; }

uint32_t dropped() { return droppedN; }

}  // namespace ebuf
