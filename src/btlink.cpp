#include "btlink.h"

#include <BluetoothSerial.h>
#include <esp_mac.h>

#include "config.h"
#include "gnss.h"

namespace btlink {

// BluetoothSerial (SPP) je v jádru 3.x označený jako zastaralý pro 4.0 – verze platformy je připnutá
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
static BluetoothSerial bt;
#pragma GCC diagnostic pop
static char btName[25] = "ROVER";
static Stats st;
static bool started = false;
static volatile Event pendingEvent = Event::None;

// ---------------------------------------------------------------- RTCM3 (jen statistika, data jdou do X20 beze změny)
static uint32_t crc24q(const uint8_t *d, size_t n) {
  uint32_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint32_t)d[i] << 16;
    for (int b = 0; b < 8; b++) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

static uint8_t rtcmBuf[1029];  // 3 hlavička + 1023 data + 3 CRC
static uint16_t rtcmPos = 0, rtcmLen = 0;

// ---------------------------------------------------------------- UBX z mobilu (asistence MGA)
// Rámec jde do X20 beze změny; uvnitř se nehledá začátek JSON (bajt 0x7B v datech by rámec rozbil).
static uint16_t ubxPos = 0, ubxLen = 0;
static uint8_t ubxCkA = 0, ubxCkB = 0, ubxRxA = 0;

// Vrací false, když bajt nepatří do rámce UBX (pak se zpracuje jako RTCM)
static bool ubxFeed(uint8_t c) {
  if (ubxPos == 0) {
    if (c != 0xB5) return false;
    ubxPos = 1;
    return true;
  }
  if (ubxPos == 1) {
    if (c != 0x62) {
      ubxPos = 0;
      return false;
    }
    ubxPos = 2;
    ubxCkA = ubxCkB = 0;
    return true;
  }
  uint16_t i = ubxPos++;  // 2 třída, 3 ID, 4–5 délka, pak data, 2 B součet
  if (i < 6 + ubxLen || i < 6) {
    ubxCkA += c;
    ubxCkB += ubxCkA;
    if (i == 4) ubxLen = c;
    if (i == 5) {
      ubxLen |= (uint16_t)c << 8;
      if (ubxLen > 2048) {  // nesmysl – nebyl to rámec
        st.ubxBad++;
        ubxPos = 0;
        ubxLen = 0;
      }
    }
    return true;
  }
  if (i == 6 + ubxLen) {
    ubxRxA = c;
    return true;
  }
  if (ubxRxA == ubxCkA && c == ubxCkB) st.ubxOk++;
  else st.ubxBad++;
  ubxPos = 0;
  ubxLen = 0;
  return true;
}

// Řádek příkazu JSON z mobilu (set profiles s 10 profily má ~2,5 kB)
static char cmdBuf[4096];
static size_t cmdLen = 0;
static bool cmdOverflow = false;
static CmdHandler cmdHandler = nullptr;

static void rtcmFeed(uint8_t c) {
  if (rtcmPos == 0) {
    if (c != 0xD3) return;
    rtcmBuf[rtcmPos++] = c;
    return;
  }
  rtcmBuf[rtcmPos++] = c;
  if (rtcmPos == 3) {
    rtcmLen = ((rtcmBuf[1] & 0x03) << 8) | rtcmBuf[2];
    if (rtcmBuf[1] & 0xFC) {  // rezervované bity musí být 0 – není to začátek zprávy
      rtcmPos = 0;
      return;
    }
  }
  if (rtcmPos >= 3 && rtcmPos == rtcmLen + 6) {
    uint32_t crc = (rtcmBuf[rtcmLen + 3] << 16) | (rtcmBuf[rtcmLen + 4] << 8) | rtcmBuf[rtcmLen + 5];
    if (crc24q(rtcmBuf, rtcmLen + 3) == crc) {
      st.rtcmOk++;
      st.rtcmLastMs = millis();
      if (rtcmLen >= 2) st.rtcmLastType = (rtcmBuf[3] << 4) | (rtcmBuf[4] >> 4);
    } else {
      st.rtcmBad++;
    }
    rtcmPos = 0;
  }
}

// ---------------------------------------------------------------- fronta odesílání
// write() data jen zařadí do fronty (až 32 dávek); když telefon nečte, fronta roste a zpoždění
// narůstá na desítky sekund. Počítá se, kolik bajtů odešlo a kolik zásobník BT potvrdil (WRITE_EVT).
static volatile uint32_t txQueued = 0;   // předáno bt.write
static volatile uint32_t txAcked = 0;    // potvrzeno ESP_SPP_WRITE_EVT
static volatile uint32_t lastAckMs = 0;

static void sppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_WRITE_EVT && param->write.status == ESP_SPP_SUCCESS) {
    txAcked += param->write.len;
    lastAckMs = millis();
  } else if (event == ESP_SPP_SRV_OPEN_EVT || event == ESP_SPP_CLOSE_EVT) {
    txQueued = 0;
    txAcked = 0;
  }
}

uint32_t backlog() {
  uint32_t q = txQueued, a = txAcked;
  return q > a ? q - a : 0;
}

// ---------------------------------------------------------------- veřejné API
static char defName[16] = "";

const char *defaultName() {
  if (!defName[0]) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(defName, sizeof(defName), "ROVER-%02X%02X", mac[4], mac[5]);
  }
  return defName;
}

bool begin(const char *customName) {
  strlcpy(btName, (customName && customName[0]) ? customName : defaultName(), sizeof(btName));
  // Párování „Just Works“ bez potvrzování čísla (přístroj nemá displej ani klávesnici).
  // Bez toho Android při párování čeká na potvrzení, které ESP nemá komu předat, a spojení SPP se nenaváže.
  bt.enableSSP(false, false);
  bt.onAuthComplete([](boolean success) {
    if (success) st.pairOk++;
    else st.pairFail++;
    pendingEvent = success ? Event::PairOk : Event::PairFail;
  });
  // Potvrzení odeslaných dat (WRITE_EVT) – kolik bajtů ještě čeká v zásobníku BT
  bt.register_callback(sppEvent);
  // Jen BT Classic: paměť řadiče pro BLE se uvolní. BLE pak do restartu nejde zapnout.
  started = bt.begin(btName, false, true);
  return started;
}

void end() {
  if (!started) return;
  bt.end();
  started = false;
  st.connected = false;
}

const char *name() { return btName; }

bool active() { return started; }

void poll() {
  if (!started) return;
  bool c = bt.hasClient();
  if (c && !st.connected) {
    st.connects++;
    pendingEvent = Event::Connected;
  } else if (!c && st.connected) {
    pendingEvent = Event::Disconnected;
  }
  st.connected = c;

  uint8_t buf[256];
  uint8_t fwd[256];  // bajty pro X20 (vše kromě řádků JSON)
  int n;
  while ((n = bt.available()) > 0) {
    n = bt.readBytes(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
    if (n <= 0) break;
    st.rxBytes += n;
    int f = 0;
    for (int i = 0; i < n; i++) {
      uint8_t c = buf[i];
      // Řádek JSON začíná '{' mimo rámec RTCM a UBX a končí \n; do X20 nejde
      if (cmdLen == 0 && rtcmPos == 0 && ubxPos == 0 && c == '{') {
        cmdBuf[cmdLen++] = c;
        continue;
      }
      if (cmdLen > 0) {
        if (c == '\n' || c == '\r') {
          if (f) {
            gnss::writeRaw(fwd, f);
            f = 0;
          }
          if (!cmdOverflow && cmdHandler) {
            cmdBuf[cmdLen] = 0;
            st.cmds++;
            cmdHandler(cmdBuf, cmdLen);
          }
          cmdLen = 0;
          cmdOverflow = false;
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
          cmdBuf[cmdLen++] = c;
        } else {
          cmdOverflow = true;  // příliš dlouhý řádek – zahodit až do konce řádku
        }
        continue;
      }
      fwd[f++] = c;
      if (rtcmPos != 0 || !ubxFeed(c)) rtcmFeed(c);
    }
    if (f) gnss::writeRaw(fwd, f);
  }
}

bool connected() { return started && st.connected; }

static uint32_t lastResetMs = 0;

void setCmdHandler(CmdHandler h) { cmdHandler = h; }

// Věty jedné epochy se skládají do dávky a odesílají jedním zápisem.
// Když v zásobníku BT čeká víc než BT_MAX_BACKLOG bajtů (telefon nestíhá nebo je spojení přerušované),
// dávka se zahodí – po obnovení přijdou nejnovější data místo staré fronty.
static char txBuf[2048];
static size_t txLen = 0;

void send(const char *s, size_t n) {
  if (txLen + n > sizeof(txBuf)) {
    st.txDropped++;
    return;
  }
  memcpy(txBuf + txLen, s, n);
  txLen += n;
}

void flush() {
  if (!txLen) return;
  size_t n = txLen;
  txLen = 0;
  if (!started || !st.connected) return;
  uint32_t b = backlog();
  if (b > BT_MAX_BACKLOG) {
    // Zásobník BT může data při chybě zahodit bez potvrzení – po 10 s bez potvrzení se počítadlo vynuluje
    if (millis() - lastAckMs > 10000 && millis() - lastResetMs > 10000) {
      txAcked = txQueued;
      lastResetMs = millis();
    }
    st.txDropped++;
    return;
  }
  if (b > st.backlogMax) st.backlogMax = b;
  // Zásobník BT při zahlcení část dat zahodí bez WRITE_EVT – počítadlo by pak trvale ukazovalo zbytek.
  // Když fronta 5 s ani jednou neklesne na nulu a potvrzení přitom chodí, nejmenší zbytek je ztracený.
  static uint32_t winStartMs = 0, winMin = UINT32_MAX;
  uint32_t now = millis();
  if (b < winMin) winMin = b;
  if (now - winStartMs >= 5000) {
    if (winMin != UINT32_MAX && winMin > 0 && now - lastAckMs < 1000) {
      txAcked += winMin;
    }
    winStartMs = now;
    winMin = UINT32_MAX;
  }
  bt.write((const uint8_t *)txBuf, n);
  txQueued += n;
  st.txBytes += n;
}

void sendNow(const uint8_t *d, size_t n) {
  if (!started || !st.connected) return;
  flush();
  bt.write(d, n);
  txQueued += n;
  st.txBytes += n;
}

Stats stats() { return st; }

Event takeEvent() {
  Event e = pendingEvent;
  pendingEvent = Event::None;
  return e;
}

}  // namespace btlink
