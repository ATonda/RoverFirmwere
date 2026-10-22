#include "btlink.h"

#include <BluetoothSerial.h>
#include <esp_mac.h>
#include <freertos/stream_buffer.h>

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

// ---------------------------------------------------------------- příjem z mobilu (0.15.1)
// BluetoothSerial má pevnou frontu příjmu 512 B a při zaplnění zbytek paketu tiše zahodí
// (CZEPOS MSM5 posílá 1–2 kB najednou). Data se proto berou přímo v tasku BT (onData)
// do vlastního bufferu a hlavní smyčka je čte v poll().
static constexpr size_t BT_RX_BUF = 8192;
static StreamBufferHandle_t rxSb = nullptr;

static void onBtData(const uint8_t *d, size_t n) {
  if (!rxSb || !n) return;
  size_t w = xStreamBufferSend(rxSb, d, n, 0);
  if (w < n) st.rxDropped += n - w;
}

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
  // Příjem přímo do vlastního bufferu (fronta BluetoothSerial 512 B přetéká); bez paměti zůstane výchozí fronta
  if (!rxSb) rxSb = xStreamBufferCreate(BT_RX_BUF, 1);
  if (rxSb) bt.onData(onBtData);
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

// Bajty pro X20 (vše kromě řádků JSON) se skládají a posílají po dávkách
static uint8_t fwd[256];
static size_t fwdLen = 0;

static void fwdFlush() {
  if (fwdLen) {
    gnss::writeRaw(fwd, fwdLen);
    fwdLen = 0;
  }
}

static void fwdByte(uint8_t c) {
  fwd[fwdLen++] = c;
  if (fwdLen == sizeof(fwd)) fwdFlush();
  if (rtcmPos != 0 || !ubxFeed(c)) rtcmFeed(c);
}

// Binární bajt uvnitř „řádku JSON“: '{' byl bajt binárních dat (RTCM/UBX po ztrátě synchronizace).
// Pohlcené bajty se vrátí do proudu pro X20, statistika RTCM se podle nich znovu zasynchronizuje.
static void cmdAbort(uint8_t c) {
  st.cmdAborted++;
  size_t n = cmdLen;
  cmdLen = 0;
  cmdOverflow = false;
  for (size_t i = 0; i < n; i++) fwdByte((uint8_t)cmdBuf[i]);
  fwdByte(c);
}

static void rxByte(uint8_t c) {
  // Řádek JSON začíná '{' mimo rámec RTCM a UBX a končí \n; do X20 nejde
  if (cmdLen == 0 && rtcmPos == 0 && ubxPos == 0 && c == '{') {
    cmdBuf[cmdLen++] = c;
    return;
  }
  if (cmdLen > 0) {
    if (c == '\n' || c == '\r') {
      fwdFlush();
      if (!cmdOverflow && cmdHandler) {
        cmdBuf[cmdLen] = 0;
        st.cmds++;
        cmdHandler(cmdBuf, cmdLen);
      }
      cmdLen = 0;
      cmdOverflow = false;
    } else if (c < 0x20 && c != '\t') {
      cmdAbort(c);  // řídicí bajt v JSON být nemůže (UTF-8 nad 0x7F je povolené)
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = true;  // příliš dlouhý řádek – zahodit až do konce řádku
    }
    return;
  }
  fwdByte(c);
}

void poll() {
  if (!started) return;
  bool c = bt.hasClient();
  if (c && !st.connected) {
    st.connects++;
    pendingEvent = Event::Connected;
    // nové spojení: rozpracované rámce a řádek z minulého spojení zahodit
    cmdLen = 0;
    cmdOverflow = false;
    rtcmPos = 0;
    ubxPos = 0;
    ubxLen = 0;
  } else if (!c && st.connected) {
    pendingEvent = Event::Disconnected;
  }
  st.connected = c;

  uint8_t buf[256];
  if (rxSb) {
    size_t fill = xStreamBufferBytesAvailable(rxSb);
    if (fill > st.rxMaxFill) st.rxMaxFill = fill;
    size_t n;
    int rounds = 0;  // nejvýš 8 kB za jedno volání, ať smyčka nestojí
    while (rounds++ < 32 && (n = xStreamBufferReceive(rxSb, buf, sizeof(buf), 0)) > 0) {
      st.rxBytes += n;
      for (size_t i = 0; i < n; i++) rxByte(buf[i]);
    }
  } else {
    int n;
    while ((n = bt.available()) > 0) {
      n = bt.readBytes(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
      if (n <= 0) break;
      st.rxBytes += n;
      for (int i = 0; i < n; i++) rxByte(buf[i]);
    }
  }
  fwdFlush();
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
