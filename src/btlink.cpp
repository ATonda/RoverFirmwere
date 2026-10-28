#include "btlink.h"

#include <BluetoothSerial.h>
#include <esp_mac.h>
#include <freertos/stream_buffer.h>

#include "config.h"
#include "gnss.h"
#include "rxparse.h"

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

// ---------------------------------------------------------------- příjem z mobilu (FW 0.17.0)
// Řádky JSON, rámce RTCM3 a UBX (asistence MGA) – do X20 jdou jen celé platné rámce (rxparse).
// Řádek příkazu JSON z mobilu: set profiles s 10 profily má ~2,5 kB.
static uint8_t pbuf[4096], prep[4096];
static RxParser parser(pbuf, prep, sizeof(pbuf), 2048);
static CmdHandler cmdHandler = nullptr;
static bool rtcmBlocked = false;   // režim USB host: RTCM z USB má přednost, z BT se zahazuje
static uint32_t fwdMs = 0;         // poslední RTCM předané do X20
static uint32_t rtcmDropped = 0;

static void onFrame(const uint8_t *f, size_t n, bool rtcm) {
  if (rtcm && rtcmBlocked) {
    rtcmDropped++;
    return;
  }
  gnss::writeRaw(f, n);  // celý rámec najednou
  if (rtcm) fwdMs = millis();
}

static void onCmd(const char *line, size_t len) {
  if (cmdHandler) cmdHandler(line, len);
}

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
  parser.setHandlers(onCmd, onFrame);
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

void poll() {
  if (!started) return;
  bool c = bt.hasClient();
  if (c && !st.connected) {
    st.connects++;
    pendingEvent = Event::Connected;
    // nové spojení: rozpracované rámce a řádek z minulého spojení zahodit
    parser.reset();
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
      for (size_t i = 0; i < n; i++) parser.put(buf[i]);
    }
  } else {
    int n;
    while ((n = bt.available()) > 0) {
      n = bt.readBytes(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
      if (n <= 0) break;
      st.rxBytes += n;
      for (int i = 0; i < n; i++) parser.put(buf[i]);
    }
  }
}

bool connected() { return started && st.connected; }

static uint32_t lastResetMs = 0;

void setCmdHandler(CmdHandler h) { cmdHandler = h; }

void setRtcmBlocked(bool b) { rtcmBlocked = b; }
uint32_t rtcmFwdMs() { return fwdMs; }

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

Stats stats() {
  Stats r = st;
  const RxParser::Stats &p = parser.stats();
  r.rtcmOk = p.rtcmOk;
  r.rtcmBad = p.rtcmBad;
  r.rtcmLastType = p.rtcmLastType;
  r.rtcmLastMs = p.rtcmLastMs;
  r.ubxOk = p.ubxOk;
  r.ubxBad = p.ubxBad;
  r.cmds = p.cmds;
  r.cmdAborted = p.cmdAborted;
  r.junk = p.junk;
  r.rtcmDropped = rtcmDropped;
  return r;
}

Event takeEvent() {
  Event e = pendingEvent;
  pendingEvent = Event::None;
  return e;
}

}  // namespace btlink
