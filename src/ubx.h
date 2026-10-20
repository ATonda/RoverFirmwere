// UBX protokol – parser, odesílání, CFG-VALSET
#pragma once
#include <Arduino.h>

namespace ubx {

// Třídy a ID zpráv
static constexpr uint8_t CLS_NAV = 0x01;
static constexpr uint8_t CLS_RXM = 0x02;
static constexpr uint8_t ID_RXM_SFRBX = 0x13;
static constexpr uint8_t ID_RXM_RAWX = 0x15;
static constexpr uint8_t CLS_ACK = 0x05;
static constexpr uint8_t CLS_CFG = 0x06;
static constexpr uint8_t CLS_MON = 0x0A;
static constexpr uint8_t CLS_MGA = 0x13;  // asistence (MGA-INI z mobilu, MGA-ACK, MGA-DBD)

static constexpr uint8_t ID_NAV_DOP    = 0x04;
static constexpr uint8_t ID_NAV_PVT    = 0x07;
static constexpr uint8_t ID_NAV_COV    = 0x36;
static constexpr uint8_t ID_NAV_HPPOSLLH = 0x14;
static constexpr uint8_t ID_NAV_SAT    = 0x35;
static constexpr uint8_t ID_NAV_EOE    = 0x61;  // konec epochy navigace
static constexpr uint8_t ID_NAV_SIG    = 0x43;  // signály (zdroj korekcí po signálech)
static constexpr uint8_t ID_ACK_ACK    = 0x01;
static constexpr uint8_t ID_ACK_NAK    = 0x00;
static constexpr uint8_t ID_CFG_VALSET = 0x8A;
static constexpr uint8_t ID_CFG_RST    = 0x04;
static constexpr uint8_t ID_MON_VER    = 0x04;
static constexpr uint8_t ID_MON_COMMS  = 0x36;
static constexpr uint8_t ID_MGA_ACK    = 0x60;  // potvrzení asistenční zprávy (CFG-NAVSPG-ACKAIDING)
static constexpr uint8_t ID_MGA_DBD    = 0x80;  // navigační databáze (výpis / obnova)

// Vrstvy konfigurace
static constexpr uint8_t LAYER_RAM   = 0x01;
static constexpr uint8_t LAYER_BBR   = 0x02;
static constexpr uint8_t LAYER_FLASH = 0x04;

// Klíče konfigurace (ID dle u-blox interface description, ověřeno v u-center 2)
static constexpr uint32_t KEY_UART1_BAUDRATE        = 0x40520001;  // U4
static constexpr uint32_t KEY_UART1INPROT_UBX       = 0x10730001;  // L
static constexpr uint32_t KEY_UART1INPROT_RTCM3X    = 0x10730004;  // L
static constexpr uint32_t KEY_UART1OUTPROT_UBX      = 0x10740001;  // L
static constexpr uint32_t KEY_UART1OUTPROT_NMEA     = 0x10740002;  // L
static constexpr uint32_t KEY_MSGOUT_NAV_PVT_UART1  = 0x20910007;  // U1
static constexpr uint32_t KEY_MSGOUT_NAV_SAT_UART1  = 0x20910016;  // U1
static constexpr uint32_t KEY_MSGOUT_NAV_DOP_UART1  = 0x20910039;  // U1
static constexpr uint32_t KEY_MSGOUT_NAV_COV_UART1  = 0x20910084;  // U1
static constexpr uint32_t KEY_MSGOUT_NAV_HPPOSLLH_UART1 = 0x20910034;  // U1
static constexpr uint32_t KEY_MSGOUT_NAV_EOE_UART1 = 0x20910160;  // U1
static constexpr uint32_t KEY_MSGOUT_RXM_RAWX_UART1 = 0x209102a5;  // U1 surová měření (PPK)
static constexpr uint32_t KEY_MSGOUT_RXM_SFRBX_UART1 = 0x20910232; // U1 navigační zprávy (PPK)
static constexpr uint32_t KEY_RATE_MEAS             = 0x30210001;  // U2 [ms]
static constexpr uint32_t KEY_RATE_NAV              = 0x30210002;  // U2
static constexpr uint32_t KEY_NAVSPG_DYNMODEL       = 0x20110021;  // E1
static constexpr uint32_t KEY_NAVSPG_INFIL_MINELEV  = 0x201100a4;  // I1 [°] elevační maska
static constexpr uint32_t KEY_NAVSPG_ACKAIDING      = 0x10110025;  // L potvrzovat asistenční zprávy (MGA-ACK)
static constexpr uint32_t KEY_MSGOUT_NAV_SIG_UART1  = 0x20910346;  // U1 signály 1 Hz
// Korekce (ID z databáze klíčů pyubx2 pro X20 HPG 2.1x – ověřit na HW, NAK se hlásí ve STA)
static constexpr uint32_t KEY_SBAS_USE_DIFFCORR     = 0x10360004;  // L používat korekce SBAS
static constexpr uint32_t KEY_NAVCOR_ENABLE_HOST    = 0x100d0001;  // L korekce od hostitele (RTCM/SPARTN z mobilu)
static constexpr uint32_t KEY_NAVCOR_ENABLE_GAL_HAS = 0x100d0002;  // L Galileo HAS (E6-B)

// Sestavení payloadu CFG-VALSET (max. 64 klíčů dle u-blox, zde omezeno bufferem)
class ValSet {
 public:
  explicit ValSet(uint8_t layers);
  void addU1(uint32_t key, uint8_t v);
  void addU2(uint32_t key, uint16_t v);
  void addU4(uint32_t key, uint32_t v);
  void addL(uint32_t key, bool v) { addU1(key, v ? 1 : 0); }
  const uint8_t *data() const { return buf_; }
  uint16_t size() const { return len_; }
  bool overflow() const { return overflow_; }

 private:
  void put(uint32_t key, uint32_t v, uint8_t n);
  uint8_t buf_[256];
  uint16_t len_ = 0;
  bool overflow_ = false;
};

// Sestaví kompletní rámec UBX do dst (musí mít len + 8 B). Vrací délku rámce.
uint16_t frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *dst);

// Bajtový parser rámců UBX
class Parser {
 public:
  using Handler = void (*)(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
  explicit Parser(Handler h) : handler_(h) {}
  void feed(uint8_t c);
  uint32_t okCount() const { return ok_; }
  uint32_t errCount() const { return err_; }

 private:
  static constexpr uint16_t MAX_LEN = 3072;  // NAV-SIG až 8 + 16 × počet signálů
  Handler handler_;
  uint8_t state_ = 0;
  uint8_t cls_ = 0, id_ = 0;
  uint16_t len_ = 0, idx_ = 0;
  uint8_t ckA_ = 0, ckB_ = 0;
  bool overflow_ = false;
  uint8_t buf_[MAX_LEN];
  uint32_t ok_ = 0, err_ = 0;
};

// Pomocné čtení little-endian
inline uint16_t rdU2(const uint8_t *p) { return p[0] | (p[1] << 8); }
inline uint32_t rdU4(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
inline int32_t  rdI4(const uint8_t *p) { return (int32_t)rdU4(p); }

}  // namespace ubx
