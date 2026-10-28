// Příjem proudu z mobilu (BT SPP) nebo z ESP32-S3 (USB, režim USB host) – FW 0.17.0
// Rozlišuje řádky JSON ('{"' … \n), rámce RTCM3 (0xD3) a UBX (0xB5 0x62).
// Rámce jdou dál jen celé a s platným CRC / součtem; bajty mimo rámce se zahodí.
// Po chybě (CRC, nesmyslná délka, řídicí bajt nebo neplatné UTF-8 v řádku JSON) se bajty za začátkem rámce
// zpracují znovu – synchronizace se najde i uvnitř zahozeného „rámce“.
#pragma once
#include <Arduino.h>

class RxParser {
 public:
  struct Stats {
    uint32_t rtcmOk = 0;        // celé zprávy RTCM3 se správným CRC
    uint32_t rtcmBad = 0;       // RTCM s chybným CRC
    uint16_t rtcmLastType = 0;  // typ poslední správné zprávy
    uint32_t rtcmLastMs = 0;    // millis() poslední správné zprávy
    uint32_t ubxOk = 0;         // celé rámce UBX se správným součtem
    uint32_t ubxBad = 0;        // rámce UBX s chybným součtem nebo délkou
    uint32_t cmds = 0;          // přijaté řádky JSON
    uint32_t cmdAborted = 0;    // začátky „JSON“ zrušené binárním bajtem
    uint32_t junk = 0;          // zahozené bajty mimo rámce (bez \r \n)
  };
  using CmdFn = void (*)(const char *line, size_t len);                  // řádek bez \n, ukončený 0
  using FrameFn = void (*)(const uint8_t *f, size_t n, bool rtcm);       // celý platný rámec

  // buf a replay musí mít velikost size (≥ 1100 B); řádek JSON smí mít nejvýš size − 1 znaků
  RxParser(uint8_t *buf, uint8_t *replay, size_t size, uint16_t ubxMax);
  void setHandlers(CmdFn cmd, FrameFn frame);
  void put(uint8_t c);
  void reset();  // rozpracovaný rámec / řádek zahodit (nové spojení)
  const Stats &stats() const { return st; }

 private:
  enum class S : uint8_t { Idle, Rtcm, Ubx, Cmd, SkipLine };
  void step(uint8_t c);
  void fail();  // bajty buf[1..pos) zpracovat znovu
  void done();
  bool utf8Ok(uint8_t c);

  uint8_t *buf, *rp;
  size_t size;
  uint16_t ubxMax;
  size_t pos = 0, rpPos = 0, rpLen = 0;
  uint16_t len = 0;
  uint8_t utf8Need = 0;  // zbývající bajty znaku UTF-8 v řádku JSON
  S s = S::Idle;
  Stats st;
  CmdFn onCmd = nullptr;
  FrameFn onFrame = nullptr;
};
