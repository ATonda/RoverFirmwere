#include "rxparse.h"

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

RxParser::RxParser(uint8_t *b, uint8_t *r, size_t sz, uint16_t um) : buf(b), rp(r), size(sz), ubxMax(um) {
  if (ubxMax + 8 > size) ubxMax = size - 8;
}

void RxParser::setHandlers(CmdFn cmd, FrameFn frame) {
  onCmd = cmd;
  onFrame = frame;
}

void RxParser::reset() {
  done();
  rpPos = rpLen = 0;
}

// Kontrola UTF-8 v řádku JSON: náhodná binární data (RTCM po ztrátě synchronizace) ho poruší rychle
bool RxParser::utf8Ok(uint8_t c) {
  if (utf8Need) {
    if ((c & 0xC0) != 0x80) return false;
    utf8Need--;
    return true;
  }
  if (c < 0x80) return true;
  if (c >= 0xC2 && c <= 0xDF) utf8Need = 1;
  else if (c >= 0xE0 && c <= 0xEF) utf8Need = 2;
  else if (c >= 0xF0 && c <= 0xF4) utf8Need = 3;
  else return false;
  return true;
}

void RxParser::done() {
  utf8Need = 0;
  s = S::Idle;
  pos = 0;
  len = 0;
}

// Chybný „rámec“: první bajt zahodit, zbytek (a nezpracovaný zbytek dřívější chyby) projít znovu.
// Během opakování pochází obsah buf jen z rp, takže buf[1..pos) + zbytek rp se do rp vejde.
void RxParser::fail() {
  size_t extra = pos > 1 ? pos - 1 : 0;
  size_t rest = rpLen - rpPos;
  if (extra + rest > size) extra = size - rest;  // nemá nastat – pojistka
  memmove(rp + extra, rp + rpPos, rest);
  memcpy(rp, buf + 1, extra);
  rpPos = 0;
  rpLen = extra + rest;
  done();
}

void RxParser::put(uint8_t c) {
  step(c);
  while (rpPos < rpLen) step(rp[rpPos++]);
}

void RxParser::step(uint8_t c) {
  switch (s) {
    case S::Idle:
      if (c == 0xD3) {
        s = S::Rtcm;
      } else if (c == 0xB5) {
        s = S::Ubx;
      } else if (c == '{') {
        s = S::Cmd;
      } else {
        if (c != '\r' && c != '\n') st.junk++;
        return;
      }
      buf[0] = c;
      pos = 1;
      return;

    case S::Rtcm:
      buf[pos++] = c;
      if (pos == 3) {
        len = ((buf[1] & 0x03) << 8) | buf[2];
        if (buf[1] & 0xFC) {  // rezervované bity musí být 0 – není to začátek zprávy
          fail();
          return;
        }
      }
      if (pos >= 3 && pos == (size_t)len + 6) {
        uint32_t crc = ((uint32_t)buf[len + 3] << 16) | ((uint32_t)buf[len + 4] << 8) | buf[len + 5];
        if (crc24q(buf, len + 3) == crc) {
          st.rtcmOk++;
          st.rtcmLastMs = millis();
          if (len >= 2) st.rtcmLastType = (buf[3] << 4) | (buf[4] >> 4);
          size_t n = pos;
          done();
          if (onFrame) onFrame(buf, n, true);
        } else {
          st.rtcmBad++;
          fail();
        }
      }
      return;

    case S::Ubx:
      buf[pos++] = c;
      if (pos == 2) {
        if (c != 0x62) fail();
        return;
      }
      if (pos == 6) {
        len = buf[4] | ((uint16_t)buf[5] << 8);
        if (len > ubxMax) {  // nesmysl – nebyl to rámec
          st.ubxBad++;
          fail();
          return;
        }
      }
      if (pos >= 6 && pos == (size_t)len + 8) {
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < (size_t)len + 6; i++) {
          a += buf[i];
          b += a;
        }
        if (a == buf[len + 6] && b == buf[len + 7]) {
          st.ubxOk++;
          size_t n = pos;
          done();
          if (onFrame) onFrame(buf, n, false);
        } else {
          st.ubxBad++;
          fail();
        }
      }
      return;

    case S::Cmd:
      if ((c == '\n' || c == '\r') && utf8Need) {  // řádek skončil uprostřed znaku UTF-8
        st.cmdAborted++;
        fail();
      } else if (c == '\n' || c == '\r') {
        buf[pos] = 0;
        size_t n = pos;
        done();
        st.cmds++;
        if (onCmd) onCmd((const char *)buf, n);
      } else if ((pos == 1 && c != '"') || (c < 0x20 && c != '\t') || c == 0x7F || !utf8Ok(c)) {
        // řádek JSON začíná vždy {" (objekt s klíčem) – jinak byl '{' bajt binárních dat
        // řídicí bajt ani neplatné UTF-8 v JSON být nemůže – '{' byl bajt binárních dat
        st.cmdAborted++;
        buf[pos++] = c;
        fail();
      } else if (pos < size - 1) {
        buf[pos++] = c;
      } else {
        s = S::SkipLine;  // příliš dlouhý řádek – zahodit až do konce řádku
        pos = 0;
      }
      return;

    case S::SkipLine:
      if (c == '\n' || c == '\r') done();
      return;
  }
}
