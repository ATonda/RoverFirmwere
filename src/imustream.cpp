#include "imustream.h"

#include "btlink.h"
#include "epochbuf.h"
#include "gnss.h"
#include "proto.h"
#include "ubx.h"

namespace imustream {

static constexpr uint8_t CLS = 0xF5;
static constexpr uint8_t ID = 0x01;
static constexpr int DECIM = 2;        // záznam: každý druhý vzorek (60 Hz)
static constexpr int DECIM_SPEED = 1;  // měření rychlosti (FW 0.16.0): všechny vzorky (120 Hz)
static int decim = DECIM;
static constexpr int PER_FRAME = 30;
static constexpr int HEAD = 4 + 8 + 4 + 8 + 8 + 12 + 4;
static constexpr int SAMPLE = 14;

static uint8_t payload[HEAD + PER_FRAME * SAMPLE];
static uint8_t out[sizeof(payload) + 8];
static int n = 0;
static int skip = 0;
static int64_t t0 = 0;
static uint32_t framesN = 0;

template <typename T>
static void put(int &o, T v) {
  memcpy(payload + o, &v, sizeof(T));
  o += sizeof(T);
}

static int16_t clamp16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static void send() {
  const gnss::Pvt &p = gnss::pvt();
  gnss::Stats gs = gnss::stats();
  float bx, by, bz;
  imu::gyroBias(bx, by, bz);
  int o = 0;
  imu::BiasInfo bi = imu::biasInfo();
  put<uint8_t>(o, 2);
  put<uint8_t>(o, (uint8_t)n);
  put<uint8_t>(o, (uint8_t)decim);
  put<uint8_t>(o, (uint8_t)((p.timeValid ? 1 : 0) | (gs.ppsCount ? 2 : 0) | (bi.still ? 4 : 0)));
  put<int64_t>(o, t0);
  put<uint32_t>(o, p.iTOW);
  put<int64_t>(o, p.rxUs);
  put<int64_t>(o, gs.ppsLastUs);
  put<float>(o, bx);
  put<float>(o, by);
  put<float>(o, bz);
  put<float>(o, imu::temperature());
  // vzorky už jsou v payload od HEAD
  uint16_t len = HEAD + n * SAMPLE;
  uint16_t fl = ubx::frame(CLS, ID, payload, len, out);
  btlink::flush();  // rozpracovaná dávka vět nejdřív
  btlink::send((const char *)out, fl);
  btlink::flush();
  framesN++;
}

void onImu(const imu::Sample &s) {
  bool speed = proto::speedMode();
  if ((!ebuf::recording() && !speed) || !btlink::connected()) {
    n = 0;
    skip = 0;
    return;
  }
  int want = speed ? DECIM_SPEED : DECIM;
  if (want != decim) {  // změna režimu – rozpracovaný rámec odeslat se starou decimací
    if (n) send();
    n = 0;
    skip = 0;
    decim = want;
  }
  if (skip++ % decim) return;
  if (n == 0) t0 = s.tUs;
  int64_t dt = (s.tUs - t0) / 100;
  if (dt < 0 || dt > 65535) {  // díra ve vzorcích – začít nový rámec
    if (n) send();
    n = 0;
    t0 = s.tUs;
    dt = 0;
  }
  int o = HEAD + n * SAMPLE;
  put<uint16_t>(o, (uint16_t)dt);
  put<int16_t>(o, clamp16(s.ax * 1000.0f));
  put<int16_t>(o, clamp16(s.ay * 1000.0f));
  put<int16_t>(o, clamp16(s.az * 1000.0f));
  put<int16_t>(o, clamp16(s.gx / 0.02f));
  put<int16_t>(o, clamp16(s.gy / 0.02f));
  put<int16_t>(o, clamp16(s.gz / 0.02f));
  if (++n >= PER_FRAME) {
    send();
    n = 0;
  }
}

uint32_t frames() { return framesN; }

}  // namespace imustream
