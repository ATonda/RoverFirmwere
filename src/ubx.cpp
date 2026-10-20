#include "ubx.h"

namespace ubx {

ValSet::ValSet(uint8_t layers) {
  buf_[0] = 0x00;    // version
  buf_[1] = layers;  // layers
  buf_[2] = 0x00;    // reserved
  buf_[3] = 0x00;
  len_ = 4;
}

void ValSet::put(uint32_t key, uint32_t v, uint8_t n) {
  if (len_ + 4 + n > sizeof(buf_)) {
    overflow_ = true;
    return;
  }
  for (int i = 0; i < 4; i++) buf_[len_++] = (key >> (8 * i)) & 0xFF;
  for (int i = 0; i < n; i++) buf_[len_++] = (v >> (8 * i)) & 0xFF;
}

void ValSet::addU1(uint32_t key, uint8_t v) { put(key, v, 1); }
void ValSet::addU2(uint32_t key, uint16_t v) { put(key, v, 2); }
void ValSet::addU4(uint32_t key, uint32_t v) { put(key, v, 4); }

uint16_t frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *dst) {
  dst[0] = 0xB5;
  dst[1] = 0x62;
  dst[2] = cls;
  dst[3] = id;
  dst[4] = len & 0xFF;
  dst[5] = len >> 8;
  if (len) memcpy(dst + 6, payload, len);
  uint8_t a = 0, b = 0;
  for (uint16_t i = 2; i < 6 + len; i++) {
    a += dst[i];
    b += a;
  }
  dst[6 + len] = a;
  dst[7 + len] = b;
  return len + 8;
}

void Parser::feed(uint8_t c) {
  switch (state_) {
    case 0:
      if (c == 0xB5) state_ = 1;
      break;
    case 1:
      state_ = (c == 0x62) ? 2 : (c == 0xB5 ? 1 : 0);
      break;
    case 2:
      cls_ = c;
      ckA_ = c;
      ckB_ = c;
      state_ = 3;
      break;
    case 3:
      id_ = c;
      ckA_ += c;
      ckB_ += ckA_;
      state_ = 4;
      break;
    case 4:
      len_ = c;
      ckA_ += c;
      ckB_ += ckA_;
      state_ = 5;
      break;
    case 5:
      len_ |= (uint16_t)c << 8;
      ckA_ += c;
      ckB_ += ckA_;
      idx_ = 0;
      overflow_ = len_ > MAX_LEN;
      state_ = len_ ? 6 : 7;
      break;
    case 6:
      if (!overflow_) buf_[idx_] = c;
      idx_++;
      ckA_ += c;
      ckB_ += ckA_;
      if (idx_ >= len_) state_ = 7;
      break;
    case 7:
      if (c == ckA_) {
        state_ = 8;
      } else {
        err_++;
        state_ = 0;
      }
      break;
    case 8:
      if (c == ckB_) {
        ok_++;
        if (!overflow_ && handler_) handler_(cls_, id_, buf_, len_);
      } else {
        err_++;
      }
      state_ = 0;
      break;
  }
}

}  // namespace ubx
