// Konzole na USB (Serial, UART0) s možností umlčení – FW 0.17.0
// Všechny výpisy firmwaru jdou přes Con. V režimu USB host (ESP32-S3) je Con umlčená a na Serial
// jdou jen věty $PRVR a odpovědi JSON (usblink::send zapisuje do Serial přímo).
#pragma once
#include <Arduino.h>

class Console : public Stream {
 public:
  void mute(bool m) { muted_ = m; }
  bool muted() const { return muted_; }

  void begin(unsigned long baud) { Serial.begin(baud); }
  size_t setTxBufferSize(size_t n) { return Serial.setTxBufferSize(n); }
  size_t setRxBufferSize(size_t n) { return Serial.setRxBufferSize(n); }
  void updateBaudRate(unsigned long baud) { Serial.updateBaudRate(baud); }
  size_t readBytes(uint8_t *b, size_t n) { return Serial.readBytes(b, n); }

  int available() override { return Serial.available(); }
  int read() override { return Serial.read(); }
  int peek() override { return Serial.peek(); }
  void flush() override {
    if (!muted_) Serial.flush();
  }
  int availableForWrite() override { return Serial.availableForWrite(); }
  size_t write(uint8_t c) override { return muted_ ? 1 : Serial.write(c); }
  size_t write(const uint8_t *b, size_t n) override { return muted_ ? n : Serial.write(b, n); }
  using Print::write;

 private:
  bool muted_ = false;
};

extern Console Con;
