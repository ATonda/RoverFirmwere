// Surová data IMU do BT při záznamu v aplikaci (ladění krokového dopočtu / EKF offline)
//
// Rámec UBX třídy 0xF5 (vlastní), ID 0x01, každých 0,5 s (30 vzorků 60 Hz – každý druhý ze 120 Hz).
// Aplikace ho neinterpretuje, ukládá se se surovým proudem trasy (track_N.log).
// Obsah (little endian):
//   u8  verze = 1
//   u8  n – počet vzorků
//   u8  decimace (2 = každý druhý vzorek IMU)
//   u8  příznaky: bit0 čas GNSS platný, bit1 PPS
//   i64 t0Us – esp_timer prvního vzorku [µs]
//   u32 pvtITOW – čas týdne GPS poslední epochy [ms]
//   i64 pvtRxUs – esp_timer příjmu této NAV-PVT [µs]
//   i64 ppsUs – esp_timer poslední hrany PPS [µs]
//   f32 ×3 bias gyra odečtený ve vzorcích [°/s]
//   n × { u16 dt [100 µs od t0], i16 ax, ay, az [mg], i16 gx, gy, gz [0,02 °/s] }
#pragma once
#include <Arduino.h>

#include "imu.h"

namespace imustream {

void onImu(const imu::Sample &s);  // každý vzorek IMU (z fronty v loop)
uint32_t frames();                 // odeslaných rámců (od startu)

}  // namespace imustream
