// Rover GNSS – piny a konstanty
// Zapojení: docs/rover-zapojeni-stav.md
#pragma once
#include <Arduino.h>

#define FW_NAME    "Rover"
#define FW_VERSION "0.12.0"

// ---------------------------------------------------------------- piny
// X20 UART1 (ELT0421: TX/SPI_MISO, RX/SPI_MOSI)
static constexpr int PIN_GNSS_RX  = 16;   // <- X20 TX
static constexpr int PIN_GNSS_TX  = 17;   // -> X20 RX (UBX příkazy + RTCM)
static constexpr int PIN_GNSS_RX2 = 4;    // -> X20 RX(2) – UART2 neaktivní, rezerva
static constexpr int PIN_PPS      = 36;   // <- X20 TIME(1), jen vstup

// IMU LSM6DSV16X (I2C)
static constexpr int PIN_SDA      = 21;
static constexpr int PIN_SCL      = 22;
static constexpr int PIN_IMU_INT1 = 15;

// Ostatní
static constexpr int PIN_VBAT     = 35;   // dělič 100k/100k z BAT 6092
static constexpr int PIN_BUTTON   = 13;   // tlačítko Wi-Fi / profil proti GND (pár 1 bílá/šedá)
static constexpr int PIN_BOOT     = 0;    // tlačítko BOOT proti GND (pár 6 fialová/modrá), za běhu čitelné
static constexpr int PIN_BUZZER   = 25;   // piezo (pár 7 oranžová/žlutá)
static constexpr int PIN_LED      = 26;   // WS2812 DIN (pár 2 černá/hnědá + VCC), zatím nezapojeno

// ---------------------------------------------------------------- GNSS
static constexpr uint32_t GNSS_BAUD_TARGET = 460800;          // provozní rychlost UART1
static constexpr uint32_t GNSS_BAUDS[] = {460800, 38400, 230400, 115200, 9600};
static constexpr uint16_t GNSS_MEAS_RATE_MS = 200;            // 5 Hz
static constexpr uint8_t  GNSS_DYNMODEL = 3;                  // 3 = pedestrian

// ---------------------------------------------------------------- IMU
static constexpr uint8_t IMU_ADDR_A = 0x6A;
static constexpr uint8_t IMU_ADDR_B = 0x6B;

// ---------------------------------------------------------------- tlačítko
static constexpr uint32_t BTN_DEBOUNCE_MS = 30;
static constexpr uint32_t BTN_LONG_MS     = 2000;   // provoz -> konfigurační režim
static constexpr uint32_t BTN_EXIT_MS     = 5000;   // konfigurace/test -> restart do provozu (BT)

// ---------------------------------------------------------------- baterie
static constexpr float VBAT_DIVIDER_DEFAULT = 2.0f;

// ---------------------------------------------------------------- konfigurační režim
static constexpr int PROFILES_MAX = 10;
// Wi-Fi AP + web: pod touto volnou RAM se BT před startem Wi-Fi vypne (s BT bývá volných jen ~56 kB)
static constexpr uint32_t WIFI_MIN_HEAP = 100000;
static constexpr uint32_t WIFI_MIN_BLOCK = 60000;
// Bluetooth: víc nepotvrzených dat v zásobníku = dávka se zahodí (~1 s dat v režimu aplikace)
static constexpr uint32_t BT_MAX_BACKLOG = 2048;
static constexpr uint8_t RESCUE_CRASHES = 3;  // tolik pádů po sobě -> záchranný režim (AP + OTA)
