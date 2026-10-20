# ZED-X20P (ELT0421) – postup konfigurace

## První nastavení (před složením)
1. ELT0421 připojit USB-C přímo do PC (s anténou, bez ESP32).
2. V u-center 2 zkontrolovat verzi firmwaru; kvůli Galileo HAS aktualizovat na aktuální verzi.
3. Nastavit:
   - rychlost UART1 (např. 460800 Bd),
   - výstupní zprávy na UART1 (NAV-PVT, RAWX/SFRBX, …),
   - vstup RTCM na UART2,
   - navigační frekvence 5 Hz,
   - zapnout Galileo HAS.
4. Uložit do Flash (vrstvy RAM + BBR + Flash), jinak se nastavení po vypnutí ztratí.
5. Odpojit USB, připojit ESP32 na piny.

## Konfigurace ve složeném přístroji
- **Varianta A – USB-C ELT0421 vyvedený ven** (výřez nebo krátký prodlužovací kabel).
  USB a UART1 jsou na X20P samostatná rozhraní → konfigurace přes USB funguje i s připojeným ESP32.
  Ověřit, že při současném napájení z USB a baterie nedochází ke zpětnému napájení (diody u PWR).
- **Varianta B – průchozí režim v ESP32.** Firmware na povel přepne ESP32 do režimu mostu USB ESP32 ↔ UART1 X20P.
  u-center 2 se připojí na COM port ESP32 jako na přijímač. Lze tak i aktualizovat firmware X20P (aktualizace přes UART1 je podporovaná).

## Doporučení pro firmware ESP32
ESP32 po startu samo pošle kritické nastavení (rychlost UART, zprávy, frekvence) přes UBX-CFG-VALSET.
Přístroj tak funguje i po výměně modulu nebo továrním resetu a nezávisí jen na nastavení uloženém ve flash X20P.

## Pinout ELT0421 (14 pinů, rozteč 2,54 mm)
| Horní řada | Dolní řada |
|---|---|
| NC | NC |
| RX(2) | TX(2) |
| TIME(2) | TIME(1) |
| EXTINT | READY |
| SCL / SPI_CLK | SDA / SPI_CS |
| RX / SPI_MOSI | TX / SPI_MISO |
| +5V | GND |

Zapojení s ESP32:
- TX/SPI_MISO → RX ESP32, RX/SPI_MOSI ← TX ESP32 (UART1 – hlavní data)
- RX(2) ← TX ESP32 (RTCM korekce)
- TIME(1) → GPIO ESP32 s přerušením (PPS pro synchronizaci IMU)
- UART1 je sdílený se SPI – plôška SEL u USB musí být nastavena na UART.
