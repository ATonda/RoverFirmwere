# Rover GNSS – firmware ESP32

Firmware roveru ZED-X20P + ESP32 (DFRobot FireBeetle 2 ESP32-E, 4 MB flash), PlatformIO (pioarduino 55.03.312, Arduino 3.3.12).

- Specifikace: [docs/rover-firmware-specifikace.md](docs/rover-firmware-specifikace.md)
- Protokol s aplikací RoverGnss (repozitář `RoverGnss`): [docs/rover-app-specifikace.md](docs/rover-app-specifikace.md), kap. 5 a 10
- Zapojení: [docs/rover-zapojeni-stav.md](docs/rover-zapojeni-stav.md)

Nahrání jen přes USB (bez OTA a bez Wi-Fi od 0.13.0). Konzole 115200 Bd, `h` = nápověda, řádek začínající `{` = příkaz JSON jako z aplikace.

## Licence

GPL-3.0 – viz [LICENSE](LICENSE).
