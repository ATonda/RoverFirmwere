# Rover GNSS – hardware, zapojení a stav (9. 10. 2026)

## Hardware

| Díl | Typ | Pozn. |
|---|---|---|
| MCU | DFRobot FireBeetle 2 ESP32-E (ESP32-WROOM-32E, 4 MB) | USB-UART CH340, na PC COM28 |
| GNSS | u-blox ZED-X20P, deska ELT0421 (gnss.store) | FW **EXT HPG 2.11**, PROTVER 50.11 |
| Anténa | AllBand+ ELT0242 (E6/HAS) | |
| IMU | SparkFun 6DoF LSM6DSV16X (SEN-21325) | I2C, adresa 0x6B, WHO_AM_I 0x70 |
| Nabíječka | Adafruit 6092 (bq25185) | 1 A, power-path; výstup 3V se nepoužívá |
| Zdroj 5 V | Pololu U3V16F5 (step-up 5 V) | piny jen VIN, GND, VOUT (bez EN) |
| Baterie | LiPo 1S 3 Ah, JST-PH | připojená do 6092 |
| Vypínač | mechanický | mezi power-path 6092 a U3V |
| Tlačítko | Wi-Fi (podržení 2 s) / profil (krátký stisk) / konec průchozího režimu | GPIO13 proti GND |
| Tlačítko BOOT | nahrávání firmwaru (podržet při resetu), za běhu čitelné | GPIO0 proti GND |
| Panelová USB-C zásuvka | s vývody (VBUS, GND, D+, D−) | zatím nezapojená |
| Piezo | pasivní | GPIO25 (pár 7) |
| Stavová LED | WS2812 – zatím nekoupená | vodiče připravené |

## Napájení

```
USB-C (na 6092, později panelová zásuvka) → 6092 VIN
LiPo 3 Ah → 6092 BAT
6092 power-path out (~4,8 V z USB / 3,0–4,2 V z baterie) → vypínač → U3V16F5 VIN
U3V16F5 VOUT 5 V → ploška VIN na ESP (= VBUS, ověřeno 5,1 V)
                 → X20 +5V (u pinů X20: 10 µF elektrolyt + 100 nF keramika, bez tlumivky)
ESP 3V3 (výstup LDO) → IMU 3V3
ESP VCC (~4,7 V) → jen budoucí WS2812 VDD
```

- Výstup 3V na 6092 se nezapojuje (dva regulátory proti sobě).
- Na pin VCC ESP vede jen vodič k WS2812.
- Pin 13/D7 je volný (původně EN měniče, U3V16F5 EN nemá).

## USB data (jeden konektor pro vše)

Ustřižený USB-C kabel zasunutý do ESP, vodiče připájené na 6092:

| Vodič kabelu | 6092 |
|---|---|
| zelený (D+) | D+ |
| bílý (D−) | D− |
| černý (GND) | G |
| červený (VBUS) | odstřižený, nezapojený |
| žlutý, modrý (CC / VCONN) | zaizolované |

Panelová zásuvka (při kompletaci): červený → 6092 VIN, černý → G, bílý → D−, zelený → D+. Ověřit barvy měřákem a CC odpory 5,1 kΩ (bez nich nabíječka C-C nedá 5 V).

## Zapojení ESP (FireBeetle)

| Pin ESP | Připojení |
|---|---|
| 16/D11 | X20 TX / SPI_MISO (UART1 RX ESP) |
| 17/D10 | X20 RX / SPI_MOSI (UART1 TX ESP) |
| 4/D12 | X20 RX(2) – UART2, **neaktivní, rezerva** |
| 36/A0 | X20 TIME(1) – PPS |
| SDA (21) | IMU PICO (= SDA) |
| SCL (22) | IMU SCLK (= SCL) |
| 15/A4 | IMU INT1 |
| 3V3 | IMU 3V3 |
| GND | IMU GND |
| 35/A3 | střed děliče 2 × 100 kΩ (BAT+ 6092 → R → uzel → R → GND), 100 nF keramika uzel–GND |
| 13 | tlačítko Wi-Fi/profil proti GND (interní pull-up) – pár 1 bílá/šedá |
| 0/D5 | tlačítko BOOT proti GND – pár 6 fialová/modrá |
| 25/D2 | piezo proti GND (sériově 100 Ω) – pár 7 oranžová/žlutá |
| 26/D3 + VCC | WS2812 (DIN přes 330 Ω, VDD) – pár 2 černá/hnědá; **GND k LED chybí, doplnit** (+ 100 nF) |
| VIN / GND plošky | U3V16F5 VOUT / GND |

Nepoužívat: 0, 2, 5, 12 (strapping), 1/3 (USB), 6–11 (flash).

## Naměřeno

| Veličina | Hodnota |
|---|---|
| Odběr bez nabíjení | 0,17 A při 5,1 V (0,85 W) → z 3 Ah cca 11–12 h |
| Nabíjení | 0,98 A ze zdroje (~0,8 A do baterie), plné nabití ~4–4,5 h |
| Dělič baterie | 3,86 V → 1,93 V; ADC v ESP souhlasí (4,18 V plná) |
| IMU v klidu | \|a\| ≈ 1,01–1,02 g, bias gyra X ≈ −0,75 dps, teplota 26–28 °C |
| IMU INT1 | 115–118 Hz (nominál 120 Hz, tolerance oscilátoru) |
| IMU FIFO | 117–119 Hz podle časových značek čipu |
| IMU po RST/nahrání | občas nenalezeno – IMU drží SDA v nule (přerušené čtení); od FW 0.3.0 uvolnění sběrnice při startu |
| X20 → ESP UART1 | 38400 Bd, UBX i NMEA bez chyb kontrolní sumy |

## Konfigurace X20 (uložena RAM + BBR + Flash)

| Klíč | Hodnota | Pozn. |
|---|---|---|
| CFG-UART1-ENABLED | 1 | **výchozí hodnota u tohoto FW je 0** – bez ní UART1 mlčí |
| CFG-UART1-BAUDRATE | 38400 | později 460800 |
| CFG-UART1INPROT-RTCM3X | 1 | RTCM korekce půjdou přes UART1 |
| CFG-MSGOUT-UBX_NAV_PVT_UART1 | 1 | |
| CFG-UART2-ENABLED | 1 (výchozí) | **UART2 přesto v UBX-MON-COMMS chybí – neaktivní** |
| CFG-UART2-BAUDRATE | 38400 | |
| CFG-UART2INPROT-RTCM3X | 1 | |

Konfigurace ve složeném přístroji: příkaz `b` (průchozí režim) a u-center 2 na COM28, 38400 Bd. Konec průchozího režimu stiskem tlačítka. USB-C na ELT0421 je ve složené krabičce nepřístupné.

## Firmware (testovací)

- Projekt: `Documents\PlatformIO\Projects\Rover` (PlatformIO, platforma pioarduino, Arduino jádro 3.3.12, board esp32dev, `huge_app.csv`).
- Příkazy v monitoru (115200 Bd): `h` nápověda, `s` živý výpis, `d` detekce rychlosti X20, `v` MON-VER, `p` zapnout NAV-PVT (RAM), `r` test vstupu UART2 + výpis MON-COMMS, `i` sken I2C a init IMU, `m` vzorky IMU, `k3860` kalibrace baterie, `b` průchozí režim, `x` diagnostika vodičů 16/17, `w` prohození RX/TX v SW (uloženo, teď „normálně“).

**Praktické poznámky**
- Nahrávání: automatický reset nepřepne do download módu (boot 0x17). Před Uploadem propojit **GPIO0 (0/D5) s GND**, po `SUCCESS` drát odpojit a stisknout RST.
  - Reset přes RTS (EN) funguje, **linka DTR nestáhne GPIO0**. Vyzkoušeno bez úspěchu: kondenzátor 10 µF EN–GND (odpájen), `esptool.cfg` s `reset_delay = 0.5` i vlastní `custom_reset_sequence` (vráceno na výchozí).
  - Tlačítko u plošky „27“ na ESP je uživatelské tlačítko GPIO27, ne BOOT.
- `esptool.cfg` ve složce projektu: esptool ho načítá automaticky (teď prázdná sekce = výchozí chování).
- Monitor VS Code: **Ctrl+C monitor ukončí**, ke kopírování používat Ctrl+Shift+C. COM port nepadá.
- Když Claude upravuje `main.cpp`, mít ho ve VS Code zavřený (jinak ho uložení editoru přepíše starší verzí).
- COM28 smí mít otevřený jen jeden program (VS Code monitor / Arduino IDE / u-center).
- **Studený start X20 uvnitř vypadá jako vadná anténa:** 0 satelitů, PPS 0, nesvítí/nebliká zelená LED na desce X20, UBX-MON-RF bez rozdílu (AGC ~25 %, antStatus UNKNOWN – dohled antény není zapojený). Napětí pro anténu na SMA 3,2 V = v pořádku. Řešení: anténa ven/k oknu, počkat na fix (pak už startuje rychle).
- **Bez baterie** ukazuje měření baterie nesmysl (~6 V) – výstup BAT nabíjecího modulu bez článku nedrží napětí.
- **GPIO35 – obdélník ~100 kHz, 0 ↔ 2,9 V** (osciloskop, 14. 10.): příčina nalezena 20. 10. – kondenzátor 100 nF byl připájený mezi GPIO35 a sousední SCL místo GND (signál I2C přes kondenzátor na vstup ADC, zároveň zátěž sběrnice I2C). Po opravě vzorky na pinu 2067–2108 mV (rozptyl ~40 mV). Firmware od 0.14.5 vzorkuje v náhodných intervalech a průměruje (10 s / 5 min).
- Páry vodičů od ESP (vlastní číslování): 1 bílá/šedá = 13 + GND (tlačítko Wi-Fi), 2 černá/hnědá = 26/D3 + VCC (WS2812), 6 fialová/modrá = 0/D5 + GND (BOOT), 7 oranžová/žlutá = 25/D2 + GND (piezo).
- Testy hardwaru v konzoli (FW 0.3.3): `t1` tlačítka, `t2` piezo, `t3` LED, `t4` souhrn.
- Panelová USB-C zásuvka: CC odpory 5,1 kΩ jsou v zásuvce (žlutý/modrý vodič nezapojovat). Zapojení: červený → 6092 VIN, černý → G, zelený → D+, bílý → D−. Při „neznámém zařízení“ zkusit otočit konektor, zkontrolovat spoje D+/D−.
- 9. 10.: upadlé vodiče (opakované přepojování kvůli BOOT) → X20 do ESP nic neposílala (0 B, PPS běželo). Po opravě vodičů OK.

## Stav testů

| Periferie | Stav |
|---|---|
| Napájení, nabíjení, měření baterie | ✅ |
| IMU I2C + INT1 | ✅ |
| X20 UART1 oba směry | ✅ |
| Konfigurace X20 přes ESP (průchozí režim) | ✅ |
| PPS | ✅ (perioda 0,999994 s) |
| Fix GNSS | ✅ 3D, s NTRIP RTK Float |
| Tlačítko GPIO13 | přepájeno ze 14 na 13 (FW 0.3.3), test `t1` |
| UART2 (RTCM) | ❌ neaktivní, náhrada: RTCM přes UART1 |
| Piezo | zapojeno, test `t2` |
| WS2812 | nezapojeno (chybí GND), test `t3` |

## Otevřené body

1. **Oprava nahrávání přes USB (DTR → GPIO0):** aktualizovat ovladač CH340 (CH341SER od WCH); při `Connecting…` změřit GPIO0 – pokud neklesne k 0 V, je vadný obvod auto-download na desce ESP.
   - **Tlačítko „27“ jako BOOT:** ohmmetrem (bez napájení) ověřit, že při stisku spíná 27 ↔ GND (~0 Ω). Pokud ano, propojit plošku 27 drátkem s pinem 0/D5; GPIO27 ve firmwaru nepoužívat jako výstup. Pokud spíná proti 3V3, použít samostatné tlačítko GPIO0 → GND.
   - Ovladač CH340K aktualizován na WCH 4.0.2026.2 (port teď COM6). Test po aktualizaci (`No serial data received`) proběhl ještě s vlastní sekvencí resetu v `esptool.cfg` – ta se do PC nevrátila. `esptool.cfg` je teď výchozí (prázdná sekce) → **znovu otestovat Upload bez drátu s novým ovladačem**.
2. **OTA aktualizace firmwaru přes Wi-Fi** – ve FW 0.3.0 (konfigurační režim → Přístroj → Aktualizace), otestovat.
3. UART2: zjistit, proč je neaktivní (restart napájení X20, MON-COMMS v u-centru), nebo definitivně RTCM přes UART1.
4. UART1 460800 Bd a zprávy posílá ESP po startu do RAM ✅; RAWX/SFRBX až v kroku 7.
5. Ověřit HAS.
6. Bzučák (NPN + dioda), WS2812, panelová USB-C zásuvka, kompletace.
