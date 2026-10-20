# Rover GNSS – specifikace firmwaru ESP32 (12. 10. 2026)

Hardware a zapojení: viz `rover-zapojeni-stav.md`.

## 1. Režimy přístroje

| Režim | Vstup | Rádio | Pozn. |
|---|---|---|---|
| **Provoz** | výchozí po každém zapnutí | BT Classic SPP + BLE | Wi-Fi vypnutá |
| **Konfigurace** | podržení tlačítka 2 s (bez restartu) | Wi-Fi AP; **BT vypnuto** (nevejde se do RAM), po vypnutí AP se zapne | bzučák 2× |
| **Průchozí X20** | volba v menu (bez restartu) | Wi-Fi AP zůstává | USB (UART0) napřímo na UART1 X20 |

- Režimy Konfigurace a Průchozí X20 se **neukládají** – po výpadku napájení vždy naběhne Provoz.
- Konfigurace se ukončí tlačítkem „Uložit a vypnout Wi-Fi“ nebo podržením tlačítka **5 s** – **bez restartu** (AP se vypne, BT, výpočet i měření bodu běží dál; dva klesající tóny). Restart jen po změně hesla Wi-Fi / názvu BT, a to jen když neběží měření. **Časový limit nečinnosti zrušen** (od 0.5.1).
- Průchozí X20: stránka ukáže jen banner „USB připojeno k X20 (rychlost)“ a tlačítko „Ukončit průchozí režim“, ostatní volby zašedlé. Ukončení: tlačítko na stránce, podržení fyzického tlačítka, nebo vypnutí/zapnutí. Bez časového limitu.

## 2. Webové menu (Wi-Fi AP)

- AP `ROVER-xxxx`, **WPA2 s pevným heslem na štítku** (kvůli OTA nesmí být otevřený).
- **Captive portal** (vlastní DNS) – mobil/tablet/PC po připojení stránku otevře sám; záložně `192.168.4.1`.
- Funkce: nastavení profilů, správa profilů, nastavení přístroje, OTA, průchozí režim X20, **stránka Statistika**.

### Stránka Statistika (živě, obnova 1 Hz – dotaz `/api/stats`, JSON)

| Oblast | Zobrazení |
|---|---|
| **Satelity** | sky plot; sloupce CN0 po satelitech, barevně podle konstelace (GPS, Galileo, BeiDou, GLONASS, QZSS, NavIC) a pásma (L1/L2/L5/E6); počet sledovaných / použitých ve výpočtu |
| **Řešení** | typ fixu (žádný / 2D / 3D / HAS / RTK Float / RTK Fix / jen IMU), hAcc, vAcc, σ z EKF, PDOP/HDOP/VDOP, rozptyl polohy za posledních N minut (graf bodů v metrech) |
| **Korekce** | zdroj (HAS / RTCM z mobilu), stáří korekcí, přijaté typy zpráv RTCM a jejich četnost, počet zahozených/vadných RTCM, doba do Float a do Fix, stav HAS |
| **Chybovost a komunikace** | UART1 X20: bajty, chybné kontrolní součty UBX/NMEA, přetečení bufferu; BT: připojení SPP/BLE, odeslané/přijaté bajty, odpojení |
| **Výpadky** | počet výpadků GNSS, nejdelší výpadek, aktuální doba přemostění IMU |
| **Rušení a integrita** | indikátor rušení, AGC (UBX-MON-RF), stav OSNMA, stav antény (OK / odpojena / zkrat) |
| **Přístroj** | baterie (V, %), teplota IMU, doba běhu, aktivní profil, verze FW ESP a X20 |

- Statistika je dostupná v konfiguračním režimu (Wi-Fi). Do budoucna vše přes geodetickou aplikaci (viz `rover-app-specifikace.md`).
- Tlačítko „Vynulovat statistiku“ a export záznamu (CSV).

## 3. Uživatelské profily

- Profil = kompletní sada nastavení pod vlastním jménem; uložení v LittleFS (JSON), cca 10 profilů.
- Menu: Uložit jako…, Načíst, Přejmenovat, Smazat, Výchozí po startu.
- **Přepnutí bez Wi-Fi:** krátký stisk tlačítka cyklicky přepíná profily; bzučák pípne podle pořadí (1×, 2×, …), LED barvou profilu.
- Přepnutí i příkazem přes BT (aplikace).

### Obsah profilu

| Sekce | Nastavení |
|---|---|
| **Výpočet** | frekvence výstupu 1 / 5 / 10 Hz; EKF zap/vyp; průměrování bodu (statika); dynamický model X20: chodec / auto / statika / **automaticky**; elevační maska |
| **Korekce** | žádná / Galileo HAS / RTCM z mobilu (NTRIP přes SPP) / později vlastní base; při výpadku RTCM automatický návrat na HAS |
| **Výstupy** | NMEA přes SPP (výběr vět, GSV/GSA 1 Hz); vlastní zpráva s polohou, časem, σ a kvalitou pro aplikaci; volitelně RAWX/SFRBX přes SPP pro PPK (1 Hz) |
| **Geodetická tyč** | volitelně: zap/vyp, výška tyče, korekce náklonu |
| **Signalizace** | bzučák zap/vyp, hlasitost, jas LED |

### Automatický dynamický model
- > 15 km/h po 10 s → automotive; < 8 km/h po 30 s → pedestrian; stání > 60 s → volitelně stationary.
- Zápis jen do RAM X20 (`CFG-NAVSPG-DYNMODEL`).

### Nastavení přístroje (mimo profily)
Heslo AP, název BT, kalibrace IMU a baterie, OTA, průchozí režim X20.

## 4. Výpočet GNSS + IMU

- Po startu ESP pošle X20 **jen potřebné klíče a jen do vrstvy RAM** (UART1 460800, výstupní zprávy, frekvence, dyn. model). Ostatní (konstelace, HAS, maska, …) zůstává podle Flash X20 = nastavení z u-centra.
- IMU (LSM6DSV16X, gyro + akcelerometr) 120 Hz přes INT1, PPS pro časovou synchronizaci.
- EKF 5 Hz – vyhodnocuje automaticky, nic se ručně nenastavuje:

| Veličina | Z čeho | Pozn. |
|---|---|---|
| Náklon (roll/pitch) | akcelerometr + gyro | vždy |
| Kurz | gyro, za pohybu opravovaný kurzem z GNSS | při dlouhém stání pomalu ujíždí (bez magnetometru) |
| Rychlost/poloha při výpadku GNSS | akcelerometr + gyro | sekundy, pak roste nejistota |
| Klid / pohyb | akcelerometr + gyro | auto model, průměrování, ZUPT |

- EKF vypnutý = čistá poloha z X20.

## 5. Výstupy

- **BT Classic SPP:** NMEA z EKF – GGA, RMC, GST 5 Hz; GSA, GSV 1 Hz. Kvalita v GGA odpovídá skutečnosti (4 RTK fix, 5 float, 2 HAS/DGNSS, 1 GNSS, 6 jen IMU).
- **SPP zpět:** RTCM z NTRIP klienta v mobilu (SW Maps, Lefebure) → ESP → UART1 X20.
- **Vlastní zpráva pro aplikaci (SPP):** čas GNSS, poloha, σ, zdroj polohy, délka výpadku, rychlost/kurz/klid, satelity a PDOP, rušení a OSNMA, baterie, stav korekcí (stáří RTCM, Float/Fix), stav logování RAW.

## 6. Signalizace a napájení

- Bzučák (pasivní, PWM) + WS2812: Fix, výpadek, slabá baterie, konfigurační režim, profil.
- Baterie: varování, pak řízené vypnutí – X20 do úsporného režimu (UBX-RXM-PMREQ), ESP deep sleep.

## 7. Servis

- USB: průchozí režim pro u-center, ladicí výpis.
- ~~OTA přes Wi-Fi~~ – zrušeno ve 0.6.0, nahrávání jen přes USB (automatický reset do bootloaderu teď funguje bez drátu na GPIO0).

## 8. Doporučení k doplnění

1. **Tabulka oddílů s OTA:** současná `huge_app.csv` nemá druhý oddíl pro OTA. Použít vlastní tabulku 4 MB: 2 × app (~1,9 MB) + LittleFS pro profily a web. Ověřit, že BT Classic + BLE + Wi-Fi + web se do 1,9 MB vejde.
2. **Záchranný režim:** při opakovaném pádu firmwaru po startu (crash loop) naběhnout rovnou do AP s OTA, aby šel přístroj opravit bez USB. Po nepovedeném OTA automatický návrat na předchozí verzi (rollback).
3. ~~Stavová stránka~~ – zapracováno jako stránka Statistika (kap. 2).
4. **Kontrola X20 po startu:** automatická detekce rychlosti UART1, kontrola verze FW; při chybě varování bzučákem/LED.
5. **Stav antény** (UBX-MON-RF): varování při odpojené nebo zkratované anténě; indikátor rušení.
6. **Export/import profilů** (soubor JSON) – přenesení nastavení na další kus (např. sestava pro kolegu).
7. Watchdog na hlavní úlohy (čtení X20, IMU, BT).

## 9. Pořadí vývoje

1. Jádro: konfigurace X20 po startu, čtení UBX, IMU, PPS.
2. BT SPP: NMEA výstup + příjem RTCM.
3. Konfigurační režim: AP, captive portal, profily, OTA, průchozí režim, tabulka oddílů.
4. Signalizace a hlídání baterie.
5. Test přesnosti (kap. 12).
6. Protokol pro geodetickou aplikaci (SPP).
7. EKF naostro, auto dynamický model.
8. Volitelné: RAWX přes SPP, geodetická tyč.

## 12. Test přesnosti (záložka webu „Test“)

**Kontrolní bod**
- Zadání: název, druh, φ, λ (ETRS89), elipsoidická výška h, S-JTSK Y, X, Bpv, kvalita polohy (přesná / přibližná / nevyhodnocovat), poznámka. Zdroj: geodetické údaje ČÚZK (TB mají ETRS89 φ, λ, h; nivelační body jen Bpv a digitalizované S-JTSK).
- Bez φ, λ se poloha dopočte ze S-JTSK ve stránce (Křovák + 7 parametrů Bessel → GRS80, ověřeno na TB 10 Holý kopec: 0,35 m) a bod se označí jako přibližný.
- Výška: h má přednost (ΔU z elipsoidu); bez h se Bpv porovnává s h_msl přijímače (geoid u-blox, ne kvazigeoid ČR).
- Seznam bodů uložený v přístroji (LittleFS, JSON) – dostupný z libovolného mobilu.
- Výška antény v profilu: značka → ARP + posun fázového centra (PCO). Stejnou hodnotu později použije geodetická tyč.

**Živé porovnání**
- ΔN, ΔE, ΔU a 2D vzdálenost od bodu [m]; srovnání skutečné chyby s hAcc/vAcc z X20 (odhalí přehnaný optimismus).
- Graf rozptylu se středem ve skutečném bodě, měřítko v cm.
- Průměrování po zvolenou dobu: průměr, σ, RMS, CEP50/95, 2DRMS.

**Automatický test doby do fixu (běží v ESP, ne v prohlížeči)**
- Volba: horký / teplý / studený start, počet opakování, časový limit.
- Měří čas do: 3D fixu, korekcí (HAS/DGNSS), RTK float, RTK fix, dosažení zadané přesnosti vůči bodu (např. < 1 m, < 10 cm).
- Tabulka pokusů, min / medián / max, export CSV. Test pokračuje i při zhasnutém mobilu, výsledky se načtou po obnovení stránky.

**Souřadnicový systém**
- X20 se samotným HAS dává polohu v ITRF, body ČÚZK jsou v ETRS89 – rozdíl v ČR cca 0,8–0,9 m (pohyb desky). RTK z VESOG dává ETRS89.
- Přepočet ITRF2020 → ETRS89 (transformace s epochou) přímo ve stránce, volba podle zdroje korekcí.

**Testovací režim: Wi-Fi AP + BT současně**
- Test RTK potřebuje NTRIP z mobilu přes BT; konfigurační režim BT vypíná. Testovací režim drží Wi-Fi AP i BT SPP najednou (ESP32 koexistence, nižší propustnost, RAM cca 90 kB volná).
- Mobil připojený k Wi-Fi roveru, NTRIP jde přes mobilní data.

## 10. Budoucí rozšíření (po zprovoznění HW a FW)

- **Konfigurace a statistika přes BT bez Wi-Fi AP:** BLE GATT služba „Rover config“ (čtení/zápis profilů a nastavení, živá statistika, přepnutí profilu).
- Nahrazeno samostatnou geodetickou aplikací pro Android (viz `rover-app-specifikace.md`).
- Protokol navrhnout už teď tak, aby web (AP) i BLE používaly stejný JSON formát nastavení a statistiky.
- OTA zůstává přes Wi-Fi (přes BLE by trvalo minuty).

## 11. Stav vývoje

**Krok 1 (verze 0.1.0) – napsáno, přeloženo, na HW zatím netestováno.** Projekt `Documents\PlatformIO\Projects\Rover`, testovací firmware přesunut do `Projects\Rover_test`.

- Moduly: `config.h` (piny, konstanty), `ubx` (parser, CFG-VALSET), `gnss` (detekce rychlosti, přepnutí UART1 na 460800 jen v RAM, konfigurace RAM: UBX in/out, RTCM3 in, NMEA out vypnuto, NAV-PVT 5 Hz, NAV-SAT 1 Hz, dyn. model pedestrian; při NAK zkouší klíče jednotlivě; PPS s časem a zpožděním PVT za PPS; průchozí režim), `imu` (120 Hz přes INT1 s časovou značkou, kalibrace biasu gyra do NVS), `power` (baterie, kalibrace do NVS, odhad %), `button` (krátký stisk / podržení 2 s).
- Konzole: `h s i c m g k b r`.
- Tabulka oddílů `partitions.csv`: app0/app1 po 1,875 MB (OTA), LittleFS 128 kB (oddíl `spiffs`), coredump 64 kB.
- Ověřeno překladem: BT Classic + BLE + Wi-Fi AP + DNS + web + OTA + LittleFS = 1,70 MB z 1,97 MB (86 %). Rezerva cca 270 kB pro vlastní kód; HTML stránky ukládat do LittleFS.

**Krok 1 – ověřeno na HW (9. 10.):** X20 na 460800 Bd, konfigurace RAM OK, NAV-PVT 5 Hz, NAV-SAT, PPS (perioda 0,999994 s), zpoždění NAV-PVT za PPS 13,5–14 ms, 0 chyb UBX.

**Verze 0.1.1:** IMU přes FIFO s časovými značkami čipu (převod na esp_timer s průběžnou korekcí měřítka), fronta vzorků pro EKF, větší vysílací buffer konzole, opravený první řádek živého výpisu.

**Verze 0.2.0 (krok 2) – přeloženo, na HW netestováno:**
- Bluetooth SPP `ROVER-xxxx` (podle MAC), duální režim BT Classic + BLE ponechán.
- NMEA 4.10 z dat X20: GGA, RMC, GST 5 Hz; GSA (po konstelacích, system ID), GSV (GP/GL/GA/GB/GQ/GI) 1 Hz. Kvalita GGA: 1 GNSS, 2 korekce/HAS, 4 RTK fix, 5 float, 6 DR. Stáří korekcí z NAV-PVT (flags3). GST ze směrodatných odchylek NAV-COV, jinak z hAcc/vAcc. HDOP z NAV-DOP.
- Příjem z mobilu přes SPP → beze změny do UART1 X20; RTCM3 rámce se jen počítají (CRC-24Q, typ, stáří).
- NMEA se odesílá po dávkách; když klient nečte (zápis > 50 ms), dávky se 1 s zahazují – smyčka se neblokuje.
- Konzole: nový příkaz `n` (NMEA do konzole), v živém výpisu řádek BT/RTCM/kvalita/σ.
- Platforma připnutá na pioarduino 55.03.312 (Arduino 3.3.12) – jádro 4.x už BluetoothSerial (SPP) nemá.
- Velikost: 1,11 MB z 1,97 MB (56 %).

**Verze 0.2.1–0.2.2 – ověřeno na HW (9. 10.):**
- BT SPP: párování „Just Works“ (bez potvrzování čísla), události připojení/párování v konzoli. SW Maps: Generic NMEA, NTRIP přes mobil → RTK Float 11 cm / 21 cm v autě (VESOG Plzeň, ~68 km).
- IMU: reset čipu, zápis registrů s ověřením zpětným čtením (3 pokusy), hlídání – 1 s bez vzorků → nové nastavení.
- Občasné „IMU: nenalezeno“ po RST/nahrání: zablokovaná sběrnice I2C (IMU drží SDA v nule) – po vypnutí napájení IMU nalezeno, 119 Hz.

**Verze 0.3.0 (krok 3) – přeloženo, na HW netestováno:**
- IMU: před startem I2C uvolnění sběrnice (9 pulzů SCL + STOP); vyprázdnění FIFO po inicializaci (odstraní přetečení při startu).
- Profily v LittleFS (`/profiles.json`, max. 10): název, frekvence 1/5/10 Hz, dynamický model (chodec/auto/statika/automaticky – zatím jako chodec), elevační maska (CFG-NAVSPG-INFIL_MINELEV), výběr vět NMEA; EKF, bzučák, hlasitost, jas LED se ukládají, účinek od kroků 4 a 6. Výchozí profily: Chodec 5 Hz, Auto 5 Hz, Statika 1 Hz.
- Krátký stisk tlačítka = další profil (hned se pošle do RAM X20). Konzole `p` / `p2`.
- Nastavení přístroje (`/device.json`): heslo Wi-Fi (výchozí = název malými písmeny, např. `rover-303e`), název BT a Wi-Fi.
- Konfigurační režim: podržení tlačítka 2 s nebo `w` → BT vypnuto, Wi-Fi AP `ROVER-xxxx` (WPA2), captive portal (DNS), `http://192.168.4.1/`. Konec: „Uložit a restartovat“, podržení tlačítka, 10 min bez požadavku z prohlížeče.
- Webové menu (stránka vložená ve firmwaru, zdroj `web/index.html` → `web/build_page.py` → `src/webpage.h`): Stav (řešení, rozptyl polohy s exportem CSV, komunikace, IMU, přístroj), Satelity (sky plot, C/N0 po konstelacích), Profily (úpravy, aktivace, výchozí, kopie, smazání, export/import JSON), Přístroj (Wi-Fi/BT, kalibrace baterie a gyra, průchozí režim X20, OTA).
- Průchozí režim X20 z webu: USB ↔ UART1 bez blokování, web běží dál; konec tlačítkem na webu nebo podržením tlačítka přístroje, pak se X20 znovu nastaví.
- OTA: nahrání `firmware.bin` přes web do druhého oddílu, restart.
- Záchranný režim: 3 pády po sobě (panic/watchdog) → jen Wi-Fi AP s OTA a restartem. Počítadlo se nuluje po 60 s běhu nebo jiném důvodu startu.
- Knihovna ArduinoJson 7.4.2.
- Velikost: 1,80 MB z 1,97 MB (92 %). **Pro krok 5 (BLE) bude místa málo** – řešení: zvětšit oddíly aplikace na úkor coredump (64 kB) a LittleFS, rozhodnout před krokem 5.

**Verze 0.3.1:** oprava grafů webu na mobilu (výška plátna při hustotě displeje > 1). Ověřeno na mobilu.

**Verze 0.3.2 – přeloženo, na HW netestováno:**
- Při startu hledání X20 opakovaně až 15 s – po zapnutí napájení X20 nabíhá déle než ESP (jinak „X20 neodpovídá“ a 0 B až do restartu).
- Hlídání za provozu i v konfiguraci: 3 s bez platné zprávy UBX → X20 znovu najít a nastavit (další pokus nejdřív za 10 s). Počet na webu „Nové hledání X20“.
- Restart příjmu X20 (UBX-CFG-RST, resetMode 0x02 = jen GNSS, konfigurace RAM zůstane): horký / teplý / studený – web Přístroj, konzole `xh` `xw` `xc`. Doba do fixu (TTFF) na webu ve Stavu a v konzoli.
- Otevřené: přesnost v SW Maps (GST) se nemění (3,8 / 2,5 m) – ověřit, zda se mění σ z NAV-COV, nebo jen hAcc/vAcc.

**Verze 0.3.3 – přeloženo, na HW netestováno:**
- Obsahuje 0.3.2 (opakované hledání X20, hlídání UBX, restart X20 horký/teplý/studený).
- Tlačítko Wi-Fi/profil přesunuto na **GPIO13** (přepájeno). BOOT (GPIO0) vyveden, za běhu čitelný.
- NMEA bez fixu: GST s prázdnými poli místo nesmyslných odchylek; GLONASS s neznámým slotem (svId 255) se nevypisuje v GSA/GSV (dříve číslo 319).
- Testy hardwaru v konzoli: `t1` tlačítka (stav, krátký stisk, podržení, BOOT), `t2` piezo (tóny 500–4000 Hz, signály fix/varování/profil), `t3` WS2812, `t4` souhrn (I2C sken, X20, PPS, baterie, tlačítka, paměť).
- Modul `buzzer` (LEDC tón) – základ pro krok 4.
- Velikost: 1,83 MB z 1,97 MB (93 %).

**Verze 0.4.0 (krok 5 – test přesnosti) – přeloženo, na HW netestováno:**
- **Nová tabulka oddílů:** app0/app1 po 0x1F0000 (1,94 MB), LittleFS 64 kB, coredump zrušen. Nutné jednou nahrát přes USB; profily, body a nastavení Wi-Fi/BT se smažou (výchozí hodnoty). NVS (kalibrace baterie, bias gyra) zůstává.
- Profil: výška antény (značka → ARP) a fázové centrum (ARP → APC).
- Záložka **Test**: kontrolní body (ETRS89, max. 30, LittleFS `/points.json`, zadání desetinně i °'"), volba systému polohy přijímače (auto/ETRS89/ITRF), živá odchylka ΔN/ΔE/ΔU a 2D, poměr chyba/hAcc, graf se středem v bodě a kruhem hAcc, průměrování s historií (10 výsledků) a CSV, automatický test TTFF (horký/teplý/studený, cíl 3D/korekce/float/fix/přesnost, opakování, limit, pauza; tabulka, min/medián/max, CSV).
- Výpočty v ESP (`testctl`, `geo`): ECEF/ENU, ITRF2020 → ETRF2020 (rotace Eurasie od 1989.0: 0,086/0,519/−0,753 mas/rok; v ČR 2026 ≈ N −57 cm, E −74 cm). ČÚZK používá ETRF2000 – rozdíl řádu cm.
- **Testovací režim** (Wi-Fi AP + BT současně): z webu (Test → Restart do testovacího režimu) nebo konzole `wt`; platí jen do dalšího restartu, bez 10min limitu. Konfigurační režim se během běžícího testu TTFF sám neukončí.
- Velikost: 1,88 MB z 1,94 MB (93 %). **BLE (krok 6) se do zbývajících ~150 kB nejspíš nevejde** – rozhodnout před krokem 6 (BLE místo SPP, nebo NimBLE, nebo modul s 8 MB flash).

**Verze 0.5.0 – přeloženo, na HW netestováno (12. 10.):**
- Vyhodnocení měření 11. 10.: Kraví hora (TB 10) 0,46 m po přepočtu ITRF → ETRS89 při hAcc 0,12 m (3,7× optimistické), výška +0,9 m; Lázně Kynžvart (Aa4-18.1, les) rozptyl 2DRMS 1,7 m, výška −0,3 m vůči Bpv. Korekce byly SBAS (HAS vypnuté). Mezery v exportu (10–18 %) způsobilo dotazování z prohlížeče, ne X20.
- Webová stránka ve firmwaru zabalená gzipem (66,6 kB → 20,2 kB, `Content-Encoding: gzip`), `web/build_page.py` generuje `PAGE_GZ`.
- Tlačítko: podržení 2 s v provozu = konfigurační režim (2 pípnutí); podržení **5 s** v konfiguraci/testu = restart do provozu s BT (dva klesající tóny). Stisk, kterým se konfigurace zapnula, ji neukončí.
- Bzučák: neblokující signály (`buzzer::beep`, `tones2`, `poll`), respektuje volbu bzučáku v profilu.
- Kontrolní body: rozšířené pole (druh, S-JTSK, Bpv, kvalita polohy, poznámka), tolerantní zadání úhlu, výběr bodu nezahazuje rozepsané změny bez potvrzení, „Nový bod“ už neukládá okamžitou polohu (samostatné tlačítko „Vložit aktuální polohu“ s odečtenou anténou).
- **Měření bodu** (modul `survey`, nahrazuje průměrování): odpočet → ustálení (hAcc 10 s neklesá o > 1 cm, volitelně pod mezí, s limitem) → sběr 1 Hz po zadanou dobu → vyřazení odlehlých (> 3σ, min. 2 cm / 3 cm) → průměr, σ N/E/U, CEP50/95, počty RTK fix/float/korekce, hAcc/vAcc → uložení. Cíl: vybraný bod (Δ vůči bodu), nový bod (z průměru se založí bod – značka pod anténou), bez bodu. Bzučák: 1× ustálení, 2× start sběru, dva tóny hotovo, 3× hluboký chyba. Běží v ESP, konfigurační režim se během měření sám neukončí.
- Souhrny měření v LittleFS `/meas.json` (max. 20, nejstarší se přepisuje), číslování měření po bodech, tabulka „Uložená měření“ s Δ vůči bodu a vůči průměru měření bodu (opakovatelnost, RMS), export CSV, mazání jednotlivě i po bodu.
- Vzorky posledního měření v RAM (28 B/s, při nedostatku paměti se ukládá každá n-tá sekunda), export `/api/meas/raw` (CSV: čas, φ, λ, h_ell, h_msl, fix, RTK, korekce, hAcc, vAcc, SV, vyřazeno).
- Export rozptylu na záložce Stav doplněn o h_ell.
- Velikost: 1,89 MB z 1,94 MB, volné ~135 kB.
- Otevřené: rozlišení SBAS / HAS ve zdroji korekcí a přepínače SBAS/HAS v profilu (ověřit klíče CFG pro X20P); dlouhý záznam a RAWX až s microSD (SPI: SCK 18, MISO 19, MOSI 23, CS 27/32).

**Verze 0.5.1 – přeloženo, na HW netestováno (12. 10.):**
- Wi-Fi AP a BT běží v konfiguraci současně (dřívější „testovací režim“ je teď běžný stav konfigurace; tlačítko „Restart do testovacího režimu“ odstraněno).
- Wi-Fi zap: podržení tlačítka 2 s; vyp: podržení 5 s nebo „Uložit a vypnout Wi-Fi“ – bez restartu, data měření v RAM zůstanou. 10min limit nečinnosti zrušen.
- Odhad spotřeby: provoz 0,85 W (~230 mA z baterie, 11–12 h z 3 Ah); Wi-Fi AP přidá cca 80–110 mA → 8–9 h. Ověřit měřením.
- Měření bodu – pohyb přístroje (IMU 120 Hz): pohyb = |ω| > 15 °/s nebo ||a| − 1 g| > 0,2 g. Krátký otřes (pták, vítr) jen vyřadí epochy během něj a 1,5 s po něm; souvislý pohyb > 3 s (mezery do 0,4 s) = odneseno → konec, uloží se epochy před pohybem (bez posledních 2). Záloha z GNSS: 5 epoch po sobě dál než max(2 m, 10σ) od průměru. Během odpočtu se pohyb nehlídá, během ustálení otřes ustálení restartuje.
- Souhrn měření: epochy vyřazené kvůli pohybu, počet otřesů, důvod konce (doba / ručně / odneseno). CSV vzorků: sloupec vyřazeno 1 = odlehlé, 2 = pohyb.
- Velikost 1,90 MB z 1,94 MB, volné ~133 kB.

**Verze 0.6.0 – přeloženo, na HW netestováno (12. 10.):**
- Na HW se ukázalo: Wi-Fi AP vedle BT Classic + BLE nenaběhne (málo RAM, „Expected to init 4 rx buffer, actual is 0“), ani při startu (`wt`). Testovací režim z 0.4.0 tedy nikdy nefungoval.
- BT jen Classic: `bt.begin(name, false, true)` uvolní paměť řadiče pro BLE. **BLE do restartu nejde zapnout** – data pro aplikaci jdou přes SPP.
- Pomocná kopie bodů při načítání jen dočasně na haldě (−4,3 kB statické RAM).
- Záloha: když AP ani tak nenaběhne, BT se po dobu zapnuté Wi-Fi vypne a po vypnutí AP znovu spustí (bez restartu). Konzole vypisuje volnou RAM před a po startu AP.
- **OTA zrušeno**: tabulka oddílů nvs + factory 3,625 MB + LittleFS 320 kB (`spiffs` 0x3B0000). Nutné nahrát přes USB; LittleFS (profily, body, měření, nastavení Wi-Fi/BT) se smaže. NVS (kalibrace baterie, bias gyra) zůstává. Souhrnů měření max. 30.
- Velikost 1,89 MB z 3,625 MB.

**Verze 0.6.1 – přeloženo, na HW netestováno (12. 10.):**
- Naměřeno 0.6.0: s BT Classic (BLE uvolněno) volných 55,9 kB RAM, největší blok 49 kB – Wi-Fi AP nenaběhne. Vypnutí BT po nepovedeném startu Wi-Fi zamrzlo (ovladač Wi-Fi zůstal napůl inicializovaný).
- Oprava: před startem AP kontrola volné RAM (`WIFI_MIN_HEAP` 100 kB, `WIFI_MIN_BLOCK` 60 kB); při nedostatku se BT vypne **předem**. Po vypnutí AP se BT znovu spustí bez restartu (ověřit na HW).
- Důsledek: s otevřenou stránkou neběží BT (NTRIP, NMEA do SW Maps). Měření bodu běží dál, RTK korekce ale během zapnuté Wi-Fi nepřicházejí.
- Možnost do budoucna: zmenšit paměť BT/Wi-Fi vlastním sdkconfig (pioarduino `custom_sdkconfig`, překlad knihoven ESP-IDF) – např. vypnout BLE v Bluedroid, méně bufferů Wi-Fi.

## Plán – další úpravy (12. 10.)

**Rozptyl na záložce Stav v přístroji, jen v klidu**
1. Rozptyl se počítá v ESP, ne v prohlížeči (dnes se maže každým novým připojením / obnovením stránky).
2. Počítá se jen v klidu: IMU bez pohybu (meze jako u měření bodu) a rychlost GNSS < 0,3 m/s, obojí aspoň 10 s.
3. Při pohybu se rozptyl zastaví, stránka ukáže „pohyb – rozptyl se neměří“; ostatní hodnoty (fix, hAcc, satelity, poloha, rychlost) běží dál.
4. Po uklidnění (10 s) začne rozptyl znovu od nuly.
5. Statistika (průměr, σ N/E/U, CEP, počet epoch, doba klidu) průběžně z celé doby klidu; body pro graf a export v kruhovém bufferu 15 min po 1 s (~14 kB RAM).
6. Export CSV rozptylu z přístroje (s h_ell, bez mezer).
7. Detekce pohybu ve společném modulu `motion` (rozptyl + měření bodu).

**Paměť a Bluetooth**
- Varianta A: externí BT SPP modul (HC-05 / JDY-31) na UART2 – TXD → GPIO34, RXD ← GPIO33, STATE → GPIO39, EN/KEY ← GPIO32 nebo 27. ESP bez BT stacku: volná RAM ~150 kB (provoz), ~85 kB s Wi-Fi AP, Wi-Fi + BT současně, program o ~700 kB menší.
- Varianta B (bez HW změny): BT vypnuté po dobu měření bodu a zapnuté Wi-Fi, vypnutí Wi-Fi bez měření = restart; vzorky posledního měření do LittleFS `/raw.bin`. Výpis `t4` u vypnutého BT „BT vypnuto“.
- Naměřeno 0.6.1: vypnutí Wi-Fi a nový start BT bez restartu nechá volných jen 23,7 kB (únik paměti ovladače Wi-Fi / BT) – nepoužitelné.

**Bluetooth – fronta (oprava ve FW 0.8.2)**
- Počítadlo fronty BT po výpadku spojení trvale ukazuje zbytek (test 12. 10.: 942 B, pak 691 B) – zásobník BT část dat při zahlcení zahodí bez WRITE_EVT. Oprava: když fronta během 5 s ani jednou neklesne na nulu a potvrzení přitom chodí, zbytek odečíst jako ztracený.
- Část dat zůstává pod úrovní WRITE_EVT (řadič BT) – po výpadku přijde několik epoch starých až 16 s. Pro záznam nevadí (čas GNSS), aplikace to ukazuje ve zpoždění.
- Test 0.8.1: HPPOSLLH u 100 % epoch, přenos 2,0 kB/s, uvnitř bez ztrát (zpoždění 22 ms); výpadky 7 a 17 s jen při stínění zdí/tělem, ne při přepínání aplikací ani provozu Wi-Fi 2,4 GHz.

**Filtrace polohy**
- Pro statický bod Kalmanův filtr nad výstupem X20 přesnost nezlepší (X20 má vlastní navigační filtr; chyba je hlavně pomalá a korelovaná – vícecestné šíření, atmosféra, korekce). Pro statiku stačí průměr; zvážit vážení 1/hAcc² a delší dobu měření.
- EKF GNSS + IMU má smysl pro pohyb (přemostění výpadků, ZUPT, kurz) – krok 7.
- Skutečné zpřesnění: RTK, zkonvergovaný HAS, PPK z RAWX (RTKLIB) – vyžaduje microSD.

## Plán – měření za chůze (12. 10.)

Hlavní použití: měření za chůze (mapování), statické geodetické měření bodů. Auto občas, bez nároků na přesnost.

**Výstup roveru (5 Hz)** – vlastní zpráva přes SPP: čas GNSS (UTC s ms), φ, λ, h_ell, σN, σE, σU, kvalita (RTK fix / float / HAS / SBAS / 3D / DR), rychlost, kurz, počet SV, stáří korekcí, příznak pohyb/klid; navíc NMEA pro kompatibilitu. Formát v `rover-app-specifikace.md`.

**Rover za chůze**
- Dynamický model chodec, 5 Hz; automotive automaticky podle rychlosti.
- EKF GNSS + IMU: vyhlazení trajektorie, přemostění výpadků pod stromy, kurz i při pomalé chůzi, σ pro každou polohu.
- Záznam RAWX → PPK (RTKLIB, CZEPOS / vlastní báze) a zpětné vyhlazení trajektorie (RTS) – záznam v aplikaci (telefon), microSD nepotřeba.

**Pořadí** – viz `rover-app-specifikace.md` (etapy aplikace a úpravy ESP).

## Test 12. 10. – Kraví hora (TB 10), FW 0.6.1, HAS

- Měření bodu 300 s, 300 epoch, ustálení 8 s (HAS už zkonvergovaný), σ N/E/U 4,7 / 5,6 / 4,8 cm, CEP95 14 cm, hAcc 4,5 cm, 32 SV.
- Souhrn: ΔN +0,695, ΔE +0,728, 2D 1,006 m, ΔU −0,085 m – **chybně bez přepočtu ITRF → ETRS89**. Po přepočtu (N −0,57, E −0,74): ΔN +0,125, ΔE −0,014, **2D 0,13 m**, ΔU −0,085 m.
- **Chyba:** X20 hlásí řešení HAS jako `carrSoln = 1` („RTK float“) i bez RTCM. Režim „automaticky“ to bere jako RTK ze sítě (ETRS89) a nepřepočítá.
- **Oprava (připravit):** ETRS89 jen když za posledních 30 s přišlo platné RTCM z mobilu, jinak ITRF; bez RTCM zobrazovat „HAS (PPP)“ místo „RTK float“ (stránka, statistika, NMEA kvalita GGA, souhrn měření). Do souhrnu a vzorků ukládat příznak „RTCM přijímáno“.
- Horký start (UBX-CFG-RST) při testu TTFF vynuluje konvergenci HAS (10:53:06 pád na 3D, hAcc 6,8 m, „float“ po 14 s s hAcc 1,3 m). Pro měření doby konvergence HAS: cíl přesnost 0,2 m vůči bodu, limit 1800 s, 2–3 pokusy.
- TTFF horký start: 3D fix 0,6–0,7 s; korekce 8,6–14,7 s.
- Export rozptylu ze Stavu (prohlížeč) má mezery 2–60 s – řeší plán „rozptyl v přístroji“.

**Měření 2 (11:26–11:31 UTC, 300 s, vzorky z přístroje bez mezer, ITRF → ETRS89 dopočteno ručně):**
- Celé: ΔN −0,228, ΔE +0,055, 2D 0,235 m, ΔU +0,091 m; σN 16 cm (drift), σE 3,6 cm, σU 10 cm.
- Po minutách ΔN: −0,49 / −0,29 / −0,18 / −0,11 / −0,06 m, hAcc 7,4 → 4,9 cm – **HAS ještě konvergoval** (po předchozím horkém startu). Poslední minuta: 2D 0,07 m, ΔU +0,08 m, σ 0,7 / 1,2 / 2,2 cm.
- **Ustálení selhalo:** kritérium „hAcc za 10 s neklesne o > 1 cm“ splní i konvergující HAS (hAcc klesá ~1 cm/min). Navrhnout: ustálení podle polohy (posun průměru 30 s oken < 2 cm za 60 s) a/nebo hAcc pod mezí; delší okno.
- Rozlišení polohy z NAV-PVT je 1e-7° (1,1 cm N / 0,7 cm E) – pro statiku zvážit UBX-NAV-HPPOSLLH (0,1 mm).
- Fix ve vzorcích „3D“ + rtk 1 + korekce 1 = HAS (PPP) – potvrzuje chybu automatického ITRF/ETRS89.

**Verze 0.7.0 – přeloženo, na HW netestováno (12. 10.):**
- Systém polohy „automaticky“: ETRS89 jen když za posledních 30 s přišlo platné RTCM z mobilu (`gnss::rtcmActive`), jinak ITRF – i při `carrSoln > 0` (HAS/PPP).
- Označení: bez RTCM `carrSoln 1` = „HAS“, `2` = „PPP fix“; NMEA GGA kvalita 2 (ne 5/4). Měření bodu počítá epochy HAS zvlášť (`hs`), vzorky mají příznak RTCM.
- Ustálení podle stability polohy: průměr posledních 30 s proti průměru 30 s o minutu dřív, posun < 5 cm vodorovně a < 8 cm svisle (+ volitelně hAcc pod mezí); nejdřív po 90 s, výchozí limit 900 s. Meze odvozené z dat 12. 10. (zkonvergovaný HAS bloudí 1–15 cm/min, konvergence 20–50 cm/min); simulace: konvergující měření by ustálení splnilo v 11:31 (chyba 7 cm).
- Přesná poloha z UBX-NAV-HPPOSLLH (0,1 mm, klíč CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1 0x20910034), spojení s NAV-PVT podle iTOW (`Pvt.hp`).

**Verze 0.8.0 – přeloženo, na HW netestováno (12. 10.):** protokol pro aplikaci RoverGnss (popis polí: `rover-app-specifikace.md`, kap. 5).
- Modul `proto`: `$PRVR,POS` každá epocha, `$PRVR,STA` 1 Hz, `$PRVR,SAT` 1 Hz (po 12 družicích) – do SPP vedle NMEA, jen když je klient připojen.
- `btlink`: z proudu z mobilu se oddělují řádky JSON (`{` mimo rámec RTCM, do `\n`, max. 511 znaků); ostatní jde beze změny do X20.
- Příkazy JSON: `get info`, `get profiles`, `profile` (přepnutí), `x20 reset` (hot/warm/cold); odpověď `{"id":N,"ok":…}`.
- Pohyb pro POS z IMU (15 °/s nebo 0,2 g, drží 1,5 s) – nezávisle na měření bodu.
- `survey::phase()`, `survey::phaseSeconds()` pro STA. Wi-Fi a web beze změny.

**Test 0.8.0 s aplikací (12. 10., parapet, HAS):** zkonvergovaný HAS σN 7, σE 13, σU 14 cm (multipath od zdi). HPPOSLLH jen u 37 % epoch – NAV-PVT spouštěl odeslání dřív, než přišel HPPOSLLH téže epochy. Při přepínání aplikací v telefonu se BT na 9–17 s zasekl (spojení drželo); rover hromadil data ve frontě BluetoothSerial (32 dávek, volná RAM 47 → 35 kB), pak zahazoval a zpoždění dosáhlo 17,5 s. Podezření na souběh Wi-Fi 2,4 GHz a BT v telefonu.

**Verze 0.11.0 – přeloženo, na HW netestováno (14. 10.):** asistence X20 z telefonu (`rover-app-specifikace.md`, kap. 7c).
- `btlink`: rozpoznání rámce UBX z mobilu (`0xB5 0x62`, délka, kontrolní součet). Rámec jde celý do UART1 X20, uvnitř se nehledá začátek JSON (bajt 0x7B v datech MGA-INI by dřív rámec rozbil). Počítadla rámců OK / vadných.
- `gnss`: UBX-MGA-ACK a UBX-MGA-DBD z X20 jdou celé do BT (jako RAWX). MGA-DBD mimo limit fronty `BT_MAX_BACKLOG` (`btlink::sendNow`), aby se výpis databáze nezahodil. Počítadla MGA-ACK přijato / odmítnuto a vyslaných MGA-DBD.
- Konfigurace RAM: `CFG-NAVSPG-ACKAIDING` (0x10110025) = 1 zvlášť před hlavní konfigurací; NAK neshodí ostatní klíče, výsledek ve statistice.
- `$PRVR,STA` na konci navíc: UBX z mobilu OK, UBX vadné, MGA-ACK přijato, MGA-ACK odmítnuto, vyslané MGA-DBD, ACKAIDING (1 OK / 0 NAK / −1 bez odpovědi).
- Ověřit na HW: podpora MGA-INI, MGA-DBD a ACKAIDING v HPG 2.11; velikost výpisu databáze a zda se při něm neztrácejí epochy NAV-PVT.

**Verze 0.10.0 – přeloženo, na HW netestováno (13. 10.):**
- Rozdělení flash: aplikace 2,25 MB (FW 1,89 MB), LittleFS 1,69 MB. Po nahrání se LittleFS naformátuje (profily, body, měření se smažou).
- Zásobník epoch při výpadku BT (`epochbuf`): aplikace při záznamu trasy nebo měření bodu pošle `{"cmd":"rec","op":"start"}` (příznak v NVS, platí i po restartu), po konci `stop` (zásobník se smaže). Při odpojeném BT jde každá epocha binárně (72 B) do `/buf.bin` + `/buf.old` (kruhově po polovinách, místo = volné LittleFS bez 128 kB rezervy, ~21 000 epoch ≈ 6 h při 1 Hz, 1,2 h při 5 Hz). Při zjištění výpadku se uloží i posledních 32 epoch.
- `{"cmd":"buf","op":"get"}` → odpověď `{"n":…}`, pak `$PRVR,BUF,…` (pole jako POS) po 6 větách podle volné fronty BT a `$PRVR,BUFEND,počet`; `{"cmd":"buf","op":"clear"}` zásobník smaže. Výpadek během odesílání: zásobník zůstává.
- Surová data IMU při záznamu (`imustream`): rámec UBX třídy 0xF5, ID 0x01 každých 0,5 s – 30 vzorků 60 Hz (každý druhý ze 120 Hz), zrychlení [mg] a gyro [0,02 °/s] s časem esp_timer, k tomu iTOW a čas příjmu poslední NAV-PVT, poslední PPS a bias gyra (formát v `imustream.h`). Jen při připojeném BT (do zásobníku se nepíše), ~0,9 kB/s. Aplikace je ukládá se surovým proudem trasy – podklad pro krokový dopočet a EKF.
- Statický model / RAWX měření bodu se při odpojení BT nevypíná, když aplikace zaznamenává (měření po připojení pokračuje).

**Verze 0.9.0 – přeloženo, na HW netestováno (13. 10.):**
- Příkaz JSON `{"cmd":"meas","op":"start","static":true,"raw":false}` / `{"cmd":"meas","op":"stop"}` – měření bodu z aplikace: X20 ve statickém modelu (CFG-NAVSPG-DYNMODEL 2, jen RAM) a volitelně RXM-RAWX (1 Hz) a RXM-SFRBX na UART1. Odpověď `{"ok":true,"static":…,"raw":…,"ack":…}` (ack = X20 potvrdila VALSET).
- Rámce RAWX/SFRBX jdou do BT celé (mimo dávky vět); aplikace je ukládá do `.ubx` k měření (PPK, RTKLIB).
- Stav měření platí i při změně profilu (configure() bere statický model a RAWX v úvahu). Odpojení BT (konec režimu aplikace) měření ukončí – model z profilu, RAWX vypnuto.

**Verze 0.8.2 – přeloženo, na HW netestováno (13. 10.):**
- Tlačítko: podržení 2 s zapíná i vypíná Wi-Fi AP (dřív vypnutí až po 5 s).
- Baterie: vzorek 1× za s, exponenciální průměr ~60 s; jednotlivé špičky > 0,3 V se ignorují, trvalá změna (nabíječka) se převezme po 5 s. Procenta z vyhlazeného napětí s hysterezí (bez nabíjení jen klesají). Mimo 2,5–4,4 V = chyba měření (web, konzole; aplikace podle rozsahu).
- Kalibrace baterie: ve V i mV, desetinná čárka i tečka (konzole `k3,86`, web), průměr posledních 5 s, koeficient jen 1,6–2,4; nesmyslný uložený koeficient se při startu ignoruje.
- `t4`: okamžité i vyhlazené napětí, rozptyl vzorků na pinu, upozornění na nabíjení při USB.
- Fronta BT: když 5 s neklesne na nulu a potvrzení chodí, nejmenší zbytek se odečte jako ztracený.
- Zjištěno 13. 10.: v režimu BT (aplikace) ukazovala baterie ~5,4 V, web (Wi-Fi) správně ~3,8 V, multimetr 3,48 V – podezření na rušení vysíláním BT na vodiči k GPIO35 (vysoká impedance děliče). Návrh: 100 nF (+10 nF) přímo GPIO35–GND u ESP.

**Verze 0.8.1 – přeloženo, na HW netestováno (12. 10.):**
- Konec epochy z UBX-NAV-EOE (klíč CFG-MSGOUT-UBX_NAV_EOE_UART1 0x20910160): NMEA i `$PRVR,POS` až po EOE, takže HPPOSLLH a COV jsou z téže epochy. Když X20 EOE neposílá (3 s), odesílá se hned po NAV-PVT; čekání nejvýš 150 ms.
- Fronta BT: počítání odeslaných a potvrzených bajtů (`register_callback`, ESP_SPP_WRITE_EVT); nad `BT_MAX_BACKLOG` 2048 B se dávky zahazují. Po 10 s bez potvrzení se počítadlo vynuluje (data ztracená v zásobníku).
- Režim aplikace (první příkaz JSON, do odpojení): NMEA jen GGA a RMC, `$PRVR,SAT` každé 2 s.
- STA: fronta BT, režim aplikace, EOE, počet epoch s HPPOSLLH.
- Síla signálu (RSSI) zatím ne: čtení `esp_bt_gap_read_rssi_delta` vrací výsledek jen do GAP callbacku, který si drží BluetoothSerial (párování) – vlastní callback by ho nahradil. Kvalitu spojení zatím hodnotí aplikace podle zpoždění, ztrát a fronty.
