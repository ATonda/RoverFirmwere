# Rover ↔ CurieFinder – rozhraní (FW 0.12.0, 19. 10. 2026; režim USB host 28. 10. 2026)

Doplněk k `rover-firmware-specifikace.md` a `rover-app-specifikace.md` (kap. 5 – pole POS/STA).

## FW 0.12.0 – změny (přeloženo nebylo, na HW netestováno)
- Odpověď na `{"cmd":"profile","i":N}` obsahuje navíc `"hz"` (frekvence aktivního profilu).
- `$PRVR,STA`: nové poslední pole „frekvence navigace [Hz]“ aktivního profilu (za ACKAIDING z 0.11.0).
- `FW_VERSION` 0.12.0.

## Příkazy pro přepnutí na 5 Hz
```
→ {"id":1,"cmd":"get","what":"profiles"}\n
← {"id":1,"ok":true,"data":{"active":0,"default":0,"max":10,"profiles":[{"name":"…","rate":5,"dyn":"pedestrian",…},…]}}\r\n
→ {"id":2,"cmd":"profile","i":0}\n
← {"id":2,"ok":true,"i":0,"name":"…","hz":5}\r\n
← {"id":2,"ok":false,"err":"neplatný profil"}\r\n
→ {"id":3,"cmd":"get","what":"info"}\n
← {…,"prof":{"i":0,"n":3,"def":0,"name":"…","hz":5},…}\r\n
```
- id profilu = pozice v poli `profiles`; `rate` ∈ {1, 5, 10}.
- Výběr: první profil s `rate == 5`, přednostně `dyn == "pedestrian"`; jinak indikace „X20 1 Hz“.
- Platnost: do restartu roveru (jen RAM X20); odpojení klienta profil nevrací. Profil mění i krátký stisk tlačítka – stav hlídat přes STA / `get info`.

## Ověřeno ve FW
- POS každou epochu i v režimu aplikace; NMEA omezeno na GGA+RMC.
- POS i bez fixu: kvalita 0, prázdná poloha (čas jen při platném čase).
- Kvalita 6 = X20 jen dopočet (fixType 1) – existuje už dnes.
- `$PRVR` končí `*hh\r\n`, JSON odpověď `\r\n`; příkazy přijímá s `\n` i `\r`.
- Bez JSON příkazu plný proud NMEA.
- Znovupřipojení SPP do ~2 s: netestováno.

## Požadavky na CurieFinder (doplněk)
1. NTRIP v záložce externí GPS (caster, port, mountpoint, jméno, heslo); RTCM surově do SPP socketu roveru, GGA casteru ~10 s (VRS). S RTCM kvalita 4/5, poloha v systému sítě (CZEPOS = ETRS89).
2. Čas vzorku podle UTC z POS (pole 1), převod na hodiny telefonu mediánem (čas příjmu − UTC) z posledních desítek epoch; zpožděné epochy (BT zaseknutí až 16 s) podle UTC, ne času příjmu.
3. Kvalita 6: platná s omezenou dobou (např. ≤ 5 s), pak jako 0; do CSV q=6.
4. JSON odpověď může přijít ve více paketech – skládat do `\n`; řádky `{` oddělit od vět `$`.
5. Nepoužívat `rec` a `buf` (zásobník epoch a `$PRVR,BUF` patří RoverGnss).
6. Používat `id` pro párování odpovědí.
7. Jeden SPP klient: současně s CurieFinderem nepojede RoverGnss.

## Režim USB host (ESP32-S3) – FW 0.17.0, dohodnuto 28. 10. 2026, přeloženo, na HW netestováno

Sestava CurieFinder „dvě čidla“: ESP32-S3 je USB host, rover připojený kabelem USB-C. Protějšek: `CurieFinder_dve_cidla_spec.md` v projektu CurieFinder.

### Fyzická vrstva
- USB-C → CH340K → `Serial` (UART0) roveru, 115200 8N1. RX buffer `Serial` 4096 B (`Serial.setRxBufferSize`).
- RTS → EN na FireBeetle přerušené (reset po nahrání firmwaru tlačítkem RST). S3 drží DTR i RTS neaktivní.
- CH340K je napájený z roveru (VIN/VBUS desky z U3V 5 V, VBUS kabelu k ESP nevede) – připojení kabelu ESP neresetuje.
- VBUS z S3 → VIN nabíječky 6092, odběr až ~1 A (nabíjení) + rover. Rover jinak běží z baterie.
- Rover VBUS z USB nesleduje – odpojení pozná jen podle heartbeatu.

### Aktivace a heartbeat
```
→ {"cmd":"usbhost","hz":5,"model":"pedestrian","id":…}\n
← {"ok":true,"cmd":"usbhost","usbhost":true,"hz":5,"model":"pedestrian","profile":…,"name":"…","id":…}\r\n
← {"ok":false,"cmd":"usbhost","err":"hz: 1|5|10",…}\r\n
```
- `hz` ∈ {1, 5, 10}; `model` ∈ {`pedestrian`, `automotive`, `stationary`, `auto`} (`auto` = přepínání podle stavu pohybu).
- Režim mimo seznam profilů, jen v RAM (vzor měření rychlosti, vlastní stav). Nic do NVS ani LittleFS.
- Jen ze `Serial`; z BT odmítnuto (`err` „jen ze Serial (USB)“).
- Heartbeat = stejný příkaz 1×/s, odpověď na každý. Beze změny parametrů se X20 nepřenastavuje; při změně se přenastaví.
- 3 s bez heartbeatu → konec režimu, `applyProfile(active)`, konzole obnovena.
- Frekvence a model z příkazu; při pevném modelu `motion::setAuto(false)`. Maska, SBAS, HAS, výška antény, PCO z aktivního profilu.
- `navRateHz()` vrací frekvenci režimu. Zapnutí ukončí měření rychlosti a měření bodu (statický model, RAWX).

### Odmítnuto v režimu (`{"ok":false,"err":"režim USB host","usbhost":true,"hz":…,"id":…}`)
- `profile`, `speed start`, `meas start` (statický model při chůzi = chybná poloha, RAWX zatíží UART1), `passthrough` (průchozí režim používá `Serial`).
- `set profiles`: uloží se, aplikuje se až po konci režimu.
- Krátký stisk tlačítka: ignoruje se, krátké pípnutí 800 Hz „odmítnuto“. Podržení 5 s (restart) funguje.

### Výstup na `Serial`
- `$PRVR,POS` každou epochu (po NAV-EOE), `$PRVR,STA` 1×/s – nezávisle na připojení BT (dnes `sendPos`/`sendSta` bez BT končí).
- Formát stejný jako do BT. Do BT jde výstup dál beze změny.
- STA – nová pole na konci (0.17.0): `usbhost(0/1)`, `zdrojRtcm(0 žádný / 1 BT / 2 USB)`, `rtcmUsbOk`, `rtcmUsbChybné`. Pole `rtcmOk`, `rtcmChybné`, `stáříRtcm`, `typRtcm` uprostřed STA ukazují zdroj, který jde do X20. Pole BT (`btZahozeno`, `btFronta`, `režimAplikace`, `btPřijem…`) beze změny.
- Ostatní výpisy jdou přes `Con` (`console.h`) a jsou v režimu umlčené; příkazy písmeny a ozvěna NMEA (`n`) nejdou.
- Před první odpovědí může na `Serial` dojít ještě řádek konzole `[usb] režim USB host ZAP …`.
- Mimo kontrolu firmwaru: hláška ROM bootloaderu po resetu, výpis výjimky při pádu.

### Vstup ze `Serial`
- V režimu jen řádky JSON a rámce RTCM, stejný parser jako SPP s odděleným stavem pro každý zdroj.
- Do X20 jen celé rámce s ověřeným CRC – RTCM i UBX (MGA z mobilu), **obecně, i mimo režim a i pro BT**, každý rámec jedním zápisem. Ostatní bajty se zahodí. Parser `rxparse`: BT 2 × 4 kB, USB 2 × 2 kB.
- Řádek JSON musí začínat `{"` a mít platné UTF-8 (RoverGnss posílá `JSONObject.toString()`; CurieFinder a S3 ověřit), jinak se `{` bere jako binární bajt a data se zpracují znovu.
- V režimu má přednost RTCM z USB, RTCM z BT se zahazuje.
- `rtcmActive` (pole `rtcm` v POS, kvalita FIX/FLOAT vs. HAS) a `stáříKor` počítají i se zdrojem USB.

### Ochrana konzole
- Po konci režimu a po každém binárním bajtu na vstupu konzole se 5 s neprovádějí příkazy písmeny, řádky JSON ano (S3 může režim hned obnovit). Řádek s řídicím bajtem se neprovede nikdy.

### Restart ESP
- X20 se neresetuje (konfigurace jen do RAM, žádné `CFG-RST`); fix i konvergence HAS zůstanou.
- Spadne BT, platí výchozí profil, režim USB skončí – S3 ho obnoví dalším heartbeatem.
- Zásobník epoch (příznak v NVS, soubory v LittleFS) restart přežije; nezapsaný konec bloku se může ztratit.

### Požadavky na S3
- Parser bere jen `$PRVR` s platným kontrolním součtem a řádky JSON; ostatní ignoruje.

### Požadavky na aplikace
- `RoverSppManager` (CurieFinder) musí snést odmítnutý `profile` a vzít `hz` z chybové odpovědi.
- `RoverSppManager` čte STA podle pozic – nová pole na konci ignoruje (ověřit v kódu při úpravě aplikace).
