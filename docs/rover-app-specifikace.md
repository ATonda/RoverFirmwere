# Rover GNSS – geodetická aplikace pro Android a protokol rover ↔ aplikace (12. 10. 2026)

Samostatný projekt. Rover (ZED-X20P + ESP32) měří a řídí časově kritické věci, aplikace v telefonu
zajišťuje ovládání, NTRIP z internetu, zpracování, ukládání a export. Navazuje na `rover-firmware-specifikace.md`.

## 1. Rozdělení práce

| Rover (ESP32) | Aplikace (Android) |
|---|---|
| Čtení X20: NAV-PVT, NAV-HPPOSLLH, NAV-SAT, NAV-COV, NAV-DOP; volitelně RXM-RAWX/SFRBX | Spojení BT s roverem, automatické obnovení |
| IMU, detekce pohybu, později EKF GNSS + IMU | Klient NTRIP (RTCM do roveru, GGA zpět do casteru) |
| Měření bodu (odpočet, ustálení, sběr, souhrn) – běží i bez telefonu | Řízení měření, zobrazení průběhu, stažení výsledků a vzorků |
| Uložené nastavení, profily a **úloha po startu** (LittleFS) | Nastavení roveru (profily, X20, úlohy po startu, kalibrace) |
| Výstup polohy 5 Hz + NMEA | Mapa, body, trasy, vytyčení, transformace, export, záznam pro PPK |
| BT Classic SPP; Wi-Fi jen servisní režim | Databáze v telefonu |

- Rover bez telefonu: po zapnutí provede uloženou úlohu (např. změř bod / zaznamenávej / čekej). RTK z internetu jen s telefonem.
- **Wi-Fi v roveru:** jen servisní režim (podržení tlačítka při zapnutí, BT vypnuto) – nouzové nastavení a diagnostika. Běžně vše přes aplikaci. Uvolní RAM (~64 kB) a program (~0,5 MB).

## 2. Technologie aplikace

- **Kotlin, Jetpack Compose** (UI), **MapLibre Native** (mapy – vektorové i rastrové vrstvy, offline balíčky), **Room** (SQLite), **Kotlin Coroutines**.
- **Služba na popředí** (foreground service): BT, NTRIP, záznam běží se zhasnutým displejem.
- Vlastní implementace: Křovák a transformace ČÚZK s mřížkou oprav, kvazigeoid ČR (mřížka), ITRF2020 → ETRS89 s epochou, klient NTRIP (HTTP/TCP, NTRIP 1/2), parser protokolu roveru.
- Mapové podklady: OSM (vektor), ČÚZK WMS/WMTS (ortofoto, ZABAGED, katastrální mapa), offline dlaždice pro terén.

## 3. Funkce a obrazovky

| Obrazovka | Obsah |
|---|---|
| **Připojení** | výběr roveru (spárovaná BT zařízení), stav spojení, profily NTRIP (VESOG, CZEPOS, vlastní báze; caster, port, mountpoint, jméno/heslo, odesílání GGA), stáří a přenos RTCM |
| **Živě / mapa** | poloha na mapě, fix (RTK fix / float / HAS / SBAS / 3D / DR), σ N/E/U, hAcc/vAcc, PDOP, počet SV, stáří korekcí, rychlost, kurz, pohyb/klid, baterie roveru; souřadnice ETRS89 / S-JTSK / Bpv |
| **Satelity** | skyplot, C/N0 podle konstelací a pásem, použité / sledované |
| **Body** | seznam bodů zakázky, import (CSV, geodetické údaje ČÚZK – ruční zadání, S-JTSK + Bpv), kvalita polohy (přesná / přibližná / nevyhodnocovat), poznámka, foto |
| **Měření bodu** | cíl (existující / nový bod / bez bodu), odpočet, ustálení (stabilita polohy, hAcc, limit), doba sběru, volitelně jen RTK fix; průběh živě; souhrn (průměr, σ, CEP, počty fix/float/HAS, Δ od bodu); opakovaná měření a opakovatelnost |
| **Trasy** | záznam 1–10 Hz s σ a kvalitou, za chůze; zobrazení, statistika, filtrace podle σ / kvality |
| **Vytyčení** | navigace k bodu: vzdálenost, směr, ΔN/ΔE/ΔU, zvuková signalizace přiblížení |
| **Export** | CSV (ETRS89 / S-JTSK / Bpv, volba sloupců), GPX, KML, GeoJSON, DXF; vzorky měření; sdílení souboru |
| **PPK** | záznam RAWX/SFRBX do souboru `.ubx` (RTKLIB), metadata (výška antény, čas, bod) |
| **Rover** | profily (frekvence, dynamický model, elevační maska, C/N0 maska, SBAS/HAS, výstupy NMEA), výška antény a PCO, úloha po startu, bzučák, kalibrace baterie a gyra, verze FW, restart X20 (horký/teplý/studený) |
| **Diagnostika** | statistika komunikace (UBX, RTCM, BT), PPS, IMU, rušení / AGC, stav antény, OSNMA, paměť roveru, konzole |
| **Porovnání** | GNSS telefonu (čip / Fused / Network) proti roveru – viz kap. 7b |

### Zakázky a data
- **Zakázka** (projekt) → body, měření, trasy, soubory PPK, poznámky.
- Bod: název, druh, ETRS89 φ/λ/h, S-JTSK Y/X, Bpv, kvalita polohy, zdroj (zadaný / změřený), poznámka, foto.
- Měření: bod, čas, nastavení, souhrn, vzorky (1 Hz), příznaky (RTCM, HAS, pohyb, vyřazení).
- Trasa: body s časem GNSS, polohou, σ, kvalitou, rychlostí, kurzem.

## 4. Souřadnice a výšky

- Poloha z roveru: s RTCM ze sítě v ČR = ETRS89; bez RTCM (HAS, SBAS) = ITRF2020 → přepočet do ETRS89 (rotace Eurasie s epochou; ČR 2026 ≈ N −0,57 m, E −0,74 m).
- **ETRS89 ↔ S-JTSK:** postup ČÚZK s opravnou mřížkou (cm úroveň); záloha 7 parametrů (~0,4 m, ověřeno na TB 10 Holý kopec: 0,35 m).
- **Výška:** elipsoidická h → normální Bpv přes kvazigeoid ČR (mřížka ČÚZK); h_msl z X20 (geoid u-blox) jen orientačně.
- Výška antény a PCO z profilu roveru; odečítá se při ukládání bodů.

## 5. Protokol rover ↔ aplikace (BT SPP)

Jeden obousměrný proud bajtů. Druhy dat se rozlišují podle prvního znaku:

| Směr | Začátek | Obsah |
|---|---|---|
| rover → app | `$G` | NMEA (GGA, RMC, GST, GSA, GSV) – kompatibilita s jinými aplikacemi |
| rover → app | `$PRVR,` | vlastní věty s kontrolním součtem (živá data) |
| rover → app | `{` | JSON na jednom řádku: odpovědi na příkazy, události, souhrny |
| rover → app | `0xB5 0x62` | rámce UBX (RAWX/SFRBX) při záznamu pro PPK |
| app → rover | `0xD3` | RTCM3 z NTRIP → X20 |
| app → rover | `{` | JSON příkaz na jednom řádku |

### Živá data `$PRVR` (protokol 1, FW 0.8.0)
Kontrolní součet jako NMEA (XOR bajtů UTF-8 mezi `$` a `*`). Prázdné pole = hodnota není k dispozici.

`$PRVR,POS,` – každá epocha (frekvence profilu, 5 Hz):

| # | pole | jednotka / význam |
|---|---|---|
| 1 | čas UTC | `hhmmss.sss` |
| 2 | datum | `ddmmyy` |
| 3, 4 | φ, λ | ° (9 des. míst, z NAV-HPPOSLLH) |
| 5, 6 | h_ell, h_msl | m |
| 7–9 | σN, σE, σU | m (NAV-COV; prázdné, když neplatná) |
| 10, 11 | hAcc, vAcc | m |
| 12 | kvalita | 0 žádná, 1 GNSS, 2 SBAS/DGNSS, 3 HAS (bez RTCM, ITRF2020), 4 RTK fix, 5 RTK float, 6 dopočet |
| 13 | carrSoln | 0 / 1 float / 2 fix (u HAS stav konvergence) |
| 14 | RTCM | 1 = z mobilu chodí RTCM (posledních 30 s) |
| 15 | stáří korekcí | s (horní mez) |
| 16 | SV | počet v řešení |
| 17 | PDOP | |
| 18, 19 | rychlost, kurz | m/s, ° |
| 20 | pohyb | 1 = IMU hlásí pohyb (15 °/s nebo 0,2 g, drží 1,5 s) |
| 21 | hp | 1 = poloha z NAV-HPPOSLLH |

`$PRVR,STA,` – 1 Hz: FW, baterie V, baterie %, aktivní profil (od 0), počet profilů, název profilu, RTCM přijato, RTCM chybné, stáří RTCM [s], typ poslední RTCM, chyby UBX, volná RAM [B], fáze měření bodu (0 nic, 1 odpočet, 2 ustálení, 3 sběr, 4 hotovo, 5 selhalo), čas ve fázi [s], vzorků, zahozené dávky BT; od FW 0.8.1 navíc: fronta BT v roveru [B] (odesláno, zásobníkem BT nepotvrzeno), režim aplikace (0/1), konec epochy X20 (1 = X20 posílá NAV-EOE), počet epoch s HPPOSLLH.

Od FW 0.11.0 na konci STA navíc: rámce UBX z mobilu OK, vadné, MGA-ACK přijato, odmítnuto, vyslané MGA-DBD, ACKAIDING (1 OK / 0 NAK / −1 bez odpovědi).

`$PRVR,SAT,` – 1 Hz (v režimu aplikace každé 2 s), po 12 družicích: část, počet částí, celkem sledovaných, pak opakovaně `gnssId,svId,C/N0,elevace,azimut,použit` (jen C/N0 > 0).

Režim aplikace (FW 0.8.1): první příkaz JSON ho zapne, odpojení BT vypne. Z NMEA jde jen GGA a RMC (GGA potřebuje NTRIP), GSA/GSV/GST se neposílají (data jsou v `$PRVR`). Přenos ~1,7 kB/s místo ~3,6 kB/s.

Omezení zpoždění (FW 0.8.1): rover počítá odeslaná a zásobníkem BT potvrzená data (ESP_SPP_WRITE_EVT); nad 2 kB nepotvrzených dat se dávky zahazují – po zaseknutí spojení přijde nejnovější poloha, ne stará fronta.

Příkazy z mobilu: řádek JSON začínající `{` a ukončený `\n`. Rover ho rozpozná jen mimo rámec RTCM – aplikace proto posílá RTCM po celých rámcích. Etapa 1 (FW 0.8.0): `get info`, `get profiles`, `profile`, `x20 reset`.

### Příkazy (JSON, `id` pro spárování odpovědi)
- `{"id":1,"cmd":"get","what":"info|profiles|device|points|meas|task"}`
- `{"id":2,"cmd":"set","what":"profiles|device|task","data":{…}}`
- `{"id":3,"cmd":"profile","i":2}` – aktivní profil
- `{"id":4,"cmd":"meas","op":"start","cfg":{…}}` / `"op":"finish|cancel"`
- `{"id":5,"cmd":"raw","meas":12,"from":0,"n":200}` – vzorky měření po dávkách
- `{"id":6,"cmd":"log","what":"rawx","on":true}` – UBX RAWX/SFRBX do proudu
- `{"id":7,"cmd":"x20","op":"reset","kind":"hot|warm|cold"}`, `"op":"cfg"` (vybrané klíče)
- `{"id":8,"cmd":"cal","what":"battery|gyro",…}`
- Odpověď: `{"id":N,"ok":true,…}` / `{"id":N,"ok":false,"err":"…"}`; události bez `id`: `{"ev":"meas_done",…}`, `{"ev":"moved",…}`.

### Úloha po startu (uložená v roveru)
- `none` – jen výstup polohy, čeká na příkazy.
- `meas` – změř bod (bod, ustálení, doba, opakování); výsledek uloží, bzučák signalizuje.
- `track` – záznam (v roveru jen souhrn / značky; vzorky do telefonu, když je připojen).
- Platí po každém zapnutí, dokud ji aplikace nezmění.

## 6. Úpravy roveru (ESP)

1. Protokol `$PRVR` + JSON příkazy přes SPP (místo webového API); NMEA zůstává.
2. Wi-Fi a web jen jako servisní režim (podržení tlačítka při zapnutí, bez BT).
3. Úloha po startu, ukládání v LittleFS.
4. Předávání RAWX/SFRBX do SPP na povel.
5. EKF GNSS + IMU, rozptyl v klidu (modul `motion`) – podle plánu ve specifikaci firmwaru.

## 7. Etapy

1. **Základ:** protokol v roveru, spojení BT, NTRIP, živé zobrazení + mapa, záznam trasy, export CSV/GPX.
2. **Body a měření:** zakázky, body, řízení měření bodu, stažení vzorků, souhrny a porovnání, ETRS89 ↔ S-JTSK, kvazigeoid.
3. **Vytyčení, PPK, nastavení roveru:** vytyčení, záznam RAWX, profily a úloha po startu, diagnostika.
4. **Rover:** EKF, rozptyl v klidu, servisní režim Wi-Fi.

## 7a. Úpravy po testu 13. 10. (Kraví hora, HAS)

**Výška antény v cm**
- Zadání i zobrazení výšky antény v **cm** (uložení dál v m). Souhrn měření a export ukazují výšku v cm.
- Kontrola rozsahu při zadání (např. 0–300 cm), u hodnoty pod 1 cm upozornit.

**Jedno místo pro profil a nastavení měření**
- Problém: profil (dynamický model) se mění na obrazovce Rover, příznak statiky na obrazovce Měření bodu. Souhrn pak ukazuje „Chodec 5 Hz“ a zároveň `staticky_model = 1`, takže není jasné, co skutečně platilo.
- **Obrazovka Rover** = jediné místo pro trvalé nastavení: profily (frekvence, dynamický model, masky, SBAS/HAS), výška antény a PCO.
- **Měření bodu** nemá vlastní přepínače nastavení roveru. Nahoře ukazuje aktivní profil a výšku antény a nabízí tlačítko „Změnit“, které otevře obrazovku Rover.
- Statika při měření bodu: volba v profilu „při měření bodu přepnout na stacionární model“. Rover ji provede sám na začátku sběru a po skončení vrátí model profilu.
- Souhrn a vzorky ukládají **skutečně použité** nastavení podle potvrzení roveru: dynamický model, frekvenci a výšku antény. Samotný název profilu nestačí.

## 7b. Porovnání GNSS telefonu s roverem (14. 10.)

Cíl: ukázat skutečnou přesnost polohy telefonu a vliv asistence (A-GNSS, Fused) proti referenci z roveru.

**Zdroje telefonu (každý zapnout/vypnout zvlášť)**

| Zdroj | API | Obsah |
|---|---|---|
| **GNSS čip** | `LocationManager`, `GPS_PROVIDER` | čistá poloha z čipu (A-GNSS data už čip využívá); φ, λ, h_ell, rychlost, kurz, čas, přesnost horizontální (68 %) a vertikální (Android 8+) |
| **Fused** | Google Play Services `FusedLocationProviderClient`, vysoká přesnost | GNSS + Wi-Fi + BTS + senzory – poloha „s asistencí“, jak ji vidí běžné aplikace |
| **Network** | `NETWORK_PROVIDER` | jen Wi-Fi / BTS, bez GNSS |
| **Satelity telefonu** | `GnssStatus` | konstelace, svId, C/N0, elevace, azimut, použit, nosná frekvence (L1/L5, E1/E5a) |

- Volitelně `GnssMeasurement` (surová měření, jen některé telefony) – zjistit podporu a uložit příznak v diagnostice; záznam pro RTKLIB až později.
- Při budoucí mock location z roveru číst telefon jen z hardwarových providerů, ne z mock.

**Reference a párování**
- Reference = poloha roveru (RTK fix / float / zkonvergovaný HAS); u každé epochy porovnání se ukládá kvalita reference.
- Páruje se podle času GNSS (telefon ~1 Hz, rover 5 Hz → nejbližší epocha roveru, max. rozdíl 100 ms; Fused/Network podle času polohy, s uvedením stáří).
- Systém: telefon = WGS84 ≈ ITRF2020 → stejný přepočet do ETRS89 jako HAS; reference podle příznaku RTCM.
- Odsazení telefonu od antény: zadání ΔN/ΔE/ΔU [cm] nebo „telefon u antény“ (0).

**Obrazovka Porovnání**
- Živě pro každý zdroj: ΔN, ΔE, ΔU, 2D chyba, hlášená přesnost, poměr chyba / hlášená přesnost, stáří polohy, počet SV (čip), frekvence (L1 / L1+L5).
- Graf rozptylu se středem v poloze roveru, všechny zdroje v jednom měřítku (barvy podle zdroje), kruhy hlášené přesnosti.
- Časový graf 2D chyby zdrojů.
- Statistika za zvolené okno / celé porovnání: průměr, σ, RMS, CEP50/95, max, podíl epoch, kdy byla chyba > hlášená přesnost.
- Na mapě (Živě, Trasy) volitelně stopy zdrojů telefonu vedle roveru.
- Satelity: přepínač rover / telefon.
- Uložení porovnání k zakázce (volitelně k trase nebo měření bodu), export CSV (čas, zdroj, poloha, hlášená přesnost, Δ vůči roveru, kvalita reference).

**Test doby do fixu telefonu (vliv A-GNSS)**
- Smazání asistenčních dat čipu: `sendExtraCommand(GPS_PROVIDER, "delete_aiding_data")`; vynucení stažení: `"force_psds_injection"` (dříve `"force_xtra_injection"`), čas `"force_time_injection"`. Oprávnění `ACCESS_LOCATION_EXTRA_COMMANDS`. Podpora závisí na výrobci – výsledek příkazu zaznamenat.
- Varianty: studený start bez internetu (uživatel zapne režim letadlo, aplikace ověří), studený start s A-GNSS, horký start.
- Měří: čas do první polohy GNSS čipu, do přesnosti < 10 m a < 5 m vůči roveru, počet SV v čase; pro Fused čas do první polohy a do < 10 m.
- Opakování, tabulka pokusů, min / medián / max, export CSV.

**Nelze zjistit**
- zda čip použil SBAS nebo jiné korekce,
- váhy zdrojů ve Fused (GNSS / Wi-Fi / BTS),
- zda čip skutečně použil stažená A-GNSS data (jen nepřímo z doby do fixu).

## 7c. Asistence X20 z telefonu (14. 10.)

Cíl: kratší studený start X20 po delším vypnutí bez účtu a bez internetu. AssistNow (Thingstream) se nepoužívá.

**1. Čas a přibližná poloha z telefonu (UBX-MGA-INI)**
- Po připojení BT, když rover hlásí fix „žádný“: aplikace pošle UBX-MGA-INI-TIME_UTC (čas telefonu, přesnost podle zdroje – GNSS telefonu / síť) a UBX-MGA-INI-POS_LLH (poloha telefonu, přesnost z `Location.accuracy`, min. 100 m pro Network).
- Rámce UBX jdou z mobilu do roveru a ESP je předá beze změny do UART1 X20 (FW 0.11.0: `btlink` rozpozná rámec `0xB5 0x62` a uvnitř nehledá JSON).
- Výsledek: UBX-MGA-ACK (pokud X20 posílá), zobrazit v Diagnostice; zapnutí/vypnutí v nastavení Roveru.

**2. Uložení navigační databáze X20 (UBX-MGA-DBD)**
- Aplikace periodicky (např. 1× za 30 min při fixu) a při odpojení / slabé baterii vyžádá výpis databáze (poll UBX-MGA-DBD); rover předá odpovědi MGA-DBD z X20 do BT (**úprava FW ESP** – dnes jen RAWX/SFRBX).
- Aplikace uloží databázi s časem (Room / soubor), po dalším připojení při studeném startu ji pošle zpět do X20 (po MGA-INI-TIME).
- Platnost: efemeridy ~4 h, almanach týdny – starší databázi poslat jen pro almanach; stáří zobrazit.
- Ověřit v popisu rozhraní HPG 2.11, že X20P podporuje MGA-DBD a MGA-INI.

**Ověření přínosu:** test TTFF roveru (studený start) bez asistence / s bodem 1 / s body 1+2.

## 8. Otevřené otázky

- Zdroj a formát mřížek ČÚZK (transformace S-JTSK, kvazigeoid) – ověřit aktuální soubory a licenci.
- NTRIP: dostupné castery (VESOG, CZEPOS) a vzdálenost referenčních stanic od místa měření (fix spolehlivě do ~20–30 km).
- Mock location pro ostatní aplikace v telefonu – volitelně.

## 9. Stav aplikace

- **0.1.0 (etapa 1):** spojení BT, protokol `$PRVR`, NTRIP, mapa, satelity, záznam tras, export CSV/GPX, diagnostika (DB v3).
- **0.2.0 (etapa 2a):** zakázky (výběr, nová, přejmenování, smazání), záložka Body (ruční zadání S-JTSK+Bpv nebo ETRS89, úhly i 49°55′9,8368″, dopočet druhého systému 7 parametry, import/export CSV), body na mapě s popisky, vrstva bodových polí ČÚZK (rastr ze služby BodovaPole od přiblížení 13) a dotaz klepnutím (do 30 m, vrstvy ZhB, ZPBP, PPBP, ZVBP, PVBP) s převzetím do zakázky; trasy podle zakázky. DB v4 (starší data se smažou).
  - Pole služby BodovaPole (ZhB): CISLO, ZTLTL, DRUH, Y, X, VYSKA, B, L, HEL, GPS, GEODETICKE_UDAJE, NAZEV_KU, NAZEV_OKRES; formát hodnot ověřit na skutečných datech (karta bodu ukazuje všechna pole).
- **0.7.0 (14. 10., napsáno bez překladu – ověřit překladem v Android Studiu, na HW netestováno):** karta **Telefon** a asistence X20 (kap. 7b, 7c), výška antény v cm (kap. 7a).
  - Zdroje polohy telefonu GNSS čip / Fused / Network (přepínače, uloženo), satelity telefonu s pásmy (přepínač Rover / Telefon i v Živě → Satelity), zjištění podpory surových měření.
  - Porovnání: párování s roverem (čip podle času GNSS do 100 ms, Fused/Network podle času příjmu do 600 ms), přepočet telefonu ITRF2020 → ETRS89, odsazení telefonu od antény [cm], živé Δ, poměr chyba / hlášená přesnost, graf rozptylu, časový graf 2D chyby, statistika (průměr, σ, RMS, CEP50/95, max, podíl chyba > přesnost), stopy na mapě, ukládání k zakázce (DB v8, migrace 7 → 8, data zůstávají), export CSV.
  - Test doby do fixu telefonu: studený bez internetu / s A-GNSS / horký, opakování, limit, pauza; čas do první polohy, < 10 m, < 5 m, Fused; min / medián / max, CSV.
  - Asistence X20 (FW 0.11.0+): po připojení bez fixu MGA-INI-TIME_UTC + POS_LLH (telefon, jinak poslední poloha roveru), záloha MGA-DBD každých 30 min při fixu (uloží se jen úplný výpis podle počítadla roveru), vrácení databáze mladší 7 dní; ruční tlačítka, studený start X20.
  - Oprávnění k poloze, služba na popředí s typem „poloha“ (poloha telefonu se zhasnutým displejem), knihovna `play-services-location` 21.3.0.
  - Kap. 7a: měření bodu už přepínače roveru nemá (statika a RAWX jsou v uživatelském profilu v Nastavení) a souhrn ukládá statiku podle potvrzení roveru – zbývalo zadání a zobrazení výšky antény v cm (rozsah 0–300 cm, varování pod 1 cm), export měření `vyska_anteny_cm`.
- **0.7.1 (14. 10.):** úpravy po prvním testu (byt u okna, rover 3D).
  - Párování GNSS čipu: když čas GNSS v poloze telefonu nesedí s roverem do 100 ms (Qualcomm GEN9: z 1230 poloh spárováno 2), páruje se podle hodin telefonu proti času GNSS roveru do 300 ms. Zobrazení posunu času (čas GNSS telefonu − rover, hodiny telefonu − čas GNSS, medián 60 poloh) a počtu párování podle způsobu.
  - Test doby do fixu: „první poloha“ (i vložená z asistence – po smazání dat 1–2 s při 1 družici) zvlášť od „fixu“ s aspoň 4 družicemi v řešení; hlášená přesnost fixu; čas do hlášené přesnosti < 10 / < 5 m (bez fixu roveru pokus končí podle ní); chyba vůči roveru jen u poloh se 4+ družicemi.
  - Družice v řešení se u telefonu počítají jako různé družice (dvoupásmová = 1), signály zvlášť.
  - Záložka „Srovnání“.
- **0.7.2 (14. 10.):** po jízdě autem (nepravidelná stopa telefonu, mezi tím telefonní hovor).
  - Stopa telefonu na mapě ze všech poloh, i nespárovaných s roverem (dřív jen spárované – výpadek BT nebo párování = mezera ve stopě).
  - U každého zdroje počet poloh, frekvence za posledních 60 s, mezery > 3 s, nejdelší a poslední mezera s časem.
  - Stav služby na popředí s typem „poloha“ (poloha se zhasnutým displejem) a tlačítko Obnovit.
  - Test doby do fixu: u studeného startu se Fused a Network během pokusu vypnou (drží čip v chodu; test 9:10–9:15 dal fix za 0,1–1,1 s); fix se 4+ družicemi do 5 s = neplatný studený start, označí se a do statistiky nejde.
  - Otevřené: posun času GNSS čipu telefonu proti roveru přesně 1000 ms (hodiny telefonu − čas GNSS −115 ms) – rozhodne zpoždění dat z roveru, pak oprava v párování nebo ve FW.
- **0.7.3 (14. 10.):** legenda mapy v Živě při zapnutých stopách telefonu (rover s barvou kvality, GNSS čip, Fused, Network), klepnutím sbalit / rozbalit.

## 10. Protokol od FW 0.13.0 (20. 10.) – rover bez Wi-Fi, nastavení z aplikace

**`$PRVR,STA` – nová pole na konci** (za ACKAIDING): frekvence [Hz] (0.12.0), zdroj korekcí z UBX-NAV-SIG (−1 žádný, 1 SBAS, 2 BeiDou, 3 RTCM2, 4 RTCM3 OSR, 5 RTCM3 SSR, 6 QZSS SLAS, 7 SPARTN, 8 CLAS, vyšší = HAS podle FW X20), signálů použitých s korekcí, výsledek nastavení korekcí SBAS/HAS (1 OK / 0 NAK / −1), průchozí režim (0/1), rámce NAV při záznamu (0/1). Pole měření bodu (fáze, čas, vzorků) jsou 0 – rover měření bodu nemá, řídí ho aplikace.

**Kvalita v `$PRVR,POS`:** 2 = SBAS (korekce bez fáze, zdroj SBAS nebo neznámý), 3 = HAS (bez RTCM: carrSoln > 0 nebo korekce z jiného zdroje než SBAS podle NAV-SIG).

**Záznam (`rec start`):** rover posílá navíc celé rámce UBX NAV-PVT, NAV-COV a NAV-HPPOSLLH (před `$PRVR,POS` téže epochy, ~1,1 kB/s při 5 Hz) – aplikace je ukládá do surového proudu trasy spolu s IMU (0xF5). Podklad pro EKF offline.

**Nové příkazy JSON** (stejné přijímá i konzole USB – řádek začínající `{`):
- `{"cmd":"set","what":"profiles","data":{"active":0,"default":0,"profiles":[…]}}` – formát jako `get profiles`; profil má navíc `sbas`, `has` (bool). Odpověď s uloženými profily, profil se hned pošle do X20.
- `{"cmd":"get","what":"device"}` → `{"data":{"btName":"…"},"default":"ROVER-xxxx","name":"…"}`; `{"cmd":"set","what":"device","data":{"btName":"…"}}` – platí po restartu (`"restart":true`).
- `{"cmd":"get","what":"cal"}` → `{"gyro":[x,y,z],"div":…,"bat":…,"batOk":…}`; `{"cmd":"cal","what":"battery","v":3.86}`; `{"cmd":"cal","what":"gyro"}` (3 s v klidu).
- `{"cmd":"passthrough","op":"start|stop"}` – průchozí režim USB ↔ X20 (u-center na PC), konec i krátkým stiskem tlačítka.
- `{"cmd":"restart"}` – restart ESP po odeslání odpovědi.
- `get info` navíc: `caps` (`set,cal,pass,navrec,sig,boot`), `boot` {fix, corr, float, fixRtk} = ms od zapnutí do prvního 3D fixu, korekcí, float, RTK fix (−1 = zatím ne), `upMs`, `reset` (esp_reset_reason: 1 zapnutí napájení, 3 SW restart).
- Délka řádku příkazu až 4 kB (dřív 511 znaků).

**Tlačítko roveru:** krátký stisk = další profil, podržení 5 s = pípnutí a restart ESP (X20 běží dál).

## 11. Stav aplikace 0.8.0 (20. 10., napsáno bez překladu – ověřit v Android Studiu)

- **Karta Test** (nová, mezi Telefon a Nastavení):
  - Testovací záznam pro EKF: scénáře Klid 30 min (sám skončí), 6 poloh (řízené kroky 30 s s odpočtem a pípnutím), Orientace při nošení (stání, předklon, záklon, úklony, rovná chůze), Volné prostranství s RTK, Město, Les, Auto. U každého postup, metadata (nošení, orientace čela krabičky, poznámka), tlačítka značek s časem telefonu i GNSS.
  - Záznam = trasa „TEST …“ se surovým proudem vždy (i bez volby v profilu) a s každou epochou; metadata, značky a geometrie IMU (osy, rameno páky) v `files/tests/test_<trasa>.json`.
  - Uložené testy: export ZIP (test.json, body.csv se všemi epochami, diag.csv, raw.log).
  - Doba do fixu roveru: horký / teplý / studený start příkazem, cíl 3D / korekce / float / RTK fix, opakování, limit, pauza; čas do 3D, korekcí, float, fix; min / medián / max; CSV.
  - Start po vypnutí (záloha X20): po zapnutí napájení roveru (FW 0.13.0+) zapíše dobu vypnutí (od posledních dat v aplikaci) a dobu od zapnutí do 3D fixu a korekcí; dotaz na rover každých 10 s, dokud časy nejsou známé (do 30 min od zapnutí).
- **Nastavení → Nastavení roveru** (FW 0.13.0+): editor profilů roveru (název, 1/5/10 Hz, dynamický model, elevační maska, SBAS, HAS, bzučák, věty NMEA; výchozí profil, kopie, smazání, uložení do roveru), název Bluetooth, kalibrace baterie a gyra, průchozí režim USB ↔ X20, restart roveru; zdroj korekcí z NAV-SIG a výsledek nastavení SBAS/HAS.
- Kvalita 2 se zobrazuje jako „SBAS“.
- Diagnostika k trase (Diag CSV) už ukládá frontu BT, zpoždění, ztráty a volnou RAM – podklad k poklesu kvality BT v autě.

## 12. Stav aplikace 0.8.1 (20. 10.)

- Stav přístroje z roveru (FW 0.14.0): Měřím / Manipulace / Chodím / Jedu v Živě vedle kvality a v Nastavení roveru (s modelem X20 a znaky σ|a|, ω max, kroky). Profil roveru lze nastavit na model „automaticky“.
- Test „Celý den (lehký)“: bez surového proudu, každá 5. epocha, ruční značky skutečného stavu (Měřím, Manipulace, Chodím, Jedu) a automatické značky „rover: …“ při změně stavu; `motion.csv` (1 Hz: stav, model, znaky) v ZIPu. „Celý den (plný)“ se surovou IMU.
- Osy IMU v `test.json` opraveny podle testu 6 poloh (X doprava, Z dovnitř, rameno −17,5 / −16 / +45 mm).
- Zdroj korekcí 12 = HAS. Název ZIPu se scénářem (`RG_TEST_20261020_161249_Les.zip`).

## 13. Požadavky na další verzi (20. 10.)

- Automatické připojení k naposledy připojenému roveru po startu aplikace (bez výběru v Nastavení).
- Sekce v Nastavení (Rover, Nastavení roveru, Korekce NTRIP, Uživatelský profil, …) sbalené; v záhlaví jen stav, např. „Korekce NTRIP – zapnuto, VESOG, RTCM 2 s“, „Rover – připojeno, ROVER_Tonda, 0.14.0“. Rozbalení klepnutím.

## 14. Stav aplikace 0.8.2 (21. 10.)

- **Automatické připojení:** po startu aplikace (s oprávněním Bluetooth) připojí naposledy použitý rover; zkouší nejdéle 5 min, pak přestane a ukáže výběr. Po ručním odpojení se znovu nepřipojuje (až po dalším startu aplikace).
- **Sbalené sekce v Nastavení** (Rover, Nastavení roveru, Korekce NTRIP, Profil, Aplikace): nadpis se stavem (např. „připojeno ROVER_Tonda, FW 0.14.5, baterie 95 %“, „Automat, 5 Hz, HAS, Měřím“, „zapnuto, VESOG, RTCM 2 s“), rozbalení klepnutím; aplikace si pamatuje, které sekce jsou rozbalené.
- **Profily:** jen jeden pojem „Profil“ (dřív uživatelský profil). Výchozí čtyři, dřívější uživatelské profily se při první instalaci 0.8.2 smažou:

| Profil | Přijímač (profil v roveru) | Anténa | Záznam trasy | Měření kontrolního bodu |
|---|---|---|---|---|
| Automat | Automat – 5 Hz, model automaticky (podle stavu Měřím / Chodím / Jedu) | 4 cm | každá epocha, surový proud | 3 min, statický model |
| Chůze | Chůze – 5 Hz, pedestrian | 4 cm | každá epocha, surový proud | 3 min, statický model |
| Měření | Měření – 1 Hz, stationary | 4 cm | každá epocha, surový proud | 5 min, statický model, RAWX |
| Auto | Auto – 5 Hz, automotive | 4 cm | každá 5. epocha, bez surového proudu | ručně |

- Přepínání v Živě štítkem vlevo nahoře, úpravy, přidání a mazání v Nastavení → Profil. Profil určuje „Nastavení přijímače“ **podle názvu** profilu v roveru (dřív podle pořadí). Když rover profil s tím názvem nemá, štítek ukáže „přijímač nenalezen“ a přijímač se nepřepne.
- **Profily v roveru:** při prvním připojení každého roveru (podle adresy BT) aplikace zapíše do roveru čtyři profily Automat, Chůze, Měření, Auto, výchozí po zapnutí Automat (masky, SBAS/HAS, bzučák a NMEA převezme z dosavadního prvního profilu). Dosavadní profily roveru se přepíšou. Firmware bez stavu přístroje (bez „motion“, starší než 0.14.0) dostane u Automatu model pedestrian.
- Krátký stisk tlačítka na roveru přepne profil roveru a aplikace přepne profil se stejným názvem přijímače. Bez aplikace (např. CurieFinder přes NMEA) rover běží podle svého profilu, po zapnutí vždy Automat.
- V Nastavení roveru se seznam profilů roveru jmenuje „Nastavení přijímače“.

## 15. Stav aplikace 0.8.3 (21. 10.) – FW 0.15.0

- **Test „Kontrolní okruh“** (karta Test, jeden záznam, jeden ZIP):
  1. Výběr bodu lokality se souřadnicemi ETRS89; profil se přepne na Automat (po testu zpět). Tlačítko „Studený start a vypnout“: studený start X20 a výzva vypnout rover.
  2. Po odpojení a novém připojení roveru (nebo „Rover je zapnutý – pokračovat“) začne záznam (surový proud, IMU, NAV) se značkou „rover připojen – příprava“; manipulace se zaznamená.
  3. „Položeno na bodě“: studený start X20, měření bodu 30 min uvnitř záznamu testu (statický model podle profilu), velký odpočet; doba do 3D fixu a do korekcí od tohoto okamžiku (značky „3D fix“, „korekce (…)“), živá odchylka 2D, N/E a výšky od bodu.
  4. Okruh: značky nesu v ruce, nastupuju, jedu, vystupuju, chodím, les – vstup / výstup, klid, poznámka.
  5. „Položeno na bodě (15 min)“: druhé měření, po odpočtu konec testu.
  - U každé fáze ⓘ s postupem. `test.json` navíc `circuit`: bod, profil, časy studených startů, `ttffFixS`, `ttffCorrS`, `meas1`/`meas2` (začátek, konec, epochy, průměr, dN, dE, dU, d2, dH, id měření). Měření se uloží i k bodu (historie měření).
- Měření kontrolního bodu může běžet uvnitř běžícího záznamu (`startMeasurement(…, inTrack)`) – trasa se nezakládá ani neukončuje.
- STA z FW 0.15.0 (teplota IMU, klid, kroky, bias gyra) se zapisuje do `motion.csv` (sloupce `teplota_imu;klid;kroky_celkem;bias_aktualizaci;bias_x;bias_y;bias_z`).
- Kalibrace gyra 30 s na pozadí (čeká na událost `cal_gyro`), se starším FW beze změny.
- Pozn.: `raw.log` limit velikosti nemá (zápis i export ZIP průběžně). Konec surových dat v testu 21. 10. v 6482 s způsobilo ukončení záznamu trasy, ne velikost – příčina neověřena.

## 16. Požadavky z testu 22. 10. (Město, RTK, Blanka; návrh, nerozhodnuto)

- **Metadata jízdních scénářů:** rover je v autě vždy na palubní desce – u scénářů Město / Auto nošení „palubní deska“ místo výchozího „levý bok“.
- **Výška v kvalitě 2 (DGNSS):** po výjezdu z tunelu o 6 m mimo – nebrat jako platnou (zobrazení, měření bodu, export s příznakem).
- **NTRIP bez polohy (23. 10.):** po zapnutí roveru se špatným příjmem caster CZEPOS (RTK3-MSM) odpovídá 401 („špatné jméno nebo heslo“), po naskočení družic se připojí – požadavek odešel bez GGA (bez fixu aplikace GGA neposílá). Úpravy:
  1. Mountpoint s GGA > 0 bez fixu nepřipojovat; stav „NTRIP čeká na polohu“, připojit samo po prvním 3D fixu.
  2. 401 u požadavku bez GGA = „caster odmítl spojení bez polohy“, opakovat po fixu (ne konečná chyba).
  3. „Špatné jméno nebo heslo“ jen u 401 s polohou v požadavku; i pak 2–3 pokusy po 30–60 s (obsazený účet po pádu aplikace).
  4. Tlačítko RTK v mapě (návrh): zapnutí bez fixu se odloží, žlutá „čekám na polohu“.
- **NTRIP po restartu aplikace:** připojil se až za ~50 s (23. 10.) – obnovit hned po startu služby (s podmínkou polohy výše).
- **Zpoždění a kvalita BT bez fixu:** rover a telefon cm od sebe, bez signálu družic zpoždění ~2 s a „slabý signál“, fronta 0 – zpoždění se počítá z času GNSS v POS, který bez fixu není platný. Bez fixu (kvalita 0) zpoždění nepočítat (zobrazit „—“), kvalitu BT hodnotit z fronty, ztrát a pravidelnosti příjmu.
- **Živě – informační řádek družic:** počet použitých / sledovaných družic (po konstelacích), průměrné C/N0.
- **Pád aplikace 23. 10.:** na vjezdu do tunelu při ztrátě fixu, aplikace na pozadí, během hovoru – příčinu zjistit (logcat / záznam chyby).
- **Test doby do fixu roveru – chyba (23. 10., studený start, cíl 3D, mezi budovami, RTCM):** pokus 2 dal 3D za 0,0 s (fix změřen hned po odeslání resetu, X20 ještě hlásil starou polohu); reset z pokusu 2 se projevil až 0,4 s po něm, pokus 3 pak měřil od svého resetu o 10 s později (33,6 s místo 43,8 s). Oprava: po resetu čekat na skutečnou ztrátu fixu a čas měřit od ní; fix do 1 s po resetu = neplatný pokus (označit, do statistiky nepočítat). Pozn.: s cílem 3D test po fixu a pauze hned resetuje – float naskočil vždy do ~3 s po 3D, fix nedosažen; pro RTK cíl RTK fix, limit 180 s, pauza 30 s. Souběh testu doby do fixu se záznamem testu nevadí (záznam zachytí každou epochu).
- **Záznam trasy – zrušit volbu „Zahrnout průměr od …“ (rozhodnutí 24. 10.):** volba vkládala do trasy polohy z běžícího průměru a posouvala začátek trasy, surový proud a diagnostika ale začínají až stiskem Start – záznam nebyl úplný. Záznam trasy jen od stisku Start, zpětně nic. Ostatní použití průměru (Zaměřit bod, obnovení průměru po restartu aplikace) beze změny.
- **K prošetření – skok 0,95 m při přepnutí RTK ↔ HAS (25. 10., dálnice):** při zapnutí NTRIP poloha v aplikaci skočila o N +0,57 / E +0,76 m, při vypnutí (po 28,6 s, přechod na HAS) zpět o N −0,56 / E −0,73 m. Surová poloha z roveru (`$PRVR,POS`) skok nemá (jen 0,3–0,4 m při dokonvergování float → fix) – skok dělá přepočet ITRF2020 → ETRS89 v aplikaci. HAS po předchozím RTK zůstal ve stejném systému jako RTK (v prvním úseku HAS byla poslední RTCM 75–209 s stará); HAS bez předchozího RTK (12. 10., TB 10) byl v ITRF a přepočet seděl (0,13 m). Pravidlo „RTCM za posledních 30 s = ETRS89, jinak přepočet“ to nerozliší. Test: známý bod ETRS89 (např. 1115-010) – 10 min RTK fix, pak NTRIP vypnout a 20–30 min HAS; zda a jak rychle se surová poloha HAS posune o ~0,95 m k ITRF. Pak pravidlo (např. držet posun z přechodu RTK → HAS a postupně ho rozpouštět).
- **K prošetření – záznam trasy v testu skončil bez zásahu (25. 10., test Auto):** `body.csv`, `diag.csv` a `raw.log` skončily ve stejné sekundě 11:14:50,9, test běžel dál do cíle (11:31, `motion.csv` a značky pokračovaly). Aplikace na popředí, nad ní malé okno navigace Google; v 11:14:44,9 hlášen návrat na popředí, v 11:14:43–45 krátký výpadek GNSS v podjezdu. Uživatel nic neovládal. Ověřit v kódu na PC (ne GitHub): co ukončí nebo zneplatní záznam trasy při návratu na popředí / změně okna / výpadku polohy; na kartě Trasy zjistit čas konce trasy (ukončení vs. zastavený zápis). Nejspíš stejně skončil surový proud 21. 10. (6482 s).
- Rozbor: `ekf-vyhodnoceni.md`, kap. 9 a 10; úpravy FW: `rover-firmware-specifikace.md`, „Požadavky z testu 22. 10.“.

## 17. Stav aplikace 0.9.0 (25. 10.) – FW 0.16.0; přeloženo v CI, na HW netestováno

- **NTRIP:** mountpoint s GGA se bez fixu nepřipojuje („čeká na polohu“, připojí se samo po prvním 3D fixu; GGA s kvalitou 0 se nepočítá). 401 bez polohy = opakovat po fixu; 401 s polohou se opakuje po 45 s, „špatné jméno nebo heslo“ až po 3 odmítnutích. Po startu aplikace jen ručně (beze změny).
- **Tlačítko RTK v Živě pod REC:** šedá vypnuto, žlutá připojuje / čeká na polohu, zelená s dobou spojení (a stářím RTCM nad 5 s), červená chyba; profil z Nastavení → Korekce NTRIP.
- **Doba a cena RTK** (Nastavení → Korekce NTRIP): cena [Kč/h] ručně, každé spojení zaokrouhleno nahoru na celou sekundu, celková doba, počet spojení a cena od vynulování; probíhající spojení se ukládá po 5 s (po pádu aplikace se započte). Nečinnost: upozornění (bez vypnutí), když je rover odpojený > 1 min, nebo neběží záznam ani měření a rover stojí > 10 min.
- **Kvalita BT bez fixu:** zpoždění a ztráty se bez fixu nepočítají (čas GNSS neplatný), kvalita jen podle příjmu dat a fronty.
- **Živě – řádek družic:** použité / sledované celkem a po konstelacích (G, E, B, R), průměrné C/N0 použitých.
- **Testy:** nošení výchozí „palubní deska“ (Auto, výchozí volba; „leží na stole“ a „různě“ u klidu a celodenních testů beze změny). Test nejde spustit během měření rychlosti.
- **Test doby do fixu:** čas od skutečné ztráty fixu po restartu (u horkého startu i mezera v epochách > 0,6 s); bez ztráty fixu do 5 s nebo fix do 1 s po ztrátě (studený / teplý) = neplatný pokus, do statistiky se nepočítá, v CSV sloupec `neplatny`.
- **Záznam trasy:** volba „Zahrnout průměr od …“ zrušena. Tlačítka STOP v Živě a Ukončit v Trasách záznam testu neukončí (test se ukončuje na kartě Test).
- **Deník událostí** `files/events.log` (začátek a konec záznamu s důvodem, testy, NTRIP, popředí / pozadí, spojení s roverem, přechod RTK → HAS, pády se zásobníkem volání); v ZIPu testu `events.log` za dobu testu – k prošetření konce záznamu 25. 10. a pádu 23. 10.
- **DGNSS při RTK** (kvalita 2 s RTCM) se nepoužije v měření bodu ani průběžném průměru (výška až o 6 m mimo).
- **HAS po RTK (dočasně, do testu na známém bodě):** při přechodu RTK → HAS se porovná surová poloha s předpovědí z rychlosti; skok < 0,3 m = HAS zůstává v ETRS89 („frameRtk“, bez přepočtu ITRF → ETRS89) do ztráty fixu > 30 s, odpojení roveru nebo restartu X20. Výsledek přechodu v deníku událostí.
- **Záložka Auto:** „Aktivovat měření“ = rover do profilu Rychlost (každých 5 s „žiju“), „Ukončit měření“ = návrat. Velká rychlost, maximum, průměr za jízdu a 10 s, dráha, podélné zrychlení / brzdění (IMU, posun sklonu a biasu odečtený proti GNSS) a příčné (rychlost × otáčení z gyra) v g s maximy. Orientace krabičky se určí sama (svislice z průměru za jízdy, směr jízdy regresí proti změně rychlosti GNSS). Testy: 0–50, 0–100, vlastní 0–X, 60–100, 80–120, 100–0, 50–0, 100 / 200 / 400 / 1000 m; nabití klepnutím, start i konec samy (rozjezd z IMU, překročení rychlostí interpolací, dráha integrací rychlosti), sklon nad 1 % a výpadek GNSS u výsledku, historie (200) v telefonu. Odchod ze záložky při měření: dotaz Ukončit / Pokračovat s výčtem omezení; na mapě štítek „Rychlost – měření“. Měření bodu a testy během měření rychlosti nejdou.
- **Dopočet při výpadku GNSS** (stav Jedu nebo měření rychlosti, když chodí IMU – záznam, test, měření rychlosti; nejdéle 120 s): kurz z gyra, rychlost z poslední platné epochy (i DGNSS), prvních 10 s z akcelerometru, pak držená (rozbor umělých výpadků, `ekf-vyhodnoceni.md`, kap. 12). Body s kvalitou 6 do záznamu a stopy na mapě, ne do měření bodu.
- Verze 0.9.0 (versionCode 19).
