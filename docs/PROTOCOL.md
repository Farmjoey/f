# XBOT-protokol i NOVA RIDE OS (v0.4.0, læsning + begrænset skrivning)

Kilde: open-source komponenten *esphome-xbot* (`components/xbot/*` og dens tests).
Den er **verificeret på ét 36 V-køretøj**. Din E-Wheels e2s Urban Max (48 V, controller a5.9.6)
er ikke verificeret. Koden i `src/XbotProtocol.h` er en port af den komponent.

## Automatisk protokolfinding (v0.2.0)
Når du vælger hjulet på DIAG, tester firmwaren selv: hver dokumenteret profil, som hjulet har (både write og notify), og for hver profil to dokumenterede nøgler til variant 3 (nøglen fra navnet først, derefter den anden: `0x34` eller klartekst). Under testen sendes kun læseforespørgsler. En kombination accepteres **kun**, når hjulet svarer med en ramme med `55 AA`-header og gyldig checksum. Findes der ingen, står årsagen på DIAG (fx `no reply to read requests`), og den prøver igen. Hjul, hvis navn peger på en anden variant (andre rammeformater), afvises uden at sende noget. Resultatet gemmes ikke; testen kører ved hver forbindelse (få sekunder).

## Fra kilden (dokumenteret, ikke verificeret på dit hjul)
| Del | Indhold |
|---|---|
| Transportprofiler | 5 (nus, ae00, ffe0, fff0-f3f7, fff0-f2f1). Firmwaren vælger den første, hvor hjulet *har både* write- og notify-karakteristik |
| Ramme | `55 AA <len+2> <dest> <cmd> <reg> <payload> <~sum LE>`; checksum fra længde-byten |
| Obfuskering | Hele rammen XOR'es med nøgle fra enhedens navn (variant 3: `0x34`, nogle navne i klartekst) |
| Læseforespørgsler | 20 blokke (`kPollTable`), payload = antal bytes |
| Registre (læses) | hastighed 0x27, batteri 0x22, spænding 0x47, strøm 0x48, trip 0x2F, total 0x29/0x2A, fejl 0x1B, advarsel 0x1C, controller-version 0x1E+0x1D, køretilstand 0x7E |
| Test | Én optaget svarramme fra kilden afkodes korrekt i enhedstesten |

## Antagelser (ikke bekræftet for dit hjul)
* At dit hjul bruger variant 3. Navnet "Urban"/"e2s" står ikke i navnetabellen, så fallback er variant 3 med nøgle `0x34`. Firmwaren sender kun læseforespørgsler; en forkert nøgle gør bare, at hjulet ikke svarer.
* At registerkortet og skaleringerne gælder for en 48 V-controller.
* Køretilstand: kilden mapper 0..3 til Sport/Drive/Eco/Walking. Det er kun bekræftet på ét køretøj, så dashboardet viser kun råindekset (`MODE #n`).

## Skrivning (v0.3.0) – hvad der sendes
Skrivefunktionen findes kun under **CTRL**, når hjulet er forbundet og DEMO er slået fra. Rammen er `55 AA 04 20 03 <reg> <lo> <hi> <~sum>` (dest `0x20`, cmd `0x03`); formatet er testet mod kildens egen testvektor.
| Funktion | Register | Hvad sendes |
|---|---|---|
| LIGHT | `0xF2`, bit 0 | frisk læsning først, så resten af registret bevares (read-modify-write) |
| ZERO START | `0x7D`, bit 0 | som ovenfor |
| Køretilstand | `0x7E` | indeks 0=Sport, 1=Drive, 2=Eco, 3=Walking (kildens rækkefølge, set på ét køretøj) |

### 20 km/t-begrænser og lås (swipe op)
Swipe op på dashboardet åbner sikkerhedsskærmen (med hjulet forbundet og DEMO slået fra).
| Knap | Hvad sker | Register |
|---|---|---|
| HOLD 1.5s: LIMIT TO 20 | Hver tilstands grænse, der er over 20 km/t, sættes ned til 20 (Sport, Drive, Eco; Walking har controlleren selv). Grænser under 20 røres ikke | `0xF0`, `0xEF`, `0xF1`, talt i enheder ud fra hjulets egen "wheel factor" `0xEE` (`5794,65 / wheel factor` tæller pr. km/t) |
| HOLD 1.5s: RESTORE LIMITS | Sætter grænserne tilbage til de værdier, hjulet selv havde før (gemmes i hjulet-uafhængig hukommelse, overlever genstart) | samme |
| LOCK / UNLOCK (hold 1,5 s) | Låser eller låser hjulet op. LOCK afvises, hvis hastigheden ikke er kendt eller over 0,5 km/t | `0x7F` (1 = låst, 0 = oplåst) |
* "20 KM/H ACTIVE" vises kun, når alle tre grænser faktisk **aflæses** som ≤ 20 km/t fra hjulet. Ellers står der NOT LIMITED eller UNKNOWN.
* Begrænseren kan kun **sænke**. Gendannelse sætter kun værdier tilbage, som hjulet selv har rapporteret, og kun hvis wheel factor er den samme.
* Hver knap kræver et 1,5 s hold og giver kun én skrivning (derefter låses skrivning igen).
* Den lokale demo kører kun, når DEMO er slået til.

Sikkerhed i koden:
* **Hvidliste:** kun lys (`0xF2`), zero start (`0x7D`), tilstand (`0x7E`), lås (`0x7F`) og de tre hastighedsgrænser (`0xEF/0xF0/0xF1`) kan skrives. Fartpilot (`0x7C`), wheel factor og alt andet kan ikke sendes.
* **Oplåsning:** CTRL åbner låst. Hold UNLOCK 1,5 s for at låse op. Forlader du CTRL, eller mister du linket, låses den igen.
* **Kun efter validering:** der skrives kun, når autodetekteringen har set en gyldig svarramme.
* **Aldrig "bekræftet" uden aflæsning:** efter en skrivning læser firmwaren registret igen. Skærmen skriver CONFIRMED først, når hjulet selv melder den ønskede værdi, ellers NOT CONFIRMED. Knapperne viser hjulets egen tilstand, ikke hvad der blev bedt om.
* Alle skrivninger logges (`WRITE reg=.. val=..`).

## Ikke implementeret med vilje
* **Fartpilot:** kan ikke skrives. Hastighedsgrænser skrives kun gennem 20 km/t-begrænseren (ingen slider).
* **Havarilys:** findes ikke i kilden; knappen er grå (`N/A`) på rigtigt hjul.

## Sådan verificerer du på hjulet (DIAG-skærmen)
1. Luk X-BOT-appen helt (hjulet tager kun én forbindelse).
2. Settings → BLE ON. DIAG viser enhederne; tryk på dit hjul for at vælge det.
3. DIAG skal vise `Link <profil> key <XX> var 3`. Skriv profilen og nøglen ned.
4. `ntf` og `frm` tæller notifikationer og afkodede rammer. `ver` er controller-versionen (hex).
   Sammenlign med X-BOT: `a5.9.6 (06051153)`.
5. Slå DEMO fra i Settings. Dashboardet viser nu LIVE-data. Kontrollér mod hjulet:
   spænding ≈ 48 V-pakken, batteri %, total km, trip km, og at hastigheden følger virkeligheden.
6. Står der en gul linje (fx `wire key 34, using 00` eller `unknown framing`), så send den til mig.
Når tallene passer, kan du teste skrivning: CTRL, hold UNLOCK, tryk på LIGHT. Lyset skal ændre sig, og skærmen skal vise CONFIRMED. Prøv ZERO START og køretilstand bagefter, ét ad gangen.
