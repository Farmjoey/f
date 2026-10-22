# Funktionsstatus v0.4.0

## Verificeret (på PC, uden hardware)
* Pure logik (gestus, hold-tracker, touch-kalibrering, sikkerhedsregler, demo-data, "--"-formatering): **2844 checks, 0 fejl** (`tools/run_host_tests.sh`, C++11).
* Alle `.cpp`-filer syntakstjekket (`g++ -fsyntax-only`) mod midlertidige stub-headere for Arduino/TFT_eSPI/NimBLE.
  Stubs afspejler *mine* antagelser om bibliotekernes API – det er **ikke** en rigtig kompilering.

## IKKE verificeret
* Kompilering med PlatformIO, linkning, firmwarestørrelse og versionslåste biblioteker (kræver netværk – gøres af GitHub Actions).
* Opstart, display, farver/inversion, touch, kalibrering, lysstyrke, hastighed/flicker på dit board.
* BLE-scanning (NimBLE) og RAM-forbrug under scanning.
* Alt vedr. E-Wheels e2s Urban Max: **ingen forbindelse, ingen protokol, ingen telemetri**.

## Funktioner
| Funktion | Status |
|---|---|
| Dashboard (hastighed, batteri, spænding, effekt, trip/total, mode/fejl) | Implementeret; viser kun demo-data (mærket DEMO) eller `--` |
| Klokkeslæt | Ikke tilgængeligt (ingen RTC/tidskilde) – viser `--:--` |
| BLE | Scanning + læse-only forbindelse til valgt hjul (profil-detektion, notifikationer, læseforespørgsler). **Ikke testet på hardware.** Ingen skrivning |
| Scooter-protokol | XBOT læsning ported fra kilde (afkodning testet mod optaget ramme). Verificeret **ikke** på Urban Max. Skrivning ikke understøttet (`kVerified=false`, `kWriteSupported=false`) |
| 20 km/t + lås | Swipe op: på rigtigt hjul sænkes Sport/Drive/Eco-grænserne til 20 (hold 1,5 s), kan gendannes, og LOCK/UNLOCK. Vises kun aktiv efter aflæsning. **Ikke testet på hjulet.** Uden forbindelse: lokal demo |
| Musik (iPhone) | Ikke implementeret (planlagt AMS); knapper inaktive |
| Beskeder | Ikke implementeret (planlagt ANCS); intet gemmes |
| Ride controls | CTRL: demo = lokal simulation. Rigtigt hjul: lys, zero start og køretilstand kan skrives efter UNLOCK (hold 1,5 s), kun hvidlistede registre, og kun markeret bekræftet efter aflæsning. **Ikke testet på hjulet.** Hazard: ingen kendt kommando |
| Indstillinger gemmes | Implementeret (NVS) – ikke testet på hardware |
