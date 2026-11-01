# Testabdeckung der seriellen Kommandos

Grundlage: die in [`AI_AGENT_COMMANDS.md`](../AI_AGENT_COMMANDS.md) dokumentierten Kommandos.
Getestet wird im nativen Testbuild (`tests/CMakeLists.txt`, GoogleTest + arduino-mock). Die Tests für Sendekommandos beschreiben den Stand nach den Fixes in `src/send.h` (siehe Abschnitt "Behobene Fehler").

Legende Status: ✅ getestet · ⏸ Test vorhanden, aber deaktiviert (wartet auf Fix) · ⛔ im nativen Build nicht testbar

Die CC1101-Kommandos (Schritt 4) laufen in einem eigenen Test-Programm `TestProjectCC1101`, das mit `CMP_CC1101` gebaut wird. Dort läuft das unveränderte `src/cc1101.cpp` gegen ein CC1101-Modell (`tests/fakeCC1101.h`), das über die AVR-SPI-Register angesprochen wird.

Spalte "Schritt": 2 = System-/Konfigurationskommandos, 3 = Sendekommandos, 4 = CC1101-Kommandos (eigenes Test-Target).

## Systembefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `V` | Versionsstring `V <PROGVERS><PROGNAME> - compiled at …`; mit CC1101 inkl. `cc1101 (chip CC110x)` | Zusatzzeichen, Chipversionen 03/04/14/05/07/17/unbekannt, Version `00`/`FF` (Rückfall auf Revision 0), `hasCC1101` aus | 2 / 4 | ✅ |
| `?` | Liste `? Use one of V R t X S P C r W s`, mit CC1101 zusätzlich `w x e` | Zusatzzeichen, `hasCC1101` aus | 2 / 4 | ✅ |
| `R` | freier RAM (`freeRam()`) | Zusatzzeichen | 2 | ✅ |
| `t` | Uptime in Sekunden (Wert gegen `millis()` des Mocks geprüft) | Überlauf nicht testbar: der arduino-mock leitet `millis()` aus der Systemzeit ab | 2 | ✅ |
| `P` | `OK` | Zusatzzeichen | 2 | ✅ |
| `s` | ohne `CMP_CC1101` keine Ausgabe; mit CC1101: `OK` bzw. Hinweis auf Factory Reset (PKTCTRL0/IOCFG2 geprüft) | falsches PKTCTRL0, falsches IOCFG2, `hasCC1101` aus | 2 / 4 | ✅ |
| `XQ` / `XE` | Empfänger aus/an | `X` allein, `Xz` (unbekanntes Flag) | 2 | ✅ |
| `e` | Factory Reset: EEPROM- und Chip-Register = `initVal`, PATABLE, SRES, danach RX | `hasCC1101` aus, anschließend `s` = `OK` | 4 | ✅ |
| unbekannt | `Unsupported short command` | leerer Puffer, Steuerzeichen | 2 | ✅ |

## Konfigurationsbefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `CG` | `MS=…;MU=…;MC=…;Mred=…` | – | 2 | ✅ |
| `CE<x>` / `CD<x>` (S, U, C, R) | Flag gesetzt/gelöscht | unbekanntes `<x>`, fehlendes `<x>` | 2 | ✅ |
| `CEA`/`CDA` | AFC an/aus, bei aus FSCTRL0 = 0 und Offset zurückgesetzt | Kleinbuchstaben (`CEa`, `Cea`, `CDa`), `hasCC1101` aus; jedes `CE<x>`/`CD<x>` schreibt bei AFC aus FSCTRL0 | 4 | ✅ |
| `CEW`/`CDW`, `CET`/`CDT` | WMBus / WMBus_T | werden nativ ignoriert (geprüft) | – | ⛔ benötigt zusätzlich ESP8266/ESP32/MAPLEMINI |
| `CG` (FSK) | `MN=1;AFC=<0/1>` bei `ccmode` ≠ 3 | Zusatzzeichen | 4 | ✅ |
| `CSmcmbl=<val>` | `<val> bits set` | kein Wert, negativ, nicht numerisch, sehr groß, falscher Schlüssel | 2 | ✅ |
| `C<reg>` | Config- (`00`–`2E`) und Statusregister (`30`–`3D`; die Firmware liest ab `2F` als Status), `C3E` PATABLE, `C99` alle, `C<reg>n<x>` Bereich | Kleinbuchstaben, eine Stelle, kein Hex, `3F`/`98`/`FF`, `n` mit Nicht-Hex (`nG`), mehrstellige Anzahl (`n1F`), Bereich über `2F` und über den ganzen Statusbereich, Überlauf der Anzahl (`C10nEE`, Endlosschleife), `hasCC1101` aus | 4 | ✅ |
| `W<reg><val>` | EEPROM-Schreibzugriff; mit CC1101 zusätzlich Register `<reg>-2` und Antwort `W<reg><val>` | ungültiges Hex, zu kurz, `00`/`01`/`40`/`FF`, MDMCFG2 → `ccmode`, MDMCFG4 → FREND1/FIFOTHR (Grenzen `56`/`57`, `C6`/`C7`), `hasCC1101` aus | 2 / 4 | ✅ |
| `WS<id>` | Strobe + Chip-Status `cmdStrobeReg <id> chipStatus <s> delay1 <s>` | `WS3`, `WS3G`, `WS3E`/`WS3F`, `WS2F`, mehr Ziffern (`WS3412`), Kleinbuchstaben, `hasCC1101` aus | 2 / 4 | ✅ |
| `r<addr>` | `EEPROM <addr> = <val>` | ungültiges Hex, eine Stelle, `FF`; im CC1101-Build mit und ohne `hasCC1101` | 2 / 4 | ✅ |
| `r<addr>n` | 16 Bytes ab `<addr>` | Adresse am Ende des Bereichs | 2 | ✅ |
| `x<val>` | PATABLE in EEPROM und Chip, `write <val> to PATABLE done` | `00`/`FF`, eine Stelle, kein Hex, `hasCC1101` aus | 4 | ✅ |

## Sendebefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `SR` | Rohsequenz mit `P0`…`P7` und `D=`, Echo, Pegel am Sendepin | `R=0`, `R=256`/`R=257` (Überlauf), `P8` (ignoriert), ±32767 µs, Abbruch ohne Daten / mitten im Teil / ohne Zeilenende, längstes akzeptiertes (261 Zeichen) und zu langes Kommando (262) | 3 | ✅ |
| `SM` | Manchester mit `C=` und `D=`, Echo, `delay(1)` | `R=2`, `0`/`F`, Kleinbuchstaben, ungültiges Hex (`G`), fehlendes `C=` | 3 | ✅ |
| `SC` | kombinierte Sequenz mit `R=` | `R=0`, ohne `R=`, Abbruch | 3 | ✅ |
| `SN` | ohne `CMP_CC1101`: Echo + `no supported device to send`; mit CC1101: Daten über TX-FIFO, STX, danach RX | Wiederholungen, Kleinbuchstaben, `N=`, ungerade Anzahl Hexziffern, zwei `SN`-Teile in `SC`, `hasCC1101` aus, STX/SRX schlägt fehl | 3 / 4 | ✅ |
| `S…` ohne führendes `S` | Zeichen vor dem ersten `S` werden übersprungen (kein Echo), das Kommando wird mit seinen `R=` gesendet; ohne `S` oder mit unbekanntem Typ (`SX;`) `send cmd corrupt` ([#267](https://github.com/RFD-FHEM/SIGNALDuino/issues/267), vgl. [#268](https://github.com/RFD-FHEM/SIGNALDuino/pull/268)) | `X;P0=500;D=0;`, nur `;`, `SX;…` | 3 | ✅ |
| `F=` (bei `SR`/`SM`/`SC`) | bis zu 5 Register ab FREQ2 setzen und nach dem Senden zurückschreiben | 5 Register (Adressen und Werte), 6 Register, `hasCC1101` aus; 7 und 36 Register als `DISABLED_` (Lesezugriff außerhalb von `ccReg[6]`) | 4 | ✅ / ⏸ |

### Fehlerhafte Übertragungen, Timing und ungewöhnliche Sendekommandos

Spalte "vorher" beschreibt das Verhalten vor den Fixes in `src/send.h`.

| Bereich | Fall | Verhalten | vorher | Test |
| :--- | :--- | :--- | :--- | :--- |
| Empfänger | erfolgreiches Senden | `disableReceive()` am Anfang, `enableReceive()` am Ende | gleich | `receiverIsEnabledAgainAfterSend` |
| Empfänger | `send cmd corrupt` / `send cmd to long` | Empfänger wird wieder eingeschaltet | blieb aus bis `XE` oder Neustart | `corruptCommandEnablesReceiverAgain`, `tooLongCommandEnablesReceiverAgain`, `rawCarriageReturnOnlyIsCorrupt` |
| Timeout | Pause im Kommando (z. B. zwei TCP-Pakete) | `send cmd corrupt`, der Rest der Zeile wird verworfen (wartet dafür bis zum Serial-Timeout) | Rest blieb im Puffer und wurde als neues Kommando ohne `S` gelesen (#267) | `serialTimeoutInsideCommandDiscardsRestOfLine` |
| Timeout | Rest kommt erst nach dem Timeout | wird als Kommando ohne `S` abgelehnt | Endlosschleife | `serialTimeoutRestIsNotSentAsNewCommand` |
| Timeout | Pause direkt vor `\n` | vollständiges Kommando wird verworfen, `\n` mit verworfen | `\n` blieb im Puffer | `serialTimeoutBeforeLineEndDropsCompleteCommand` |
| Länge | Kommando mit 600 Datenzeichen | `send cmd to long`, die ganze Zeile wird verworfen | nur 255 weitere Bytes verworfen, 102 Bytes blieben im Puffer | `veryLongCommandIsDiscardedCompletely` |
| Länge | Zeilenende genau an der Längengrenze, danach ein weiteres Kommando | `send cmd to long`, das folgende Kommando bleibt erhalten | das folgende Kommando wurde mit verworfen | `tooLongAtLineEndKeepsNextCommand` |
| Länge | Teile, die in `buf` bleiben (`C=`, `D=`, `F=`, unbekannte Teile, Teile nach dem ersten `S`-Teil) | Grenze über die Schreibposition: höchstens 255 Bytes + `\0` in `buf`, sonst `send cmd to long`. `SM;C=400;D=` hat höchstens 245 Datenzeichen | `buffer_left` zählte das `;` dieser Teile nicht, Schreibzugriff hinter `buf[256]` | `manchesterLongestDataThatFitsIntoBuf`, `manchesterDataBeyondBufIsTooLong`, `manyShortPartsBeyondBufAreTooLong`, `combinedPartsBeyondBufAreTooLong` (der letzte erkennt den alten Fehler nur unter ASan) |
| Zeilenende | `\r\n` | wird akzeptiert | gleich | `rawCrLfLineEndIsAccepted` |
| Zeilenende | nur `\r` | `send cmd corrupt` (Timeout) | gleich, Empfänger blieb aus | `rawCarriageReturnOnlyIsCorrupt` |
| Zeilenende | `D=0101` ohne abschließendes `;` | `send cmd corrupt` | Segmentation fault | `rawLastPartWithoutSemicolonIsRejected` |
| Zeilenende | Zeichen außer `\r` zwischen letztem `;` und `\n` (z. B. `…;D=0101; \n`) | `send cmd corrupt` | gesendet, der Rest wurde ignoriert | `rawLastPartWithoutSemicolonIsRejected` (nur Fall ohne `;`) |
| leerer Teil | `…;D=0101;;\n` | wird gesendet, Rest der Zeile verworfen, Echo mit Zeilenende | Echo ohne Zeilenende, `\n` blieb im Puffer | `rawEmptyPartAfterDataEndsCommand` |
| leerer Teil | `…;;D=0101;\n` | `send cmd corrupt` | Segmentation fault | `rawEmptyPartBeforeDataIsRejected` |
| leerer Teil | `D=;` | nichts gesendet | Bytes hinter dem `;` als Pattern-Index gesendet | `rawEmptyDataPartSendsNothing` |
| Daten | kein `D=` | `send cmd corrupt` | `datastart`/`dataend` nicht initialisiert | `rawWithoutDataPartIsRejected` |
| Daten | Pattern nicht gesetzt (`D=012` ohne `P2`) | `send cmd corrupt` | zufällige Pulslänge | `rawUnsetPatternIsRejected`, `combinedPartWithoutPatternsIsRejected` |
| Daten | Index ≥ 8 oder kein Ziffernzeichen (`D= 01`, `D=018`) | `send cmd corrupt` | Lesezugriff außerhalb von `buckets[]` | `rawInvalidPatternIndexIsRejected` |
| Wertebereich | `R=-1` | 255 Wiederholungen | gleich | `rawRepeatNegativeBecomes255` |
| Wertebereich | `R=` ohne Wert | nichts gesendet, keine Meldung | gleich | `rawRepeatWithoutValueSendsNothing` |
| Wertebereich | `P0=40000` | als `int16_t` −25536 gespeichert, also LOW statt HIGH | gleich | `rawPatternAboveInt16FlipsLevel` |
| Wertebereich | `C=70000` | als `int16_t` 4464 gespeichert | gleich | `manchesterClockAboveInt16Wraps` |
| Wertebereich | `C=-400` | keine Wartezeit, Bits ohne Dauer | gleich | `manchesterNegativeClockSendsWithoutDelay` |
| Wertebereich | `C=-400` bei `micros()` < 400 (nach Start / Überlauf) | keine Wartezeit | Warteschleife endete nicht (auf dem Gerät bis ~71 min) | `manchesterNegativeClockDoesNotBlock` |
| Timing | `micros()`-Überlauf während des Sendens (alle 2³² µs ≈ 71,6 min) | Pulslängen bleiben erhalten | Pulse nach dem Überlauf schrumpften auf ~0 µs | `rawMicrosOverflowKeepsPulseLengths`, `manchesterMicrosOverflowKeepsBitLengths` |
| mehrere Teile | zweiter `SR`-Teil ohne `SC` | wird wie `SC` gesendet | gleich | `rawSecondSendPartWithoutCombinedIsSentToo` |
| mehrere Teile | `SC` + 4 Teile | Maximum, wird gesendet | gleich | `combinedFivePartsIsTheMaximum` |
| mehrere Teile | `SC` + 5 Teile | `send cmd corrupt` | Schreibzugriff auf `command[5]` (außerhalb des Arrays) | `combinedMoreThanFivePartsIsRejected` |

Unverändert gelassen (Wertebereiche): `R=`, `Pn=` und `C=` werden weiterhin ohne Bereichsprüfung übernommen.

Nur im Code gelesen, im nativen Build nicht kompiliert (`serialEvent()` in `signalduino.h`, `signalSTM.h`, `signalesp.h`):

- ESP: `IB_1[idx + 1] = '\0'` schrieb bei `;` an Position 13 nach `IB_1[14]`, also ein Byte hinter den Puffer.
- Alle: bei vollem Puffer gab `Command to long: ` mit `MSG_PRINTLN(IB_1)` 14 Zeichen ohne `\0` aus und las über `IB_1` hinaus. AVR/STM terminierten `IB_1` nie, `DBG_PRINT(IB_1)` in `send_cmd()` las ebenso weiter.
- Behoben: `IB_1` hat jetzt 15 Bytes (14 Zeichen + `\0`). Alle drei `serialEvent()`-Varianten terminieren nach jedem gelesenen Zeichen, `IB_1` ist also immer ein gültiger String. Die drei `serialEvent()`-Varianten sind nicht kompiliert getestet. Ein Test liest ihre Quelltexte und prüft Terminierung direkt nach dem Lesen, Puffergröße und `idx == 14` (`serialEventVariantsNullTerminateIB_1AfterEveryCharacter`, reine Textprüfung, führt `serialEvent()` nicht aus); dazu die Puffergröße des Testhelfers (`loadCommandRespectsBufferSize`). Der volle Puffer mit `;` an Position 13 ist nativ getestet (`garbagePrefixFillingIB_1IsSkipped`).
- AVR/STM: ein `;` an Position 0 (Zeile beginnt mit `;`) ruft `send_cmd()` auf. ESP prüft vorher `idx > 0`. `send_cmd()` lehnt das jetzt mit `send cmd corrupt` ab (`onlySemicolonIsRejectedAsCorrupt`).
- Bei Timeouts oder zu langen Kommandos verwirft `send_cmd()` jetzt den Rest der Zeile. Kommt der Rest erst nach dem Timeout, liest `serialEvent()` ihn als neues Kommando, das wegen des fehlenden `S` abgelehnt wird.

### Behobene Fehler in `send_cmd()` / `send_raw()` / `send_mc()`

Diese Fälle hatten undefiniertes Verhalten (Absturz, Endlosschleife, Speicherzugriff außerhalb von Arrays). Sie werden jetzt mit `send cmd corrupt` abgelehnt und sind getestet (siehe Tabelle oben):

- Sendekommando ohne `D=`, letzter Teil ohne `;`, leerer Teil `;;` vor `D=`: `datastart`/`dataend` waren nicht initialisiert. Jetzt sind sie mit `nullptr` vorbelegt und werden vor dem Senden geprüft.
- Erster Teil in `IB_1` beginnt nicht mit `S` (#267): `cmdNo` blieb 255, `command[255]` lag außerhalb des Arrays.
- `D=` mit Index eines nicht gesetzten Patterns oder Ziffer ≥ 8: gesetzte Patterns werden jetzt pro Teil gemerkt und die Daten vor dem Senden geprüft.
- Mehr als 5 Teile: `command[5]` lag außerhalb des Arrays.

Die Tests für Schritt 2 stehen in `tests/test_serialCommand_system.cpp` (Fixture `ShortCommandTest`, parametrisiert `DecoderFlagTest`), die für Schritt 3 in `tests/test_serialCommand_send.cpp` (Fixture `SendCommandTest`) und die für Schritt 4 in `tests/test_serialCommand_cc1101.cpp` (Fixture `Cc1101Test`, Programm `TestProjectCC1101`).

## Auffälligkeiten im aktuellen Verhalten

Die Tests halten das Verhalten fest (für Sendekommandos nach den Fixes in `src/send.h`). Folgende Punkte weichen von der Doku ab oder sind bei Grenzwerten auffällig (Fundstellen in `src/commands.h`, `src/send.h` bzw. `src/cc1101.cpp`):

| Kommando | Beobachtung | Fundstelle | Test |
| :--- | :--- | :--- | :--- |
| `r<addr>` | wird nur beantwortet, wenn `hasCC1101` gesetzt ist, obwohl nur das EEPROM gelesen wird | `cmd_read`, Bedingung `&& hasCC1101` | `readEepromRequiresCC1101Flag` |
| `r<addr>n` | liest immer 16 Bytes, ab `F1` also über Adresse `FF` hinaus | Schleife `EEPROM.read(reg + i)` | `readEepromBlockAtEndReadsBeyond0xFF` |
| `CSmcmbl=` | leerer oder nicht numerischer Wert setzt `0`; Werte > 255 bzw. negative laufen im `uint8_t` über | `configSET()`, `strtol` ohne Prüfung | `setMcMinBitLen*` |
| `CSmcmbl=` | Antwort `<n> bits set` ohne Zeilenende | `configSET()`, `MSG_PRINT` statt `MSG_PRINTLN` | `setMcMinBitLen` |
| `s` | ohne `CMP_CC1101` keine Antwort (Doku: `OK` oder Fehlermeldung) | `cmd_status` | `statusWithoutCC1101SupportPrintsNothing` |
| `SR`/`SM`/`SC` | behoben: Echo enthielt Reste vorheriger Teile, wenn der `D=`-Teil kürzer war als der vorherige Teil oder ein `P8=`-Teil übersprungen wurde (z. B. `…D=01;\n;`) | `send_cmd()`, jetzt Nullterminierung nach `\n` | `rawEchoHasNoStaleBytesWhenDataPartIsShorter`, `rawPatternIndexAboveSevenIsIgnored` |
| `SR`/`SM`/`SC` | `R=` ist `uint8_t`: `R=256` sendet nichts, `R=257` einmal | `strtoul` → `repeats` | `rawRepeatOverflowWraps*` |
| `SC` | `R=0` wird ignoriert, es wird einmal gesendet | `command[0].repeats > 0` | `combinedRepeatZeroSendsOnce` |
| `SM` | ungültige Hexzeichen werden ohne Fehlermeldung als Nibble gesendet (`G` → `0`) | `send_mc()` | `manchesterInvalidHexIsSentAsZeroNibble` |
| `S…` | behoben: nach `send cmd corrupt` / `send cmd to long` blieb der Empfänger aus (`return` vor `enableReceive()`; `enableReceive()` steht sonst nur in `setup()` und bei `XE`) | `send_cmd()` | `corruptCommandEnablesReceiverAgain`, `tooLongCommandEnablesReceiverAgain` |
| `S…` | behoben: Wartezeiten mit `while (stoptime > micros())` waren nicht überlaufsicher, jetzt `(long)(micros() - stoptime) < 0` | `send_raw()`, `send_mc()` | `*MicrosOverflow*`, `manchesterNegativeClockDoesNotBlock` |
| `S…` | Meldung lautet `to long` statt `too long` | `TXT_TOLONG` in `output.h` | `rawTooLongCommandIsRejected` |
| `e` | Antwort ist `ccFactoryReset done` (Doku: `Init eeprom to defaults...`) | `cc1101::ccFactoryReset()` | `factoryResetWritesDefaultsToEepromAndChip` |
| `x<val>` | Antwort ist `write 12 to PATABLE done` (Doku: `Write 12 to patable`) | `cmd_patable`, `TXT_WRITE`/`TXT_TPATAB` | `patableWrite` |
| `C<reg>n<x>` | Bereich wird nicht auf die Config-Register begrenzt: ab Adresse `30` wird mit Single-Read-Header gelesen, das ist für den CC1101 ein Strobe (`C2En1` sendet SRES) | `cc1101::readCCreg()`, nur `reg < 0x2F` geprüft | `readRegisterRangeBeyond0x2FSendsStrobe` |
| `W30…W3F` | EEPROM `30`–`37` ist der PATABLE-Speicher, wird aber zusätzlich an Chip-Adresse `2E`–`3D` geschrieben; `30`–`3D` sind Strobes (`W3201` sendet SRES) | `cc1101::writeCCreg()`, Bedingung `reg < 0x40` | `writeEepromPatableAreaSendsStrobe` |
| `WS<id>` | prüft `hasCC1101` nicht (anders als `C`, `x`, `e`) | `cmd_write`, Zweig `WS3` | `strobeIgnoresMissingCC1101Hardware` |
| `SN` | Parameter `N=` aus dem Doku-Beispiel wird nicht ausgewertet | `send_cmd()` kennt kein `N=` | `xfskIgnoresUnknownParameterN` |
| `SN` | bei ungerader Anzahl Hexziffern wird das letzte Nibble mit `;` kombiniert (`D=91C` → `91 FB`) | `cc1101::sendFIFO()`, `hex2int()` | `xfskOddNumberOfHexDigits` |
| `SN` | ohne `hasCC1101` wird der TX-FIFO trotzdem beschrieben (nur `setTransmitMode()` entfällt) | `send_cmd()` | `xfskWithoutCC1101HardwareStillWritesFifo` |
| `F=` | wird `F=` nicht angewendet (mehr als 5 Register oder `hasCC1101` aus), schreibt `send_cmd()` trotzdem nicht initialisierte Werte aus `ccReg[]` in die Register ab `0D` zurück | `send_cmd()`, `ccParamAnz` wird vor der Prüfung gesetzt | `frequencyRegisters*ButWrittenBack` |
| `F=` | ab 7 Registern liest das Zurückschreiben außerhalb von `ccReg[6]` (Stack; ASan: `stack-buffer-overflow`), ab 36 Registern erreicht es Adresse `30` = SRES und setzt den Chip zurück | `send_cmd()`, Schleife über `ccParamAnz` ohne Obergrenze | `DISABLED_frequencyRegistersSevenAreNotWrittenBack`, `DISABLED_frequencyRegistersLongListDoesNotResetChip` |
| `C<reg>n<x>` | die Anzahl wird mit `strtol` über alle Hexziffern gelesen (`C00n1F` liest 33 Register); `n + 2` läuft im `uint8_t` über, und `for (uint8_t i = reg; i < reg + n; i++)` endet nicht, wenn `reg + n` 256 erreicht (`C10nEE`: Firmware hängt) | `cc1101::readCCreg()` | `readRegisterRangeCountHasMoreDigits`, `readRegisterRangeCountOverflowDoesNotEnd` |
| `C<reg>n<x>` | `C2Dn10` liest `2D`–`3E` mit Single-Read-Headern und löst dabei alle Strobes `30`–`3D` aus (u. a. SRES, SPWD, SFTX) | `cc1101::readCCreg()` | `readRegisterRangeAcrossStatusAreaSendsAllStrobes` |
| `V` | liefert der Chip als VERSION `00`/`FF`, liest `chipVersion()` mit Revision 0 Adresse `31` mit Single-Read-Header (`B1`); das ist der Strobe SFSTXON, nicht das VERSION-Register | `cc1101::chipVersion()`, `CC1101_VERSION_REV00` | `versionReadsOldAddressWhenChipReturns00OrFF` |
| `WS<id>` | `strtol` liest alle Ziffern, verwendet wird das untere Byte: `WS3412` schickt `12` (Config-Adresse, kein Strobe) an den Chip | `cc1101::commandStrobes()` | `strobeWithMoreDigitsUsesLowByte` |
| `CE<x>`/`CD<x>` | bei AFC aus schreibt jedes `CE<x>`/`CD<x>` FSCTRL0 = 0 und setzt den Frequenz-Offset zurück, auch ohne `hasCC1101` | `storeFunctions()`, Bedingung `afc == 0` | `decoderFlagWithAfcOffWritesFsctrl0` |
| `SC` mit `SN` | `sendFIFO(command[cmdNo].datastart, command[c].dataend)` verwendet `cmdNo` statt `c`: bei zwei `SN`-Teilen wird nur der letzte gesendet (`…SN;D=91;SN;D=C6;` → TX-FIFO `C6`) | `send_cmd()`, xFSK-Zweig | `xfskCombinedPartsUseDataOfLastPart` |
| `S…` mit CC1101 | schlägt der Wechsel nach TX bzw. RX fehl, wird das nicht erkannt: nach `while (maxloop-- && …)` ist `maxloop` 255, nicht 0. `setTransmitMode()` schaltet `PIN_SEND` trotzdem auf Ausgang, die Nachricht wird gesendet | `cc1101::setTransmitMode()`, `cc1101::setReceiveMode()` | `transmitModeFailureIsNotDetected`, `receiveModeFailureIsNotDetected` |

## Testhelfer

`tests/fakeCC1101.h` (nur `TestProjectCC1101`): Modell des CC1101 am SPI. `SPDR`/`SPSR`/`SPCR` werden per Präprozessor auf ein Fake-Register umgeleitet, Chip-Select kommt über `pinMode(csPin, LOW/HIGH)`. Das Modell bildet das SPI-Zugriffsschema des CC1101-Datenblatts ab (Header-Byte mit Lese- und Burst-Bit, Config-, Status- und PATABLE-Zugriffe, Strobes mit Status-Byte vor Ausführung, Schreiben in den TX-FIFO). Es ist ein vereinfachtes Modell. Nicht nachgebildet sind:

- MISO / CHIP_RDYn: MISO ist immer low, die Timeout-Pfade von `wait_Miso()`/`waitTo_Miso()` laufen nie.
- MARCSTATE folgt nicht dem Zustand (bleibt `01`), das Warten auf das Ende des Sendens in `sendFIFO()` endet sofort.
- FIFO-Füllstand im Status-Byte (immer 0).
- Reset-Werte: SRES setzt alle Config-Register auf 0.
- SFTX/SFRX: `txFifo` ist ein Protokoll aller in den TX-FIFO geschriebenen Bytes.
- Adresse `2F` wird wie ein Config-Register behandelt.

Für Fehlerfälle gibt es `stuckStrobe` (dieser Strobe ändert den Zustand nicht, z. B. ein STX, das fehlschlägt). Jedes SPI-Byte wird gezählt. Nach `transferLimit` Bytes wirft das Modell `fakecc::TooManyTransfers`, eine Endlosschleife in der Firmware lässt den Test also fehlschlagen statt den Lauf hängen zu lassen. Nach jedem Test wird geprüft, dass kein SPI-Byte ohne Chip-Select übertragen wurde.

`tests/serialCommandHelper.h`:

- `serialtest::SerialCapture` zeichnet alle Ausgaben über `Serial` (`MSG_PRINT`, `MSG_PRINTLN`, `MSG_WRITE`) als String auf. `println` hängt `\n` an, Zahlen werden wie in der Arduino-`Print`-Klasse formatiert.
- `serialtest::loadCommand(cmd)` kopiert ein Kommando mit Längenprüfung in `IB_1` (max. 14 Zeichen, wie `serialEvent()` sie ablegt).
- `serialtest::runShortCommand(cmd)` (in `test_serialCommand.cpp`) hängt wie `serialEvent()` ein `\n` an und ruft `commands::HandleShortCommand()` auf.
- `SerialCapture::setInput(str, gapAt)` liefert Bytes für `Serial.readBytes()` usw., also den Rest eines Sendekommandos. Mit `gapAt` liefert der erste Lesezugriff an dieser Stelle nichts. Das simuliert einen Serial-Timeout, die restlichen Bytes bleiben im Puffer (`remainingInput()`).
- `serialtest::receiveState()` zählt die Aufrufe von `disableReceive()`/`enableReceive()` in `send.h`. Sie werden wie `micros()` per Präprozessor nur für `send.h` umgeleitet und rufen die echten Funktionen auf.
- `serialtest::fakeMicros()` ersetzt beim Einbinden von `send.h` per Präprozessor `micros()` (der arduino-mock liefert immer 0, die Warteschleifen in `send_raw()`/`send_mc()` würden nie enden). Jeder Aufruf zählt 1 µs weiter.
- `serialtest::SendRecorder` zeichnet die Pegelwechsel am Sendepin (`digitalLow`/`digitalHigh` → `pinMode(pin, LOW/HIGH)` im nativen Build) mit Zeitstempel auf und liefert das Signal als Dauern (`-` = low, `+` = high).
- `runSendCommand(cap, cmd, gapAt)` (in `test_serialCommand_send.cpp`) teilt das Kommando wie `serialEvent()`: bis zum ersten `;` in `IB_1`, der Rest kommt über `Serial`.
