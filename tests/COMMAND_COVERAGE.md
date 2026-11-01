# Testabdeckung der seriellen Kommandos

Grundlage: die in [`AI_AGENT_COMMANDS.md`](../AI_AGENT_COMMANDS.md) dokumentierten Kommandos.
Getestet wird im nativen Testbuild (`tests/CMakeLists.txt`, GoogleTest + arduino-mock) ohne Änderungen an der Firmware.

Legende Status: ✅ getestet · ⏸ Test vorhanden, aber deaktiviert (wartet auf Fix) · ⛔ im nativen Build nicht testbar

Die CC1101-Kommandos (Schritt 4) laufen in einem eigenen Test-Programm `TestProjectCC1101`, das mit `CMP_CC1101` gebaut wird. Dort läuft das unveränderte `src/cc1101.cpp` gegen ein CC1101-Modell (`tests/fakeCC1101.h`), das über die AVR-SPI-Register angesprochen wird.

Spalte "Schritt": 2 = System-/Konfigurationskommandos, 3 = Sendekommandos, 4 = CC1101-Kommandos (eigenes Test-Target).

## Systembefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `V` | Versionsstring `V <PROGVERS><PROGNAME> - compiled at …`; mit CC1101 inkl. `cc1101 (chip CC110x)` | Zusatzzeichen, Chipversionen 03/04/14/05/07/17/unbekannt, Version `00`/`FF` (Rückfall auf Revision 0), `hasCC1101` aus | 2 / 4 | ✅ |
| `?` | Liste `? Use one of V R t X S P C r W s`, mit CC1101 zusätzlich `x e` | Zusatzzeichen, `hasCC1101` aus | 2 / 4 | ✅ |
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
| `S…` ohne führendes `S` | Müllzeichen vor `SR;…` wird übersprungen, Kommando wird mit seinen `R=` gesendet; ohne `S` im Kommando `send cmd corrupt` ([#267](https://github.com/RFD-FHEM/SIGNALDuino/issues/267), Fix [#268](https://github.com/RFD-FHEM/SIGNALDuino/pull/268)) | `X;P0=500;D=0;`, nur `;` | 3 | ⏸ (Tests vorhanden, `DISABLED_` bis #268 gemergt ist) |
| `F=` (bei `SR`/`SM`/`SC`) | bis zu 5 Register ab FREQ2 setzen und nach dem Senden zurückschreiben | 5 Register (Adressen und Werte), 6 Register, `hasCC1101` aus; 7 und 36 Register als `DISABLED_` (Lesezugriff außerhalb von `ccReg[6]`) | 4 | ✅ / ⏸ |

### Fehlerhafte Übertragungen, Timing und ungewöhnliche Sendekommandos

Ohne `DISABLED_` halten die Tests das aktuelle Verhalten fest, auch wenn es fragwürdig ist. Mit `DISABLED_` stürzen sie ab oder hängen. Sie beschreiben das vorgeschlagene Verhalten und laufen mit `--gtest_also_run_disabled_tests`.

| Bereich | Fall | Aktuelles Verhalten | Test | Status |
| :--- | :--- | :--- | :--- | :---: |
| Empfänger | erfolgreiches Senden | `disableReceive()` am Anfang, `enableReceive()` am Ende | `receiverIsEnabledAgainAfterSend` | ✅ |
| Empfänger | `send cmd corrupt` / `send cmd to long` | Empfänger bleibt aus (bis `XE` oder Neustart) | `corruptCommandLeavesReceiverDisabled`, `tooLongCommandLeavesReceiverDisabled`, `rawCarriageReturnOnlyIsCorrupt` | ✅ |
| Timeout | Pause im Kommando (z. B. zwei TCP-Pakete) | `send cmd corrupt`, der Rest bleibt im Puffer und wird von `serialEvent()` als neues Kommando ohne `S` gelesen (#267) | `serialTimeoutInsideCommandLeavesRestInBuffer` | ✅ |
| Timeout | Rest nach Timeout als neues Kommando | hängt ohne Fix aus #268 | `DISABLED_serialTimeoutRestIsNotSentAsNewCommand` | ⏸ (#268) |
| Timeout | Pause direkt vor `\n` | vollständiges Kommando wird verworfen, `\n` bleibt im Puffer | `serialTimeoutBeforeLineEndDropsCompleteCommand` | ✅ |
| Länge | Kommando mit 600 Datenzeichen | `send cmd to long`, verworfen werden höchstens 255 weitere Bytes, 102 Bytes bleiben im Puffer (letzter Teil `000…0;` ohne `S`) | `veryLongCommandLeavesRestInBuffer` | ✅ |
| Zeilenende | `\r\n` | wird akzeptiert | `rawCrLfLineEndIsAccepted` | ✅ |
| Zeilenende | nur `\r` | `send cmd corrupt` (Timeout) | `rawCarriageReturnOnlyIsCorrupt` | ✅ |
| Zeilenende | `D=0101` ohne abschließendes `;` | Segmentation fault | `DISABLED_rawLastPartWithoutSemicolonIsRejected` | ⏸ |
| leerer Teil | `…;D=0101;;\n` | wird gesendet, Echo ohne Zeilenende, `\n` bleibt im Puffer | `rawEmptyPartAfterDataEndsCommandEarly` | ✅ |
| leerer Teil | `…;;D=0101;\n` | Segmentation fault | `DISABLED_rawEmptyPartBeforeDataIsRejected` | ⏸ |
| leerer Teil | `D=;` | Bytes hinter dem `;` werden als Pattern-Index gesendet (Lesezugriff außerhalb von `buckets[]`) | `DISABLED_rawEmptyDataPartSendsNothing` | ⏸ |
| Wertebereich | `R=-1` | 255 Wiederholungen | `rawRepeatNegativeBecomes255` | ✅ |
| Wertebereich | `R=` ohne Wert | nichts gesendet, keine Meldung | `rawRepeatWithoutValueSendsNothing` | ✅ |
| Wertebereich | `P0=40000` | als `int16_t` −25536 gespeichert, also LOW statt HIGH | `rawPatternAboveInt16FlipsLevel` | ✅ |
| Wertebereich | `C=70000` | als `int16_t` 4464 gespeichert | `manchesterClockAboveInt16Wraps` | ✅ |
| Wertebereich | `C=-400` | keine Wartezeit, Bits ohne Dauer | `manchesterNegativeClockSendsWithoutDelay` | ✅ |
| Wertebereich | `C=-400` bei `micros()` < 400 (nach Start / Überlauf) | Warteschleife endet nicht (auf dem Gerät bis ~71 min) | `DISABLED_manchesterNegativeClockDoesNotBlock` | ⏸ |
| Timing | `micros()`-Überlauf während des Sendens (alle 2³² µs ≈ 71,6 min) | Pulse nach dem Überlauf schrumpfen auf ~0 µs | `rawMicrosOverflowShortensPulses`, `manchesterMicrosOverflowShortensBits` | ✅ |
| mehrere Teile | zweiter `SR`-Teil ohne `SC` | wird wie `SC` gesendet | `rawSecondSendPartWithoutCombinedIsSentToo` | ✅ |
| mehrere Teile | `SC` + 4 Teile | Maximum, wird gesendet | `combinedFivePartsIsTheMaximum` | ✅ |
| mehrere Teile | `SC` + 5 Teile | Schreibzugriff auf `command[5]` (außerhalb des Arrays), zufälliges Signal | `DISABLED_combinedMoreThanFivePartsIsRejected` | ⏸ |

Nur im Code gelesen, im nativen Build nicht kompiliert (`serialEvent()` in `signalduino.h`, `signalSTM.h`, `signalesp.h`):

- ESP: `IB_1[idx + 1] = '\0'` schreibt bei `;` an Position 13 nach `IB_1[14]`, also ein Byte hinter den Puffer.
- AVR/STM: ein `;` an Position 0 (Zeile beginnt mit `;`) ruft `send_cmd()` auf. ESP prüft vorher `idx > 0`. Folge wie #267.
- Bei Timeouts oder zu langen Kommandos landen Reste im Puffer, wie oben beschrieben. `serialEvent()` liest sie als neue Kommandos. Kommt darin ein `;`, wird `send_cmd()` mit einem Teil ohne `S` aufgerufen (#267).

Nicht getestet, weil der aktuelle Code dabei undefiniertes Verhalten hat (ein Test würde zufällig bestehen oder abstürzen):

- Sendekommando ohne `D=` oder Abbruch mit `\n` vor dem ersten `D=`: `datastart`/`dataend` sind nicht initialisiert (`s_sendcmd` in `src/send.h`). Zwei Ausprägungen sind reproduzierbar und als `DISABLED_` erfasst (Segmentation fault): letzter Teil ohne `;` und leerer Teil `;;` vor `D=`.
- Sendekommando, dessen erster Teil in `IB_1` nicht mit `S` beginnt (z. B. `<Müll>SR;…`): `cmdNo` bleibt 255 und `command[255]` liegt außerhalb des Arrays (`command[5]`), im Ergebnis Endlosschleife mit `repeats` = 255 (Issue #267). Die Tests dazu (`DISABLED_garbagePrefix…`, `DISABLED_onlySemicolon…`, `DISABLED_validCommandStillWorksAfterRejectedOne`) hängen ohne den Fix aus PR #268 und sind deshalb deaktiviert.
- `D=` mit Index eines nicht gesetzten Patterns oder Ziffer ≥ 8 (`buckets` nicht initialisiert bzw. Zugriff außerhalb des Arrays in `send_raw()`).

Die Tests für Schritt 2 stehen in `tests/test_serialCommand_system.cpp` (Fixture `ShortCommandTest`, parametrisiert `DecoderFlagTest`), die für Schritt 3 in `tests/test_serialCommand_send.cpp` (Fixture `SendCommandTest`) und die für Schritt 4 in `tests/test_serialCommand_cc1101.cpp` (Fixture `Cc1101Test`, Programm `TestProjectCC1101`).

## Auffälligkeiten im aktuellen Verhalten

Die Tests halten das aktuelle Verhalten fest. Folgende Punkte weichen von der Doku ab oder sind bei Grenzwerten auffällig (Fundstellen in `src/commands.h`, `src/send.h` bzw. `src/cc1101.cpp`):

| Kommando | Beobachtung | Fundstelle | Test |
| :--- | :--- | :--- | :--- |
| `r<addr>` | wird nur beantwortet, wenn `hasCC1101` gesetzt ist, obwohl nur das EEPROM gelesen wird | `cmd_read`, Bedingung `&& hasCC1101` | `readEepromRequiresCC1101Flag` |
| `r<addr>n` | liest immer 16 Bytes, ab `F1` also über Adresse `FF` hinaus | Schleife `EEPROM.read(reg + i)` | `readEepromBlockAtEndReadsBeyond0xFF` |
| `CSmcmbl=` | leerer oder nicht numerischer Wert setzt `0`; Werte > 255 bzw. negative laufen im `uint8_t` über | `configSET()`, `strtol` ohne Prüfung | `setMcMinBitLen*` |
| `CSmcmbl=` | Antwort `<n> bits set` ohne Zeilenende | `configSET()`, `MSG_PRINT` statt `MSG_PRINTLN` | `setMcMinBitLen` |
| `s` | ohne `CMP_CC1101` keine Antwort (Doku: `OK` oder Fehlermeldung) | `cmd_status` | `statusWithoutCC1101SupportPrintsNothing` |
| `SR`/`SM`/`SC` | Echo enthält Reste vorheriger Teile, wenn der `D=`-Teil kürzer ist als der vorherige Teil oder ein `P8=`-Teil übersprungen wurde (z. B. `…D=01;\n;`) | `send_cmd()`, `MSG_PRINT(buf)` ohne Nullterminierung nach `\n` | `rawEchoContainsStaleBytesWhenDataPartIsShorter`, `rawPatternIndexAboveSevenIsIgnored` |
| `SR`/`SM`/`SC` | `R=` ist `uint8_t`: `R=256` sendet nichts, `R=257` einmal | `strtoul` → `repeats` | `rawRepeatOverflowWraps*` |
| `SC` | `R=0` wird ignoriert, es wird einmal gesendet | `command[0].repeats > 0` | `combinedRepeatZeroSendsOnce` |
| `SM` | ungültige Hexzeichen werden ohne Fehlermeldung als Nibble gesendet (`G` → `0`) | `send_mc()` | `manchesterInvalidHexIsSentAsZeroNibble` |
| `S…` | nach `send cmd corrupt` / `send cmd to long` kehrt `send_cmd()` vor `enableReceive()` zurück, nachdem es am Anfang `disableReceive()` aufgerufen hat. Der Empfänger bleibt aus. `enableReceive()` steht sonst nur im Setup und bei `XE` | `send_cmd()`, `return` vor `enableReceive()` | `corruptCommandLeavesReceiverDisabled`, `tooLongCommandLeavesReceiverDisabled` |
| `S…` | Wartezeiten mit `while (stoptime > micros())` sind nicht überlaufsicher | `send_raw()`, `send_mc()` | `*MicrosOverflow*` |
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
- `serialtest::loadCommand(cmd)` kopiert ein Kommando mit Längenprüfung in `IB_1` (max. 13 Zeichen).
- `serialtest::runShortCommand(cmd)` (in `test_serialCommand.cpp`) hängt wie `serialEvent()` ein `\n` an und ruft `commands::HandleShortCommand()` auf.
- `SerialCapture::setInput(str, gapAt)` liefert Bytes für `Serial.readBytes()` usw., also den Rest eines Sendekommandos. Mit `gapAt` liefert der erste Lesezugriff an dieser Stelle nichts. Das simuliert einen Serial-Timeout, die restlichen Bytes bleiben im Puffer (`remainingInput()`).
- `serialtest::receiveState()` zählt die Aufrufe von `disableReceive()`/`enableReceive()` in `send.h`. Sie werden wie `micros()` per Präprozessor nur für `send.h` umgeleitet und rufen die echten Funktionen auf.
- `serialtest::fakeMicros()` ersetzt beim Einbinden von `send.h` per Präprozessor `micros()` (der arduino-mock liefert immer 0, die Warteschleifen in `send_raw()`/`send_mc()` würden nie enden). Jeder Aufruf zählt 1 µs weiter.
- `serialtest::SendRecorder` zeichnet die Pegelwechsel am Sendepin (`digitalLow`/`digitalHigh` → `pinMode(pin, LOW/HIGH)` im nativen Build) mit Zeitstempel auf und liefert das Signal als Dauern (`-` = low, `+` = high).
- `runSendCommand(cap, cmd, gapAt)` (in `test_serialCommand_send.cpp`) teilt das Kommando wie `serialEvent()`: bis zum ersten `;` in `IB_1`, der Rest kommt über `Serial`.
