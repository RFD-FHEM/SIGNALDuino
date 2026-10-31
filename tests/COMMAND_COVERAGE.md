# Testabdeckung der seriellen Kommandos

Grundlage: die in [`AI_AGENT_COMMANDS.md`](../AI_AGENT_COMMANDS.md) dokumentierten Kommandos.
Getestet wird im nativen Testbuild (`tests/CMakeLists.txt`, GoogleTest + arduino-mock) ohne Änderungen an der Firmware.

Legende Status: ✅ getestet · ⏳ geplant · 🔒 im nativen Build nicht kompiliert (benötigt `CMP_CC1101`)

Spalte "Schritt": 2 = System-/Konfigurationskommandos, 3 = Sendekommandos, 4 = CC1101-Kommandos (eigenes Test-Target).

## Systembefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `V` | Versionsstring `V <PROGVERS><PROGNAME> - compiled at …` | Zusatzzeichen nach `V` | 2 | ✅ |
| `?` | Liste `? Use one of V R t X S P C r W s` | Zusatzzeichen nach `?` | 2 | ✅ |
| `R` | freier RAM (`freeRam()`) | Zusatzzeichen | 2 | ✅ |
| `t` | Uptime in Sekunden (Wert gegen `millis()` des Mocks geprüft) | Überlauf nicht testbar: der arduino-mock leitet `millis()` aus der Systemzeit ab | 2 | ✅ |
| `P` | `OK` | Zusatzzeichen | 2 | ✅ |
| `s` | ohne `CMP_CC1101` keine Ausgabe | – | 2 | ✅ |
| `XQ` / `XE` | Empfänger aus/an | `X` allein, `Xz` (unbekanntes Flag) | 2 | ✅ |
| `e` | Factory Reset | – | 4 | 🔒 |
| unbekannt | `Unsupported short command` | leerer Puffer, Steuerzeichen | 2 | ✅ |

## Konfigurationsbefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `CG` | `MS=…;MU=…;MC=…;Mred=…` | – | 2 | ✅ |
| `CE<x>` / `CD<x>` (S, U, C, R) | Flag gesetzt/gelöscht | unbekanntes `<x>`, fehlendes `<x>` | 2 | ✅ |
| `CEA`/`CDA`, `CEW`/`CDW`, `CET`/`CDT` | AFC / WMBus / WMBus_T | nativ geprüft: werden ignoriert ✅ | 4 | 🔒 |
| `CSmcmbl=<val>` | `<val> bits set` | kein Wert, negativ, nicht numerisch, sehr groß, falscher Schlüssel | 2 | ✅ |
| `C<reg>` | CC1101-Register lesen | kein Hex, eine Stelle | 4 | 🔒 |
| `W<reg><val>` | EEPROM-Schreibzugriff | ungültiges Hex, zu kurz, `00`/`FF` | 2 | ✅ |
| `WS<id>` | Strobe-Kommando | – | 4 | 🔒 (nativ geprüft: kein EEPROM-Schreibzugriff ✅) |
| `r<addr>` | `EEPROM <addr> = <val>` | ungültiges Hex, eine Stelle, `FF` | 2 | ✅ |
| `r<addr>n` | 16 Bytes ab `<addr>` | Adresse am Ende des Bereichs | 2 | ✅ |
| `x<val>` | PA Table | – | 4 | 🔒 |

## Sendebefehle

| Kommando | Gutfall | Fehler-/Grenzfälle | Schritt | Status |
| :--- | :--- | :--- | :---: | :---: |
| `SR` | Rohsequenz mit `P<n>=` und `D=` | fehlendes `D=`/`P<n>=`, abgeschnittene Übertragung, Extremwerte `R=` | 3 | ⏳ (bestehend: `serial.command_sendCorrupt*`) |
| `SM` | Manchester mit `C=` und `D=` | fehlendes `C=`, ungültiges Hex in `D=` | 3 | ⏳ |
| `SC` | kombinierte Sequenz | fehlende Teilsequenz | 3 | ⏳ |
| `SN` | xFSK | – | 3/4 | ⏳ |

Die Tests für Schritt 2 stehen in `tests/test_serialCommand_system.cpp` (Fixture `ShortCommandTest`, parametrisiert `DecoderFlagTest`).

## Auffälligkeiten im aktuellen Verhalten

Die Tests halten das aktuelle Verhalten fest. Folgende Punkte weichen von der Doku ab oder sind bei Grenzwerten auffällig (Fundstellen in `src/commands.h`):

| Kommando | Beobachtung | Fundstelle | Test |
| :--- | :--- | :--- | :--- |
| `r<addr>` | wird nur beantwortet, wenn `hasCC1101` gesetzt ist, obwohl nur das EEPROM gelesen wird | `cmd_read`, Bedingung `&& hasCC1101` | `readEepromRequiresCC1101Flag` |
| `r<addr>n` | liest immer 16 Bytes, ab `F1` also über Adresse `FF` hinaus | Schleife `EEPROM.read(reg + i)` | `readEepromBlockAtEndReadsBeyond0xFF` |
| `CSmcmbl=` | leerer oder nicht numerischer Wert setzt `0`; Werte > 255 bzw. negative laufen im `uint8_t` über | `configSET()`, `strtol` ohne Prüfung | `setMcMinBitLen*` |
| `CSmcmbl=` | Antwort `<n> bits set` ohne Zeilenende | `configSET()`, `MSG_PRINT` statt `MSG_PRINTLN` | `setMcMinBitLen` |
| `s` | ohne `CMP_CC1101` keine Antwort (Doku: `OK` oder Fehlermeldung) | `cmd_status` | `statusWithoutCC1101SupportPrintsNothing` |

## Testhelfer

`tests/serialCommandHelper.h`:

- `serialtest::SerialCapture` zeichnet alle Ausgaben über `Serial` (`MSG_PRINT`, `MSG_PRINTLN`, `MSG_WRITE`) als String auf. `println` hängt `\n` an, Zahlen werden wie in der Arduino-`Print`-Klasse formatiert.
- `serialtest::loadCommand(cmd)` kopiert ein Kommando mit Längenprüfung in `IB_1` (max. 13 Zeichen).
- `serialtest::runShortCommand(cmd)` (in `test_serialCommand.cpp`) hängt wie `serialEvent()` ein `\n` an und ruft `commands::HandleShortCommand()` auf.
