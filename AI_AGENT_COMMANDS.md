# Befehlsreferenz für AI-Agenten

Dieses Dokument listet die verfügbaren seriellen Befehle für die SIGNALDuino Firmware auf.

## Systembefehle

| Syntax | Funktion | Antwort | Beispiel |
| :--- | :--- | :--- | :--- |
| `V` | Version abfragen | Versionsstring inkl. Kompilierzeitpunkt, CC1101-Status und Frequenz | `V` -> `V 3.3.1-dev SIGNALduino - compiled at ...` |
| `?` | Hilfe anzeigen | Liste der unterstützten Befehle | `?` -> `? Use one of V R t X S P C r W s x e` |
| `R` | Freien RAM abfragen | Anzahl der freien Bytes im RAM | `R` -> `1234` |
| `t` | Uptime abfragen | Systemlaufzeit in Sekunden | `t` -> `3600` |
| `P` | Ping | Prüft Verbindung | `P` -> `OK` |
| `s` | Status abfragen | Prüft CC1101 Status (falls vorhanden) | `s` -> `OK` oder Fehlermeldung |
| `XQ` | Empfänger deaktivieren | Deaktiviert den Empfang | (keine direkte Ausgabe) |
| `XE` | Empfänger aktivieren | Aktiviert den Empfang | (keine direkte Ausgabe) |
| `e` | CC1101 Factory Reset | Setzt CC1101 zurück und lädt EEPROM-Defaults (nur mit CC1101) | `e` -> `Init eeprom to defaults...` |

## Konfigurationsbefehle

| Syntax | Funktion | Antwort | Beispiel |
| :--- | :--- | :--- | :--- |
| `CG` | Konfiguration lesen | Gibt aktuelle Decodereinstellungen aus (MS, MU, MC, Mred, AFC, WMBus) | `CG` -> `MS=1;MU=1;MC=1;Mred=1` |
| `C<FLAG><CMD>` | Decoder konfigurieren | Aktiviert/Deaktiviert Decoder (FLAG: E=Enable, D=Disable; CMD: S=MS, U=MU, C=MC, R=Mred, A=AFC) | `CER` -> (Einschalten der Datenkomprimierung (config: Mred=1)) |
| `CSmcmbl=<val>` | MC Min Bit Length setzen | Setzt minimale Bitlänge für Manchester-Decoding | `CSmcmbl=20` -> `20 bits set` |
| `C<reg>` | CC1101 Register lesen | Liest Wert eines CC1101-Registers (hex) | `C0D` -> (Liest Register 0x0D) |
| `W<reg><val>` | EEPROM/CC1101 schreiben | Schreibt Wert in EEPROM und CC1101 Register (reg/val als 2-stellige Hex-Strings) | `W0001` -> (Schreibt 0x01 nach Reg 0x00) |
| `WS<id>` | WMBus Init (CC1101) | Sendet ein strobe command zum cc1101 (Strobe 0x36 SIDLE, 0x3A SFRX, 0x34 SRX) | `WS34` |
| `r<addr>` | EEPROM lesen | Liest Byte an EEPROM-Adresse (hex) | `r00` -> `EEPROM 00 = 33` |
| `r<addr>n` | EEPROM Block lesen | Liest 16 Bytes ab EEPROM-Adresse | `r00n` -> `EEPROM 00 : 33 1D ...` |
| `x<val>` | PA Table schreiben | Schreibt Wert in CC1101 PA Table (Sendeleistung) | `x12` -> `Write 12 to patable` |

### Bedeutung der Strobe commands

*   `WS36`: **SIDLE** - Exit RX / TX, turn off frequency synthesizer
*   `WS3A`: **SFRX** - Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
*   `WS34`: **SRX** - Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.

### WMBus Konfiguration

Die Konfiguration für WMBus erfolgt ausschließlich durch direktes Schreiben der entsprechenden CC1101 Register mittels des `W<reg><val>` Befehls. Es gibt keine speziellen `C` Befehle für die WMBus Konfiguration.

Details zu den Registern finden sich im CC1101 Datenblatt von Texas Instruments:
https://www.ti.com/lit/ds/symlink/cc1101.pdf

## Sendebefehle

Die Sendebefehle beginnen mit `S` gefolgt vom Typ (`C`=Combined/Raw, `M`=Manchester, `R`=Raw, `N`=xFSK).
Parameter werden oft mit `key=value;` angehängt.

| Syntax | Funktion | Parameter | Beispiel |
| :--- | :--- | :--- | :--- |
| `SC...` | Send Combined | Startet eine kombinierte Sendesequenz. | `SC;R=4;SM;C=400;D=AFFFFFFFFE;...` |
| `SM...` | Send Manchester | Sendet Manchester-codierte Daten. | `SM;C=400;D=AFFFFFFFFE;` |
| `SR...` | Send Raw | Sendet rohe Timing-Daten. | `SR;R=3;P0=-2500;P1=400;D=010;` |
| `SN...` | Send xFSK | Sendet Daten im xFSK Modus (für CC1101). | `SN;R=13;N=2;D=91C64...;` |

### Parameter für Sendebefehle

*   `R=<n>`: Anzahl der Wiederholungen (Repeats)
*   `C=<n>`: Clock/Zeitbasis in Mikrosekunden (für Manchester)
*   `P<n>=<val>`: Definition eines Timings (Pulse/Pause) für Raw-Daten. `P0`, `P1`, etc. (Buckets). Negative Werte sind Pausen (Low), positive Pulse (High).
*   `D=<data>`: Die zu sendenden Daten (Hex-String oder Sequenz von P-Indizes für Raw).
*   `F=<hex>`: (Optional, CC1101) Frequenzregister-Update vor dem Senden.

**Beispiel RAW:** `SR;R=3;P0=1230;P1=-3120;P2=-400;P3=-900;D=030301010101010202020202020101010102020202010101010202010120202;`
(Definiert 4 Timings P0-P3 und sendet dann die Sequenz basierend auf diesen Indizes).

**Beispiel Manchester:** `SM;C=400;D=AFFFFFFFFE;`
(Sendet Hex-Daten AFFFFFFFFE mit 400us Clock Manchester-codiert).
