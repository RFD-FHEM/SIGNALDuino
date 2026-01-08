#include "main.h"

#if defined (ESP32) || defined(ESP8266)
  // Definition des Array-Objekts, das in signalesp.h als extern deklariert ist
  // Dies behebt den Linker-Fehler, nachdem die Deklaration in signalesp.h zu extern geändert wurde.
  #include "signalesp.h"
  WiFiClient serverClient[MAX_SRV_CLIENTS];

  MultiClientPrinter _MultiClientPrinterInstance;
  Stream &TelnetPrint = _MultiClientPrinterInstance;
#endif