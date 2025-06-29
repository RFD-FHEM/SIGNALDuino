#pragma once

#if defined (ESP32) || defined(ESP8266)

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
ESP8266WebServer PortalServer(80);
#elif defined(ESP32)
#include <WiFi.h>
//#include <esp_wifi.h>  
#include <WebServer.h>
WebServer PortalServer(80);             
#endif 

#define SD_SSID "SignalESP"
#define AUTOCONNECT_MENULABEL_CONFIGNEW   "WiFi scan"
#define AUTOCONNECT_MENULABEL_OPENSSIDS   "Saved networks"
#define AUTOCONNECT_MENULABEL_RESET       "Reboot"

#include <AutoConnect.h>



AutoConnect Portal(PortalServer);
AutoConnectConfig Config;       // Enable autoReconnect supported on v0.9.4
AutoConnectAux auxInfo("/info", "System Info", true);

bool connected;
unsigned long elapsed;

void exitOTAStart() {
  Serial.println("OTA started");
}

void exitOTAProgress(unsigned int amount, unsigned int sz) {
  Serial.printf("OTA in progress: received %d bytes, total %d bytes\n", sz, amount);
}

void exitOTAEnd() {
  Serial.println("OTA ended");
}

void exitOTAError(uint8_t err) {
  Serial.printf("OTA error occurred %d\n", err);
}

void onWifiConnect(IPAddress& ip) {
  Serial.print("WiFi connected: " + WiFi.SSID());
  Serial.println("\tIP: " + WiFi.localIP().toString());
  Config.ota = AC_OTA_BUILTIN;
  Config.otaExtraCaption = PROGVERS;
  connected=true;
  Portal.config(Config);
}


String onSystemInfoPage(AutoConnectAux& aux, PageArgument& args) {
  String html = "<html><meta http-equiv='refresh' content='60'><body>";
  html += "<h1>SIGNALDuino System Info</h1>";
  html += "<table style='border-collapse: collapse; font-family: monospace'>";

  // Firmware
  html += "<tr><td><b>Firmware-Version</b></td><td>" + String(PROGVERS) + "</td></tr>";
  html += "<tr><td><b>Uptime</b></td><td>" + String(getUptime()) + "</td></tr>";

#if defined(ESP8266)
  html += "<tr><td><b>Plattform</b></td><td>ESP8266</td></tr>";
  html += "<tr><td><b>Chip-ID</b></td><td>" + String(ESP.getChipId()) + "</td></tr>";
  html += "<tr><td><b>CPU-Frequenz</b></td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
  html += "<tr><td><b>Heap frei</b></td><td>" + String(ESP.getFreeHeap()) + " Bytes</td></tr>";
  html += "<tr><td><b>Flash-Größe</b></td><td>" + String(ESP.getFlashChipRealSize() / 1024) + " KB</td></tr>";
  html += "<tr><td><b>SDK-Version</b></td><td>" + String(ESP.getSdkVersion()) + "</td></tr>";

#elif defined(ESP32)
  html += "<tr><td><b>Plattform</b></td><td>ESP32</td></tr>";

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  html += "<tr><td><b>Chip-Revision</b></td><td>" + String(chip_info.revision) + "</td></tr>";
  html += "<tr><td><b>CPU-Kerne</b></td><td>" + String(chip_info.cores) + "</td></tr>";
  // html += "<tr><td><b>CPU-Frequenz</b></td><td>" + String(esp_clk_cpu_freq() / 1000000) + " MHz</td></tr>";
  html += "<tr><td><b>Heap frei</b></td><td>" + String(esp_get_free_heap_size()) + " Bytes</td></tr>";
  html += "<tr><td><b>Flash-Größe</b></td><td>" + String(spi_flash_get_chip_size() / 1024) + " KB</td></tr>";
  html += "<tr><td><b>IDF-Version</b></td><td>" + String(esp_get_idf_version()) + "</td></tr>";
#endif

#ifdef CMP_CC1101
  
  html += "<tr><td><b>CC1101 connected</b></td><td>" + String(hasCC1101) + "</td></tr>";
  html += "<tr><td><b>Funkchip</b></td><td>" + String(FPSTR(TXT_CC110)) + " " + commands::getCC1101ChipString(cc1101::chipVersion()) + "</td></tr>";
#endif

  html += "</table>";
  return html;

}

void setupWifi()
{
  Config.title = S_brand;


  #ifdef ESP8266
  Config.apid = "SignalESP-" + String(ESP.getChipId(), HEX);
  #elif defined(ESP32)
  Config.apid = "SignalESP-" + String((uint32_t)(ESP.getEfuseMac() >> 32), HEX);
  #endif
  Config.hostName = Config.apid;

  Config.autoReconnect = true;
  Config.reconnectInterval = 2;
  Config.retainPortal = true;   // Keep the captive portal open.
  Config.principle = AC_PRINCIPLE_RSSI;

  Portal.config(Config);

  Portal.onOTAStart(exitOTAStart);
  Portal.onOTAEnd(exitOTAEnd);
  Portal.onOTAProgress(exitOTAProgress);
  Portal.onOTAError(exitOTAError);
  
  Portal.onNotFound([]() {
    auxInfo.redirect("/info");  // optionaler Redirect zur Info-Seite
  });
  auxInfo.on(onSystemInfoPage);
  Portal.join({auxInfo});  // registriere Info Seite

 // PortalServer.on("/", rootPage);

  Portal.onConnect(onWifiConnect);
  Portal.begin();
}

void resetwifi() {
//	wifiManager.resetSettings();
#ifdef esp8266
	ESP.reset();
#elif defined(ESP32)
	ESP.restart();
#endif

	
}

#endif
