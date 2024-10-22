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
  Portal.config(Config);
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

  Portal.config(Config);

  Portal.onOTAStart(exitOTAStart);
  Portal.onOTAEnd(exitOTAEnd);
  Portal.onOTAProgress(exitOTAProgress);
  Portal.onOTAError(exitOTAError);
  

  //PortalServer.on("/", rootPage);

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
