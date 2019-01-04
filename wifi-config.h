#include <ArduinoJson.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#elif defined(ESP32)
#include <WiFi.h>
#include <esp_wifi.h>  
#endif 

#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager



//default custom static IP
char static_ip[16] = "0.0.0.0";
char static_gw[16] = "0.0.0.0";
char static_sn[16] = "0.0.0.0";

//flag for saving data
bool shouldSaveConfig = false;


void configModeCallback(WiFiManager *myWiFiManager) {
	Serial.println("Entered config mode");
	Serial.println(WiFi.softAPIP());
	//if you used auto generated SSID, print it
	Serial.println(myWiFiManager->getConfigPortalSSID());
}



void saveConfigCallback() {
	DBG_PRINTLN("Should save config");
	shouldSaveConfig = true;
}



void resetwifi() {
	WiFiManager wifiManager;
	wifiManager.resetSettings();
#ifdef esp8266
	ESP.reset();
#elif defined(ESP32)
	ESP.restart();
#endif

	
}
