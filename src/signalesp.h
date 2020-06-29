#pragma once

#if defined (ESP32) || defined(ESP8266)
#include "compile_config.h"

#define PROGNAME               " SIGNALESP "
#define VERSION_1              0x33
#define VERSION_2              0x1d
#define BAUDRATE               115200
#define FIFO_LENGTH            200

#define ETHERNET_PRINT
#define WIFI_MANAGER_OVERRIDE_STRINGS

// EEProm Addresscommands
#define EE_MAGIC_OFFSET        0
#define addr_features          0xff
#define MAX_SRV_CLIENTS        2

#include "compile_config.h"

void serialEvent();
void ICACHE_RAM_ATTR cronjob(void *pArg);
int freeRam();
inline void ethernetEvent();
//unsigned long getUptime();
//void enDisPrint(bool enDis);
//void getFunctions(bool *ms, bool *mu, bool *mc);
//void initEEPROM(void);
uint8_t rssiCallback() { return 0; }; // Dummy return if no rssi value can be retrieved from receiver
size_t writeCallback(const uint8_t *buf, uint8_t len = 1);
void ICACHE_RAM_ATTR sosBlink(void *pArg);

#if defined(ESP8266)
extern "C" {
#include "user_interface.h"
}
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#endif
#if defined(ESP32)
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <WiFi.h>
#include <WiFiType.h>
#endif

#include <FS.h>   
#include <EEPROM.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include "ArduinoJson.h"     //Local WebServer used to serve the configuration portal


#include "output.h"
#include "bitstore.h"  // Die wird aus irgend einem Grund zum Compilieren benoetigt.
#include "SimpleFIFO.h"

#ifdef CMP_CC1101
#include "cc1101.h"
#include <SPI.h>      // prevent travis errors
#endif

SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#include "signalDecoder.h"
#include "commands.h"
#include "functions.h"
#include "send.h"
#include "FastDelegate.h" 
#define WIFI_MANAGER_OVERRIDE_STRINGS
#include "wifi-config.h"
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager


WiFiServer Server(23);  //  port 23 = telnet
WiFiClient serverClient;

SignalDetectorClass musterDec;

#define pulseMin  90
volatile bool blinkLED = false;
String cmdstring = "";
volatile unsigned long lastTime = micros();

/*
#define digitalLow(P) digitalWrite(P,LOW)
#define digitalHigh(P) digitalWrite(P,HIGH)
#define isHigh(P) (digitalRead(P) == HIGH)
#define isLow(P) (digitalRead(P) == LOW)
#define digitalState(P)((uint8_t)isHigh(P))
*/

#ifdef ESP32
esp_timer_create_args_t cronTimer_args;
esp_timer_create_args_t blinksos_args;
esp_timer_handle_t cronTimer_handle;
esp_timer_handle_t blinksos_handle;
#elif defined(ESP8266)
os_timer_t cronTimer;
os_timer_t blinksos;
#endif

bool hasCC1101 = false;
char IB_1[14]; // Input Buffer one - capture commands




const char sos_sequence[] = "0101010001110001110001110001010100000000";
const char boot_sequence[] = "00010100111";



void ICACHE_RAM_ATTR sosBlink (void *pArg) {

	static uint8_t pos = 0;
	const char* pChar;
	pChar = (const char*)pArg; //OK in both C and C++


	digitalWrite(PIN_LED, pChar[pos] == '1' ? HIGH : LOW);
	pos++;
	if (pos == sizeof(pChar) * sizeof(pChar[1]))
		pos = 0;

}





WiFiManager wifiManager;
#ifdef ESP8266
WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
#endif

void setup() {
	//char cfg_ipmode[7] = "dhcp";
	//Server.setNoDelay(true);
#if defined(ESP8266)
	gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
	{
		//Server.stop();
		Server.begin();  // start telnet server
		Server.setNoDelay(true);
	});

	// added @Dattel #130
	disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected & event)
	{
		Server.stop();
		Serial.print("WiFi lost connection. Reason: ");
		Serial.println(event.reason);
	});
	// added @Dattel #130 - END

#elif defined(ESP32)
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Server.begin();  // start telnet server
		Server.setNoDelay(true);
	}, WiFiEvent_t::SYSTEM_EVENT_STA_CONNECTED);

	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Server.stop();  // end telnet server
		Serial.print("WiFi lost connection. Reason: ");
		Serial.println(info.sta_er_fail_reason);
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);
#endif


	//ESP.wdtEnable(2000);
#ifdef ESP32
	blinksos_args.callback = sosBlink;
	blinksos_args.dispatch_method = ESP_TIMER_TASK;
	blinksos_args.name = "blinkSOS";
	blinksos_args.arg = (void *)boot_sequence;
	esp_timer_create(&blinksos_args, &blinksos_handle);
	esp_timer_start_periodic(blinksos_handle, 300000);
#elif defined(ESP8266)
	os_timer_setfn(&blinksos, &sosBlink, (void *)boot_sequence);
	os_timer_arm(&blinksos, 300, true);
#endif

//	WiFi.setAutoConnect(false);
	WiFi.mode(WIFI_STA);

	Serial.begin(115200);
	Serial.setDebugOutput(true);
	while (!Serial)
		delay(90);

	Serial.println("\n\n");

	pinMode(PIN_RECEIVE, INPUT);
	pinMode(PIN_LED, OUTPUT);
  
	#ifdef CMP_CC1101
	cc1101::setup();
	#endif

	initEEPROM();

#ifdef CMP_CC1101
	DBG_PRINT(FPSTR(TXT_CCINIT));
	cc1101::CCinit();					 // CC1101 init
	hasCC1101 = cc1101::checkCC1101();	 // Check for cc1101

	if (hasCC1101)
	{
		DBG_PRINT(FPSTR(TXT_CC1101));
		DBG_PRINTLN(FPSTR(TXT_FOUND));
		musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
	}
	else {
		musterDec.setRSSICallback(&rssiCallback);	// Provide the RSSI Callback		
	}
#endif 

	wifiManager.setShowStaticFields(true);
	// wifiManager.setCountry("US");

/*
		1. starts a config portal in access point mode (timeout 60 seconds)
		2. Ii config portal times out, try connecting to a previous stored ssid in client mode
		3. if no connection is possible, start wps mode 
		If wps connection is successfull, ip address is retrieved via dhcp and saved. Otherwise esp is reseted and we start at 1. again  

		ip configuration can be switched between static and dhcp mode
*/
	IPAddress _ip, _gw, _sn;
/*
	DBG_PRINTLN("mounting FS...");
	
	if (SPIFFS.begin()) {
		DBG_PRINTLN("mounted file system");
		if (SPIFFS.exists("/config.json")) {
			//file exists, reading and loading
			DBG_PRINTLN("reading config file");
			File configFile = SPIFFS.open("/config.json", "r");
			if (configFile) {
				DBG_PRINTLN("opened config file");
				size_t size = configFile.size();
				// Allocate a buffer to store contents of the file.
				std::unique_ptr<char[]> buf(new char[size]);

				configFile.readBytes(buf.get(), size);
				DynamicJsonBuffer jsonBuffer;
				JsonObject& json = jsonBuffer.parseObject(buf.get());
				json.printTo(Serial);
				if (json.success()) {
					DBG_PRINTLN("\nparsed json");
					strcpy(cfg_ipmode, json["ipmode"]);
					if (strcmp(cfg_ipmode, "static")==0)
					{
						DBG_PRINT("ipmode: ");
						DBG_PRINTLN(cfg_ipmode);
						if (json["ip"]) {
							//DBG_PRINTLN("load custom ip from config");
							strcpy(static_ip, json["ip"]);
							strcpy(static_gw, json["gateway"]);
							strcpy(static_sn, json["subnet"]);
							//DBG_PRINTLN(static_ip);

							//  Load static IP to display them in the config portal
							_ip.fromString(static_ip);
							_gw.fromString(static_gw);
							_sn.fromString(static_sn);
							DBG_PRINTLN("apply static ip from config");
							wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);

						} else {
							DBG_PRINTLN("no custom ip in config");
						}
					}
				} else {
					DBG_PRINTLN("failed to load json config");
				}
				buf.release();

			}
			configFile.close();
		}
	}
	else {
		DBG_PRINTLN("failed to mount FS");
	}
	//end read
	*/

	//Todo: Add custom html code to better explain how this input field works
	
	//WiFiManagerParameter custom_ipconfig("ipmode", "static or dhcp", cfg_ipmode, 7);
	//wifiManager.addParameter(&custom_ipconfig);

	//wifiManager.setConfigPortalTimeout(60);
	//wifiManager.setSaveConfigCallback(saveConfigCallback);
	//wifiManager.setBreakAfterConfig(true); // Exit the config portal even if there is a wrong config

	//bool wps_successfull=false;
	Serial.println("Starting config portal with SSID: NodeDuinoConfig");
	/*
	if (!wifiManager.startConfigPortal("NodeDuinoConfig", NULL)) {

		wifiManager.setConfigPortalTimeout(1);
		if (wifiManager.autoConnect()) {
			//if you get here you have connected to the WiFi
			Serial.println("connected...)");
			Serial.println("local ip");
			Serial.println(WiFi.localIP());
			os_timer_disarm(&blinksos);
			// Blink if we have a connection
			for (int i = 0; i < 10; i++)
			{
				digitalHigh(PIN_LED);
				delay(1000);
				digitalLow(PIN_LED);
			}
		} else {
			Serial.println("failed to connect, we now enter WPS Mode");
			delay(3000);
			os_timer_disarm(&blinksos);
			os_timer_setfn(&blinksos, &sosBlink, (void *)sos_sequence);
			os_timer_arm(&blinksos, 300, true);

			Serial.printf("\nCould not connect to WiFi. state='%d'\n", WiFi.status());
			Serial.println("Please press WPS button on your router");
			delay(5000);
			if (!startWPS()) {
				Serial.println("Failed to connect with WPS, will restart ESP now :-(");
				delay(3500);

				//while (1);
				ESP.restart();
				ESP.reset();
			}
			strcpy(cfg_ipmode, "dhcp");
			wps_successfull = true;
			shouldSaveConfig = true;
			delay(5000);
		}
	}
	*/
	wifiManager.autoConnect("NodeDuinoConfig",NULL);

	/*
	if (shouldSaveConfig)
	{
		DBG_PRINTLN("saving config");
		DynamicJsonBuffer jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		char old_cfg_ipmode[7];
		strcpy(old_cfg_ipmode, cfg_ipmode);

		if (wps_successfull == false) {
			json["ipmode"] = custom_ipconfig.getValue();
			strcpy(cfg_ipmode, json["ipmode"]);
		} else
			json["ipmode"] = cfg_ipmode;

		if (strcmp(cfg_ipmode,"static")==0)
		{
			DBG_PRINTLN("saving static ip config");
			json["ip"] = WiFi.localIP().toString();
			json["gateway"] = WiFi.gatewayIP().toString();
			json["subnet"] = WiFi.subnetMask().toString();

			strcpy(static_ip, json["ip"]);
			strcpy(static_gw, json["gateway"]);
			strcpy(static_sn, json["subnet"]);
		}
		File configFile = SPIFFS.open("/config.json", "w");
		if (!configFile) {
			DBG_PRINTLN("failed to open config file for writing");
		}

		json.prettyPrintTo(Serial);
		json.printTo(configFile);
		configFile.close();
		if (strcmp(cfg_ipmode,"dhcp") == 0 && strcmp(old_cfg_ipmode,"dhcp") != 0)
		{
			Serial.println("Reenable DHCP client mode. Restarting ESP now");
			delay(1000);
			ESP.restart(); // Reset ESP to re-enable DHCP mode
			ESP.reset();
			//Todo: Store in eeprom that we restarted so we can skip configportal for one reboot

		}
		if (strcmp(cfg_ipmode,"static") == 0) {
			_ip.fromString(static_ip);
			_gw.fromString(static_gw);
			_sn.fromString(static_sn);
			DBG_PRINTLN("update static ip from config");
			wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn); // This should disable static ip mode for wifimanger and allow dhcp again	}
		}
	}
	*/

#ifdef ESP32
	cronTimer_args.callback = cronjob;
	cronTimer_args.name = "cronTimer";
	cronTimer_args.dispatch_method = ESP_TIMER_TASK;
	esp_timer_create(&cronTimer_args, &cronTimer_handle);
#elif defined(ESP8266)
	os_timer_disarm(&cronTimer);
	os_timer_setfn(&cronTimer, &cronjob, 0);
#endif

	musterDec.setStreamCallback(writeCallback);
#ifdef CMP_CC1101
	if (!hasCC1101 || cc1101::regCheck()) {
#endif
		enableReceive();
		DBG_PRINT(FPSTR(TXT_RECENA));
#ifdef CMP_CC1101
	}
	else {
		DBG_PRINT(FPSTR(TXT_CC1101));
		DBG_PRINT(FPSTR(TXT_DOFRESET));
		DBG_PRINTLN(FPSTR(TXT_COMMAND));
	}
#endif
	MSG_PRINTER.setTimeout(400);

//	WiFi.mode(WIFI_STA);
	wifiManager.setConfigPortalBlocking( false);
//	wifiManager.startConfigPortal();
	wifiManager.startWebPortal();
#ifdef ESP32
	esp_timer_start_periodic(cronTimer_handle, 31000);
	esp_timer_stop(blinksos_handle);
#elif defined(ESP8266)
	os_timer_arm(&cronTimer, 31, true);
	os_timer_disarm(&blinksos);
#endif
	pinAsOutput(PIN_SEND);
	digitalLow(PIN_LED);
}

void ICACHE_RAM_ATTR cronjob(void *pArg) {

	cli();
	static uint8_t cnt = 0;

	const unsigned long  duration = micros() - lastTime;
	long timerTime = maxPulse - duration + 1000;
	if (timerTime < 1000)
	    timerTime=1000;
    
#ifdef ESP32
	esp_timer_stop(cronTimer_handle);
	esp_timer_start_periodic(cronTimer_handle, timerTime);
#elif defined(ESP8266)
	os_timer_disarm(&cronTimer);
	os_timer_arm(&cronTimer, timerTime / 1000, true);
#endif


	if (duration > maxPulse) { //Auf Maximalwert pruefen.
		int sDuration = maxPulse;
		if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			sDuration = -sDuration;
		}
		FiFo.enqueue(sDuration);
		lastTime = micros();
	}
	else if (duration > 10000) {
		//os_timer_disarm(&cronTimer);
		//os_timer_arm(&cronTimer, 20, true);
	}
#ifdef PIN_LED_INVERSE	
	digitalWrite(PIN_LED, !blinkLED);
#else
	digitalWrite(PIN_LED, blinkLED);
#endif
	blinkLED = false;

	sei();
	// Infrequent time uncritical jobs (~ every 2 hours)
	if (cnt++ == 0)  // if cnt is 0 at start or during rollover
		getUptime();

}


void loop() {

	wifiManager.process();
	
	static int aktVal = 0;
	bool state;
	serialEvent();
	ethernetEvent();

	while (FiFo.count()>0) { //Puffer auslesen und an Dekoder uebergeben
		aktVal = FiFo.dequeue();
		state = musterDec.decode(&aktVal);
		if (state) blinkLED = true; //LED blinken, wenn Meldung dekodiert
		if (FiFo.count()<120) yield();
	}

}

//============================== Write callback =========================================

#define _USE_WRITE_BUFFER

#ifdef _USE_WRITE_BUFFER
const size_t writeBufferSize = 128;
size_t writeBufferCurrent = 0;
uint8_t writeBuffer[writeBufferSize];
#endif
size_t writeCallback(const uint8_t *buf, uint8_t len)
{
#ifdef _USE_WRITE_BUFFER
	if (!serverClient || !serverClient.connected())
		return 0;

	size_t result = 0;

	while (len > 0) {
		size_t copy = (len > writeBufferSize - writeBufferCurrent ? writeBufferSize - writeBufferCurrent : len);
		if (copy > 0)
		{
			memcpy(writeBuffer + writeBufferCurrent, buf, copy);
			writeBufferCurrent = writeBufferCurrent + copy;
		}
		// Buffer full or \n detected - force send
		if ((len == 1 && *buf == char(0xA)) || (writeBufferCurrent == writeBufferSize))
		{
			size_t byteswritten = 0;
			if (serverClient && serverClient.connected()) {
				byteswritten = serverClient.write(writeBuffer, writeBufferCurrent);
			}

			if (byteswritten < writeBufferCurrent) {
				memmove(writeBuffer, writeBuffer + byteswritten, writeBufferCurrent - byteswritten);
				writeBufferCurrent -= byteswritten;
			}
			else {
				writeBufferCurrent = 0;
			}
			result += byteswritten;
		}
		// buffer full
		len = len - copy;
		if (len > 0)
		{
			memmove((void*)buf, buf + copy, len);
		}
	}
	return len;
#else

	while (!serverClient.available()) {
		yield();
		if (!serverClient.connected()) return 0;
	}
	DBG_PRINTLN("Called writeCallback");

	memccpy()

		return serverClient.write(buf, len);
	//serverClient.write("test");
#endif
}


inline void ethernetEvent()
{
	//check if there are any new clients
	if (Server.hasClient()) {
		
		if (!serverClient || !serverClient.connected()) {
			if (serverClient) serverClient.stop();
			serverClient = Server.available();
			serverClient.flush();
			//DBG_PRINTLN("New client: ");
			//DBG_PRINTLN(serverClient.remoteIP());
		} else {
			WiFiClient rejectClient = Server.available();
            rejectClient.stop();
			//DBG_PRINTLN("Reject new Client: ");
			//DBG_PRINTLN(rejectClient.remoteIP());
		}
	}

	if(serverClient && !serverClient.connected())
	{
		//DBG_PRINTLN("Client disconnected: ");
		//DBG_PRINTLN(serverClient.remoteIP());
		serverClient.stop();
	}
}

void serialEvent()
{
	static uint8_t idx = 0;
	while (MSG_PRINTER.connected() > 0 && MSG_PRINTER.available())
	{
		if (idx == 14) {
			// Short buffer is now full
			MSG_PRINT("Command to long: ");
			MSG_PRINTLN(IB_1);
			idx = 0;
			return;
		}
		else {
			IB_1[idx] = (char)MSG_PRINTER.read();
			// DBG_PRINTLN(idx);
			// DBG_PRINT(IB_1[idx]);

			switch (IB_1[idx])
			{
			case '\n':
			case '\r':
			case '\0':
			case '#':
#if defined(ESP32)
				esp_task_wdt_reset();
				yield();
#elif defined(ESP8266)
				wdt_reset();
#endif
				if (idx > 0) {
					// DBG_PRINT("HSC");
					commands::HandleShortCommand();  // Short command received and can be processed now
				}
				idx = 0;
				return; //Exit function
			case ';':
				DBG_PRINT("send cmd detected ");
				// DBG_PRINTLN(idx);
				IB_1[idx + 1] = '\0';
				if (idx > 0)
					send_cmd();
				idx = 0; // increments to 1
				return; //Exit function
			}
			idx++;
		}
		yield();
	}
}


int freeRam() {
#ifdef ESP32
	return ESP.getFreeHeap();
#elif defined(ESP8266)
	return system_get_free_heap_size();
#endif
}

#endif
