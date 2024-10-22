#pragma once

#if defined (ESP32) || defined(ESP8266)
#include "compile_config.h"

#define PROGNAME               " SIGNALESP "
#define VERSION_1              0x33
#define VERSION_2              0x1d
#define BAUDRATE               115200
#define FIFO_LENGTH            200
#define S_brand                " SIGNALESP "
#define S_debugPrefix           " DB "
#define ETHERNET_PRINT

// EEProm Addresscommands
#define EE_MAGIC_OFFSET        0
#define addr_features          0xff
#define MAX_SRV_CLIENTS        2

#include "compile_config.h"

void serialEvent();
void IRAM_ATTR cronjob(void *pArg);
int freeRam();
inline void ethernetEvent();
//unsigned long getUptime();
//void enDisPrint(bool enDis);
//void getFunctions(bool *ms, bool *mu, bool *mc);
//void initEEPROM(void);
uint8_t rssiCallback() { return 0; }; // Dummy return if no rssi value can be retrieved from receiver
size_t writeCallback(const uint8_t *buf, uint8_t len = 1);
void IRAM_ATTR sosBlink(void *pArg);

#if defined(ESP8266)
  extern "C" {
    #include "user_interface.h"
  }

  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>   // Local WebServer used to serve the configuration portal
#endif

#if defined(ESP32)
  #include "esp_timer.h"
  #include "esp_task_wdt.h"
  #include <WiFi.h>
  #include <WiFiType.h>
#endif

#include <FS.h>
#include <EEPROM.h>
#include <DNSServer.h>             // Local DNS Server used for redirecting all requests to the configuration portal

#include "output.h"
#include "bitstore.h"              // Die wird aus irgend einem Grund zum Compilieren benoetigt.
#include "SimpleFIFO.h"

#ifdef CMP_CC1101
  #include "cc1101.h"
  #include <SPI.h>                 // prevent travis errors
  #include "mbus.h"
#endif

SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#include "signalDecoder.h"
#include "commands.h"
#include "functions.h"
#include "send.h"
#include "FastDelegate.h"
#include "wifi-config.h"


WiFiServer Server(23);             //  port 23 = telnet
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
bool AfcEnabled = true;
char IB_1[14];                     // Input Buffer one - capture commands
#ifdef CMP_CC1101
  bool wmbus = false;
  bool wmbus_t = false;
#endif
const char sos_sequence[] = "0101010001110001110001110001010100000000";
const char boot_sequence[] = "00010100111";



void IRAM_ATTR sosBlink (void *pArg) {
  static uint8_t pos = 0;
  const char* pChar;
  pChar = (const char*)pArg;      //OK in both C and C++

  digitalWrite(PIN_LED, pChar[pos] == '1' ? HIGH : LOW);
  pos++;
  if (pos == sizeof(pChar) * sizeof(pChar[1]))
    pos = 0;
}



#ifdef ESP8266
  WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
#endif

void restart(){
  ESP.restart();
}

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {
    delay(90); // wait for serial port to connect. Needed for native USB
  }
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
  }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Server.stop();  // end telnet server
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.wps_fail_reason);
    
  }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
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

//WiFi.setAutoConnect(false);
//WiFi.mode(WIFI_STA);


  Serial.setDebugOutput(true);
  Serial.println("\n\n");

  pinMode(PIN_RECEIVE, INPUT);
  pinMode(PIN_LED, OUTPUT);

#ifdef CMP_CC1101
  cc1101::setup();
#endif

initEEPROM();

#ifdef CMP_CC1101
  cc1101::CCinit();                                // CC1101 init
  hasCC1101 = cc1101::checkCC1101();               // Check for cc1101

  if (hasCC1101)
  {
    DBG_PRINT(FPSTR(TXT_CC1101));
    DBG_PRINTLN(FPSTR(TXT_FOUND));
    musterDec.setRSSICallback(&cc1101::getRSSI);   // Provide the RSSI Callback
  }
  else {
    musterDec.setRSSICallback(&rssiCallback);      // Provide the RSSI Callback
  }
#endif 



#ifdef ESP32
  cronTimer_args.callback = cronjob;
  cronTimer_args.name = "cronTimer";
  cronTimer_args.dispatch_method = ESP_TIMER_TASK;
  esp_timer_create(&cronTimer_args, &cronTimer_handle);
#elif defined(ESP8266)
  os_timer_disarm(&cronTimer);
  os_timer_setfn(&cronTimer, &cronjob, 0);
#endif

  setupWifi();

  musterDec.setStreamCallback(writeCallback);

#ifdef CMP_CC1101
  if (!hasCC1101 || cc1101::regCheck()) {
#endif
    enableReceive();
    DBG_PRINTLN(FPSTR(TXT_RECENA));
#ifdef CMP_CC1101
  }
  else {
    DBG_PRINT(FPSTR(TXT_CC1101));
    DBG_PRINT(FPSTR(TXT_DOFRESET));
    DBG_PRINTLN(FPSTR(TXT_COMMAND));
  }
#endif


MSG_PRINTER.setTimeout(400);

#ifdef ESP32
  esp_timer_start_periodic(cronTimer_handle, 31000);
  esp_timer_stop(blinksos_handle);
#elif defined(ESP8266)
  os_timer_arm(&cronTimer, 31, true);
  os_timer_disarm(&blinksos);
#endif

pinAsOutput(PIN_SEND);
digitalLow(PIN_LED);
#ifdef CMP_CC1101
  if (wmbus == 1) { // WMBus
    mbus_init((uint8_t)wmbus_t + 1); // WMBus mode S or T
  }
#endif
}



void IRAM_ATTR cronjob(void *pArg) {
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

  if (duration > maxPulse) {       // Auf Maximalwert pruefen.
    int sDuration = maxPulse;
    if (isLow(PIN_RECEIVE)) {      // Wenn jetzt low ist, ist auch weiterhin low
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
  Portal.handleClient();

  static int aktVal = 0;
  bool state;
  serialEvent();
  ethernetEvent();

#ifdef CMP_CC1101
  if (cc1101::ccmode == 3) {                // ASK/OOK = 3 (default)
#endif
    while (FiFo.count()>0) {                // Puffer auslesen und an Dekoder uebergeben
      aktVal = FiFo.dequeue();
      state = musterDec.decode(&aktVal);
      if (state) blinkLED = true;           // LED blinken, wenn Meldung dekodiert
      if (FiFo.count()<120) yield();
    }
#ifdef CMP_CC1101
  } else {
    if (wmbus == 0) {
      cc1101::getRxFifo(0);                   // xFSK = 0
    } else {
      mbus_task();
    }
  }
#endif
}



//============================== Write callback =========================================

#define _USE_WRITE_BUFFER

#ifdef _USE_WRITE_BUFFER
  const size_t writeBufferSize = 512;
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
      } else {
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

  while (!serverClient.accept()) {
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
      serverClient = Server.accept();
      serverClient.flush();
      //DBG_PRINTLN("New client: ");
      //DBG_PRINTLN(serverClient.remoteIP());
    } else {
      WiFiClient rejectClient = Server.accept();
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



//================================= Serielle verarbeitung ======================================
void serialEvent()
{
  static uint8_t idx = 0;
  while (MSG_PRINTER.connected() > 0 && MSG_PRINTER.available())
  {
    if (idx == 14) {
      // Short buffer is now full
      MSG_PRINT(F("Command to long: "));
      MSG_PRINTLN(IB_1);
      idx = 0;
      return;
    } else {
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



#endif  // END, defined (ESP32) || defined(ESP8266)
