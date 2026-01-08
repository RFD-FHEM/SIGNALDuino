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
#define EEPROM_RESERVED_SIZE   512
#define MAX_SRV_CLIENTS        2


#include "compile_config.h"

void serialEvent();
void IRAM_ATTR cronjob(void *pArg);
int freeRam();
inline void ethernetEvent();
void telnetClientEvent();
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
  #include "driver/gpio.h"
#endif

#include <FS.h>
#include <EEPROM.h>
#include <DNSServer.h>             // Local DNS Server used for redirecting all requests to the configuration portal

#include <Stream.h>

class MultiClientPrinter : public Stream {
public:
  size_t write(uint8_t c) override {
    return writeCallback(&c, 1);
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t n = 0;
    while (size > 0) {
      uint8_t chunk = (size > 255) ? 255 : (uint8_t)size;
      writeCallback(buffer + n, chunk);
      n += chunk;
      size -= chunk;
    }
    return n;
  }

  // Stream interface stubs
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
};

extern MultiClientPrinter _MultiClientPrinterInstance;
extern Stream &TelnetPrint;

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
extern WiFiClient serverClient[MAX_SRV_CLIENTS];

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
#define CMD_BUFFER_SIZE 14
char IB_Telnet[MAX_SRV_CLIENTS][CMD_BUFFER_SIZE];
uint8_t IB_Telnet_idx[MAX_SRV_CLIENTS] = {0};
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

#ifdef ESP32
  gpio_install_isr_service(0);
#endif

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


Serial.setTimeout(400);

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
  static uint16_t cnt = 0;         // approximately every 34 minutes
  static unsigned long stuckRxTime = 0; // for stuck RX detection

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

#ifdef CMP_CC1101
  // Stuck RX detection logic
  if (hasCC1101 && cc1101::ccmode == 0) { // Only run if CC1101 is present and in xFSK (mode 0)
    // Read MARCSTATE and PKTSTATUS registers
    uint8_t marcstate = cc1101::readReg(CC1101_MARCSTATE_REV01,CC1101_STATUS) & 0x1F; // mask state bits
    uint8_t pktstatus = cc1101::readReg(CC1101_PKTSTATUS,CC1101_STATUS);

    // Trigger condition: Duration > maxPulse && MARCSTATE == 0x0D (RX Mode) && PKTSTATUS bit 6 set (Carrier Sense active) && PKTSTATUS bit 3 clear (Sync Word NOT detected)
    if (marcstate == 0x0D && (pktstatus & 0x40) && !(pktstatus & 0x08)) {
      if (stuckRxTime == 0) {
        // First detection
        stuckRxTime = millis();
      } else if (millis() - stuckRxTime > 100) {
        // Condition persists for > 100ms (0.1s)
        DBG_PRINTLN(F("Resetting RX State (Calibrate)..."));
        cc1101::cmdStrobe(CC1101_SIDLE);
        delayMicroseconds(100);
        cc1101::cmdStrobe(CC1101_SRX);
        stuckRxTime = 0; // Reset timer
      }
    } else {
      stuckRxTime = 0; // Reset timer if condition is false
    }
  }
#endif

#ifdef PIN_LED_INVERSE
	digitalWrite(PIN_LED, !blinkLED);
#else
	digitalWrite(PIN_LED, blinkLED);
#endif

  blinkLED = false;

  sei();
  // Infrequent time uncritical jobs (~ every 2 hours)
  // Workaround for ESP RSSI issues
  if (cnt++ == 0) { // approximately every 34 minutes
    getUptime();
    musterDec.reset();
    FiFo.flush();
    //cc1101::CCinit(); // Reinitialize CC1101
  }
}

unsigned long Elapsed;

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connected = false;
    Elapsed = millis();
  }

  if ((WiFi.getMode() & WIFI_AP) && connected) {
    if (millis() - elapsed > 30000) {
      WiFi.softAPdisconnect(true);
      WiFi.enableAP(false);
    }
  }


  Portal.handleClient();

  static int aktVal = 0;
  bool state;
  serialEvent();
  telnetClientEvent();
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
  size_t writeBufferCurrent[MAX_SRV_CLIENTS] = { 0 };
  uint8_t writeBuffer[MAX_SRV_CLIENTS][writeBufferSize];
#endif

size_t writeCallback(const uint8_t *buf, uint8_t len)
{
#ifdef _USE_WRITE_BUFFER
  // Broadcast to all clients
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (!serverClient[i] || !serverClient[i].connected())
      continue;

    // Need to use a temporary copy of len and buf for each client's consumption loop
    size_t currentLen = len;
    const uint8_t *currentBuf = buf;

    while (currentLen > 0) {
      size_t copy = (currentLen > writeBufferSize - writeBufferCurrent[i] ? writeBufferSize - writeBufferCurrent[i] : currentLen);
      
      if (copy > 0)
      {
        memcpy(writeBuffer[i] + writeBufferCurrent[i], currentBuf, copy);
        writeBufferCurrent[i] += copy;
      }

      // Check for forced send condition (newline or buffer full)
      bool forceSend = (writeBufferCurrent[i] == writeBufferSize);
      if (copy > 0 && currentBuf[copy - 1] == char(0xA)) {
          forceSend = true;
      }

      if (forceSend)
      {
        size_t byteswritten = serverClient[i].write(writeBuffer[i], writeBufferCurrent[i]);
        
        if (byteswritten < writeBufferCurrent[i]) {
          memmove(writeBuffer[i], writeBuffer[i] + byteswritten, writeBufferCurrent[i] - byteswritten);
          writeBufferCurrent[i] -= byteswritten;
        } else {
          writeBufferCurrent[i] = 0;
        }
      }

      currentLen -= copy;
      currentBuf += copy;
    }
  }

  // The original implementation returned `len` on success. We keep this for compatibility.
  return len; 

#else
  // Non-buffered write (broadcast)
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClient[i] && serverClient[i].connected()) {
      serverClient[i].write(buf, len);
    }
  }
  return len;
#endif
}

inline void ethernetEvent()
{
  // Check if there are any new clients
  if (Server.hasClient()) {
    bool accepted = false;
    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (!serverClient[i] || !serverClient[i].connected()) {
        if (serverClient[i]) serverClient[i].stop();
        serverClient[i] = Server.accept();
        serverClient[i].flush();
        DBG_PRINT(F("New client accepted on slot "));
        DBG_PRINTLN(i);
        //DBG_PRINTLN(serverClient[i].remoteIP());
        accepted = true;
        break;
      }
    }

    if (!accepted) {
      WiFiClient rejectClient = Server.accept();
      rejectClient.stop();
      DBG_PRINT(F("Reject new Client: "));
      DBG_PRINTLN(rejectClient.remoteIP());
    }
  }

  // Check for disconnected clients
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if(serverClient[i] && !serverClient[i].connected())
    {
      DBG_PRINT(F("Client disconnected from slot "));
      DBG_PRINT(i);
      DBG_PRINT(F(": "));
      //DBG_PRINTLN(serverClient[i].remoteIP());
      serverClient[i].stop();
    }
  }
}



//================================= Serielle verarbeitung und Telnet ======================================

void handleStreamInput(Stream &input, char *inputBuffer, uint8_t &idx)
{
  while (input.available())
  {
    if (idx >= CMD_BUFFER_SIZE - 1) { // CMD_BUFFER_SIZE is 14
      // Short buffer is now full
      MSG_PRINT(F("Command to long: "));
      MSG_PRINTLN(inputBuffer);
      idx = 0;
      return;
    } else {
      inputBuffer[idx] = (char)input.read();
      // DBG_PRINTLN(idx);
      // DBG_PRINT(inputBuffer[idx]);

      switch (inputBuffer[idx])
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
            // Copy command to global IB_1 buffer before calling shared command handlers
            memcpy(IB_1, inputBuffer, idx);
            IB_1[idx] = '\0';
            commands::HandleShortCommand();  // Short command received and can be processed now
          }
          idx = 0;
          return; //Exit function
        case ';':
          DBG_PRINT(F("send cmd detected "));
          // DBG_PRINTLN(idx);
          
          // Copy command to global IB_1 buffer before calling shared command handlers
          memcpy(IB_1, inputBuffer, idx + 1);
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

void telnetClientEvent()
{
  // Handle commands from all connected Telnet clients
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClient[i] && serverClient[i].connected() && serverClient[i].available()) {
      handleStreamInput(serverClient[i], IB_Telnet[i], IB_Telnet_idx[i]);
    }
  }
}

void serialEvent()
{
  // Handle commands from the Serial port
  static uint8_t serial_idx = 0;
  handleStreamInput(Serial, IB_1, serial_idx);
}



int freeRam() {
  #ifdef ESP32
    return ESP.getFreeHeap();
  #elif defined(ESP8266)
    return system_get_free_heap_size();
  #endif
}



#endif  // END, defined (ESP32) || defined(ESP8266)
