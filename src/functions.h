#pragma once


#ifndef _FUNCTIONS_h
  #define _FUNCTIONS_h

  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
  //  #include "WProgram.h"
  #endif

  #include "compile_config.h"
  #include <EEPROM.h>
  #include "output.h"
  #include "SimpleFIFO.h"
  #include "cc1101.h"

  extern volatile unsigned long lastTime;
  extern SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
  extern SignalDetectorClass musterDec;
  extern void DBG_PRINTtoHEX(uint8_t b);
  extern bool AfcEnabled;
  extern bool hasCC1101;
  #ifdef CMP_CC1101
    extern int8_t cc1101::freqOffAcc;
    extern float cc1101::freqErrAvg;
    #if defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB)
      extern bool wmbus;
      extern bool wmbus_t;
    #endif 
  #endif 

  #define pulseMin  90

  #if !defined(ESP8266) && !defined(ESP32)
    #define IRAM_ATTR 
  #else
    #ifdef ESP8266
      extern os_timer_t cronTimer;
    #endif
    #ifdef ESP32
      extern esp_timer_handle_t cronTimer;
    #endif
  #endif



//========================= Pulseauswertung ================================================
void IRAM_ATTR handleInterrupt() {
  #ifdef ARDUINO_MAPLEMINI_F103CB
    noInterrupts();
  #elif ESP32
    portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mutex);
  #else
    cli();
  #endif
  const unsigned long Time = micros();
  const unsigned long  duration = Time - lastTime;
  lastTime = Time;
  if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
    int sDuration;
    if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
      sDuration = int(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
    }
    else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (isHigh(PIN_RECEIVE)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
      sDuration = -sDuration;
    }
    FiFo.enqueue(sDuration);
  } // else => trash
  #ifdef ARDUINO_MAPLEMINI_F103CB
    interrupts();
  #elif ESP32
    portEXIT_CRITICAL(&mutex);
  #else
    sei();
  #endif
}


void enableReceive() {
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), handleInterrupt, CHANGE);
  #ifdef CMP_CC1101
    if (hasCC1101) cc1101::setReceiveMode();
  #endif
}


void disableReceive() {
  detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));

  #ifdef CMP_CC1101
    if (hasCC1101) cc1101::setIdleMode();
  #endif
}


//================================= EEProm commands ======================================
#if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB))
  void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red, int8_t afc, int8_t wmbus, int8_t wmbus_t) {
#else
  void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red, int8_t afc) {
#endif
  #ifdef CMP_CC1101
    if (afc == 0) { // reset AFC
      cc1101::freqOffAcc = 0;
      cc1101::freqErrAvg = 0;
      cc1101::writeReg(static_cast<uint8_t>(0x0C), static_cast<uint8_t>(afc) ); // reset 0x0C: FSCTRL0 – Frequency Synthesizer Control
    }
    #if defined (ESP8266) || defined (ESP32)
      if (wmbus == 1) { // WMBus
        mbus_init(wmbus_t + 1); // WMBus mode S or T
      }
    #endif
  #endif
  mu = mu << 1;
  mc = mc << 2;
  red = red << 3;
  afc = afc << 4;
  #if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB))
    wmbus = wmbus << 5;
    wmbus_t = wmbus_t << 6;
    int8_t dat = ms | mu | mc | red | afc | wmbus | wmbus_t;
  #else
    int8_t dat = ms | mu | mc | red | afc;
  #endif
  EEPROM.write(addr_features, dat);
  #if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
  #endif
}

#if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB))
  void getFunctions(bool *ms, bool *mu, bool *mc, bool *red, bool *afc, bool *wmbus, bool *wmbus_t) {
#else
  void getFunctions(bool *ms, bool *mu, bool *mc, bool *red, bool *afc) {
#endif
  int8_t dat = EEPROM.read(addr_features);

  *ms = bool(dat &(1 << 0));
  *mu = bool(dat &(1 << 1));
  *mc = bool(dat &(1 << 2));
  *red = bool(dat &(1 << 3));
  *afc = bool(dat &(1 << 4));
  #if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB))
    *wmbus = bool(dat &(1 << 5));
    *wmbus_t = bool(dat &(1 << 6));
  #endif
}

void dumpEEPROM() {
  #ifdef DEBUG
    DBG_PRINT(F("dump, ")); DBG_PRINT(FPSTR(TXT_EEPROM)); DBG_PRINTLN(F(":"));
    for (uint8_t i = EE_MAGIC_OFFSET; i < 56+ EE_MAGIC_OFFSET; i++) {
      DBG_PRINTtoHEX(EEPROM.read(i));
      DBG_PRINT(FPSTR(TXT_BLANK));
        if ((i & 0x0F) == 0x0F)
        DBG_PRINTLN("");
    }
    DBG_PRINTLN("");
  #endif
}

void initEEPROM(void) { 
  #if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(512); //Max bytes of eeprom to use
  #endif
  if (EEPROM.read(EE_MAGIC_OFFSET) == VERSION_1 && EEPROM.read(EE_MAGIC_OFFSET + 1) == VERSION_2) {
    DBG_PRINT(F("Reading values from ")); DBG_PRINT(FPSTR(TXT_EEPROM)); DBG_PRINT(FPSTR(TXT_DOT)); DBG_PRINT(FPSTR(TXT_DOT));
  } else {
    #if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB))
      storeFunctions(1, 1, 1, 1, 0, 0, 0); // Init EEPROM with all flags enabled, AFC, WMBus and WMBus_T disabled
    #else
      storeFunctions(1, 1, 1, 1, 0); // Init EEPROM with all flags enabled, AFC disabled
    #endif
    MSG_PRINTLN(F("Init eeprom to defaults after clear eeprom"));
    EEPROM.write(EE_MAGIC_OFFSET, VERSION_1);
    EEPROM.write(EE_MAGIC_OFFSET + 1, VERSION_2);
    #ifdef CMP_CC1101
      cc1101::ccFactoryReset();
    #endif

    #if defined(ESP8266) || defined(ESP32)
      EEPROM.commit();
    #endif
  }
  #if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32) || defined (ARDUINO_MAPLEMINI_F103CB))
    getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled, &musterDec.MredEnabled, &AfcEnabled, &wmbus, &wmbus_t);
  #else
    getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled, &musterDec.MredEnabled, &AfcEnabled);
  #endif
  DBG_PRINTLN(F("done"));
  dumpEEPROM();
}


//================================= getUptime command Receiver ======================================
inline unsigned long getUptime() {
  unsigned long now = millis();
  static uint16_t times_rolled = 0;
  static unsigned long last = 0;
  // If this run is less than the last the counter rolled
  //unsigned long seconds = now / 1000;
  if (now < last) {
    times_rolled++;
  }
  last = now;
  return (0xFFFFFFFF / 1000) * times_rolled + (now / 1000);
}

#if defined (ESP8266) || defined (ESP32)
// This function will read IPAddress (4 byte) from the eeprom at the specified address to address + 3.
IPAddress EEPROM_read_ipaddress(int address) {
  IPAddress ip;
  for (uint8_t x = 0; x < 4; x++) {
    ip[x] = EEPROM.read(address + x);
  }
  #ifdef DEBUG
    Serial.print(F("EEPROM read IPAddress at address: "));
    Serial.print(address);
    Serial.print(F(" - "));
    Serial.println(ip);
  #endif
  return ip;
}

// This function will write IPAddress (4 byte) to the eeprom at the specified address to address + 3.
void EEPROM_write_ipaddress(int address, IPAddress ip) {
  #ifdef DEBUG
    Serial.print(F("EEPROM write IPAddress at address: "));
    Serial.print(address);
    Serial.print(F(" - "));
    Serial.println(ip);
  #endif
  for (uint8_t x = 0; x < 4; x++) {
    EEPROM.write(address + x, ip[x]);
  }
  EEPROM.commit();
}
#endif
#endif // endif _FUNCTIONS_h
