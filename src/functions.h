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
  extern bool hasCC1101;

  #define pulseMin  90


  #if !defined(ESP8266) && !defined(ESP32)
    #define ICACHE_RAM_ATTR 
  #else
    #ifdef ESP8266
      extern os_timer_t cronTimer;
    #endif
    #ifdef ESP32
      extern esp_timer_handle_t cronTimer;
    #endif
  #endif


//========================= Pulseauswertung ================================================
void ICACHE_RAM_ATTR handleInterrupt() {
  #ifdef ARDUINO_MAPLEMINI_F103CB
    noInterrupts();
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
  #else
    sei();
  #endif
}


void enableReceive() {
  // ToDo MR if(cc1101::ccmode == 0) ???
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), handleInterrupt, CHANGE);
  #ifdef CMP_CC1101
    if (hasCC1101) cc1101::setReceiveMode();
  #endif
}


void disableReceive() {
  // ToDo MR if(cc1101::ccmode == 0) ???
  detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));

  #ifdef CMP_CC1101
    if (hasCC1101) cc1101::setIdleMode();
  #endif
  FiFo.flush();
}


//================================= EEProm commands ======================================
void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red) {
  mu = mu << 1;
  mc = mc << 2;
  red = red << 3;

  int8_t dat = ms | mu | mc | red;
  EEPROM.write(addr_features, dat);
  #if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
  #endif
}

void getFunctions(bool *ms, bool *mu, bool *mc, bool *red) {
  int8_t dat = EEPROM.read(addr_features);

  *ms = bool(dat &(1 << 0));
  *mu = bool(dat &(1 << 1));
  *mc = bool(dat &(1 << 2));
  *red = bool(dat &(1 << 3));
}

void dumpEEPROM() {
  #ifdef DEBUG
    DBG_PRINT(F("dump, ")); DBG_PRINT(FPSTR(TXT_EEPROM)); DBG_PRINTLN(F(":"));
    char b[4];
    for (uint8_t i = EE_MAGIC_OFFSET; i < 56+ EE_MAGIC_OFFSET; i++) {
      sprintf(b, "%02x ", EEPROM.read(i));
      DBG_PRINT(b);
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
    storeFunctions(1, 1, 1, 1);    // Init EEPROM with all flags enabled
    //hier fehlt evtl ein getFunctions()
    MSG_PRINTLN(F("Init eeprom to defaults after flash"));
    EEPROM.write(EE_MAGIC_OFFSET, VERSION_1);
    EEPROM.write(EE_MAGIC_OFFSET + 1, VERSION_2);
    #ifdef CMP_CC1101
      cc1101::ccFactoryReset();
    #endif

    #if defined(ESP8266) || defined(ESP32)
      EEPROM.commit();
    #endif
  }
  getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled, &musterDec.MredEnabled);
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


#endif // endif _FUNCTIONS_h
