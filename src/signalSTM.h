#pragma once

#ifdef ARDUINO_ARCH_STM32
/*

*   notes different boards:
*   https://github.com/firmata/arduino/blob/master/Boards.h
*   https://danieleff.github.io/STM32GENERIC/build_macros/

*/

#include "compile_config.h"

/*

*   developer:
*   2020 S.Butzek, HomeAutoUser, elektron-bbs

*   Integration for compatibility with a similar project by Ralf9

*   tested hardware:
*    - Maple Mini STM32F103CBT6

*/

#define PROGNAME               " SIGNALSTM32 "
#define VERSION_1               0x33
#define VERSION_2               0x1d

#define BAUDRATE               115200
#define FIFO_LENGTH            170

#ifdef ARDUINO_MAPLEMINI_F103CB
  uint8_t radionr = 1;                            // variant -> Circuit board for four connected cc110x devices - standard value 1 = B (defSelRadio)
  uint8_t radio_bank[4];                          // variant -> Circuit board for four connected cc110x devices
#endif


// EEProm Address
#define EE_MAGIC_OFFSET      0
#define addr_features        0xff


// Predeclation
void serialEvent();
void cronjob();
int freeRam();
void configSET();
uint8_t rssiCallback() { return 0; };	// Dummy return if no rssi value can be retrieved from receiver
size_t writeCallback(const uint8_t *buf, uint8_t len = 1);


extern char _estack;
extern char _Min_Stack_Size;
static char *ramend = &_estack;
static char *minSP = (char*)(ramend - &_Min_Stack_Size);
extern "C" char *sbrk(int i);

//Includes
#ifdef WATCHDOG_STM32
  #include <IWatchdog.h>
  bool watchRes = false;
#endif

#include "output.h"
#include "bitstore.h"
#include "signalDecoder.h"
#include <malloc.h>
#include "commands.h"
#include "functions.h"
#include "send.h"
#include "SimpleFIFO.h"
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
SignalDetectorClass musterDec;
#include <EEPROM.h>
#ifdef CMP_CC1101
  #include "cc1101.h"
#endif
volatile bool blinkLED = false;
volatile unsigned long lastTime = micros();
bool hasCC1101 = false;
bool AfcEnabled = true;
char IB_1[14]; // Input Buffer one - capture commands

HardwareTimer *Timer1 = new HardwareTimer(TIM1);



void setup() {
  pinAsOutput(PIN_WIZ_RST);

  Serial.begin(BAUDRATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  #ifdef WATCHDOG_STM32
    if (IWatchdog.isReset(true)) {
      MSG_PRINTLN(F("Watchdog caused a reset"));
      watchRes = true;
    } else {
      watchRes = false;
    }
    IWatchdog.begin(20000000);	// Init the watchdog timer with 20 seconds timeout
    if (IWatchdog.isEnabled()) {
      MSG_PRINTLN(F("Watchdog enabled"));
    }
  #endif

  pinAsInput(PIN_RECEIVE);
  pinAsOutput(PIN_LED);
  // CC1101

  #ifdef CMP_CC1101
    cc1101::setup(); // set pins input/output, init SPI
  #endif

  initEEPROM();

  #ifdef CMP_CC1101
    cc1101::CCinit();                   // CC1101 init
    hasCC1101 = cc1101::checkCC1101();  // Check for cc1101
    musterDec.hasCC1101 = hasCC1101;
    if (hasCC1101)
    {
      DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(FPSTR(TXT_FOUND));
      musterDec.setRSSICallback(&cc1101::getRSSI);          // Provide the RSSI Callback
    } else {
      musterDec.setRSSICallback(&rssiCallback);             // Provide the RSSI Callback
    }

    }
  #endif

  pinAsOutput(PIN_SEND);
  DBG_PRINTLN(F("Starting timerjob"));
  delay(50);

  // https://github.com/stm32duino/wiki/wiki/HardwareTimer-library
  Timer1->setMode(2, TIMER_OUTPUT_COMPARE);
  Timer1->setOverflow(32001, MICROSEC_FORMAT);
  Timer1->attachInterrupt(cronjob);
  Timer1->resume();

  /*MSG_PRINT("MS:"); 	MSG_PRINTLN(musterDec.MSenabled);
  MSG_PRINT("MU:"); 	MSG_PRINTLN(musterDec.MUenabled);
  MSG_PRINT("MC:"); 	MSG_PRINTLN(musterDec.MCenabled);*/
  //cmdstring.reserve(40);

  musterDec.setStreamCallback(&writeCallback);

#ifdef CMP_CC1101
  if (!hasCC1101 || cc1101::regCheck()) {
#endif
    enableReceive();
    DBG_PRINTLN(FPSTR(TXT_RECENA));
#ifdef CMP_CC1101
  } else {
    DBG_PRINT(FPSTR(TXT_CC1101));
    DBG_PRINT(FPSTR(TXT_DOFRESET));
    DBG_PRINTLN(FPSTR(TXT_COMMAND));
  }
#endif
  MSG_PRINTER.setTimeout(400);
}



void cronjob() {
  noInterrupts();
  static uint8_t cnt = 0;
  const unsigned long  duration = micros() - lastTime;
  long timerTime = maxPulse - duration;

  if (timerTime < 1000)
    timerTime=1000;

  // Timer1->pause();         // ToDo: Timer stopt, why ??? -> no blinkLED action
  Timer1->setOverflow(timerTime, MICROSEC_FORMAT);
  if (duration > maxPulse) {  // Auf Maximalwert pruefen.
    int sDuration = maxPulse;
    if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
      sDuration = -sDuration;
    }
    FiFo.enqueue(sDuration);
    lastTime = micros();
  }

  #ifdef PIN_LED_INVERSE
    digitalWrite(PIN_LED, !blinkLED);
  #else
    digitalWrite(PIN_LED, blinkLED);
  #endif

  blinkLED = false;
  interrupts();

  // Infrequent time uncritical jobs (~ every 2 hours)
  if (cnt++ == 0)  // if cnt is 0 at start or during rollover
    getUptime();
}



	/*
		* not used now !
		* these are preparations if the project can be expanded to 4 cc110x

uint16_t getBankOffset(uint8_t tmpBank) {
	uint16_t bankOffs;
	if (tmpBank == 0) {
		bankOffs = 0;
	}
	else {
	bankOffs = 0x100 + ((tmpBank - 1) * 0x40);
	}
	return bankOffs;
}

	*/



void loop() {
  static int aktVal=0;
  bool state;

  #ifdef WATCHDOG_STM32
    IWatchdog.reload();
  #endif

#ifdef CMP_CC1101
  if (cc1101::ccmode == 3) {                // ASK/OOK = 3 (default)
#endif
    while (FiFo.count()>0 ) {               // Puffer auslesen und an Dekoder uebergeben
      aktVal=FiFo.dequeue();
      state = musterDec.decode(&aktVal);
      if (state) blinkLED=true;             // LED blinken, wenn Meldung dekodiert
    }
#ifdef CMP_CC1101
  } else {
    cc1101::getRxFifo(0);                   // xFSK = 0
  }
#endif
}



//============================== Write callback =========================================
size_t writeCallback(const uint8_t *buf, uint8_t len)
{
  while (!MSG_PRINTER.availableForWrite() )
  yield();
  //DBG_PRINTLN("Called writeCallback");

  //MSG_PRINT(*buf);
  //MSG_WRITE(buf, len);
  return MSG_PRINTER.write(buf,len);

  //serverClient.write("test");
}




//================================= Serielle verarbeitung ======================================
void serialEvent()
{
  static uint8_t idx = 0;
  while (MSG_PRINTER.available())
  {
    if (idx == 14) {
      // Short buffer is now full
      MSG_PRINT(F("Command to long: "));
      MSG_PRINTLN(IB_1);
      idx = 0;
      return;
    } else {
      IB_1[idx] = (char)MSG_PRINTER.read();
      switch (IB_1[idx])
      {
        case '\n':
        case '\r':
        case '\0':
        case '#':
          //wdt_reset();
          commands::HandleShortCommand();  // Short command received and can be processed now
          idx = 0;
          return; //Exit function
        case ';':
          DBG_PRINT("send cmd detected ");
          DBG_PRINTLN(idx);
          send_cmd();
          idx =  0; // increments to 1
          return; //Exit function
      }
      idx++;
    }
  }
}



int freeRam () {
  char *heapend = (char*)sbrk(0);
  char * stack_ptr = (char*)__get_MSP();
  struct mallinfo mi = mallinfo();
  return (((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks);
}



#endif  // END, ARDUINO_ARCH_STM32
