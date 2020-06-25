/*
*   RF_RECEIVER v3.4.0 for Arduino
*   Sketch to use an arduino as a receiver/sending device for digital signals
*
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented in the sketch,
*   there is an option to send almost any data over a send raw interface
*   2014-2015  N.Butzek, S.Butzek
*   2016-2018 S.Butzek

*   This software focuses on remote sensors like weather sensors (temperature,
*   humidity Logilink, TCM, Oregon Scientific, ...), remote controlled power switches
*   (Intertechno, TCM, ARCtech, ...) which use encoder chips like PT2262 and
*   EV1527-type and manchester encoder to send information in the 433MHz Band.
*   But the sketch will also work for infrared or other medias. Even other frequencys
*   can be used
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


// See config flags in compile_config.h:
#include "compile_config.h"


#define PROGVERS               "3.4.0-dev_20200625"
#define VERSION_1               0x33
#define VERSION_2               0x1d

#if !defined(__AVR__) && defined(MAPLE_Mini)

#define PROGNAME               " SIGNALduino_STM "

#ifdef MAPLE_Mini
	#define BAUDRATE               115200
	#define FIFO_LENGTH            170
	#define defSelRadio 1                           // variant -> Circuit board for 4 cc110x - standard value 1 = B
	const uint8_t pinReceive[] = {11, 18, 16, 14};  // variant -> Circuit board for 4 cc110x
	uint8_t radionr = defSelRadio;                  // variant -> Circuit board for 4 cc110x
	uint8_t radio_bank[4];                          // variant -> Circuit board for 4 cc110x
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



//Includes
//#include <avr/wdt.h>
#include "FastDelegate.h"
#include "output.h"
#include "bitstore.h"
#include "signalDecoder.h"

#ifdef MAPLE_Mini
	#include <malloc.h>
	extern char _estack;
	extern char _Min_Stack_Size;
	static char *ramend = &_estack;
	static char *minSP = (char*)(ramend - &_Min_Stack_Size);
	extern "C" char *sbrk(int i);
#endif

#include "commands.h"
#include "functions.h"
#include "send.h"
#include "SimpleFIFO.h"
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
SignalDetectorClass musterDec;


#include <EEPROM.h>
#include "cc1101.h"

volatile bool blinkLED = false;
//String cmdstring = "";
volatile unsigned long lastTime = micros();
bool hasCC1101 = false;
char IB_1[14]; // Input Buffer one - capture commands





void setup() {
#ifdef MAPLE_Mini
	pinAsOutput(PIN_WIZ_RST);
#endif

	Serial.begin(BAUDRATE);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB
	}

	//delay(2000);
	pinAsInput(PIN_RECEIVE);
	pinAsOutput(PIN_LED);
	// CC1101

	//wdt_reset();

	#ifdef CMP_CC1101
		cc1101::setup();
	#endif
	initEEPROM();
	#ifdef CMP_CC1101
		DBG_PRINT(FPSTR(TXT_CCINIT));
	// for variant Circuit board for 4 cc110x and compatible version with 1 cc110x
	#if defined(DEBUG) && defined (MAPLE_Mini)
		DBG_PRINT(FPSTR(("(misoPin="))); DBG_PRINT((misoPin));
		DBG_PRINT(FPSTR((" mosiPin="))); DBG_PRINT((mosiPin));
		DBG_PRINT(FPSTR((" sckPin="))); DBG_PRINT((sckPin));
		DBG_PRINT(FPSTR((" csPin="))); DBG_PRINT((csPin));
		DBG_PRINTLN(")");
	#endif

		cc1101::CCinit();                   // CC1101 init
		hasCC1101 = cc1101::checkCC1101();  // Check for cc1101



		if (hasCC1101)
		{
			//DBG_PRINTLN("CC1101 found");
			DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(FPSTR(TXT_FOUND));
			musterDec.setRSSICallback(&cc1101::getRSSI);          // Provide the RSSI Callback
		} else {
			musterDec.setRSSICallback(&rssiCallback);             // Provide the RSSI Callback
		}
	#endif

	pinAsOutput(PIN_SEND);
	DBG_PRINTLN(F("Starting timerjob"));
	delay(50);

#if defined(ARDUINO) && ARDUINO <= 100                            // to compile with PlatformIO
	#ifdef MAPLE_Mini
		TIM_TypeDef *Instance = TIM1;
		HardwareTimer *MyTim = new HardwareTimer(Instance);
		MyTim->setMode(2, TIMER_OUTPUT_COMPARE);
		MyTim->setOverflow(31*1000, MICROSEC_FORMAT);
		MyTim->attachInterrupt(cronjob);
		MyTim->resume();
	#endif
#endif

	/*MSG_PRINT("MS:"); 	MSG_PRINTLN(musterDec.MSenabled);
	MSG_PRINT("MU:"); 	MSG_PRINTLN(musterDec.MUenabled);
	MSG_PRINT("MC:"); 	MSG_PRINTLN(musterDec.MCenabled);*/
	//cmdstring.reserve(40);

	musterDec.setStreamCallback(&writeCallback);


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
}



#ifdef MAPLE_Mini /* MR neeed ??? */
	#if ARDUINO < 190
		void cronjob(HardwareTimer*) {
	#else
		void cronjob() {
	#endif
	noInterrupts();
#else
	void cronjob() {
	cli();
#endif

static uint8_t cnt = 0;
const unsigned long  duration = micros() - lastTime;

/* MR Timer1 failed
	Timer1.setPeriod(32001);
*/
	if (duration >= maxPulse) { //Auf Maximalwert pruefen.
		int sDuration = maxPulse;
		if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			sDuration = -sDuration;
		}
		FiFo.enqueue(sDuration);
		lastTime = micros();
	} 
/* MR Timer1 failed
	 else if (duration > 10000) {
		Timer1.setPeriod(maxPulse-duration+16);
	 }
*/

#ifdef PIN_LED_INVERSE
	digitalWrite(PIN_LED, !blinkLED);
#else
	digitalWrite(PIN_LED, blinkLED);
#endif
blinkLED = false;

#ifdef MAPLE_Mini
	interrupts();
#endif

	// Infrequent time uncritical jobs (~ every 2 hours)
	if (cnt++ == 0)  // if cnt is 0 at start or during rollover
		getUptime();
}


	/*
		* note use now !
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

	uint8_t tmpBank;
	uint16_t bankoff;

	#ifdef MAPLE_WATCHDOG
		IWatchdog.reload();
	#elif WATCHDOG
		wdt_reset();
	#endif

	/*
		* note use now !
		* these are preparations if the project can be expanded to 4 cc110x

	for (radionr = 0; radionr < 4; radionr++) {
		if (radio_bank[radionr] > 9) {
			continue;
		}
		tmpBank = radio_bank[radionr];
		bankoff = getBankOffset(tmpBank);

		//wdt_reset();
	*/
		while (FiFo.count()>0 ) { //Puffer auslesen und an Dekoder uebergeben
			aktVal=FiFo.dequeue();
			state = musterDec.decode(&aktVal);
			if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
		}

	/*
	}
	*/
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
			MSG_PRINT("Command to long: ");
			MSG_PRINTLN(IB_1);
			idx = 0;
			return;
		}
		else {
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

/* MR not in STM ???
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

 }
*/

#endif
