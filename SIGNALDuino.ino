/*
*   RF_RECEIVER v3.3 for Arduino
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


// Config flags for compiling correct options / boards Define only one
//#define ARDUINO_ATMEGA328P_MINICUL 1
//#define ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101 1;
//#define OTHER_BOARD_WITH_CC1101  1

// #todo: header file für die Boards anlegen
#ifdef OTHER_BOARD_WITH_CC1101
	#define CMP_CC1101     
#endif
#ifdef ARDUINO_ATMEGA328P_MINICUL
	#define CMP_CC1101     
#endif

// Get compatibility with arduino ide and visualmicro
#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
#define ARDUINO_RADINOCC1101
#endif

#ifdef ARDUINO_RADINOCC1101
	#define CMP_CC1101     
#endif





#define PROGVERS               "3.3.1-RC-X-171218"
#define PROGNAME               "RF_RECEIVER"
#define VERSION_1               0x33
#define VERSION_2               0x1d



#ifdef CMP_CC1101
	#ifdef ARDUINO_RADINOCC1101
		#define PIN_LED               13
		#define PIN_SEND              9   // gdo0Pin TX out
		#define PIN_RECEIVE				   7
		#define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
		#define PIN_MARK433			  4
		#define SS					  8  
	#elif ARDUINO_ATMEGA328P_MINICUL  // 8Mhz 
		#define PIN_LED               4
		#define PIN_SEND              2   // gdo0Pin TX out
		#define PIN_RECEIVE           3
		#define PIN_MARK433			  A0
	#else 
		#define PIN_LED               9
		#define PIN_SEND              3   // gdo0Pin TX out
	    #define PIN_RECEIVE           2
	#endif
#else
	#define PIN_RECEIVE            2
	#define PIN_LED                13 // Message-LED
	#define PIN_SEND               11
#endif


#define BAUDRATE               57600 // 500000 //57600
#define FIFO_LENGTH			   90 //150
//#define DEBUG				   1

// EEProm Address
// EEProm Address
#define EE_MAGIC_OFFSET      0
#define addr_features        0xff



// Predeclation
void serialEvent();
void cronjob();
int freeRam();
//bool command_available = false;
unsigned long getUptime();
void enDisPrint(bool enDis);
void configSET();
void getFunctions(bool *ms, bool *mu, bool *mc);
void initEEPROM(void);
void changeReceiver();
uint8_t rssiCallback() { return 0; };	// Dummy return if no rssi value can be retrieved from receiver
size_t writeCallback(const uint8_t *buf, uint8_t len = 1);



//Includes
#include <avr/wdt.h>
#include "FastDelegate.h"
#include "output.h"
#include "bitstore.h"
#include "signalDecoder.h"
#include "TimerOne.h"  // Timer for LED Blinking
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
	DBG_PRINT(F("CCInit "));

	cc1101::CCinit();					 // CC1101 init
	hasCC1101 = cc1101::checkCC1101();	 // Check for cc1101
	
	if (hasCC1101)
	{
		DBG_PRINTLN("CC1101 found");
		musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
	} else {
		musterDec.setRSSICallback(&rssiCallback);	// Provide the RSSI Callback		
	}
	#endif 

	pinAsOutput(PIN_SEND);
	DBG_PRINTLN("Starting timerjob");
	delay(50);

	Timer1.initialize(32001); //Interrupt wird jede 32001 Millisekunden ausgeloest
	Timer1.attachInterrupt(cronjob);

	/*MSG_PRINT("MS:"); 	MSG_PRINTLN(musterDec.MSenabled);
	MSG_PRINT("MU:"); 	MSG_PRINTLN(musterDec.MUenabled);
	MSG_PRINT("MC:"); 	MSG_PRINTLN(musterDec.MCenabled);*/
	//cmdstring.reserve(40);

	musterDec.setStreamCallback(&writeCallback);

	if (!hasCC1101 || cc1101::regCheck()) {
		enableReceive();
		DBG_PRINTLN(F("receiver enabled"));
	}
	else {
		DBG_PRINTLN(F("cc1101 is not correctly set. Please do a factory reset via command e"));
	}
	MSG_PRINTER.setTimeout(400);
}


void cronjob() {
	static uint8_t cnt = 0;
	cli();
	const unsigned long  duration = micros() - lastTime;

	Timer1.setPeriod(32001);
	
	if (duration >= maxPulse) { //Auf Maximalwert pruefen.
		int sDuration = maxPulse;
		if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			sDuration = -sDuration;
		}
		FiFo.enqueue(sDuration);
		lastTime = micros();
	 } else if (duration > 10000) {
		Timer1.setPeriod(maxPulse-duration+16);
	 }
	 digitalWrite(PIN_LED, blinkLED);
	 blinkLED = false;

	 sei();
	
	 // Infrequent time uncritical jobs (~ every 2 hours)
	 if (cnt++ == 0)  // if cnt is 0 at start or during rollover
		 getUptime();
}


void loop() {
	static int aktVal=0;
	bool state;
#ifdef __AVR_ATmega32U4__	
	serialEvent();
#endif
	wdt_reset();
	while (FiFo.count()>0 ) { //Puffer auslesen und an Dekoder uebergeben
		aktVal=FiFo.dequeue();
		state = musterDec.decode(&aktVal); 
		if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
	}

 }






//============================== Write callback =========================================
size_t writeCallback(const uint8_t *buf, uint8_t len = 1)
{
	while (!MSG_PRINTER.availableForWrite() )
		yield();
	//DBG_PRINTLN("Called writeCallback");

	//MSG_PRINT(*buf);
	//MSG_WRITE(buf, len);
	MSG_PRINTER.write(buf,len);
	
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
					wdt_reset();
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
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

 }






