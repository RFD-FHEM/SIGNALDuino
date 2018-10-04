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





#define PROGVERS               "3.3.1-RC8"
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
#define FIFO_LENGTH			   50 //150
//#define DEBUG				   1

// EEProm Address
#define EE_MAGIC_OFFSET      0
#define addr_features        0xff



// Predeclation
void serialEvent();
void cronjob();
int freeRam();
void HandleLongCommand();
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

#include "SimpleFIFO.h"
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
SignalDetectorClass musterDec;


#include <EEPROM.h>
#include "cc1101.h"

volatile bool blinkLED = false;
//String cmdstring = "";
volatile unsigned long lastTime = micros();
bool hasCC1101 = false;
char IB_1[10]; // Input Buffer one - capture commands






void setup() {
	bool resetflag;
	Serial.begin(BAUDRATE);
	if (MCUSR & (1 << WDRF)) {
		resetflag = true;
	}
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB
		wdt_reset();
	}
	if (resetflag)
		DBG_PRINTLN("Watchdog caused a reset");
	/*
	if (MCUSR & (1 << BORF)) {
		DBG_PRINTLN("brownout caused a reset");
	}
	if (MCUSR & (1 << EXTRF)) {
		DBG_PRINTLN("external reset occured");
	}
	if (MCUSR & (1 << PORF)) {
		DBG_PRINTLN("power on reset occured");
	}
	*/
	wdt_reset();

	wdt_enable(WDTO_2S);  	// Enable Watchdog

	//delay(2000);
	pinAsInput(PIN_RECEIVE);
	pinAsOutput(PIN_LED);
	// CC1101
	
	wdt_reset();

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

//================================= RAW Send ======================================
void send_raw(const char *startpos,const char *endpos,const int16_t *buckets)
{
	uint8_t index=0;
	unsigned long stoptime=micros();
	bool isLow;
	uint16_t dur;
	for (char *i = (char*)startpos;i<=endpos;i++ )
	{
		//DBG_PRINT(cmdstring.substring(i,i+1));
		index = *i - '0';
		//DBG_PRINT(index);
		isLow=buckets[index] >> 15;
		dur = abs(buckets[index]); 		//isLow ? dur = abs(buckets[index]) : dur = abs(buckets[index]);

		while (stoptime > micros()){
			;
		}
		isLow ? digitalLow(PIN_SEND): digitalHigh(PIN_SEND);
		stoptime+=dur;
	}
	while (stoptime > micros()){
		;
	}
	//DBG_PRINTLN("");

}
//SM;R=2;C=400;D=AFAFAF;




void send_mc(const char *startpos,const char *endpos, const int16_t clock )
{
	int8_t b;
	uint8_t bit;

	unsigned long stoptime =micros();
	for (char *i = (char*)startpos; i <= endpos; i++) {
		b = ((byte)*i) - (*i <= '9' ? 0x30 : 0x37);

		for (bit = 0x8; bit>0; bit >>= 1) {
			for (byte i = 0; i <= 1; i++) {
				if ((i == 0 ? (b & bit) : !(b & bit)))
					digitalLow(PIN_SEND);
				else
					digitalHigh(PIN_SEND);
				
				stoptime += clock;
				while (stoptime > micros())
					yield();
			}
			
		}
		
	}
	// MSG_PRINTLN("");
}


/*
bool split_cmdpart(int16_t *startpos, String *msg_part)
{
	int16_t endpos=0;
	//startpos=cmdstring.indexOf(";",startpos);   			 // search first  ";"
	endpos=cmdstring.indexOf(";",*startpos);     			 // search next   ";"

	if (endpos ==-1 || *startpos== -1) return false;
	*msg_part = cmdstring.substring(*startpos,endpos);
	*startpos=endpos+1;    // Set startpos to endpos to extract next part
	return true;
}
*/


// SC;R=4;SM;C=400;D=AFFFFFFFFE;SR;P0=-2500;P1=400;D=010;SM;D=AB6180;SR;D=101;
// SC;R=4;SM;C=400;D=FFFFFFFF;SR;P0=-400;P1=400;D=101;SM;D=AB6180;SR;D=101;
// SR;R=3;P0=1230;P1=-3120;P2=-400;P3=-900;D=030301010101010202020202020101010102020202010101010202010120202;
// SM;C=400;D=AAAAFFFF;
// SR;R=10;P0=-2000;P1=-1000;P2=500;P3=-6000;D=2020202021212020202121212021202021202121212023;

struct s_sendcmd {
	int16_t sendclock=0;
	uint8_t type;
	char * datastart;
	char * dataend;
	int16_t buckets[6];
	uint8_t repeats=1;
} ;

void send_cmd()
{
	#define combined 0
	#define manchester 1
	#define raw 2
	disableReceive();

	//Wait until command is in Buffer or Buffer is full
	/*while (MSG_PRINTER.available())
	{
		IB_1[idx] = (char)MSG_PRINTER.read();

		wdt_reset();
		switch (IB_1[idx])
		{
			case '\n':
			case '\r':
			case '\0':
			case '#':
				break;
		
		}
	}
	*/

	uint8_t repeats=1;  // Default is always one iteration so repeat is 1 if not set
	int16_t start_pos=0;
	uint8_t counter=0;
	bool extraDelay = true;

	s_sendcmd command[5];

	uint8_t ccParamAnz = 0;   // Anzahl der per F= uebergebenen cc1101 Register
	uint8_t ccReg[4];
	uint8_t val;


	uint8_t cmdNo=255;

	char *bptr = IB_1;
	
	char buf[128]; // Second Buffer 64 Bytes
	char *msg_beginptr=IB_1;
	char *msg_endptr=buf;
	do 
	{

		//if (cmdNo == 255)  msg_part = IB_1;
		

		DBG_PRINTLN(msg_beginptr);
		if (msg_beginptr[0] == 'S')
		{
			if (msg_beginptr[1] == 'C')  // send combined information flag
			{
				cmdNo++;
				command[cmdNo].type = combined;
				extraDelay = false;
			}
			else if (msg_beginptr[1] == 'M') // send manchester
			{
				cmdNo++;
				command[cmdNo].type=manchester;
				DBG_PRINTLN("Adding manchester");

			}
			else if (msg_beginptr[1] == 'R') // send raw
			{
				cmdNo++;
				command[cmdNo].type=raw;
				DBG_PRINTLN("Adding raw");
				extraDelay = false;
			}
			if (cmdNo == 0) {
				msg_endptr = buf; // rearrange to beginning of buf
			}
		}
		else if (msg_beginptr[0] == 'P' && msg_beginptr[2] == '=') // Do some basic detection if data matches what we expect
		{
			counter = msg_beginptr[1] - '0'; // Convert to dec value
			command[cmdNo].buckets[counter]= strtol(&msg_beginptr[3], &msg_endptr, 10);
			DBG_PRINTLN("Adding bucket");

		} else if(msg_beginptr[0] == 'R' && msg_beginptr[1] == '=') {
			command[cmdNo].repeats = strtoul(&msg_beginptr[2], &msg_endptr, 10);  
			DBG_PRINT("Adding repeats: "); DBG_PRINTLN(command[cmdNo].repeats);
		} else if (msg_beginptr[0] == 'D' && msg_beginptr[1] == '=') {
			command[cmdNo].datastart = msg_beginptr+2;
			command[cmdNo].dataend = msg_endptr = strchr(msg_beginptr, (int)';');
			DBG_PRINT("locating data start:");
			DBG_PRINT(command[cmdNo].datastart);
			DBG_PRINT(" end:");
			DBG_PRINTLN(command[cmdNo].dataend);
		} else if(msg_beginptr[0] == 'C' && msg_beginptr[1] == '=')
		{
			//sendclock = msg_part.substring(2).toInt();
			command[cmdNo].sendclock = strtoul(&msg_beginptr[2], &msg_endptr, 10);
			DBG_PRINTLN("adding sendclock");
		} else if(msg_beginptr[0] == 'F' && msg_beginptr[1] == '=')
		{
			ccParamAnz = strlen(msg_beginptr) / 2 - 1;
			
			if (ccParamAnz > 0 && ccParamAnz <= 5 && hasCC1101) {
				uint8_t hex;
				DBG_PRINTLN("write new ccreg  ");
				for (uint8_t i=0;i<ccParamAnz;i++)
				{
					ccReg[i] = cc1101::readReg(0x0d + i, 0x80);    // alte Registerwerte merken
					hex = (uint8_t)msg_beginptr[2 + i*2];
					val = cc1101::hex2int(hex) * 16;
					hex = (uint8_t)msg_beginptr[3 + i*2];
					val = cc1101::hex2int(hex) + val;
					cc1101::writeReg(0x0d + i, val);            // neue Registerwerte schreiben
					cc1101::printHex2(val);
					msg_endptr = msg_beginptr + (3 + i * 2) + 1;
				}
				DBG_PRINTLN("");
			}
		}
		if (msg_endptr == msg_beginptr)
		{
			// EOM detected
			break; // break the loop now

		} else {
			//msg_part = strtok(NULL, ";");
			msg_beginptr = msg_endptr;
			msg_endptr = msg_beginptr + Serial.readBytesUntil((const char)";", msg_endptr, buf+128-msg_endptr);
		}
	} while (msg_beginptr != NULL);

	#ifdef CMP_CC1101
	if (hasCC1101) cc1101::setTransmitMode();	
	#endif
	MSG_PRINT(IB_1); // echo command


	if (command[0].type == combined && command[0].repeats > 0) {
		repeats = command[0].repeats;
	}
	for (uint8_t i = 0;i < repeats; i++)
	{
		DBG_PRINT("repeat "); DBG_PRINT(i); DBG_PRINT("/"); DBG_PRINT(repeats);
		
		for (uint8_t c = 0;c <= cmdNo ;c++)
		{
			DBG_PRINT(" cmd "); DBG_PRINT(c); DBG_PRINT("/"); DBG_PRINT(cmdNo);
			DBG_PRINT(" reps "); DBG_PRINT(command[c].repeats);

			if (command[c].type == raw) { for (uint8_t rep = 0; rep < command[c].repeats; rep++) send_raw(command[c].datastart, command[c].dataend, command[c].buckets); }
			else if (command[c].type == manchester) { for (uint8_t rep = 0; rep < command[c].repeats; rep++)send_mc(command[c].datastart, command[c].dataend, command[c].sendclock); }
			digitalLow(PIN_SEND);
			DBG_PRINT(".");

		}
		DBG_PRINTLN(" ");

		if (extraDelay) delay(1);
	}

	if (ccParamAnz > 0) {
		DBG_PRINT("ccreg write back ");
		for (uint8_t i=0;i<ccParamAnz;i++)
		{
			val = ccReg[i];
			cc1101::printHex2(val);
			cc1101::writeReg(0x0d + i, val);    // gemerkte Registerwerte zurueckschreiben
		}
		DBG_PRINTLN("");
	}
	MSG_PRINTLN(buf); // echo data of command
	musterDec.reset();
	FiFo.flush();
	enableReceive();	// enable the receiver
}





//================================= Kommandos ======================================
void IT_CMDs();


void HandleLongCommand()
{
	// Try so avoid blocking as long as possible and use internal Buffer

  #define  cmd_send 'S'

 if (IB_1[0] == cmd_send) {
  	if (musterDec.getState() == searching || MSG_PRINTER.available() == SERIAL_RX_BUFFER_SIZE/2)
	{
		send_cmd(); // Part of Send
	} else {
	}

  } else {
         MSG_PRINTLN(F("Unsupported command"));
  }
  
}




void serialEvent()
{
	static uint8_t idx = 0;
	while (MSG_PRINTER.available())
	{
		if (idx == 10) {
			// Short buffer is now full
			HandleLongCommand();
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
					return;
				case ';':
					send_cmd();
					break;
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






