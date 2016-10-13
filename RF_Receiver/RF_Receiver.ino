/*
*   RF_RECEIVER v3.2 for Arduino
*   Sketch to use an arduino as a receiver/sending device for digital signals
*
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented in the sketch,
*   there is an option to send almost any data over a send raw interface
*   2014-2015  N.Butzek, S.Butzek
*   2016 S.Butzek

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
#define CMP_FIFO 1
//#define CMP_MEMDBG 1
#define CMP_NEWSD;

#define PROGNAME               "RF_RECEIVER"
#define PROGVERS               "3.3.0"

#define PIN_RECEIVE            2
#define PIN_LED                13 // Message-LED
#define PIN_SEND               11
#define BAUDRATE               57600
#define FIFO_LENGTH			   50
//#define DEBUG				   1

#include <output.h>

#include <TimerOne.h>  // Timer for LED Blinking
#include <bitstore.h>  // Die wird aus irgend einem Grund zum Compilieren benoetigt.
#ifdef CMP_FIFO
#include <SimpleFIFO.h>
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#else
#include <filtering.h> //for FiFo Buffer
RingBuffer FiFo(FIFO_LENGTH, 0); // FiFo Puffer
#endif
#ifdef CMP_NEWSD
#include <signalDecoder.h>
SignalDetectorClass musterDec;

#else
#include <patternDecoder.h> 
patternDecoder musterDec;

#endif

#include <EEPROM.h>


#define pulseMin  90
volatile bool blinkLED = false;
String cmdstring = "";
volatile unsigned long lastTime = micros();



#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))





#ifdef CMP_MEMDBG

extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
uint8_t *heapptr, *stackptr;
uint16_t diff=0;
void check_mem() {
 stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
 heapptr = stackptr;                     // save value of heap pointer
 free(stackptr);      // free up the memory again (sets stackptr to 0)
 stackptr =  (uint8_t *)(SP);           // save value of stack pointer
}
//extern int __bss_end;
//extern void *__brkval;

int get_free_memory()
{
 int free_memory;

 if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
 else
   free_memory = ((int)&free_memory) - ((int)__brkval);

 return free_memory;
}


int16_t ramSize=0;   // total amount of ram available for partitioning
int16_t dataSize=0;  // partition size for .data section
int16_t bssSize=0;   // partition size for .bss section
int16_t heapSize=0;  // partition size for current snapshot of the heap section
int16_t stackSize=0; // partition size for current snapshot of the stack section
int16_t freeMem1=0;  // available ram calculation #1
int16_t freeMem2=0;  // available ram calculation #2

#endif


// EEProm Addresscommands
#define addr_init 0
#define addr_features 1



void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void cronjob();
int freeRam();
void changeReciver();
void changeFilter();
void HandleCommand();
bool command_available=false;
unsigned long getUptime();
void getConfig();
void enDisPrint(bool enDis);
void getPing();
void configCMD();
void storeFunctions(const int8_t ms=1, int8_t mu=1, int8_t mc=1);
void getFunctions(bool *ms,bool *mu,bool *mc);




void setup() {
	Serial.begin(BAUDRATE);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB
	}
	#ifdef DEBUG
	#ifdef CMP_FIFO
	MSG_PRINTLN("Using sFIFO");
	#else

	MSG_PRINTLN(F("Startup:"));
	MSG_PRINT(F("# Bytes / Puffer: "));
	MSG_PRINTLN(sizeof(int)*FiFo.getBuffSize());
	MSG_PRINT(F("# Len Fifo: "));
	MSG_PRINTLN(FiFo.getBuffSize());

	#endif // CMP_FIFO
	#endif
	//delay(2000);
	pinMode(PIN_RECEIVE,INPUT);
	pinMode(PIN_SEND,OUTPUT);
	pinMode(PIN_LED,OUTPUT);
	Timer1.initialize(31*1000); //Interrupt wird jede n Millisekunden ausgeloest
	Timer1.attachInterrupt(cronjob);
	if (EEPROM.read(addr_init) == 0xB)
	{
		#ifdef DEBUG
		MSG_PRINTLN("Reading values fom eeprom");
		#endif
		getFunctions(&musterDec.MSenabled,&musterDec.MUenabled,&musterDec.MCenabled);
	} else {
		EEPROM.write(addr_init,0xB);
		storeFunctions(1, 1, 1);    // Init EEPROM with all flags enabled
		#ifdef DEBUG
		MSG_PRINTLN("Init eeprom to defaults after flash");
		#endif
	}

	/*MSG_PRINT("MS:"); 	MSG_PRINTLN(musterDec.MSenabled);
	MSG_PRINT("MU:"); 	MSG_PRINTLN(musterDec.MUenabled);
	MSG_PRINT("MC:"); 	MSG_PRINTLN(musterDec.MCenabled);*/


	enableReceive();
	cmdstring.reserve(40);
}

void cronjob() {

	/*
	 const unsigned long  duration = micros() - lastTime;
	 if (duration > maxPulse) { //Auf Maximalwert pr�fen.
		 //handleInterrupt();
		 //MSG_PRINTLN("PTout");
		 int sDuration = maxPulse;
		 if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			 sDuration = -sDuration;
		 }
		#ifdef CMP_FIFO
		 FiFo.enqueue(sDuration);
		#else
		 FiFo.addValue(&sDuration);
		#endif // CMP_FIFO
		lastTime = micros();

	 }*/
	 digitalWrite(PIN_LED, blinkLED);
	 blinkLED = false;
	/*
	 if (FiFo.count() >19)
	 {
		 MSG_PRINT("SF:"); MSG_PRINTLN(FiFo.count());
	 }
	 */
}

void loop() {
	static int aktVal=0;
	bool state;
	if (command_available) {
		command_available=false;
		HandleCommand();
		if (!command_available) { cmdstring = ""; }
		blinkLED=true;
	}
	#ifdef CMP_FIFO


	while (FiFo.count()>0 ) { //Puffer auslesen und an Dekoder uebergeben

		aktVal=FiFo.dequeue();
		state = musterDec.decode(&aktVal); 
		if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
	}
	#else
	while (FiFo.getNewValue(&aktVal)) { //Puffer auslesen und an Dekoder uebergeben
		state = musterDec.decode(&aktVal); //Logilink, PT2262
		if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
	}
	#endif
 }



//========================= Pulseauswertung ================================================
void handleInterrupt() {
  const unsigned long Time=micros();
  //const bool state = digitalRead(PIN_RECEIVE);
  const unsigned long  duration = Time - lastTime;
  lastTime = Time;
  if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
	int sDuration;
    if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
      sDuration = int(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
    }else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (isHigh(PIN_RECEIVE)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
      sDuration=-sDuration;
    }
    #ifdef CMP_FIFO
	//MSG_PRINTLN(sDuration);
    FiFo.enqueue(sDuration);
	#else
	FiFo.addValue(&sDuration);

	#endif // CMP_FIFO

    //++fifocnt;
  } // else => trash

}

void enableReceive() {
   attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE),handleInterrupt,CHANGE);
}

void disableReceive() {
  detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));
}



//============================== IT_Send =========================================
uint8_t ITrepetition = 6;
int ITbaseduration = 300;

void PT2262_transmit(uint8_t nHighPulses, uint8_t nLowPulses) {
  //digitalWrite(PIN_SEND, HIGH);
  digitalHigh(PIN_SEND);
  delayMicroseconds(ITbaseduration * nHighPulses);
  //digitalWrite(PIN_SEND, LOW);
  digitalLow(PIN_SEND);
  delayMicroseconds(ITbaseduration * nLowPulses);
}

void sendPT2262(char* triStateMessage) {
  disableReceive();
  for (int i = 0; i < ITrepetition; i++) {
    unsigned int pos = 0;
    PT2262_transmit(1,31);
    while (triStateMessage[pos] != '\0') {
      switch(triStateMessage[pos]) {
      case '0':
        PT2262_transmit(1,3);
        PT2262_transmit(1,3);
        break;
      case 'F':
        PT2262_transmit(1,3);
        PT2262_transmit(3,1);
        break;
      case '1':
        PT2262_transmit(3,1);
        PT2262_transmit(3,1);
        break;
      }
      pos++;
    }
  }
  enableReceive();
}


//================================= RAW Send ======================================
void send_raw(const uint8_t startpos,const uint16_t endpos,const int16_t *buckets, String *source=&cmdstring)
{
	uint8_t index=0;
	unsigned long stoptime=micros();
	bool isLow;
	uint16_t dur;
	for (uint16_t i=startpos;i<=endpos;i++ )
	{
		//MSG_PRINT(cmdstring.substring(i,i+1));
		index = source->charAt(i) - '0';
		//MSG_PRINT(index);
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
	//MSG_PRINTLN("");

}
//SM;R=2;C=400;D=AFAFAF;




void send_mc(const uint8_t startpos,const uint8_t endpos, const int16_t clock)
{
	int8_t b;
	char c;
	//digitalHigh(PIN_SEND);
	//delay(1);
	uint8_t bit;

	unsigned long stoptime =micros();
	for (uint8_t i = startpos; i <= endpos; i++) {
		c = cmdstring.charAt(i);
		b = ((byte)c) - (c <= '9' ? 0x30 : 0x37);

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
// SC;R=4;SM;C=400;D=AFFFFFFFFE;SR;P0=-2500;P1=400;D=010;SM;D=AB6180;SR;D=101;
// SC;R=4;SM;C=400;D=FFFFFFFF;SR;P0=-400;P1=400;D=101;SM;D=AB6180;SR;D=101;
// SR;R=3;P0=1230;P1=-3120;P2=-400;P3=-900;D=030301010101010202020202020101010102020202010101010202010120202;
// SM;C=400;D=AAAAFFFF;
// SR;R=10;P0=-2000;P1=-1000;P2=500;P3=-6000;D=2020202021212020202121212021202021202121212023;

struct s_sendcmd {
	int16_t sendclock;
	uint8_t type;
	uint8_t datastart;
	uint16_t dataend;
	int16_t buckets[6];
} ;

void send_cmd()
{
	#define combined 0
	#define manchester 1
	#define raw 2

	String msg_part;
	msg_part.reserve(30);
	uint8_t repeats=1;  // Default is always one iteration so repeat is 1 if not set
	//uint8_t type;
	int16_t start_pos=0;
	//int16_t buckets[6]={};
	uint8_t counter=0;
	//uint16_t sendclock;
	bool extraDelay = true;

	s_sendcmd command[5];

	disableReceive();

	uint8_t cmdNo=255;


	while (split_cmdpart(&start_pos,&msg_part))
	{
		//MSG_PRINTLN(msg_part);
		if (msg_part.charAt(0) == 'S')
		{
			if (msg_part.charAt(1) == 'C')  // send combined informatio flag
			{
				//type=combined;
				//cmdNo=255;
				extraDelay = false;
			}
			else if (msg_part.charAt(1) == 'M') // send manchester
			{
				//type=manchester;
				cmdNo++;
				command[cmdNo].type=manchester;
				//MSG_PRINTLN("Adding manchester");
			}
			else if (msg_part.charAt(1) == 'R') // send raw
			{
				//type=raw;
				cmdNo++;
				command[cmdNo].type=raw;
				//MSG_PRINTLN("Adding raw");
				extraDelay = false;

			}
		}
		else if (msg_part.charAt(0) == 'P' && msg_part.charAt(2) == '=') // Do some basic detection if data matches what we expect
		{
			counter = msg_part.substring(1,2).toInt(); // extract the pattern number
			//buckets[counter]=  msg_part.substring(3).toInt();
			command[cmdNo].buckets[counter]=msg_part.substring(3).toInt();
		    //MSG_PRINTLN("Adding bucket");

		} else if(msg_part.charAt(0) == 'R' && msg_part.charAt(1) == '=') {
			repeats= msg_part.substring(2).toInt();
		    //MSG_PRINTLN("Adding repeats");

		} else if (msg_part.charAt(0) == 'D') {
			command[cmdNo].datastart = start_pos - msg_part.length()+1;
			command[cmdNo].dataend = start_pos-2;
		    //MSG_PRINT("locating data start:");
		   // MSG_PRINT(command[cmdNo].datastart);
		    //MSG_PRINT(" end:");
			//MSG_PRINTLN(command[cmdNo].dataend);
			//if (type==raw) send_raw(&msg_part,buckets);
			//if (type==manchester) send_mc(&msg_part,sendclock);
			//digitalWrite(PIN_SEND, LOW); // turn off transmitter
			//digitalLow(PIN_SEND);
		} else if(msg_part.charAt(0) == 'C' && msg_part.charAt(1) == '=')
		{
			//sendclock = msg_part.substring(2).toInt();
			command[cmdNo].sendclock = msg_part.substring(2).toInt();
		    //MSG_PRINTLN("adding sendclock");
		}
	}

	for (uint8_t i=0;i<repeats;i++)
	{
		for (uint8_t c=0;c<=cmdNo;c++)
		{
			if (command[c].type==raw) send_raw(command[c].datastart,command[c].dataend,command[c].buckets);
			if (command[c].type==manchester) send_mc(command[c].datastart,command[c].dataend,command[c].sendclock);
			digitalLow(PIN_SEND);
		}
		if (extraDelay) delay(1);
	}

	enableReceive();	// enable the receiver
    MSG_PRINTLN(cmdstring); // echo

}





//================================= Kommandos ======================================
void IT_CMDs();

void HandleCommand()
{

  #define  cmd_Version 'V'
  #define  cmd_freeRam 'R'
  #define  cmd_intertechno 'i'
  #define  cmd_uptime 't'
  #define  cmd_changeReceiver 'X'
  #define  cmd_space ' '
  #define  cmd_help '?'
  #define  cmd_changeFilter 'F'
  #define  cmd_send 'S'
  #define  cmd_ping 'P'
  #define  cmd_config 'C'
  #define  cmd_getConfig 'G' //decrepated


  if (cmdstring.charAt(0) == cmd_ping){
	getPing();
  }  // ?: Kommandos anzeigen
  else if (cmdstring.charAt(0) == cmd_help) {
	MSG_PRINT(cmd_help);	MSG_PRINT(F(" Use one of "));
	MSG_PRINT(cmd_Version);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_intertechno);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_freeRam);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_uptime);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_changeReceiver);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_changeFilter);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_send);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_ping);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_config);MSG_PRINT(cmd_space);
	MSG_PRINT(cmd_getConfig);MSG_PRINT(cmd_space);  //decrepated

	MSG_PRINTLN("");
  }
  // V: Version
  else if (cmdstring.charAt(0) == cmd_Version) {
    MSG_PRINTLN("V " PROGVERS " SIGNALduino - compiled at " __DATE__ " " __TIME__);
  }
  // R: FreeMemory
  else if (cmdstring.charAt(0) == cmd_freeRam) {

    MSG_PRINTLN(freeRam());
  }
  // i: Intertechno
  else if (cmdstring.charAt(0) == cmd_intertechno) {
	if (musterDec.getState() != searching)
	{
		command_available=true;
	} else {
		IT_CMDs();
	}
  }
  else if (cmdstring.charAt(0) == cmd_send) {
  	if (musterDec.getState() != searching )
	{
		command_available=true;
	} else {
		send_cmd(); // Part of Send
	}
  }
    // t: Uptime
  else if (cmdstring.charAt(0) == cmd_uptime) {
		MSG_PRINTLN(getUptime());
  }
  // XQ disable receiver
  else if (cmdstring.charAt(0) == cmd_changeReceiver) {
    changeReciver();
  }
  else if (cmdstring.charAt(0) == cmd_changeFilter) {
  }
  else if (cmdstring.charAt(0) == cmd_config) {
    configCMD();
  }
  // get config
  else if (cmdstring.charAt(0) == cmd_getConfig) {
     getConfig();
  }
  else {
	  MSG_PRINTLN(F("Unsupported command"));
  }
}


void getConfig()
{
   MSG_PRINT(F("MS="));
   //enDisPrint(musterDec.MSenabled);
   MSG_PRINT(musterDec.MSenabled,DEC);
   MSG_PRINT(F(";MU="));
   //enDisPrint(musterDec.MUenabled);
   MSG_PRINT(musterDec.MUenabled, DEC);
   MSG_PRINT(F(";MC="));
   //enDisPrint(musterDec.MCenabled);
   MSG_PRINTLN(musterDec.MCenabled, DEC);
}


void enDisPrint(bool enDis)
{
   if (enDis) {
      MSG_PRINT(F("enable"));
   }
   else {
      MSG_PRINT(F("disable"));
   }
}


void configCMD()
{
  if (cmdstring.charAt(1) == 'G') {  // Get, no change to configuration
	getConfig();
	return;
  }

  bool *bptr;

  if (cmdstring.charAt(2) == 'S') {  	  //MS
	bptr=&musterDec.MSenabled;;
  }
  else if (cmdstring.charAt(2) == 'U') {  //MU
	bptr=&musterDec.MUenabled;;
  }
  else if (cmdstring.charAt(2) == 'C') {  //MC
	bptr=&musterDec.MCenabled;;
  }

  if (cmdstring.charAt(1) == 'E') {   // Enable
	*bptr=true;
  }
  else if (cmdstring.charAt(1) == 'D') {  // Disable
	*bptr=false;
  } else {
	return;
  }
  storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled);
}


void IT_CMDs() {

  // Set Intertechno receive tolerance
  if (cmdstring.charAt(1) == 't') {
    MSG_PRINTLN(cmdstring);
  }
  // Set Intertechno Repetition
  else if (cmdstring.charAt(1) == 'r') {
    char msg[3];
    cmdstring.substring(2).toCharArray(msg,3);
    ITrepetition = atoi(msg);
    MSG_PRINTLN(cmdstring);
  }
  // Switch Intertechno Devices
  else if (cmdstring.charAt(1) == 's') {
    //digitalWrite(PIN_LED,HIGH);
    digitalHigh(PIN_SEND);
    char msg[40];
    cmdstring.substring(2).toCharArray(msg,40);
   	sendPT2262(msg);
    //digitalWrite(PIN_LED,LOW);
    digitalLow(PIN_SEND);
    MSG_PRINTLN(cmdstring);
  }
  else if (cmdstring.charAt(1) == 'c') {
	ITbaseduration=cmdstring.substring(2).toInt(); // Updates Baseduration
    MSG_PRINTLN(cmdstring);
  }
  // Get Intertechno Parameters
  else if (cmdstring.charAt(1) == 'p') {
    String cPrint;
    cPrint.reserve(20);
    cPrint = "ITParams: ";
    cPrint += String(ITrepetition);
    cPrint += " ";
    cPrint += String(ITbaseduration);
    MSG_PRINTLN(cPrint);
  }

}

void serialEvent()
{
  while (MSG_PRINTER.available())
  {
    char inChar = (char)MSG_PRINTER.read();
    switch(inChar)
    {
    case '\n':
    case '\r':
    case '\0':
    case '#':
		command_available=true;
		break;
    default:
      cmdstring += inChar;
    }
  }
}


int freeRam () {
#ifdef CMP_MEMDBG

 check_mem();

 MSG_PRINT("\nheapptr=[0x"); MSG_PRINT( (int) heapptr, HEX); MSG_PRINT("] (growing upward, "); MSG_PRINT( (int) heapptr, DEC); MSG_PRINT(" decimal)");

 MSG_PRINT("\nstackptr=[0x"); MSG_PRINT( (int) stackptr, HEX); MSG_PRINT("] (growing downward, "); MSG_PRINT( (int) stackptr, DEC); MSG_PRINT(" decimal)");

 MSG_PRINT("\ndifference should be positive: diff=stackptr-heapptr, diff=[0x");
 diff=stackptr-heapptr;
 MSG_PRINT( (int) diff, HEX); MSG_PRINT("] (which is ["); MSG_PRINT( (int) diff, DEC); MSG_PRINT("] (bytes decimal)");


 MSG_PRINT("\n\nLOOP END: get_free_memory() reports [");
 MSG_PRINT( get_free_memory() );
 MSG_PRINT("] (bytes) which must be > 0 for no heap/stack collision");


 // ---------------- Print memory profile -----------------
 MSG_PRINT("\n\n__data_start=[0x"); MSG_PRINT( (int) &__data_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__data_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__data_end=[0x"); MSG_PRINT((int) &__data_end, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__data_end, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__bss_start=[0x"); MSG_PRINT((int) & __bss_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__bss_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__bss_end=[0x"); MSG_PRINT( (int) &__bss_end, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__bss_end, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__heap_start=[0x"); MSG_PRINT( (int) &__heap_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__heap_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__malloc_heap_start=[0x"); MSG_PRINT( (int) __malloc_heap_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) __malloc_heap_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__malloc_margin=[0x"); MSG_PRINT( (int) &__malloc_margin, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__malloc_margin, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__brkval=[0x"); MSG_PRINT( (int) __brkval, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) __brkval, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\nSP=[0x"); MSG_PRINT( (int) SP, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) SP, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\nRAMEND=[0x"); MSG_PRINT( (int) RAMEND, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) RAMEND, DEC); MSG_PRINT("] bytes decimal");

 // summaries:
 ramSize   = (int) RAMEND       - (int) &__data_start;
 dataSize  = (int) &__data_end  - (int) &__data_start;
 bssSize   = (int) &__bss_end   - (int) &__bss_start;
 heapSize  = (int) __brkval     - (int) &__heap_start;
 stackSize = (int) RAMEND       - (int) SP;
 freeMem1  = (int) SP           - (int) __brkval;
 freeMem2  = ramSize - stackSize - heapSize - bssSize - dataSize;
 MSG_PRINT("\n--- section size summaries ---");
 MSG_PRINT("\nram   size=["); MSG_PRINT( ramSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\n.data size=["); MSG_PRINT( dataSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\n.bss  size=["); MSG_PRINT( bssSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nheap  size=["); MSG_PRINT( heapSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nstack size=["); MSG_PRINT( stackSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nfree size1=["); MSG_PRINT( freeMem1, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nfree size2=["); MSG_PRINT( freeMem2, DEC ); MSG_PRINT("] bytes decimal");
#else
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#endif // CMP_MEMDBG

 }

unsigned long getUptime()
{
	unsigned long now = millis();
	static uint16_t times_rolled = 0;
	static unsigned long last = 0;
	// If this run is less than the last the counter rolled
	unsigned long seconds = now / 1000;
	if (now < last) {
		times_rolled++;
		seconds += ((long(4294967295) / 1000 )*times_rolled);
	}
	last = now;
	return seconds;
}

void getPing()
{
	MSG_PRINTLN("OK");
	delay(1);
}

void changeReciver() {
  if (cmdstring.charAt(1) == 'Q')
  {
  	disableReceive();
  }
  if (cmdstring.charAt(1) == 'E')
  {
  	enableReceive();
  }
}






//================================= EEProm commands ======================================



void storeFunctions(const int8_t ms, int8_t mu, int8_t mc)
{
	mu=mu<<1;
	mc=mc<<2;
	int8_t dat =  ms | mu | mc;
    EEPROM.write(addr_features,dat);
}

void getFunctions(bool *ms,bool *mu,bool *mc)
{
    int8_t dat = EEPROM.read(addr_features);

    *ms=bool (dat &(1<<0));
    *mu=bool (dat &(1<<1));
    *mc=bool (dat &(1<<2));


}
