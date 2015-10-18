/*
*   RF_RECEIVER v2.5 (201412) for Arduino
*   Sketch to use an arduino as a receiver/decoder device
*   for the home automation software fhem
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented yet
*   2014-2015  N.Butzek, S.Butzek

*   This software focuses on remote sensors like weather sensors (temperature,
*   humidity Logilink, TCM, Oregon Scientific, ...), remote controlled power switches
*   (Intertechno, TCM, ARCtech, ...) which use encoder chips like PT2262 and
*   EV1527-type and manchester encoder to send information in the 433MHz Band.
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


#define PROGNAME               "RF_RECEIVER"
#define PROGVERS               "3.1.8"

#define PIN_RECEIVE            2
#define PIN_LED                13 // Message-LED
#define PIN_SEND               11
#define BAUDRATE               57600
#define FIFO_LENGTH			   80
//#define DEBUG				   1
#include <TimerOne.h>  // Timer for LED Blinking
#include <bitstore.h>  // Die wird aus irgend einem Grund zum Compilieren benötigt.
#ifdef CMP_FIFO
#include <SimpleFIFO.h>
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#else
#include <filtering.h> //for FiFo Buffer
RingBuffer FiFo(FIFO_LENGTH, 0); // FiFo Puffer
#endif
#include <patternDecoder.h> //Logilink, IT decoder



#define pulseMin  90
volatile bool blinkLED = false;
String cmdstring = "";




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







void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void blinken();
int freeRam();
void changeReciver();
void changeFilter();
void HandleCommand();
bool command_available=false;
int getUptime();

//Decoder
patternDecoder musterDec;




void setup() {
	Serial.begin(BAUDRATE);
	#ifdef DEBUG
	#ifdef CMP_FIFO
	Serial.println("Using sFIFO");
	#else

	Serial.println(F("Startup:"));
	Serial.print(F("# Bytes / Puffer: "));
	Serial.println(sizeof(int)*FiFo.getBuffSize());
	Serial.print(F("# Len Fifo: "));
	Serial.println(FiFo.getBuffSize());

	#endif // CMP_FIFO
	#endif
	//delay(2000);
	pinMode(PIN_RECEIVE,INPUT);
	pinMode(PIN_SEND,OUTPUT);
	pinMode(PIN_LED,OUTPUT);
	Timer1.initialize(25*1000); //Interrupt wird jede n Millisekunden ausgelöst
	Timer1.attachInterrupt(blinken);

	enableReceive();
	cmdstring.reserve(20);
}

void blinken() {
     digitalWrite(PIN_LED, blinkLED);
     blinkLED=false;

}

void loop() {
	int aktVal;
	bool state;
	if (command_available) {
		command_available=false;
		HandleCommand();
		if (!command_available) { cmdstring = ""; }
		blinkLED=true;
	}
	#ifdef CMP_FIFO
	while (FiFo.count()>0 ) { //Puffer auslesen und an Dekoder übergeben
		aktVal=FiFo.dequeue();
		state = musterDec.decode(&aktVal); //Logilink, PT2262
		if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
	}
	#else
	while (FiFo.getNewValue(&aktVal)) { //Puffer auslesen und an Dekoder übergeben
		state = musterDec.decode(&aktVal); //Logilink, PT2262
		if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
	}
	#endif
}



//========================= Pulseauswertung ================================================
void handleInterrupt() {
  static unsigned long lastTime = micros();
  unsigned long Time=micros();
  const bool state = digitalRead(PIN_RECEIVE);
  const long  duration = Time - lastTime;
  lastTime = Time;
  if (duration >= pulseMin) {//kleinste zulässige Pulslänge
	int sDuration;
    if (duration <= (32000)) {//größte zulässige Pulslänge, max = 32000
      sDuration = int(duration); //das wirft bereits hier unnütige Nullen raus und vergrößert den Wertebereich
    }else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (state) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafür gilt die gemessene Dauer.
      sDuration=sDuration*-1;
    }
    #ifdef CMP_FIFO
    FiFo.enqueue(sDuration);
	#else
	FiFo.addValue(&sDuration);

	#endif // CMP_FIFO

    //++fifocnt;
  } // else => trash

}

void enableReceive() {
  attachInterrupt(0,handleInterrupt,CHANGE);
}

void disableReceive() {
  detachInterrupt(0);
}



//============================== IT_Send =========================================
byte ITrepetition = 6;
byte ITreceivetolerance= 60;
int ITbaseduration = 420;

void PT2262_transmit(int nHighPulses, int nLowPulses) {
  digitalWrite(PIN_SEND, HIGH);
  delayMicroseconds(ITbaseduration * nHighPulses);
  digitalWrite(PIN_SEND, LOW);
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

void send_raw(int16_t *buckets)
{
	uint8_t index=0;
	for (uint8_t i=cmdstring.indexOf('D')+1;i<=cmdstring.length();i++ )
	{
		index = cmdstring.charAt(i)+0;
		digitalWrite(PIN_SEND, !(buckets[index] >>15));
		if (buckets[index] > 8000) // Use delay at 8000 microseconds to be more precice
		{
			delay(buckets[index]/1000);
		} else {
			delayMicroseconds(buckets[index]);
		}
	}
}

//SR;R=3;P0=123;P1=312;P2=400;P3=600;D=010101020302030302;
void send_cmd()
{
  if (cmdstring.charAt(1) == 'R')
  {
	/*
	1. state vom decoder abfragen, ob er gerade etwas aufzeichnet
	2. daten aufsplitten
    */

	int16_t buckets[6];
	uint8_t counter=0;
	uint8_t repeats=0;
	int8_t strp1=2;
	int8_t strp2=0;
	String msg_part;
	strp1=cmdstring.indexOf(";",strp1);    // search first  ";", start after RS command

	while (cmdstring.charAt(strp1+1) != 'D' && strp1+1 < cmdstring.indexOf('D'))
	{
		strp2=cmdstring.indexOf(";",strp1+2);  // search next ";" after strp1
		msg_part = cmdstring.substring(strp1+1,strp2-1);  // substrimg

		if (msg_part.charAt(0) == 'P' && msg_part.charAt(2) == '=') // Do some basic detection if data matches what we expect
		{
			counter = msg_part.substring(1,1).toInt(); // extract the pattern number
			buckets[counter]=  msg_part.substring(3).toInt();

		} else if(msg_part.charAt(0) == 'R' && msg_part.charAt(2) == '=')
		{
			repeats= msg_part.substring(2).toInt();
		}
		strp1=strp2;
	}

	disableReceive();  // Disable the receiver
	for (uint8_t i=0;i<repeats;i++)
	{
		send_raw(buckets);
		delay(2);
	}
	enableReceive();	// enable the receiver
    Serial.println(cmdstring); // echo


  }
}





//================================= Kommandos ======================================
void IT_CMDs();

void HandleCommand()
{

  #define cmd_Version 'V'
  #define cmd_freeRam 'R'
  #define  cmd_intertechno 'i'
  #define  cmd_uptime 't'
  #define  cmd_changeReceiver 'X'
  #define  cmd_space ' '
  #define  cmd_help'?'
  #define  cmd_changeFilter 'F'
  #define  cmd_send 'S'

  // ?: Kommandos anzeigen


  if (cmdstring.charAt(0) == cmd_help) {
    //Serial.println(F("? Use one of V R i t X"));//FHEM Message
	Serial.print(cmd_help);	Serial.print(F(" Use one of "));
	Serial.print(cmd_Version);Serial.print(cmd_space);
	Serial.print(cmd_intertechno);Serial.print(cmd_space);
	Serial.print(cmd_freeRam);Serial.print(cmd_space);
	Serial.print(cmd_uptime);Serial.print(cmd_space);
	Serial.print(cmd_changeReceiver);Serial.print(cmd_space);
	Serial.print(cmd_changeFilter);Serial.print(cmd_space);
	Serial.print(cmd_send);Serial.print(cmd_space);

	Serial.println("");

  }
  // V: Version
  else if (cmdstring.charAt(0) == cmd_Version) {
    Serial.println("V " PROGVERS " SIGNALduino - compiled at " __DATE__ " " __TIME__);
  }
  // R: FreeMemory
  else if (cmdstring.charAt(0) == cmd_freeRam) {

    Serial.println(freeRam());
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
		send_cmd();
	}
  }
    // t: Uptime
  else if (cmdstring.charAt(0) == cmd_uptime) {
		Serial.println(getUptime());
  }
  // XQ disable receiver
  else if (cmdstring.charAt(0) == cmd_changeReceiver) {
    changeReciver();
    //Serial.flush();
	//Serial.end();
  }
  else if (cmdstring.charAt(0) == cmd_changeFilter) {
    changeFilter();
  }

}





void IT_CMDs() {

  // Set Intertechno receive tolerance
  if (cmdstring.charAt(1) == 't') {
    char msg[3];
    cmdstring.substring(2).toCharArray(msg,3);
    ITreceivetolerance = atoi(msg);
    Serial.println(cmdstring);
  }
  // Set Intertechno Repetition
  else if (cmdstring.charAt(1) == 'r') {
    char msg[3];
    cmdstring.substring(2).toCharArray(msg,3);
    ITrepetition = atoi(msg);
    Serial.println(cmdstring);
  }
  // Switch Intertechno Devices
  else if (cmdstring.charAt(1) == 's') {
    digitalWrite(PIN_LED,HIGH);
    char msg[13];
    cmdstring.substring(2).toCharArray(msg,13);
    if (cmdstring.length() > 14)
    {
       ITbaseduration=cmdstring.substring(14).toInt(); // Default Baseduration
    }
    else
    {
       ITbaseduration=420; // Default Baseduration
    }
    sendPT2262(msg);
    digitalWrite(PIN_LED,LOW);
    Serial.println(cmdstring);
  }
  // Get Intertechno Parameters
  else if (cmdstring.charAt(1) == 'p') {
    String cPrint = "ITParams: ";
    cPrint += String(ITreceivetolerance);
    cPrint += " ";
    cPrint += String(ITrepetition);
    cPrint += " ";
    cPrint += String(ITbaseduration);
    Serial.println(cPrint);
  }

}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
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

 Serial.print("\nheapptr=[0x"); Serial.print( (int) heapptr, HEX); Serial.print("] (growing upward, "); Serial.print( (int) heapptr, DEC); Serial.print(" decimal)");

 Serial.print("\nstackptr=[0x"); Serial.print( (int) stackptr, HEX); Serial.print("] (growing downward, "); Serial.print( (int) stackptr, DEC); Serial.print(" decimal)");

 Serial.print("\ndifference should be positive: diff=stackptr-heapptr, diff=[0x");
 diff=stackptr-heapptr;
 Serial.print( (int) diff, HEX); Serial.print("] (which is ["); Serial.print( (int) diff, DEC); Serial.print("] (bytes decimal)");


 Serial.print("\n\nLOOP END: get_free_memory() reports [");
 Serial.print( get_free_memory() );
 Serial.print("] (bytes) which must be > 0 for no heap/stack collision");


 // ---------------- Print memory profile -----------------
 Serial.print("\n\n__data_start=[0x"); Serial.print( (int) &__data_start, HEX ); Serial.print("] which is ["); Serial.print( (int) &__data_start, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__data_end=[0x"); Serial.print((int) &__data_end, HEX ); Serial.print("] which is ["); Serial.print( (int) &__data_end, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__bss_start=[0x"); Serial.print((int) & __bss_start, HEX ); Serial.print("] which is ["); Serial.print( (int) &__bss_start, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__bss_end=[0x"); Serial.print( (int) &__bss_end, HEX ); Serial.print("] which is ["); Serial.print( (int) &__bss_end, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__heap_start=[0x"); Serial.print( (int) &__heap_start, HEX ); Serial.print("] which is ["); Serial.print( (int) &__heap_start, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__malloc_heap_start=[0x"); Serial.print( (int) __malloc_heap_start, HEX ); Serial.print("] which is ["); Serial.print( (int) __malloc_heap_start, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__malloc_margin=[0x"); Serial.print( (int) &__malloc_margin, HEX ); Serial.print("] which is ["); Serial.print( (int) &__malloc_margin, DEC); Serial.print("] bytes decimal");

 Serial.print("\n__brkval=[0x"); Serial.print( (int) __brkval, HEX ); Serial.print("] which is ["); Serial.print( (int) __brkval, DEC); Serial.print("] bytes decimal");

 Serial.print("\nSP=[0x"); Serial.print( (int) SP, HEX ); Serial.print("] which is ["); Serial.print( (int) SP, DEC); Serial.print("] bytes decimal");

 Serial.print("\nRAMEND=[0x"); Serial.print( (int) RAMEND, HEX ); Serial.print("] which is ["); Serial.print( (int) RAMEND, DEC); Serial.print("] bytes decimal");

 // summaries:
 ramSize   = (int) RAMEND       - (int) &__data_start;
 dataSize  = (int) &__data_end  - (int) &__data_start;
 bssSize   = (int) &__bss_end   - (int) &__bss_start;
 heapSize  = (int) __brkval     - (int) &__heap_start;
 stackSize = (int) RAMEND       - (int) SP;
 freeMem1  = (int) SP           - (int) __brkval;
 freeMem2  = ramSize - stackSize - heapSize - bssSize - dataSize;
 Serial.print("\n--- section size summaries ---");
 Serial.print("\nram   size=["); Serial.print( ramSize, DEC ); Serial.print("] bytes decimal");
 Serial.print("\n.data size=["); Serial.print( dataSize, DEC ); Serial.print("] bytes decimal");
 Serial.print("\n.bss  size=["); Serial.print( bssSize, DEC ); Serial.print("] bytes decimal");
 Serial.print("\nheap  size=["); Serial.print( heapSize, DEC ); Serial.print("] bytes decimal");
 Serial.print("\nstack size=["); Serial.print( stackSize, DEC ); Serial.print("] bytes decimal");
 Serial.print("\nfree size1=["); Serial.print( freeMem1, DEC ); Serial.print("] bytes decimal");
 Serial.print("\nfree size2=["); Serial.print( freeMem2, DEC ); Serial.print("] bytes decimal");
#else
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#endif // CMP_MEMDBG

 }

int getUptime()
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




/* not used anymore */
void printFilter(uint8_t id)
{
	Serial.print("UPD Filter");
	Serial.print(id);
	Serial.print(";C=");	Serial.print(musterDec.protoID[id].clock);
	Serial.print(";SF=");	Serial.print(musterDec.protoID[id].syncFact);
	Serial.println(";");

}
void changeFilter()
{
	//cmdstring.concat(0);
	char tmp[10];

	cmdstring.toCharArray(tmp,10,3);

	char *param = strtok(tmp,";");
	const uint8_t id = atoi(param);
	s_sigid new_entry ={NULL,NULL,NULL,NULL,NULL,undef};

	if (cmdstring.charAt(1) == 'A')
	{
		// ADD entry to filter list    A;<num>;<Syncfact>;<clock>;

		// syncfact
		param = strtok (NULL, ";");
		new_entry.syncFact = atoi(param);
		// clock
		param = strtok (NULL, ";");
		new_entry.clock = atoi(param);

		musterDec.protoID[id] = new_entry;
	}
	if (cmdstring.charAt(1) == 'R')
	{
		// Remove entry to filter list R;<number to remove>;
		musterDec.protoID[id] = new_entry;
	}
	printFilter(id);
	if (musterDec.numprotos+1 < id)
		musterDec.numprotos = id+1;

}





