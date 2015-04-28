/*
*   RF_RECEIVER v2.5 (201412) for Arduino
*   Sketch to use an arduino as a receiver/decoder device
*   for the home automation software fhem
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented yet
*   2014  N.Butzek, S.Butzek

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


#define PROGNAME               "RF_RECEIVER"
#define PROGVERS               "3.0raw-out-alpha"

#define PIN_RECEIVE            2
#define PIN_LED                13 // Message-LED
#define PIN_SEND               11
#define BAUDRATE               57600
#define FIFO_LENGTH			   80
//#define DEBUG				   1
#include <filtering.h> //for FiFo Buffer
#include <TimerOne.h>  // Timer for LED Blinking
#include <bitstore.h>  // Die wird aus irgend einem Grund zum Compilieren benötigt.
#include <patternDecoder.h> //Logilink, IT decoder

RingBuffer FiFo(FIFO_LENGTH, 0); // FiFo Puffer
const uint8_t pulseMin = 100;
bool blinkLED = false;
String cmdstring = "";

void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void blinken();
int freeRam();
void changeReciver();
void changeFilter();


//volatile int fifocnt=0;

//Decoder
//SensorReceiver ASreceiver;
patternDecoder musterDec;
//ManchesterpatternDetector ManchesterDetect(true);
//ASDecoder   asDec  (&ManchesterDetect);
//OSV2Decoder osv2Dec (&ManchesterDetect);



void setup() {
	Serial.begin(BAUDRATE);
	#ifdef DEBUG
	Serial.println(F("Startup:"));
	Serial.print(F("# Bytes / Puffer: "));
	Serial.println(sizeof(int)*FiFo.getBuffSize());
	Serial.print(F("# Len Fifo: "));
	Serial.println(FiFo.getBuffSize());
	#endif
	//delay(2000);
	pinMode(PIN_RECEIVE,INPUT);
	pinMode(PIN_SEND,OUTPUT);
	pinMode(PIN_LED,OUTPUT);
	Timer1.initialize(25*1000); //Interrupt wird jede n Millisekunden ausgelöst
	Timer1.attachInterrupt(blinken);


	musterDec.protoID[0]=(s_sigid){-4,-8,-18,500,0,twostate}; // Logi, TCM 97001 etc.
	musterDec.protoID[1]=(s_sigid){0,0,-11,560,0,twostate}; // RSL
	musterDec.protoID[2]=(s_sigid){0,0,0,0,0,undef}; // Free Slot
	musterDec.protoID[3]=(s_sigid){-1,3,-30,-1,0,tristate}; // IT old
	musterDec.protoID[4]=(s_sigid){0,0,-10,270,0,twostate}; // IT Autolearn
	musterDec.numprotos=5;
	//musterDec.protoID[2]=(s_sigid){-1,-2,-18,500,0,twostate}; // AS

	enableReceive();
	cmdstring.reserve(20);
}

void blinken() {
     digitalWrite(PIN_LED, blinkLED);
     blinkLED=false;
}

void loop() {
  //Serial.print("fcnt: ");   Serial.println(fifocnt);


  int aktVal;
  bool state;
  while (FiFo.getNewValue(&aktVal)) { //Puffer auslesen und an Dekoder übergeben
    //Serial.print(aktVal); Serial.print(",");
    #ifdef TX_TST
    IT_TX(abs(aktVal));
    #endif
    //--fifocnt;
	//long t1= micros();
    state = musterDec.decode(&aktVal); //Logilink, PT2262
	//Serial.println(micros()-t1);
    /*
    if (ManchesterDetect.detect(&aktVal))
    {
      if (asDec.decode()) {
        state =true;
        Serial.println(asDec.getMessageHexStr());
      } else if (osv2Dec.decode()) {
        state = true;
        Serial.println(osv2Dec.getMessageHexStr());
      }
      ManchesterDetect.reset();
    }
    */
    if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
  }
  //long t2= micro();


}



//========================= Pulseauswertung ================================================
void handleInterrupt() {
  static unsigned long Time;
  static unsigned long lastTime = micros();
  static long duration;
  static int sDuration;
  Time = micros();
  bool state = digitalRead(PIN_RECEIVE);
  duration = Time - lastTime;
  lastTime = Time;
  if (duration >= pulseMin) {//kleinste zulässige Pulslänge
    if (duration <= (32000)) {//größte zulässige Pulslänge, max = 32000
      sDuration = int(duration); //das wirft bereits hier unnütige Nullen raus und vergrößert den Wertebereich
    }else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (state) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafür gilt die gemessene Dauer.
      sDuration=sDuration*-1;
    }
    FiFo.addValue(&sDuration);
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


//================================= Kommandos ======================================
void IT_CMDs(String cmd);

void HandleCommand(String cmd)
{
  //int val;
  //String strVal;

  const char cmd_Version ='V';
  const char cmd_freeRam ='R';
  const char cmd_intertechno ='i';
  const char cmd_uptime ='t';
  const char cmd_changeReceiver ='X';
  const char cmd_space =' ';
  const char cmd_help='?';
  const char cmd_changeFilter ='F';

  // ?: Kommandos anzeigen




  if (cmd.charAt(0) == cmd_help) {
    //Serial.println(F("? Use one of V R i t X"));//FHEM Message
	Serial.print(cmd_help);	Serial.print(" Use one of ");
	Serial.print(cmd_Version);Serial.print(cmd_space);
	Serial.print(cmd_intertechno);Serial.print(cmd_space);
	Serial.print(cmd_freeRam);Serial.print(cmd_space);
	Serial.print(cmd_uptime);Serial.print(cmd_space);
	Serial.print(cmd_changeReceiver);Serial.print(cmd_space);
	Serial.print(cmd_changeFilter);Serial.print(cmd_space);
	Serial.println("");

  }
  // V: Version
  else if (cmd.charAt(0) == cmd_Version) {
    Serial.println("V " PROGVERS " SIGNALduino - compiled at " __DATE__ " " __TIME__);
  }
  // R: FreeMemory
  else if (cmd.charAt(0) == cmd_freeRam) {
    Serial.println(freeRam());
  }
  // i: Intertechno
  else if (cmd.charAt(0) == cmd_intertechno) {
    IT_CMDs(cmd);
  }
  // t: Uptime
  else if (cmd.charAt(0) == cmd_uptime) {
    // tbd
  }
  // XQ disable receiver
  else if (cmd.charAt(0) == cmd_changeReceiver) {
    changeReciver();
    //Serial.flush();
	//Serial.end();
  }
  else if (cmd.charAt(0) == cmd_changeFilter) {
    changeFilter();
  }

}


void IT_CMDs(String cmd) {

	// TODO: Change check to charAt, because we only need to check the 2. char here
  // Set Intertechno receive tolerance
  if (cmd.startsWith("it")) {
    char msg[3];
    cmd.substring(2).toCharArray(msg,3);
    ITreceivetolerance = atoi(msg);
    Serial.println(cmd);
  }
  // Set Intertechno Repetition
  else if (cmd.startsWith("ir")) {
    char msg[3];
    cmd.substring(2).toCharArray(msg,3);
    ITrepetition = atoi(msg);
    Serial.println(cmd);
  }
  // Switch Intertechno Devices
  else if (cmd.startsWith("is")) {
    digitalWrite(PIN_LED,HIGH);
    char msg[13];
    cmd.substring(2).toCharArray(msg,13);
    if (cmd.length() > 14)
    {
       ITbaseduration=cmd.substring(14).toInt(); // Default Baseduration
    }
    else
    {
       ITbaseduration=420; // Default Baseduration
    }
    sendPT2262(msg);
    digitalWrite(PIN_LED,LOW);
    Serial.println(cmd);
  }
  // Get Intertechno Parameters
  else if (cmd.startsWith("ip")) {
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
      HandleCommand(cmdstring);
      cmdstring = "";
      break;
    default:
      cmdstring += inChar;
    }
  }
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
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

void changeFilter()
{
	//cmdstring.concat(0);
	char tmp[10];

	cmdstring.toCharArray(tmp,10,1);

	const char *param = strtok(tmp,";");
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
}





