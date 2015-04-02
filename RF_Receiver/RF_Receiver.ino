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
#define PROGVERS               "2.5txtst"

#define PIN_RECEIVE            2
#define PIN_LED                13 // Message-LED
#define PIN_SEND               11
#define BAUDRATE               57600
#define FIFO_LENGTH			   400
//#define TX_TST
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

void IT_TX(unsigned int duration);
void receiveProtocolIT_TX(unsigned int changeCount);

//volatile int fifocnt=0;

//Decoder
//SensorReceiver ASreceiver;
patternDecoder musterDec;
ManchesterpatternDetector ManchesterDetect(true);
ASDecoder   asDec  (&ManchesterDetect);
OSV2Decoder osv2Dec (&ManchesterDetect);



void setup() {
  Serial.begin(BAUDRATE);
#ifdef DEBUG
  Serial.println(F("Startup:"));
  Serial.print(F("# Bytes / Puffer: "));
  Serial.println(sizeof(int)*FiFo.getBuffSize());
  Serial.print(F("# Len Fifo: "));
  Serial.println(FiFo.getBuffSize());
#endif
  delay(2000);
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
  //Serial.print("fcnt: ");   Serial.println(fifocnt);


  static int aktVal;
  static bool state;
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
  int val;
  String strVal;

  // ?: Kommandos anzeigen
  if (cmd.equals("?")) {
    Serial.println(F("? Use one of V i f d h t R q"));//FHEM Message
  }
  // V: Version
  else if (cmd.startsWith("V")) {
    Serial.println("V " PROGVERS " SIGNALduino - compiled at " __DATE__ " " __TIME__);
  }
  // R: FreeMemory
  else if (cmd.startsWith("R")) {
    Serial.println(freeRam());// Fake Meldung
  }
  // i: Intertechno
  else if (cmd.startsWith("i")) {
    IT_CMDs(cmd);
  }
  // t: Uptime
  else if (cmd.startsWith("t")) {
    // tbd
  }
  else if (cmd.startsWith("XQ")) {
    disableReceive();
//    Serial.flush();
//    Serial.end();
  }
}

void IT_CMDs(String cmd) {

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



#ifdef TX_TST

/*-----------------------------------------------------------------------------------------------
/* Devices with temperatur / humidity functionality => Intertechno TX2/3/4
-----------------------------------------------------------------------------------------------*/

void dump_buffer(bool force,unsigned int changeCount)
{
  if (force || changeCount > 1 && changeCount%(FIFO_LENGTH-10) == 0)
  {
    FiFo.setFReadPointerToRead(FIFO_LENGTH*-1); //Freien Lesezeiger an den Anfang positionieren
	Serial.println("RAW IT_TX Output: (");
	int *buffVal;
	for (int i = 1; i <= FIFO_LENGTH; ++i) {
      buffVal = FiFo.getNextValue();
      Serial.print(*buffVal);
      Serial.print(", ");
    }
	Serial.println(")"); // End of output
  }

}


#define TX_MAX_CHANGES 88
unsigned int timingsTX[TX_MAX_CHANGES+20];      //  TX

// http://www.f6fbb.org/domo/sensors/tx3_th.php
void IT_TX(unsigned int duration) {

  static unsigned int changeCount;
  static unsigned int fDuration;
  static unsigned int sDuration;

  sDuration = fDuration + duration;

	/* Dump out the Buffer if received Buffer-10 valid TX Pulses to get the sync signal */

  if ((sDuration > 1550 - 200 && sDuration < 1550 + 200) || (sDuration > 2150 - 200 && sDuration < 2150 + 200)) {
    if ((duration > 520 - 100 && duration < 520 + 100) || (duration > 1250 - 100 && duration < 1250 + 100) || (duration > 950 - 100 && duration < 950 + 100)) {
      if (changeCount == 0 ) {
        timingsTX[changeCount++] = fDuration;
      }
      timingsTX[changeCount++] = duration;
    }

    if ( changeCount == TX_MAX_CHANGES - 1) {
	  dump_buffer(true,changeCount);
      receiveProtocolIT_TX(changeCount);
      changeCount = 0;
      fDuration = 0;
    }
  } else {
    changeCount = 0;
  }
  dump_buffer(false, changeCount);
  fDuration = duration;
}


void receiveProtocolIT_TX(unsigned int changeCount) {
#define TX_ONE    520
#define TX_ZERO   1250
#define TX_GLITCH  100
#define TX_MESSAGELENGTH 44

  byte i;
  unsigned long code = 0;

  String message = "TX";

  for (i = 0; i <= 14; i = i + 2)
  {
    if ((timingsTX[i] > TX_ZERO - TX_GLITCH) && (timingsTX[i] < TX_ZERO + TX_GLITCH))    {
      code <<= 1;
    }
    else if ((timingsTX[i] > TX_ONE - TX_GLITCH) && (timingsTX[i] < TX_ONE + TX_GLITCH)) {
      code <<= 1;
      code |= 1;
    }
    else {
      return;
    }
  }

  // Startsequence 0000 1010 = 0xA
  if (code != 10) {
    return;
  }

  message += String(code,HEX);
  code = 0;

  // Sensor type 0000 = Temp / 1110 = Humidity
  for (i = 16; i <= 22; i = i + 2)
  {
    if ((timingsTX[i] > TX_ZERO - TX_GLITCH) && (timingsTX[i] < TX_ZERO + TX_GLITCH))    {
      code <<= 1;
    }
    else if ((timingsTX[i] > TX_ONE - TX_GLITCH) && (timingsTX[i] < TX_ONE + TX_GLITCH)) {
      code <<= 1;
      code |= 1;
    }
    else {
      return;
    }
  }

  message += String(code,HEX);
  code = 0;

  // Sensor adress
  for (i = 24; i <= 38; i = i + 2)
  {
    if ((timingsTX[i] > TX_ZERO - TX_GLITCH) && (timingsTX[i] < TX_ZERO + TX_GLITCH))    {
      code <<= 1;
    }
    else if ((timingsTX[i] > TX_ONE - TX_GLITCH) && (timingsTX[i] < TX_ONE + TX_GLITCH)) {
      code <<= 1;
      code |= 1;
    }
    else {
      return;
    }
  }
  message += String(code,HEX);
  code = 0;

  // Temp or Humidity
  for (i = 40; i <= 62; i = i + 2)
  {
    if ((timingsTX[i] > TX_ZERO - TX_GLITCH) && (timingsTX[i] < TX_ZERO + TX_GLITCH))    {
      code <<= 1;
    }
    else if ((timingsTX[i] > TX_ONE - TX_GLITCH) && (timingsTX[i] < TX_ONE + TX_GLITCH)) {
      code <<= 1;
      code |= 1;
    }
    else {
      return;
    }
  }
  message += String(code,HEX);
  code = 0;

  // Repeated Bytes temp / Humidity
  for (i = 64; i <= 78; i = i + 2)
  {
    if ((timingsTX[i] > TX_ZERO - TX_GLITCH) && (timingsTX[i] < TX_ZERO + TX_GLITCH))    {
      code <<= 1;
    }
    else if ((timingsTX[i] > TX_ONE - TX_GLITCH) && (timingsTX[i] < TX_ONE + TX_GLITCH)) {
      code <<= 1;
      code |= 1;
    }
    else {
      return;
    }
  }
  message += String(code,HEX);
  code = 0;

  // Checksum
  for (i = 80; i <= changeCount; i = i + 2)
  {
    if ((timingsTX[i] > TX_ZERO - TX_GLITCH) && (timingsTX[i] < TX_ZERO + TX_GLITCH))    {
      code <<= 1;
    }
    else if ((timingsTX[i] > TX_ONE - TX_GLITCH) && (timingsTX[i] < TX_ONE + TX_GLITCH)) {
      code <<= 1;
      code |= 1;
    }
    else {
      return;
    }
  }
  message += String(code,HEX);
  message.toUpperCase();
  Serial.println(message);

  return;
}
#endif

