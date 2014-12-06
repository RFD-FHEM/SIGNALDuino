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
#define PROGVERS               "2.5"

#define PIN_RECEIVE            2
#define PIN_LED                13 // Message-LED
#define PIN_SEND               11
#define BAUDRATE               9600

#include <filtering.h> //for FiFo Buffer
#include <TimerOne.h>  // Timer for LED Blinking
#include <bitstore.h>  // Die wird aus irgend einem Grund zum Compilieren benötigt.
#include <patternDecoder.h> //Logilink, IT decoder

RingBuffer FiFo(200, 0); // FiFo Puffer
const int pulseMin = 100;
bool blinkLED = false;
String cmdstring = "";

void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void blinken();


//Decoder
//SensorReceiver ASreceiver;
patternDecoder musterDec;
ManchesterpatternDetector ManchesterDetect(true);
ASDecoder   asDec  (&ManchesterDetect);
OSV2Decoder osv2Dec (&ManchesterDetect);



void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("Startup:");
  Serial.print("# Bytes / Puffer: ");
  Serial.println(sizeof(int)*FiFo.getBuffSize());
  Serial.print("# Len Fifo: ");
  Serial.println(FiFo.getBuffSize());

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
  static int aktVal;
  static bool state;
  while (FiFo.getNewValue(&aktVal)) { //Puffer auslesen und an Dekoder übergeben
    //Serial.print(aktVal); Serial.print(",");
    state = musterDec.decode(&aktVal); //Logilink, PT2262
    if (ManchesterDetect.detect(&aktVal))
    {
      state &= asDec.decode();
      state &= osv2Dec.decode();
      ManchesterDetect.reset();
    }
    if (state) blinkLED=true; //LED blinken, wenn Meldung dekodiert
  }
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
      sDuration = 32001; // Maximalwert
    }
    if (state) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafür gilt die gemessene Dauer.
      sDuration=sDuration*-1;
    }
    FiFo.addValue(&sDuration);
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
    Serial.println("? Use one of V i f d h t R q");//FHEM Message
  }
  // V: Version
  else if (cmd.startsWith("V")) {
    Serial.println(F("V " PROGVERS " FHEMduino - compiled at " __DATE__ " " __TIME__));
  }
  // R: FreeMemory
  else if (cmd.startsWith("R")) {
    Serial.println(F("R500"));// Fake Meldung
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

