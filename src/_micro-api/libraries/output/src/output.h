// output.h
#ifndef _OUTPUT_h
#define _OUTPUT_h

/*
 * different debug options
 * DEBUG variable must be set separately if the library is used alone without SIGNALduino project
 * file output.h worked with DEBUG variable
 */

//#define DEBUG 0

/*
 * END debug options selection
*/

#ifdef CMP_CC1101
static const char TXT_CC1101[]          PROGMEM = "cc1101 ";
static const char TXT_CC110[]           PROGMEM = "CC110";
static const char TXT_CCFACTORYRESET[]  PROGMEM = "ccFactoryReset done";
static const char TXT_CCINIT[]          PROGMEM = "CCInit ";
static const char TXT_CCmode[]          PROGMEM = "ccmode";
static const char TXT_CHIP[]            PROGMEM = "chip";
static const char TXT_csPin[]           PROGMEM = "csPin";
static const char TXT_misoPin[]         PROGMEM = "misoPin";
static const char TXT_mosiPin[]         PROGMEM = "mosiPin";
static const char TXT_sckPin[]          PROGMEM = "sckPin";
#endif

#ifdef DEBUG
static const char TXT_CCPARTNUM[]       PROGMEM = "CCPartnum =";
static const char TXT_CCREVISION[]      PROGMEM = "CCVersion = ";
static const char TXT_VALUEFROM[]       PROGMEM = "Reading values from ";
#endif

static const char TXT_433[]             PROGMEM = "433 ";
static const char TXT_868[]             PROGMEM = "868 ";
static const char TXT_BLANK[]           PROGMEM = " ";
static const char TXT_COMMAND[]         PROGMEM = "command e";
static const char TXT_CORRUPT[]         PROGMEM = "corrupt";
static const char TXT_DOFRESET[]        PROGMEM = "is not correctly set for ASK/OOK. Please do a factory reset via ";
static const char TXT_DOT[]             PROGMEM = ".";
static const char TXT_EEPROM[]          PROGMEM = "EEPROM";
static const char TXT_EEPROMINIT[]      PROGMEM = "Init EEPROM to defaults after flash";
static const char TXT_EQ[]              PROGMEM = "=";
static const char TXT_FOUND[]           PROGMEM = "found ";
static const char TXT_FSEP[]            PROGMEM = ";";
static const char TXT_MC[]              PROGMEM = "MC";
static const char TXT_MHZ[]             PROGMEM = "Mhz";
static const char TXT_MS[]              PROGMEM = "MS";
static const char TXT_MU[]              PROGMEM = "MU";
static const char TXT_READ[]            PROGMEM = "read ";
static const char TXT_RECENA[]          PROGMEM = "receiver enabled" ;
static const char TXT_SENDCMD[]         PROGMEM = "send cmd ";
static const char TXT_TOLONG[]          PROGMEM = "to long ";
static const char TXT_TPATAB[]          PROGMEM = " to PATABLE done";
static const char TXT_UNSUPPORTED1[]    PROGMEM = "Unsupported short command";
static const char TXT_WRITE[]           PROGMEM = "write ";


#if !defined(ESP8266) && !defined(ESP32)

	#define FPSTR(s) ((__FlashStringHelper*)(s))

	#ifdef ARDUINO_RADINOCC1101
		#define portOfPin(P) \
		((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &PORTD : (((P) == 5 || (P) == 13) ? &PORTC : (((P) >= 18 && (P) <= 23)) ? &PORTF : (((P) == 7) ? &PORTE : &PORTB)))
		#define ddrOfPin(P) \
		((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &DDRD : (((P) == 5 || (P) == 13) ? &DDRC : (((P) >= 18 && (P) <= 23)) ? &DDRF : (((P) == 7) ? &DDRE : &DDRB)))
		#define pinOfPin(P) \
		((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &PIND : (((P) == 5 || (P) == 13) ? &PINC : (((P) >= 18 && (P) <= 23)) ? &PINF : (((P) == 7) ? &PINE : &PINB)))
		#define pinIndex(P) \
		(((P) >= 8 && (P) <= 11) ? (P) - 4 : (((P) >= 18 && (P) <= 21) ? 25 - (P) : (((P) == 0) ? 2 : (((P) == 1) ? 3 : (((P) == 2) ? 1 : (((P) == 3) ? 0 : (((P) == 4) ? 4 : (((P) == 6) ? 7 : (((P) == 13) ? 7 : (((P) == 14) ? 3 : (((P) == 15) ? 1 : (((P) == 16) ? 2 : (((P) == 17) ? 0 : (((P) == 22) ? 1 : (((P) == 23) ? 0 : (((P) == 24) ? 4 : (((P) == 25) ? 7 : (((P) == 26) ? 4 : (((P) == 27) ? 5 : 6 )))))))))))))))))))
	#else
	#ifndef ARDUINO_MAPLEMINI_F103CB
		#define portOfPin(P)\
		  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
		#define ddrOfPin(P)\
		  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
		#define pinOfPin(P)\
		  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
		#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
    #else
      #define pinAsInput(pin) pinMode(pin, INPUT)
      #define pinAsOutput(pin) pinMode(pin, OUTPUT)
      #define pinAsInputPullUp(pin) pinMode(pin, INPUT_PULLUP)
      #define digitalLow(pin) digitalWrite(pin, LOW)
      #define digitalHigh(pin) digitalWrite(pin, HIGH)
      #define isHigh(pin) (digitalRead(pin) == HIGH)
      #define isLow(pin) (digitalRead(pin) == LOW)
    #endif
	#endif

	#if defined(WIN32) || defined(__linux__)
		#define digitalLow(P) pinMode(P,LOW)
		#define digitalHigh(P) pinMode(P,HIGH)
	#else
		#ifndef ARDUINO_MAPLEMINI_F103CB
			#define pinMask(P)((uint8_t)(1<<pinIndex(P)))
			#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
			#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
			#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
			#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
			#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
			#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
			#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
			#define digitalState(P)((uint8_t)isHigh(P))
		#endif
	#endif
#else
	#define pinAsInput(pin) pinMode(pin, INPUT)
	#define pinAsOutput(pin) pinMode(pin, OUTPUT)
	#define pinAsInputPullUp(pin) pinMode(pin, INPUT_PULLUP)

	#ifndef digitalLow
		#define digitalLow(pin) digitalWrite(pin, LOW)
	#endif
	#ifndef digitalHigh
		#define digitalHigh(pin) digitalWrite(pin, HIGH)
	#endif
	#ifndef isHigh
		#define isHigh(pin) (digitalRead(pin) == HIGH)
	#endif
	#ifndef isLow
		#define isLow(pin) (digitalRead(pin) == LOW)
	#endif

#endif



#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
//	#include "WProgram.h"
#endif

#ifdef ETHERNET_PRINT
  #include <WiFiClient.h>
  extern WiFiClient serverClient;
  #define MSG_PRINTER serverClient
#else
  #define MSG_PRINTER Serial
#endif


#ifdef ETHERNET_DEBUG         // variable is not defined
  #define DBG_PRINTER Client  // now used ???
#else
  #define DBG_PRINTER Serial
#endif


#define MSG_PRINT(...) { MSG_PRINTER.print(__VA_ARGS__); }
#define MSG_PRINTLN(...) { MSG_PRINTER.println(__VA_ARGS__); }
#define MSG_WRITE(...) { MSG_PRINTER.write(__VA_ARGS__); }

#ifdef DEBUG
	#define DBG_PRINT(...) {  DBG_PRINTER.print(__VA_ARGS__); }
	#define DBG_PRINTLN(...) { DBG_PRINTER.println(__VA_ARGS__); }
#else
	#define DBG_PRINT(...) 
	#define DBG_PRINTLN(...) 
#endif



#endif
