// output.h
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


#ifndef _OUTPUT_h
#define _OUTPUT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#ifdef ETHERNET_PRINT
#include <wificlient.h>

extern WiFiClient serverClient;


#define MSG_PRINTER serverClient // Not Implemented at this time
#else
#define MSG_PRINTER Serial
#endif

#ifdef ETHERNET_DEBUG
#define DBG_PRINTER Client // Not Implemented at this time
#else
#define DBG_PRINTER Serial
#endif


#define MSG_PRINT(...) { MSG_PRINTER.print(__VA_ARGS__); }
#define MSG_PRINTLN(...) { MSG_PRINTER.println(__VA_ARGS__); }

#ifdef DEBUG
	#define DBG_PRINT(...) { DBG_PRINTER.print(__VA_ARGS__); }
	#define DBG_PRINTLN(...) { DBG_PRINTER.println(__VA_ARGS__); }
#else
	#define DBG_PRINT(...) 
	#define DBG_PRINTLN(...) 
#endif



#endif
