// output.h

#ifndef _OUTPUT_h
#define _OUTPUT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#ifdef ETHERNET_PRINT
#define MSG_PRINTER Client // Not Implemented at this time
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
#define DBG_PRINT(...) { DBG_PRINTER.print(__VA_ARGS__); }
#define DBG_PRINTLN(...) { DBG_PRINTER.println(__VA_ARGS__); }



#endif

