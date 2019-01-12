#pragma once


// Config flags for compiling correct options / boards Define only one!
//#define CMP_CC1101
//#define ARDUINO_ATMEGA328P_MINICUL 1
//#define ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101 1;
//#define OTHER_BOARD_WITH_CC1101  1


//Enable debug option here:
//#define DEBUG





/*

  Do not Change anything below this line 

*/

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

#ifdef ESP8266

#endif

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
	#elif defined(ESP8266)
		#define PIN_RECEIVE            5// D1
		#define PIN_LED                16
		#define PIN_SEND               4// D2  // gdo0Pin TX out
		#define ETHERNET_PRINT
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




