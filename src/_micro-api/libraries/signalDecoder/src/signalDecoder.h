/*
*   Pattern Decoder Library V3
*   Library to decode radio signals based on patternd detection
*   2014-2015  N.Butzek, S.Butzek
*   2015-2018  S.Butzek
*   2018-2020  S.Butzek, HomeAutoUser

*   This library contains different classes to perform detecting of digital signals
*   typical for home automation. The focus for the moment is on different sensors
*   like weather sensors (temperature, humidity Logilink, TCM, Oregon Scientific, ...),
*   remote controlled power switches (Intertechno, TCM, ARCtech, ...) which use
*   encoder chips like PT2262 and EV1527-type and manchester encoder to send
*   information in the 433MHz or 868 Mhz Band.
*
*   The classes in this library follow the approach to detect a recurring pattern in the
*   recived signal. For Manchester there is a class which decodes the signal.
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


#ifndef _SIGNALDECODER_h
#define _SIGNALDECODER_h


#if defined(WIN32) || defined(__linux__)  /* ** for which variant or system is this required? ** */
  #define ARDUINO 101
  #define NOSTRING
#endif

#if defined(__linux__)
  #include <stdlib.h>
#endif

#ifndef DEC
  #define DEC 10
#endif

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  //#include "WProgram.h"
#endif


/*
 * different debug options
 * DEBUG variable must be set separately if the library is used alone without SIGNALduino project
 * file signalDecoder.h worked with DEBUG, DEBUGDETECT, DEBUGDECODE variable
 */

//#define DEBUG 0

/* all variations DEBUGDETECT | output serial */
//#define DEBUGDETECT 0
//#define DEBUGDETECT 1
//#define DEBUGDETECT 2
//#define DEBUGDETECT 3
//#define DEBUGDETECT 255    /* very verbose output */ // ->> ESP8266, >>>stack>>> error

/* all variations DEBUGDECODE | output serial */
//#define DEBUGDECODE 1
//#define DEBUGDECODE 255    /* very verbose output */

/*
 * END debug options selection
*/

#if defined(DEBUGDETECT) || defined(DEBUGDECODE)
  #define DEBUG 1
#endif

#ifdef _COMPILE_CONFIG_h                      /* to break Dependency, _COMPILE_CONFIG_h is only available in the SIGNALduino project */
  #ifdef PLATFORMIO                           /* intern variable only in software PlatformIO (example 40304), in Arduino IDE undef */
    #include "../../../../compile_config.h"   /* PlatformIO   - need for right options in output.h */
  #else
    #include "compile_config.h"               /* Arduino IDE  - need for right options in output.h */
  #endif
#endif

#ifndef WIFI_ESP              /* variable is not defined, where is this used ??? */
  #include "output.h"         /* Dependency is needed */
#else
/*
 * this code no needed, variable WIFI_ESP not defined
 * 
 * #include <ESP8266WiFi.h>
 * extern WiFiClient serverClient;
 * #define DBG_PRINTER Serial
 * #define DBG_PRINT(...) { DBG_PRINTER.print(__VA_ARGS__); }
 * #define DBG_PRINTLN(...) { DBG_PRINTER.println(__VA_ARGS__); }
 */
#endif

/* for Unittest - output object over callback */
#define SDC_PRINT(...)    write(__VA_ARGS__)
#define SDC_WRITE(b)      write((const uint8_t*)b,(uint8_t) 1) 
#define SDC_PRINTLN(...)  write(__VA_ARGS__); write(char(0xA));

#ifndef F 
  #define F(V1) V1
#endif

#include "bitstore.h"         /* Dependency is needed */
#include "FastDelegate.h"     /* Dependency is needed */

#define maxNumPattern 8
#define maxMsgSize 254
#define minMessageLen 40
#define syncMinFact 6
#define syncMaxFact 44
#define syncMaxMicros 17000
#define maxPulse 32001  // Magic Pulse Length

constexpr const uint8_t SERIAL_DELIMITER = 59;
constexpr const uint8_t MSG_START = 2;
constexpr const uint8_t MSG_END = 3;

//#define SERIAL_DELIMITER  59 //char(';')
//#define MSG_START char(0x2)		// this is a non printable Char
//#define MSG_END   char(0x3)			// this is a non printable Char

enum status { searching, clockfound, syncfound, detecting, mcdecoding };

char* myitoa(int num, char* str);   // selfmade myitoa function

class ManchesterpatternDecoder;
class SignalDetectorClass;

class SignalDetectorClass
{
	friend class ManchesterpatternDecoder;

public:
	SignalDetectorClass() : first(buffer), last(nullptr), message(4) { 
																		 buffer[0] = 0; reset(); mcMinBitLen = 17; 	
																		 MsMoveCount = 0; 
																		 MredEnabled = 1;      // 1 = compress printmsg 
																		 mcdecoder = nullptr;
																		};

	void reset();
	bool decode(const int* pulse);
	const status getState();
	typedef fastdelegate::FastDelegate0<uint8_t> FuncRetuint8t;
	typedef fastdelegate::FastDelegate2<const uint8_t*, uint8_t, size_t> Func2pRetuint8t;

	void setRSSICallback(FuncRetuint8t callbackfunction) { _rssiCallback = callbackfunction; }
	void setStreamCallback(Func2pRetuint8t callbackfunction) { _streamCallback = callbackfunction; }

	//private:
  void SDC_PRINT_intToHex(unsigned int numberToPrint);

	int8_t clock;                           // index to clock in pattern
	bool MUenabled;
	bool MCenabled;
	bool MSenabled;
	bool MredEnabled;                       // 1 = compress printMsgRaw
	uint8_t MsMoveCount;

	uint8_t histo[maxNumPattern];
	//uint8_t message[maxMsgSize];
	ManchesterpatternDecoder *mcdecoder;  // Pointer to mcdecoder object

	uint8_t messageLen;					  // Todo, kann durch message.valcount ersetzt werden
	uint8_t mstart;						  // Holds starting point for message
	uint8_t mend;						  // Holds end point for message if detected
	bool success;                         // True if a valid coding was found
	bool m_truncated;					// Identify if message has been truncated
	bool m_overflow;
	void bufferMove(const uint8_t start);

	uint16_t tol;                           // calculated tolerance for signal
											//uint8_t bitcnt;
	status state;                           // holds the status of the detector
	int buffer[2];                          // Internal buffer to store two pules length
	int* first;                             // Pointer to first buffer entry
	int* last;                              // Pointer to last buffer entry
	BitStore<maxMsgSize / 2> message;       // A store using 4 bit for every value stored. 
	float tolFact;                          //
	int pattern[maxNumPattern];				// 1d array to store the pattern
	uint8_t patternLen;                     // counter for length of pattern
	uint8_t pattern_pos;
	int8_t sync;							// index to sync in pattern if it exists
											//String preamble;
											//String postamble;
	bool mcDetected;						// MC Signal alread detected flag
	uint8_t mcMinBitLen;					// min bit Length
	uint8_t rssiValue=0;					// Holds the RSSI value retrieved via a rssi callback
	FuncRetuint8t _rssiCallback= nullptr;	// Holds the pointer to a callback Function
	Func2pRetuint8t _streamCallback=nullptr;// Holds the pointer to a callback Function
	//Stream * msgPort;						// Holds a pointer to a stream object for outputting


	void addData(const int8_t value);
	void addPattern();
	inline void updPattern(const uint8_t ppos);

	void doDetect();
	void processMessage();
	void compress_pattern();
	void calcHisto(const uint8_t startpos = 0, uint8_t endpos = 0);
	bool getClock(); // Searches a clock in a given signal
	bool getSync();	 // Searches clock and sync in given Signal
	//int8_t printMsgRaw(uint8_t m_start, const uint8_t m_end, const String *preamble = NULL, const String *postamble = NULL);
	//void printMsgStr(const String *first, const String *second, const String *third);
	const bool inTol(const int val, const int set, const int tolerance); // checks if a value is in tolerance range

	void printOut();
	const size_t write(const uint8_t *buffer, size_t size);   // for the return value
	const size_t write(const char *str);                      // for the return value
	const size_t write(uint8_t b);                            // for the return value

	int8_t findpatt(const int val);              // Finds a pattern in our pattern store. returns -1 if te pattern is not found
												 //bool validSequence(const int *a, const int *b);     // checks if two pulses are basically valid in terms of on-off signals
	const bool checkMBuffer(const uint8_t begin = 0);
};


class ManchesterpatternDecoder
{
public:
	ManchesterpatternDecoder(SignalDetectorClass *ref_dec) : ManchesterBits(1), longlow(-1), longhigh(-1), shorthigh(-1), shortlow(-1) { pdec = ref_dec; 	reset(); };
	~ManchesterpatternDecoder();
	const bool doDecode();
	void setMinBitLen(const uint8_t len);
#ifdef NOSTRING
	const char* getMessageHexStr();
	const char* getMessagePulseStr();
	const char* getMessageClockStr();
	const char* getMessageLenStr();
#else
	void getMessageHexStr(String *message);
	void getMessagePulseStr(String *str);
	void getMessageClockStr(String* str);
	void getMessageLenStr(String* str);
#endif
	void printMessageHexStr();
	char nibble_to_HEX(uint8_t nibble);
	void HEX_twoDigits(char* cbuffer, uint8_t val);

	const bool isManchester();
	void reset();
#ifndef UNITTEST
	//private:
#endif
	BitStore<50> ManchesterBits;       // A store using 1 bit for every value stored. It's used for storing the Manchester bit data in a efficent way
	SignalDetectorClass *pdec;
	int8_t longlow;
	int8_t longhigh;
	int8_t shorthigh;
	int8_t shortlow;
	int clock; // Manchester calculated clock		
	int8_t minbitlen;

	bool mc_start_found = false;
	bool mc_sync = false;

	const bool isLong(const uint8_t pulse_idx);
	const bool isShort(const uint8_t pulse_idx);
	unsigned char getMCByte(const uint8_t idx); // Returns one Manchester byte in correct order. This is a helper function to retrieve information out of the buffer
};

#endif
