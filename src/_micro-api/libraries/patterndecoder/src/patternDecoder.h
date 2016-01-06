/*
*   Pattern Decoder Library V3
*   Library to decode radio signals based on patternd detection
*   2014  N.Butzek, S.Butzek
*   2015  S.Butzek

*   This library contains different classes to perform decoding of radio signals
*   typical for home automation. The focus for the moment is on different sensors
*   like weather sensors (temperature, humidity Logilink, TCM, Oregon Scientific, ...),
*   remote controlled power switches (Intertechno, TCM, ARCtech, ...) which use
*   encoder chips like PT2262 and EV1527-type and manchester encoder to send
*   information in the 433MHz Band.
*
*   The classes in this library follow the approach to detect a typical pattern in the
*   first step and to try to find sensor (sender) specific information in the pattern in the
*   second step. The decoded information is send to the house automation fhem to be
*   processed there.
*   As everything that you find in here is in an early state, please consider, that
*   the purpose of the library is to provide a startingpoint for experiments and
*   discussions rather than to provide a ready to use functionality.
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

#ifndef PATTERNDEC_H
#define PATTERNDEC_H

#include "Arduino.h"
#include <bitstore.h>
//#include <filtering.h>

#define maxNumPattern 6
#define maxMsgSize 30
#define minMessageLen 40
#define syncMinFact 7
#define syncMaxFact 39
#define syncMaxMicros 17000

//#define prefixLen 3

#define maxPulse 32001  // Magic Pulse Length


#define SERIAL_DELIMITER ';'
#define MSG_START char(0x2)			// this is a non printable Char
#define MSG_END char(0x3)			// this is a non printable Char

//#define DEBUGDETECT 3
//#define DEBUGDETECT 255  // Very verbose output
//#define DEBUGDECODE 0

#define DEBUG_BEGIN(i) Serial.print(F("(D:"));Serial.print(i);
#define DEBUG_END Serial.println(F(")"));

//#define DEBUG

// Message Type
//enum mt {twostate,tristate,undef};
enum status {searching, clockfound, syncfound,detecting};

// Struct for signal identificaion
/* struct s_sigid {
	int8_t lowFact;			// not used
	int8_t highFact;		// not used
	int8_t syncFact;		// used
	int clock;				// used
	uint8_t len;			// not used
	mt messageType;			// not used
};

// Struct for reference to pattern low-, highfact and clock index
 struct s_pidx {							// Not used anymore
	int8_t lf_idx;	// low fact
	int8_t hf_idx;	// High fact
	int8_t ck_idx;	// Clock
	int8_t sc_idx;  // Sync

};

*/

/*
 base class for pattern detector subclasses. Containing only the toolset used to define a pattern detector
*/
 class patternBasic {
    public:
        patternBasic() : patternStore(1), success(false) ,first(buffer), last(first+1){	buffer[0] = buffer[1] = 0; reset(); };
		virtual bool detect(const int* pulse);        // Runs the detection engine, must be implemented in child class
		void reset();                           // resets serval internal vars to start with a fresh pattern
		void swap(int* a, int* b);              // Swaps the two vars
		int8_t findpatt(const int val);              // Finds a pattern in our pattern store. returns -1 if te pattern is not found
		bool inTol(const int val, const int set);           // checks if a value is in tolerance range
		static bool inTol(const int val, const int set, const int tolerance);
        bool validSequence(const int *a, const int *b);     // checks if two pulses are basically valid in terms of on-off signals

        virtual void doSearch();                // Virtual class which must be implemented in a child class
		virtual void doDetect();                // Virtual class which must be implemented in a child class
		virtual void processMessage();          // Virtual class which must be implemented in a child class
		int8_t clock;                           // index to clock in pattern

    protected:
		status state;                           // holds the status of the detector

		int buffer[2];                          // Internal buffer to store two pules length
        int* first;                             // Pointer to first buffer entry
		int* last;                              // Pointer to last buffer entry
		uint16_t tol;                                // calculated tolerance for signal
        float tolFact;                          //
		int pattern[maxNumPattern];				// 1d array to store the pattern
		BitStore<0> patternStore;                 // Store for saving detected bits or pattern index
		uint8_t patternLen;                     // counter for length of pattern
		bool success;                           // True if a valid coding was found

};



/*
 Class currently not based on patternBacis, to detect simple on-off signals
 Todo: Make it child from patternBasic
*/
class patternDetector : protected patternBasic {

	public:
		patternDetector();
		bool detect(const int* pulse);
		void doDetectwoSync();
		void reset();
		bool getSync();	 // Searches clock and sync in given Signal
		bool getClock(); // Searches a clock in a given signal
		bool validSignal();						// Checks if stored message belongs to a validSignal
		virtual void processMessage();
		const status getState();
		void compress_pattern();

		void printOut();
		void calcHisto(const uint8_t startpos=0, uint8_t endpos=0);
		int8_t sync;                        // index to sync in pattern if it exists
		uint8_t bitcnt;
		uint8_t message[maxMsgSize*8];
		uint8_t messageLen;
  		bool m_truncated;     // Identify if message has been truncated
		bool m_overflow;

	    uint8_t histo[maxNumPattern];
	    uint8_t mstart; // Holds starting point for message
	    uint8_t mend; // Holds end point for message if detected

//    	s_sigid protoID[10]; // decrepated
//    	uint8_t numprotos;// decrepated

};


/*
 Decoder class for some on-off protocols
 currently implemented as cild of the detector.
*/
class patternDecoder : public patternDetector{
	friend class ManchesterpatternDecoder;
	public:
        patternDecoder() : patternDetector(), MUenabled(true), MCenabled(true), MSenabled(true) { preamble.reserve(70); postamble.reserve(20); reset(); };

		void reset();

		bool decode(const int* pulse);
		void processMessage();

		void printMsgStr(const String *first, const String *second, const String *third);
		int8_t printMsgRaw(uint8_t start, const uint8_t end,const String *preamble=NULL,const String *postamble=NULL);

		bool MUenabled;
		bool MCenabled;
		bool MSenabled;

	private:
		String preamble;
		String postamble;

		//uint8_t byteMessage[maxMsgSize];
		//byte byteMessageLen;

};


class ManchesterpatternDecoder
{
	public:
	ManchesterpatternDecoder(patternDecoder *ref_dec) : ManchesterBits(1), longlow(-1),longhigh(-1),shorthigh(-1),shortlow(-1) {	pdec = ref_dec; 	reset(); };
	~ManchesterpatternDecoder();
	bool doDecode();
	void setMinBitLen(uint8_t len);
    void getMessageHexStr(String *message);
    void getMessagePulseStr(String *str);
    void getMessageClockStr(String* str);
    bool isManchester();
	void reset();

	private:
	bool isLong(uint8_t pulse_idx);
	bool isShort(uint8_t pulse_idx);
	unsigned char getMCByte(uint8_t idx); // Returns one Manchester byte in correct order. This is a helper function to retrieve information out of the buffer

    BitStore<30> ManchesterBits;       // A store using 1 bit for every value stored. It's used for storing the Manchester bit data in a efficent way

    patternDecoder *pdec;

    int8_t longlow;
    int8_t longhigh;
    int8_t shortlow;
    int8_t shorthigh;
    int clock;						    // Manchester calculated clock
//    int dclock;						// Manchester calculated double clock

	uint8_t minbitlen;
};






#endif //PATTERNDEC_H
