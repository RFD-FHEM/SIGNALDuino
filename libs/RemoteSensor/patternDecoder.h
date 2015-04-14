/*
*   Pattern Decoder Library V0.1
*   Library to decode radio signals based on patternd detection
*   2014  N.Butzek, S.Butzek

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
#include <filtering.h>

const uint8_t maxNumPattern=6;
const uint8_t maxMsgSize=30;
const uint8_t minMessageLen=40;
const uint8_t syncMinFact=9;
const uint8_t prefixLen=3;

const int16_t maxPulse = 32001;  // Magic Pulse Length


const char SERIAL_DELIMITER =';';
const char MSG_START =0x2;			// this is a non printable Char
const char MSG_END =0x3;			// this is a non printable Char

//#define DEBUGDETECT 0
//#define DEBUGDETECT 255  // Very verbose output
//#define DEBUGDECODE 1

#define PATTERNSIZE 2

#define DEBUG_BEGIN(i) Serial.print(F("(D:"));Serial.print(i);
#define DEBUG_END Serial.println(F(")"));

//#define DEBUG

// Message Type
enum mt {twostate,tristate,undef};

// Struct for signal identificaion
 struct s_sigid {
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



/*
 base class for pattern detector subclasses. Containing only the toolset used to define a pattern detector
*/
 class patternBasic {
    public:
        patternBasic();
		virtual bool detect(int* pulse);        // Runs the detection engine, must be implemented in child class
		void reset();                           // resets serval internal vars to start with a fresh pattern
		void swap(int* a, int* b);              // Swaps the two vars
		int8_t findpatt(int *seq);              // Finds a pattern in our pattern store. returns -1 if te pattern is not found
		bool inTol(int val, int set);           // checks if a value is in tolerance range
		bool inTol(int val, int set, int tolerance);
        bool validSequence(int *a, int *b);     // checks if two pulses are basically valid in terms of on-off signals

		enum status {searching, clockfound, syncfound,detecting};

        virtual void doSearch();                // Virtual class which must be implemented in a child class
		virtual void doDetect();                // Virtual class which must be implemented in a child class
		virtual void processMessage();          // Virtual class which must be implemented in a child class

		uint16_t clock;                         // calculated clock of signal
    protected:
		int buffer[2];                          // Internal buffer to store two pules length
        int* first;                             // Pointer to first buffer entry
		int* last;                              // Pointer to last buffer entry
		uint16_t tol;                                // calculated tolerance for signal
        float tolFact;                          //
		int pattern[maxNumPattern][PATTERNSIZE];// 2d array to store the pattern
		BitStore *patternStore;                 // Store for saving detected bits or pattern index
		uint8_t patternLen;                     // counter for length of pattern
		status state;                           // holds the status of the detector
		bool success;                           // True if a valid coding was found
};



/*
 Class currently not based on patternBacis, to detect simple on-off signals
 Todo: Make it child from patternBasic
*/
class patternDetector : protected patternBasic {

	public:
		//enum status {searching, detecting};
		patternDetector();
		bool detect(int* pulse);
		void doDetectwoSync();
		void reset();
		bool getSync();	 // Searches clock and sync in given Signal
		bool getClock(); // Searches a clock in a given signal
		virtual void processMessage();
		int8_t getPatternIndex(int key);

		//void swap(int* a, int* b);
		void sortPattern();

		void printOut();
		void calcHisto();
        bool isPatternInMsg(int *key);

		//void ArraySort(int arr[maxNumPattern][PATTERNSIZE], int n);
		//int pattern[maxNumPattern*2];
		int sync;
		//uint8_t patternLen;
		//int syncFact;
		uint8_t bitcnt;
		uint8_t message[maxMsgSize*8];
		uint8_t messageLen;
		/*
		int buffer[2];
		int* first;
		int* last;
		uint16_t clock;
		int tol;
		status state;
		bool success;
		float tolFact;
		*/
	    uint8_t histo[maxNumPattern];
	    uint8_t mstart; // Temp Variable zum Testen
    	s_sigid protoID[10]; // Speicher für Protokolldaten
    	uint8_t numprotos;// Number of protocols in protoID

};


/*
 Decoder class for some on-off protocols
 currently implemented as cild of the detector.
*/
class patternDecoder : public patternDetector{
	public:
		patternDecoder();

		uint8_t twoStateMessageBytes(const s_pidx s_patt);
		void twoStateMessageBytes() {};
		void triStateMessageBytes();

		void printMsgRaw(uint8_t start, uint8_t end,String *preamble=NULL,String *postamble=NULL);
		void printMessageHexStr();
		uint8_t printTristateMessage(const s_pidx s_patt);
		void printNewITMessage();

		bool decode(int* pulse);

		void processMessage();

		bool checkSignal(const s_sigid s_signal);
		bool checkEV1527type(s_sigid match);
		void checkLogilink();
		void checkITold();
		void checkITautolearn();
		void checkAS();
		void checkTCM97001();

		byte byteMessage[maxMsgSize];
		byte byteMessageLen;

};

/*
    Detector for manchester Signals. It uses already the toolset from patternBasic.
    Todo:  Make a interface for retrieving the Manchesterbits easily for furher processing in other classes.
 */
class ManchesterpatternDetector : public patternBasic {
	public:
		ManchesterpatternDetector(bool silentstate=true);
        bool detect(int* pulse);        // Runs the detection engine, It's mainly the only function which is calles from ourside. Input is one pule
        void reset();                   // Resets servals vars to start with a fresh engine
        void printMessageHexStr();      // Gehört eigenlich in die Decoder Klasse
        bool manchesterfound();         // returns true if the detection engine has found a manchester sequence. Returns true not bevore other signals will be processed
        unsigned char getMCByte(uint8_t idx); // Returns one Manchester byte in correct order. This is a helper function to retrieve information out of the buffer
        BitStore *ManchesterBits;       // A store using 1 bit for every value stored. It's used for storing the Manchester bit data
	private:
		bool isShort(int *pulse);       // Returns true if it's a short pulse
		bool isLong(int *pulse);        // Returns true if it's a long pulse

        void doSearch();                // Seatchs for a valid manchester sequence
		void updateClock(int *pulse);   // Updates the clock with given pulse
		void updateClock(int newClock); // Updates clock with the provided newClock and sets clock to the new average

		void doDetect();                // Detect the manchester signal and store the detected manchester bits
        void processMessage();          // Checks if there are enough bits to have a message
        void printOut();                // Prints out the detected message, mor for debug purpose


        unsigned char reverseByte(unsigned char original);  // Not needed anymore
   		//uint8_t message[maxMsgSize*8];
		bool silent;                    // True to suspress output, which is the default



};

/*
 base class for decoding subclasses. Containing only the toolset used to decode a signal
*/

class decoderBasic {
    public:
        decoderBasic(); 								// Constructor
  		String getMessageHexStr() const;             	// Returns the hex message on serial
		bool decode();                        			// Currently not implemented, must be defined in the subclass

		virtual bool processMessage() {} ;              // Currently not implemented, must be defined in the subclass
	protected:
		bool message_decoded;							// Is set to true if we have decoded a valid signal
		ManchesterpatternDetector *mcdetector;
    	String protomessage;							// Holds the message from the protocol (hex)
        bool checkMessage(uint16_t min_clock, uint16_t max_clock, uint8_t min_Length,uint8_t max_Length);			 // Checks the pattern data against protocol
        bool checkSync(unsigned char pattern, uint8_t startpos, uint8_t mincount,uint8_t maxcount,uint8_t *syncend); // Checks two bits against pattern beginning at startpos returns last position at syncend
        unsigned char getNibble(uint8_t startingPos);           													 // returns data bits 4 bits (nibble) in received order
        unsigned char getDataBits(uint8_t startingPos,uint8_t numbits); 											 // returns data bits numbits bits (max 8) in received order
		String valToHex(unsigned char Value, uint8_t desiredStringLength=2);													 // returns a hex string with padded 0 up to desired length
};


/*
 Class for decoding oregon scientific protocol from a manchester bit signal.
 Currently only working if there is a delay between two transmissions of a signal
*/
class OSV2Decoder : public decoderBasic {
    public:
        OSV2Decoder(ManchesterpatternDetector *detector);
	private:
        bool processMessage();                                  // Checks the Protocol
        unsigned char getNibble(uint8_t startingPos);           // returns data bits 4 bits (nibble) in correct order
        unsigned char getByte(uint8_t startingPos);				// returns data bits 8 bits (byte) in correct order
		bool checkMessage();									// Verify if we have OSV2 Message Type
		uint8_t syncend;										// Variable to hold the last syncend counter
		unsigned char getDataBits(uint8_t startingPos,uint8_t numbits);


};

/*
 Class for decoding Arduinosensor protocol from a manchester bit signal.
*/
class ASDecoder : public decoderBasic {
    public:
        ASDecoder(ManchesterpatternDetector *detector);
	private:
        bool processMessage();                              // Check, conversion and outpuz
   		bool checkMessage();									// Verify if we have OSV2 Message Type
		uint8_t syncend;										// Variable to hold the last syncend counter
};


#endif //PATTERNDEC_H
