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

#define maxNumPattern 4
#define maxMsgSize 30

#define minMessageLen 20
#define syncMinFact 9

#define prefixLen 3

//#define DEBUGDETECT 1
//#define DEBUGDETECT 255  // Very verbose output
#define DEBUGDECODE 255


#define PATTERNSIZE 2

#define DEBUG_BEGIN(i) Serial.print("(D:");Serial.print(i);
#define DEBUG_END Serial.println(")");

//#define DEBUG

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
        bool validSequence(int *a, int *b);     // checks if two pulses are basically valid in terms of on-off signals

		enum status {searching, detecting};

        virtual void doSearch();                // Virtual class which must be implemented in a child class
		virtual void doDetect();                // Virtual class which must be implemented in a child class
		virtual void processMessage();          // Virtual class which must be implemented in a child class

		uint16_t clock;                              // calculated clock of signal
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
class patternDetector {

	public:
		enum status {searching, detecting};
		patternDetector();
		bool detect(int* pulse);
		void doSearch();
		void doDetect();
		void reset();
		virtual void processMessage();

		void swap(int* a, int* b);
		void sortPattern();

		int find();
		bool inTol(int val, int set);
		bool inTol(int val, int set, int tolerance);
		void printOut();

		int pattern[maxNumPattern*2];
		int sync;
		uint8_t patternLen;
		//int syncFact;
		uint8_t bitcnt;
		uint8_t message[maxMsgSize*8];
		byte messageLen;
		int buffer[2];
		int* first;
		int* last;
		uint16_t clock;
		int tol;
		status state;
		bool success;
		float tolFact;
};


/*
 Decoder class for some on-off protocols
 currently implemented as cild of the detector.
*/
class patternDecoder : public patternDetector {
	public:
		patternDecoder();

		void twoStateMessageBytes();
		void triStateMessageBytes();

		void printMessageHexStr();
		void printTristateMessage();
		void printNewITMessage();

		bool decode(int* pulse);

		void processMessage();
		bool checkEV1527type(int clockTst, int syncFact, int lowFact, int highFact, byte Length);
		void checkLogilink();
		void checkITold();
		void checkITautolearn();
		void checkAS();

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
		void updateClock(int *pulse);   // Updates the clock
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

class decoderBacis {
    public:
        decoderBacis(); 								// Constructor
  		String getMessageHexStr() const;             	// Returns the hex message on serial
		bool decode();                        			// Currently not implemented, must be defined in the subclass

		virtual bool processMessage() {} ;              // Currently not implemented, must be defined in the subclass
	protected:
		bool message_decoded;							// Is set to true if we have decoded a valid signal
		ManchesterpatternDetector *mcdetector;
    	String protomessage;							// Holds the message from the protocol (hex)
        bool checkMessage(uint16_t min_clock, uint16_t max_clock, uint8_t min_Length,uint8_t max_Length);			 // Checks the pattern data against protocol
        bool checkSync(unsigned char pattern, uint8_t startpos, uint8_t mincount,uint8_t maxcount,uint8_t *syncend); // Checks tow bits against pattern beginning at startpos returns last position at syncend
        unsigned char getNibble(uint8_t startingPos);           													 // returns data bits 4 bits (nibble) in correct order
};

/*
 Class for decoding oregon scientific protocol from a manchester bit signal.
 Currently only working if there is a delay between two transmissions of a signal
*/
class OSV2Decoder : public decoderBacis {
    public:
        OSV2Decoder(ManchesterpatternDetector *detector);
	private:
        bool processMessage();                                  // Checks the Protocol
        unsigned char getNibble(uint8_t startingPos);           // returns data bits 4 bits (nibble) in correct order
        unsigned char getByte(uint8_t startingPos);				// returns data bits 8 bits (byte) in correct order
		bool checkMessage();									// Verify if we have OSV2 Message Type
		bool checkSync(uint8_t startpos, uint8_t mincount,uint8_t maxcount,uint8_t *syncend); // Checks for a valid sync sequence between start and endpos

		uint8_t syncend;										// Variable to hold the last syncend counter
};

/*
 Class for decoding Arduinosensor protocol from a manchester bit signal.
*/
class ASDecoder : public decoderBacis {
    public:
        ASDecoder(ManchesterpatternDetector *detector);
	private:
        bool processMessage();                              // Check, conversion and outpuz
   		bool checkMessage();									// Verify if we have OSV2 Message Type
		uint8_t syncend;										// Variable to hold the last syncend counter
};


#endif //PATTERNDEC_H
