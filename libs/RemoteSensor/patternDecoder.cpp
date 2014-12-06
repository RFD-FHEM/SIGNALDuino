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


#include "Arduino.h"
#include "patternDecoder.h"
#include "bitstore.h"



/*
********************************************************
********** Generic detection methods and vars **********
********************************************************
*/
patternBasic::patternBasic() {
	patternStore = new BitStore(2); // Init our Patternstore, default is 10 bytes. So we can save 40 Values.

	buffer[0] = buffer[1] = 0;
	first = buffer;
	last = first+1;
	success = false;
	//tolFact = 3.0; ///10
	//tolFact = 3.0; ///10
	reset();
}

void patternBasic::reset()
{
	patternLen = 0;
	state = searching;
	clock = 0;
	patternStore->reset();
    //tol=0;
}

bool patternBasic::detect(int* pulse){
	success = false;
	//swap buffer
	swap(first, last);
	*last = *pulse;
}

int8_t patternBasic::findpatt(int *seq) {
	//seq[0] = Länge  //seq[1] = 1. Eintrag //seq[2] = 2. Eintrag ...
	// Iterate over patterns (1 dimension of array)
	for (uint8_t idx=0; idx<patternLen; ++idx)
    {
        uint8_t x;
        // Iterate over sequence in pattern (2 dimension of array)
        for (x=1; x<=seq[0]; x++)
        {
            if (!inTol(seq[x],pattern[idx][x-1]))  // Skip this iteration, if seq[x] <> pattern[idx][x]
            {
                x=0;
                break;
            }
            if (x==seq[0])  // Pattern was found
            {
                return idx;
            }
        }
    }
    // sequence was not found in pattern
    return -1;
}



bool patternBasic::inTol(int value, int set){
	return (abs(value-set)<tol);
}

void patternBasic::swap(int* a, int* b){
	int temp;
	temp = *b;
	*b = *a;
	*a = temp;
}


/*
 Check if a pulse is followd with a pulse singed at the oppsit so + - / - + is valied - - or + + is not valid!
*/
bool patternBasic::validSequence(int *a, int *b)
{
    return ((*a> 0 and 0 > *b) or (*b > 0  and 0 > *a));
    //return ((*a)*(*b)<0);
    //return ((a ^ b) < 0); // true iff a and b have opposite signs

}


void patternBasic::doSearch() {}
void patternBasic::doDetect() {}
void patternBasic::processMessage() {}


/*
********************************************************
************ Return to zero detector *******************
********************************************************
*/


//	constructor
patternDetector::patternDetector() {
	buffer[0] = 0; buffer[1] = 0;
	first = buffer;
	last = first+1;
	success = false;
	tol = 200; //
	tolFact = 0.4;
	reset();
}

void patternDetector::reset() {
	messageLen = 0;
	patternLen = 0;
	bitcnt = 0;
	state = searching;
	clock = 0;
	//Serial.println("reset");
}

bool patternDetector::detect(int* pulse){
	//swap buffer
	success = false;
	swap(first, last);
	//add new value to buffer
	*last = *pulse;
	switch(state) {
		case detecting: doDetect();
		case searching: doSearch();
		//default: reset();
	}
	return success;
}

void patternDetector::doSearch() {
	//Serial.println("doSearch");
	//Serial.print(*first); Serial.print(", ");Serial.println(*last);
	if ((0<*first) & (*first<3276) & (syncMinFact* (*first) <= -1* (*last))) {//n>10 => langer Syncpulse (als 10*int16 darstellbar
		clock = *first;
		sync = *last;
		//clock = ~(int) *last/(sync/clock);
		state = detecting;
		bitcnt=0;
		tol = (int)round(clock*tolFact);
#ifdef DEBUGDETECT
		//debug
		Serial.print("PD sync: ");
		Serial.print(*first); Serial.print(", ");Serial.print(*last);
		Serial.print(", TOL: ");Serial.print(tol);
		Serial.print(", sFACT: ");Serial.println(sync/(float)clock);
#endif
	}
}

void patternDetector::doDetect() {
	//Serial.print("bitcnt:");Serial.println(bitcnt);
	if (bitcnt >= 1) {//nächster Satz Werte (je 2 Neue) vollständig
		//Serial.println("doDetect");
		//Serial.print(*first); Serial.print(", ");Serial.println(*last);
		bitcnt = 0;
		//valides Muster prüfen: ([1,-n] oder [n, -1])
		if (((inTol(*first, clock) & (syncMinFact*clock> - *last) & (*last<0)) | ((0<*first) & (*first<syncMinFact*clock) & (inTol(*last, -clock))))){
			//wenn nicht vorhanden, aufnehmen
			int fidx = find();
#ifdef DEBUG2
		 	Serial.print("Pulse: ");Serial.print(*first); Serial.print(", ");Serial.print(*last);
			Serial.print(", TOL: "); Serial.print(tol); Serial.print(", Found: "); Serial.println(fidx);
#endif
			if (0<=fidx){
				//gefunden
				*(message+messageLen) = fidx;
				messageLen++;
			} else {
				if (patternLen <maxNumPattern){
					//neues hinzufügen
					//Serial.println("hinzufügen");
					*(pattern+2*patternLen)   = *first;
					*(pattern+2*patternLen+1) = *last;
					patternLen++;
					*(message+messageLen) = patternLen-1; //Index des letzten Elements
					messageLen++;
				} else {
					processMessage();
					reset();
				}
			}
#ifdef DEBUGDETECT
			printOut();//debug
#endif
		} else { //kein valides Muster (mehr)
		 	//Serial.print("TrashPulse: ");Serial.print(*first); Serial.print(", ");Serial.print(*last); Serial.print(", MSGLen: "); Serial.println(messageLen);
			processMessage();
	        reset();
		}
	} else { //zweiten Wert für Muster sammeln
            bitcnt++;
	}
}

int patternDetector::find() {
	//Todo: Use find from patternBasic :  int tmp[2]; tmp[0] = *first; tmp[1] = *last; return find(tmp);
	int findex = -1;
	for (int idx=0; idx<patternLen; ++idx) {
		if ((abs(*first-(*(pattern+2*idx)))<tol) & (abs(*last-(*(pattern+2*idx+1)))<tol)){
			findex = idx;
		}
	}
	return findex;
}

bool patternDetector::inTol(int value, int set){
	return inTol(value, set, tol);
}
bool patternDetector::inTol(int value, int set, int tolerance){
	return (abs(value-set)<tol);
}

void patternDetector::swap(int* a, int* b){
	int buffer;
	buffer = *b;
	*b = *a;
	*a = buffer;
}

void patternDetector::sortPattern(){
	// sort the pattern, such that the second value of the first pattern is the greatest (values are considered negative => the absolute value of the first is then the smallest)
	// Example for the result: [1, -2]*c, [1, -6]*c, with the clock c
	// other example: [3, -1]*c, [1, -3]*c, with the clock c
	// for the moment only patterns of length 2 are considered
	if (*(pattern+1)<*(pattern+3)) {
		swap(pattern, pattern+2);
		swap(pattern+1, pattern+3);
		// Swap the Message
		for (byte idx = 0; idx<messageLen; ++idx){
			if (message[idx] == 0)
				message[idx] = 1;
			else //then it must be 1
				message[idx] = 0;
		}
	}
}

void patternDetector::printOut() {
	Serial.print("Sync: ");Serial.print(sync);
	Serial.print(" -> SyncFact: ");Serial.print(sync/(float)clock);
	Serial.print(", Clock: "); Serial.print(clock);
	Serial.print(", Tol: "); Serial.print(tol);
	Serial.print(", PattLen: "); Serial.print(patternLen); Serial.print(" ");
	Serial.print(", Pulse: "); Serial.print(*first); Serial.print(", "); Serial.print(*last);
	Serial.println();Serial.print("MSG: ");
	for (int idx=0; idx<messageLen; ++idx){
		Serial.print(*(message+idx));
	}
	Serial.print(". ");Serial.print(" [");Serial.print(messageLen);Serial.println("]");
	Serial.print("Pattern: ");
	for (int idx=0; idx<patternLen; ++idx){
 		Serial.print(" P");Serial.print(idx);Serial.print(":");
		Serial.print(*(pattern+2*idx)); Serial.print(", ");Serial.print(*(pattern+2*idx+1));Serial.print(", ");
 		Serial.print(" Pclock");Serial.print(idx);Serial.print(":");
		Serial.print(*(pattern+2*idx)/(float)clock); Serial.print(", ");Serial.print(*(pattern+2*idx+1)/(float)clock);Serial.print(", ");
 		//Serial.print(" Pfact");Serial.print(idx);Serial.print(":");
		//Serial.print(*(pattern+2*idx+1)/(float)*(pattern+2*idx));
		//Serial.print("-i-> ");
		//Serial.print(*(pattern+2*idx+1)/ *(pattern+2*idx));
		//Serial.print(", ");
		}
	Serial.println();
}

void patternDetector::processMessage()
{
        if (messageLen >= minMessageLen)
        {
            //mindestlänge der Message prüfen
            Serial.println("Message detected:");
            //printOut();
            if (patternLen=2)
            {
                sortPattern();
                printOut();
            }
        }
}

//-------------------------- Decoder -------------------------------

patternDecoder::patternDecoder(): patternDetector() {
	tol = 200;
	tolFact = 0.5;
}

bool patternDecoder::decode(int* pulse){
	return detect(pulse);
}

void patternDecoder::processMessage()
{
        if (messageLen >= minMessageLen){ //mindestlänge der Message prüfen
		//Serial.println("Message decoded:");
		//printOut();
		if (patternLen=2){
			sortPattern();
			#ifdef DEBUGDECODE
				printOut();
			#endif
			#ifdef DEBUGDECODE
				Serial.print("Logi: ");
			#endif
			checkLogilink();
			#ifdef DEBUGDECODE
				Serial.print("ITold: ");
			#endif
			checkITold();
			#ifdef DEBUGDECODE
				Serial.print("ITauto: ");
			#endif
			checkITautolearn();
			#ifdef DEBUGDECODE
				Serial.print("AS: ");
			#endif
			checkAS();
		}
	}
}

bool patternDecoder::checkEV1527type(int clockTst, int syncFact, int lowFact, int highFact, byte Length){
	bool valid = true;
	valid &= messageLen==Length;
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= inTol(clock, clockTst);//clock in tolerance
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= inTol(round(sync/(float)clock), -syncFact, 1); //sync in tolerance
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= inTol(*(pattern+0), clockTst) & inTol(*(pattern+2), clockTst); //[1, ][1, ] patternclocks in tolerance
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= inTol(*(pattern+1), -lowFact* clockTst); //p0=[ ,-nc]
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= inTol(*(pattern+3), -highFact* clockTst); //p1=[ ,-mc]
	#ifdef DEBUGDECODE
		Serial.println(valid);
	#endif
	return valid;
}

void patternDecoder::checkAS(){
/*
AS:
	clock: 		300
	Sync factor: 	14
	start sequence: [300, -4200] => [1, -14]
	high pattern:	[300, -1500] => [1, -5]
	low pattern:	[300, -600] => [1, -2]
	message length:	32 bit

FHEM Schnittstelle:
    ASTTIIDDDDDD
    AS  - Arduino Sensor
    TT  - Bit 0..6 type
	    - Bit 7 trigger (0 auto, 1 manual)
    II  - Bit 0..5 id
		- Bit 6,7 battery status (00 - bad, 01 - change, 10 - ok, 11 - optimal)
    DD1 - LowByte
    DD2 - HighByte
*/
	//checkEV1527type(int clockTst, byte syncFact, byte lowFact, byte highFact, byte Length)
	bool valid = checkEV1527type(500, 18, 1, 2, 32); //14 kommt hier nicht durch =>12
	twoStateMessageBytes();
	if (valid) {//ok, it's AS
		Serial.print("AS");
		printMessageHexStr();
		success = true;
	}
}

void patternDecoder::checkLogilink(){
/*
Logilink NC_WS:
	clock: 		500
	Sync factor: 	18
	start sequence: [500, -9000] => [1, -18]
	high pattern:	[500, -4000] => [1, -8]
	low pattern:	[500, -2000] => [1, -4]
	message length:	36 bit
	characteristic:	first bits allways 0101
			checksum
*/
	//checkEV1527type(int clockTst, byte syncFact, byte lowFact, byte highFact, byte Length)
	bool valid = checkEV1527type(500, 18, 4, 8, 36);
	twoStateMessageBytes();
	valid &= ((byteMessage[0]&0b11110000)==0b01010000); //first bits has to be 0101
	if (valid) {//ok, it's Logilink
		byteMessage[byteMessageLen-1] <<=4; //shift last bits to align left in bitsequence
		Serial.print("W03");
		printMessageHexStr();
		success = true;
	}
}

void patternDecoder::checkITautolearn(){
/*
IT self learning:
	clock: 		270
	Sync factor: 	10
	start sequence: [270, -2700] => [1, -10]
	low pattern:	[270, -270], [270, -1350]  => [1, -1], [1, -5]
	high pattern:	[270, -1350], [270, -270] => [1, -5], [1, -1]
	message length:	32 bit
*/
	//checkEV1527type(int clockTst, byte startFact, byte lowFact, byte highFact, byte Length)
	bool valid = checkEV1527type(270, 10, 1, 5, 64);
	// two bits in Message give one final bit:
	// 01 => 0, 10 => 1
	if (valid) {//ok, it's new IT selflearning
		Serial.print("AR");
		printNewITMessage();
		Serial.println();
		success = true;
	}
}

void patternDecoder::checkITold() {
/*
IT old with selector:
	clock: 		360(400)
	Sync factor: 	32(31)
	start sequence: [400, -13200] => [1, -31]
	high pattern:	[1200, -400][1200, -400] => [3, -1][3, -1]
	low pattern:	[400, -1200][400, -1200] => [1, -3][1, -3]
	float pattern:	[400, -1200][1200, -400] => [1, -3][3, -1]
	message length:	12 bit
	characteristic: IT only uses 2 states: low, float
					clock varies between controls from 340 to 400 => using measured clock leads to lower (required) tolerances
*/
	bool valid = true;
	valid &= messageLen==24;
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	int clockTst = 360;
	valid &= inTol(clock, clockTst);//clock
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= inTol(sync/clock, 31, 2); //syncValues
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= (inTol(*(pattern+0),3*clock) & inTol(*(pattern+1),-1*clock));//p0=[3, -1]
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	valid &= (inTol(*(pattern+2),1*clock) & inTol(*(pattern+3),-3*clock));//p1=[1, -3]
	#ifdef DEBUGDECODE
		Serial.println(valid);
	#endif
	if (valid){
		//printOut();
		triStateMessageBytes();
		byteMessage[byteMessageLen-1] <<=4; //shift last bits to align left in bitsequence
		Serial.print("IR");
		printTristateMessage();
		Serial.print("_");Serial.print(clock);
		Serial.println();
		success = true;
	}
}

void patternDecoder::printMessageHexStr(){
	char hexStr[] ="00";
	for (byte cnt=0; cnt<byteMessageLen; ++cnt){
		sprintf(hexStr, "%02x",byteMessage[cnt]);
		Serial.print(hexStr);
	}
	Serial.println();
}

void patternDecoder::printTristateMessage(){
	/* Message Code
		0: [1, -3], [1, -3] => message 1,1
		1: [3, -1], [3, -1] => message 0,0
		F: [1, -3], [3, -1] => message 1,0
	*/
	char msgChar;
	for (byte idx=0; idx<messageLen; idx +=2){
		if (message[idx]==1 & message[idx+1]==1)
			msgChar = '0';
		else if (message[idx]==0 & message[idx+1]==0)
			msgChar = '1';
		else if (message[idx]==1 & message[idx+1]==0)
			msgChar = 'F';
		else
			msgChar = '?';
		Serial.print(msgChar);
	}
}

void patternDecoder::printNewITMessage(){
	/* Message Code
		0: message 01
		1: message 10
	*/
	char msgChar;
	for (byte idx=0; idx<messageLen; idx +=2){
		if (message[idx]==0 & message[idx+1]==1)
			msgChar = '0';
		else if (message[idx]==1 & message[idx+1]==0)
			msgChar = '1';
		else
			msgChar = '?';
		Serial.print(msgChar);
	}
}

void patternDecoder::twoStateMessageBytes(){
	byte byteCode = 0;
	byte byteCnt = 0;
	for(byte idx=0; idx<messageLen; ++idx) {
		byteCode <<= 1;
		if (message[idx]==1){
			byteCode |=1;
		}
		if (((idx+1) % 8) ==0){ //byte full
			//Serial.println(byteCode,HEX);
			byteMessage[byteCnt]=byteCode;
			byteCnt++;
			byteCode = 0;
 		}
	}
	if ((messageLen % 8) != 0) {//non full byte
			//Serial.println(byteCode,HEX);
			byteMessage[byteCnt]=byteCode;
			byteMessageLen = byteCnt+1;
	} else
		byteMessageLen = byteCnt;
}

void patternDecoder::triStateMessageBytes(){
	//the function interpretes only two states: 0([1, -3][1, -3]),floating([1, -3][3, -1])
	// I wouldn't know how to represent three digits => As 3^n?
	byte byteCode = 0;
	byte byteCnt = 0;
	for(byte idx=0; idx<messageLen; idx+=2) {
		byteCode <<= 1;
		if (message[idx] != message[idx+1]){// two succeeding equals mark 0
			byteCode |=1;
		}
		if (((idx+2) % 16) ==0){ //byte full
			//Serial.println(byteCode);
			byteMessage[byteCnt]=byteCode;
			byteCnt++;
			byteCode = 0;
 		}
	}
	if ((messageLen % 16) != 0) {//non full byte
			//Serial.println(byteCode);
			byteMessage[byteCnt]=byteCode;
			byteMessageLen = byteCnt+1;
	} else
		byteMessageLen = byteCnt;
}

/*
********************************************************
************* Manchester detection class ***************
********************************************************
*/

ManchesterpatternDetector::ManchesterpatternDetector(bool silentstate){
	//tol = 200; //
	tolFact = 5.0;
	patternStore = new BitStore(2,40); // Init our Patternstore, with 40 bytes. So we can save 160 Values.
	ManchesterBits=new BitStore(1,20); // use 1 Bit for every value stored, reserve 20 Bytes = 160 Bits
	this->silent=silentstate;

}

void ManchesterpatternDetector::reset() {
  tol=0;
//  messageLen = 0;
  patternBasic::reset();
  ManchesterBits->reset();
#ifdef DEBUGDETECT
        Serial.println("");
        Serial.println("         **  RESET       ** ");
        Serial.println("");
#endif // DEBUGDETECT

}


bool ManchesterpatternDetector::detect(int* pulse){
    patternBasic::detect(pulse);

   	switch(state) {
		case searching: doSearch(); if (state == searching) break;
		case detecting: doDetect(); break;
		//default: reset();
	}
	return success;
}


/*
 Puls is a short one
*/
bool ManchesterpatternDetector::isShort(int *pulse)
{
	return inTol(abs(*pulse), clock);
}

/*
 Puls is a long one
*/
bool ManchesterpatternDetector::isLong(int *pulse)
{
	return inTol(abs(*pulse), 2*clock);
}

/*
  Updates the Mancheser Clock every time, the function is called
*/
void ManchesterpatternDetector::updateClock(int *pulse)
{
    /* Todo Auswetung der Pulse kann in die Schleife doDetect verlagert werden */
    static uint8_t sample=0;
    static uint16_t average=0;

    if (clock == 0) // Reset the counter
    {
        sample=0;
        average=0;
#ifdef DEBUGDETECT
        Serial.print("reinit clock math...");
#endif // DEBUGDETECT
        reset();
        clock=abs(*pulse)/2;
        sample=0;
        average=0;
#ifdef DEBUGDETECT
        Serial.print("  init Clock:");
#endif // DEBUGDETECT

    }
    if (abs(*pulse) < int(clock*0.5))
    {
        clock=abs(*pulse);
    } else if (isShort(pulse)) {
        average=average+abs(*pulse);
        sample++;
        clock=average/sample;
#ifdef DEBUGDETECT
        Serial.print("  recalc Clock with short:");
        Serial.print(*pulse);
        Serial.print(" to clock :");
#endif // DEBUGDETECT
    } else if (isLong(pulse))  {
        average=average+(abs(*pulse)/2);
        sample++;
        clock=average/sample;
#ifdef DEBUGDETECT
        Serial.print("  recalc Clock with long pulse: ");
        Serial.print(*pulse);
        Serial.print(" to clock :");

#endif // DEBUGDETECT
    }
    else {
    }

#ifdef DEBUGDETECT
        Serial.println(clock);
#endif // DEBUGDETECT
}



void ManchesterpatternDetector::doSearch()
{
	//patternDecoder.doSearch()
	//Serial.println("doSearch");
	//Serial.print(*first); Serial.print(", ");Serial.println(*last);
    if (!validSequence(first,last)) return;

    updateClock(last);
    tol = (int)round(clock*tolFact/10);

    if (isLong(first) || isShort(first))
    {
      // Valid
        state = detecting;
        updateClock(first);
#ifdef DEBUGDETECT
		Serial.println("");
		Serial.println("         ** MC sync:     **");
		Serial.println("");
		Serial.print(*first); Serial.print(", ");Serial.print(*last);
		Serial.print(", TOL: ");Serial.print(tol);
		Serial.print(", clock: ");Serial.println(clock);
#endif // DEBUGDETECT
        // Eventuell Clock an diesem Punkt wieder auf 0 setzen.. und die Berechnung komplett in doDetect machen

    } else {
      // not Valid
        state = searching;
        reset();
    }
}


void ManchesterpatternDetector::doDetect() {
    static bool skip=false;
    static uint8_t bcnt=0;
	//Serial.print("bitcnt:");Serial.println(bitcnt);
	if (skip)
    {
        skip=false;
        return;
    }
    if (!validSequence(first,last))
    {
      //Serial.println("Invalid sequence found, aborting");
      processMessage(); // May this does not work for manchester due to sorting
      return;
    }
    updateClock(first);

    // May a valid manchester pulse
    int seq[3] = {0,0,0};
    if (isLong(first))
    {
        // valid it is a long pulse
        seq[0] = 1;
        seq[1] = *first;
        //ManchesterBits->addValue(!(*first & (1<<7))); // Check if bit 7 is set
        //Serial.println(*first,BIN);
        //ManchesterBits->addValue((*first >>15)? 0 :1 ); // Check if bit 7 is set
        ManchesterBits->addValue(!(*first >>15)); // Check if bit 7 is set
    }
    else if(isShort(first) && isShort(last) )
    {
         // valid
        seq[0] = 2;
        seq[1] = *first;
        seq[2] = *last;
        updateClock(last); // Update Clock also with last pulse, because we will skip this in the next iteration
        skip=true; // Skip next iteration
        //ManchesterBits->addValue(!(*last & (1<<7)));  // Check if bit 7 is set
        ManchesterBits->addValue(!(*first >>15)); // Check if bit 7 is set
/*  } else if(isShort(first))
    {
        return;
*/
    } else {
        // not valid
        //Serial.println("Invalid sequence found, aborting");

		processMessage(); // May this does not work for manchester due to sorting
        state=searching;
        //reset();
        return;
    }

	int8_t fidx = findpatt(seq);
#ifdef DEBUGDETECT
	Serial.print("MC Pulse: ");Serial.print(*first); Serial.print(", ");Serial.print(*last);
	Serial.print(", Clock: "); Serial.print(clock); Serial.print(", Found: "); Serial.println(fidx);
#endif
    if (0<=fidx)
    {
        //gefunden
#ifdef DEBUGDETECT
	Serial.println("pattern found");
#endif

        //patternStore->addValue(fidx); // Add current pattern index to our store
        //*(message+messageLen) = fidx;
        //messageLen++;
    }
    else
	{
        if(patternLen <maxNumPattern) // Max 4 pattern are valid for manchester coding
        {
#ifdef DEBUGDETECT
	Serial.println("pattern not found, adding a new one...");
#endif

            //neues hinzufügen
			//Serial.println("hinzufügen");
            pattern[patternLen][0] = seq[1];
            pattern[patternLen][1] = seq[2];

            //patternStore->addValue(patternLen); // Add current pattern index to our store
			//*(message+messageLen) = patternLen; //Index des eben angefügten Elementes

			patternLen++;
			//messageLen++;
        } else {
            //Serial.print("Max len reached, printing out");
			processMessage(); // May this does not work for manchester due to sorting
			//reset();
		}
    }
#ifdef DEBUGDETECT
	printOut();//debug
#endif

}


void ManchesterpatternDetector::processMessage()
{
        if (ManchesterBits->valcount >= minMessageLen)
        {
           // Serial.println("Message decoded:");
            if (!silent) printOut();
            success=true;
        } else {
            reset();
        }
}



void ManchesterpatternDetector::printOut() {


//#ifdef DEBUGDETECT

	Serial.print("Clock: "); Serial.print(clock);
	Serial.print(", Tol: "); Serial.print(tol);
	Serial.print(", PattLen: "); Serial.print(patternLen);
	Serial.print(", Pattern: ");
	for (int idx=0; idx<patternLen; ++idx){
        Serial.print(" P");Serial.print(idx);Serial.print(": [");
        for (uint8_t x=0; x<PATTERNSIZE;++x)
        {
            if (pattern[idx][x] != 0)
            {
                if (x>0) Serial.print(",");
                Serial.print(pattern[idx][x]);
            }
        }
        Serial.print("]");
	}
	Serial.println();
    Serial.print("MC MSG: ");
    for (uint8_t idx=0; idx<ManchesterBits->valcount; ++idx){
		//Serial.print(*(message+idx));
		Serial.print(ManchesterBits->getValue(idx),DEC);
	}
	Serial.print("  ["); Serial.print(ManchesterBits->valcount); Serial.print("]");
	Serial.println();
//#endif
    Serial.print("0x");
	printMessageHexStr();

}

/* Reverse a byte and output the reversed byte*/
unsigned char ManchesterpatternDetector::reverseByte(unsigned char original){

  return original; // Da die Bits bereits im Bitstore richtig herum stehen, können wir uns das schenken.
/*
  unsigned char reversed;
  for(uint8_t position=7; position>0; position--){
    reversed+=((original&1)<<position);
    original >>= 1;
  }
  return reversed;
*/
}

/* Prints out manchester Bits in HEX format*/
void ManchesterpatternDetector::printMessageHexStr(){
	char hexStr[] ="00"; // Not really needed
    // Bytes are stored from left to right in our buffer. We reverse them for better readability
	for (uint8_t idx=4; idx<ManchesterBits->bytecount; ++idx){
        //Serial.print(getMCByte(idx),HEX);
        //sprintf(hexStr, "%02X",reverseByte(ManchesterBits->getByte(idx)));
        sprintf(hexStr, "%02X",getMCByte(idx));
		Serial.print(hexStr);
	}
	Serial.println();
}

bool ManchesterpatternDetector::manchesterfound(){

    return success;
}

unsigned char ManchesterpatternDetector::getMCByte(uint8_t idx){

    return ManchesterBits->getByte(idx);
}

//-------------------------- Decoder -------------------------------

//decoderBacis::decoderBacis(patternBasic *detector)
decoderBacis::decoderBacis()
{
    //this->detector =detector;
}

void decoderBacis::printMessageHexStr()
{


}

bool decoderBacis::decode()
{

}

bool decoderBacis::processMessage()
{
}


OSV2Decoder::OSV2Decoder(ManchesterpatternDetector *detector)
{
    //*mcdetector = new ManchesterpatternDetector(true);
    mcdetector = detector;
}

bool OSV2Decoder::decode(){
	//return detect(pulse);

    if (mcdetector->manchesterfound()) {
        return processMessage();
        //mcdetector->reset();
        //return  true; // We will loose this value
    }
    //mcdetector->detect(pulse);
    return false;
}

/* State: Debugging */
bool OSV2Decoder::processMessage()
{

    if (mcdetector->ManchesterBits->valcount < 80) return false; // Message to short

    // Bytes are stored from left to right in our buffer. We reverse them for better readability and check first 4 Bytes for our Sync Signal
    unsigned char cdata;

    // Check sync sginal
    for (uint8_t idx=0; idx<4; idx++)
    {
        cdata = mcdetector->getMCByte(idx);
        Serial.println(cdata,HEX);
        //if (cdata != 0xAA) return;
	}

	//Todo: Convert to OSV2 Data, use only every second bit, work with this information

    // Check Preamble
    cdata = mcdetector->getMCByte(4);
    Serial.println(cdata,HEX);
    //if (cdata != 0xAA) return;


    //Serial.println(mcdetector->ManchesterBits->bytecount);
    // Bytes are stored from left to right in our buffer. We reverse them for better readability
    char hexStr[] ="00"; // Not really needed
    Serial.print("OSV2 MC:");  // Still a Manchester Bit message, not really OSV2
	for (uint8_t idx=0; idx<(mcdetector->ManchesterBits->bytecount); ++idx){
         cdata = mcdetector->getMCByte(idx);
         Serial.print(cdata,HEX);
        //Serial.println(ManchesterBits->datastore[idx],DEC);
        // Here we have to do some more stuff to extract osv2 Protocol out of the data
        //sprintf(hexStr, "%02X",mcdetector->getMCByte(idx));
		//Serial.print(hexStr);
	}

    Serial.print("OSV2 Proto (hex):");  // Now we look at the osv2 protocol. We use only every seocnd bit
    uint8_t bcnt=7;
    for (uint8_t idx=1; idx<(mcdetector->ManchesterBits->valcount); idx=idx+2){
        //cdata = cdata | mcdetector->ManchesterBits->getValue(idx);
        if (bcnt ==0){
            Serial.print(cdata,HEX);
            bcnt=7;
        }

        //Serial.print(mcdetector->ManchesterBits->getValue(idx));
        cdata =cdata | mcdetector->ManchesterBits->getValue(idx) <<(bcnt);
        bcnt--;
//        if (idx%8==0 && idx>0)
//            Serial.print(cdata,HEX);
        //Serial.println(ManchesterBits->datastore[idx],DEC);
        // Here we have to do some more stuff to extract osv2 Protocol out of the data
        //sprintf(hexStr, "%02X",mcdetector->getMCByte(idx));
		//Serial.print(hexStr);
	}


	Serial.println();

}


ASDecoder::ASDecoder(ManchesterpatternDetector *detector)
{
    //*mcdetector = new ManchesterpatternDetector(true);
    mcdetector = detector;
}

bool ASDecoder::decode(){
	//return detect(pulse);
    if (mcdetector->manchesterfound()) {
        return processMessage();
        //mcdetector->reset();
        //return  true; // We will loose this value
    }
    return false;

}


bool ASDecoder::processMessage()
{
    if (mcdetector->ManchesterBits->bytecount < 4) return false; // Message to short
    // Bytes are stored from left to right in our buffer. We reverse them for better readability and check first 4 Bytes for our Sync Signal

	// Check the sync Bit
	for (uint8_t idx=0; idx<1; ++idx)
    {
        if (mcdetector->getMCByte(idx) != 0xB3) return false;
	}
	char hexStr[] ="00"; // required for proper allocation of memory
	uint8_t val =0;
    // Bytes are stored from left to right in our buffer. We reverse them for better readability
	Serial.print("AS");
	// Check the Data Bits
	for (byte idx=1; idx<(mcdetector->ManchesterBits->bytecount); ++idx){
        //Serial.println(ManchesterBits->datastore[idx],DEC);
        // Here we have to do some more stuff to extract osv2 Protocol out of the data
        sprintf(hexStr, "%02x",mcdetector->getMCByte(idx));
		Serial.print(hexStr);
	}
	Serial.println();

}

