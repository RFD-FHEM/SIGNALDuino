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
#include <util/crc16.h>



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

    //return ((*a> 0 and 0 > *b) or (*b > 0  and 0 > *a));
    //return ((*a)*(*b)<0);
    return ((*a ^ *b) < 0); // true iff a and b have opposite signs

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
	patternStore = new BitStore(2,66); // Init our Patternstore, with 40 bytes. So we can save 224 Values.
	ManchesterBits=new BitStore(1,28); // use 1 Bit for every value stored, reserve 28 Bytes = 224 Bits
	this->silent=silentstate;

}

void ManchesterpatternDetector::reset() {
  tol=0;
//  messageLen = 0;
  patternBasic::reset();
  ManchesterBits->reset();
/*
#ifdef DEBUGDETECT
        Serial.println("");
        Serial.println("         **  RESET       ** ");
        Serial.println("");
#endif // DEBUGDETECT
*/
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
    static uint32_t average=0;      // Need 32 Bit, because 16 bit it will overflow shortly.


    if (clock == 0) // Reset the counter
    {
        sample=0;
        average=0;
#if DEBUGDETECT==255
        Serial.print("reinit clock math...");
#endif // DEBUGDETECT
        reset();
        clock=abs(*pulse)/2;
        sample=1;
        average=clock;

    }
    if (abs(*pulse) < int(clock*0.5))
    {
        clock=abs(*pulse);
    } else if (isShort(pulse)) {
        average=average+abs(*pulse);
        sample++;
        clock=average/sample;
#if DEBUGDETECT==255
        Serial.print("  recalc Clock with short:");
        Serial.print(*pulse);
        Serial.print(" to clock :");
#endif // DEBUGDETECT
    } else if (isLong(pulse))  {
        average=average+(abs(*pulse)/2);
        sample++;
        clock=average/sample;
#if DEBUGDETECT==255
        Serial.print("  recalc Clock with long pulse: ");
        Serial.print(*pulse);
        Serial.print(" to clock :");

#endif // DEBUGDETECT
    }

#if DEBUGDETECT==255
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
#if DEBUGDETECT==255
		Serial.println("MCPD Sync: ");
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
#if DEBUGDETECT==255
	Serial.print("MCPD Pulse: ");Serial.print(*first); Serial.print(", ");Serial.print(*last);
	Serial.print(", Clock: "); Serial.print(clock); Serial.print(", Found: "); Serial.println(fidx);
#endif
    if (0<=fidx)
    {
        //gefunden
#if DEBUGDETECT==255
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
#if DEBUGDETECT==255
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
	if (patternLen > 3) printOut();//debug
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


#ifdef DEBUGDETECT
	Serial.print("MCPD Clock: "); Serial.print(clock);
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
    Serial.print("Manchester Bits: ");
    for (uint8_t idx=0; idx<ManchesterBits->valcount; ++idx){
		//Serial.print(*(message+idx));
		Serial.print(ManchesterBits->getValue(idx),DEC);
	}
	Serial.print("  ["); Serial.print(ManchesterBits->valcount); Serial.print("]");
	Serial.println();

    Serial.print("Hex: 0x");
	printMessageHexStr();

#endif
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

decoderBacis::decoderBacis()
{
}

String decoderBacis::getMessageHexStr()  const                    // Returns a Pointer the hex message on serial
{
	return protomessage;
}

bool decoderBacis::decode(){
    if (mcdetector->manchesterfound()) {
        message_decoded= processMessage();
        return message_decoded;
    }
    return false;
}

/*
	Check if the clock and the bitlength fit's our protocol
*/
bool decoderBacis::checkMessage(uint16_t min_clock, uint16_t max_clock, uint8_t min_Length,uint8_t max_Length)
{
    #if DEBUGDECODE==255
	//Serial.print(" (D255 Clock: ");
	DEBUG_BEGIN(DEBUGDECODE)
	Serial.print("  Clock: ");
	Serial.print(mcdetector->clock);
	Serial.print("  num bits : ");
	Serial.print(mcdetector->ManchesterBits->valcount);
	DEBUG_END
	#endif // DEBUGDECODE
	bool valid;
	valid = ((min_clock <= mcdetector->clock) && (mcdetector->clock <= max_clock)) ;
#ifdef DEBUGDECODE
	Serial.print(valid);
#endif
	valid &= ((min_Length <= mcdetector->ManchesterBits->valcount) && (mcdetector->ManchesterBits->valcount <= max_Length));
#ifdef DEBUGDECODE
	Serial.print(valid);
#endif
	return valid;
}

/*
	Checks for Sync signal, starting at startpos, returning true if mincount sync is counted. Returns true if maxcount has been reached. returns the end of the loop via syncend
*/
bool decoderBacis::checkSync(unsigned char pattern, uint8_t startpos, uint8_t mincount,uint8_t maxcount,uint8_t *syncend)
{
	bool valid=true;

    uint8_t twobit=0; // Check two bits of the sync signal
    const uint8_t endcount = startpos+maxcount;
    uint8_t idx;
	for (idx=startpos; idx<endcount;idx+=2)
	{
		twobit = mcdetector->ManchesterBits->getValue(idx) <<1 | mcdetector->ManchesterBits->getValue(idx+1); // Combine  two sync bits
        if ( twobit == pattern)																				  // Check the sync bits against the pattern
        {
            continue;
        } else if (idx-startpos >= mincount)  {    	// minimum of sync bits must be reveived
            valid=true;              			// Valid Sync sequence of mincount bits are valid
            break;
        } else {
            valid=false;              			// Not a valid Sync sequence, to less sync bits
			break;
        }
	}
	if (valid){
		*syncend = idx+startpos;
	}
	return valid;
}

/*
returns the 4 bits one nibble, begins at position deliverd via startingPos
returns the nibble in received order
*/
unsigned char decoderBacis::getNibble(uint8_t startingPos)
{
    uint8_t bcnt=3;
    unsigned char cdata=0;
	for (uint8_t idx=startingPos; idx<startingPos+4; idx++, bcnt--){
       cdata = cdata | (mcdetector->ManchesterBits->getValue(idx) << (bcnt)); // add bits in reversed order
	}
    return cdata;
}

/*
	Function to check if it's a valid OSV2 Protocol. Checks clock, Sync and preamble
*/
bool OSV2Decoder::checkMessage()
{
	bool valid;
#ifdef DEBUGDECODE
	Serial.print("Check OSV:");
#endif
	valid = decoderBacis::checkMessage(440,540,150,220);						// Valid clock and length
	valid &= checkSync(uint8_t (0),uint8_t (24),uint8_t (33),&syncend);			// Valid sync sequence
#ifdef DEBUGDECODE
	Serial.print(valid);
#endif
	valid &= (getNibble(syncend) == 0xA);  										// Valid preamble
#ifdef DEBUGDECODE
	Serial.print(valid);
	Serial.println();
#endif
	return  valid;
}

/*
	Checks for Sync signal, starting at startpos, returning true if mincount sync is counted. Returns true if maxcount has been reached. returns the end of the loop via syncend
*/
bool OSV2Decoder::checkSync(uint8_t startpos, uint8_t mincount,uint8_t maxcount,uint8_t *syncend)
{
	bool valid=true;

    uint8_t twobit=0;
    uint8_t endcount = startpos+maxcount;
    uint8_t idx;
	for (idx=startpos; idx<endcount;idx+=2)
	{
		twobit = mcdetector->ManchesterBits->getValue(idx) <<1 | mcdetector->ManchesterBits->getValue(idx+1);
        if ( twobit == 0x2)             		// Check if the two bis are b10 / 0x2
        {
            continue;
        } else if (idx-startpos >= mincount)  {    	// minimum of 24 sync bits must be reveived
            valid=true;              			// Valid OSV2 Sync sequence of 24 bits are valid
            break;
        } else {
            valid=false;              			// Not a valid OSV2 Sync sequence, to less sync bits
			break;
        }
	}
	if (valid){
		*syncend = idx+startpos;
	}
	return  valid;
}



OSV2Decoder::OSV2Decoder(ManchesterpatternDetector *detector)
{
    //*mcdetector = new ManchesterpatternDetector(true);
    mcdetector = detector;
}


/*
The Sync signal begins really with 01, so we need to start decoding at first 01 pattern, not 10. 10 is caused by the startingpoint of the signal

Sync
10 10 10 10 10 10 10 10
10 10 10 10 10 10 10 10

Preamble:
10 01 10 01
 0  1  0  1     (Second bit)
 1  0  1  0     (Reversed nibbles)
    A

Data
10 10 01 10  10 01 01 10  10 10 01 10  10 10 10 01
0  0  1  0   0  1  1  0   0  0  1  0   0  0  0  1  (Second bit)
0  1  0  0   0  1  1  0   0  1  0  0   1  0  0  0  (Reversed nibbles)
      4            6            4          8


01 10 01 10  01 01 10 10  10 10 01 01  10 01 01 01

???
10 1

Jeelib Ausgabe Data hex: Flip=33=33 lange pulse abgewartet ohne zu decodieren!
DA=1101 1010
DC=1101 1100
53=0101 0011
9E=1001 1110
18=0001 1000
27=0010 0111
70=0111 0000
55=0101 0101,

Jeelib hinzufügen der Bits Reihenfolge des Aufrufes gotbit() (jedes 2. bit)
0101 1011
0011 1011
1100 1010
01111001
0001100011100100000011101
01010100101Binary (8) :

jeelip Manchester bits stimmen mit denen diese decoders überein!
01 10 01 10   10 01 10 10   01 01 10 10   10 01 10 10   10 10 01 01    10 01 10 01   01 10 10 10   10 01 01 10   01 01 01 101001010110101001011001010101010110101001100110011001100101100110

This Decoder: Sync bits        synccnt  (+2 every iteration!)
10 10 10 10 10 10 10 10         16
10 10 10 10 10 10 10 10         32
10                              34

Preamble<-|     Data ->>
01 10 01 10   10 01 10 10   01 01 10 10   10 01 10 10   10 10 01 01    10 01 10 01   01 10 10 10   10 01 01 10   01 01 01 10  10010101 1010100 1011001 01010101 01101010 0110011 00110011 00101100 11 [169]
   1010         1101

01 10 01 10   10 01 10 10   01 01 10 10   10 01 10 10   10 10 01 01    10 01 10 01   01 10 10 10   10 01 01 10   01 01 01
 1010           0100           1100         0110          0011            0101         1000         0110                    (Second bit)
 0101           0010           0011         0110          1100            1010         0001         0110                    (Reversed nibbles)
 0101           1011           0011         1001                                                                            (Inverted nibbles)
 1010           0100           1100         0110                                                                            (Inverted +reversed nibbles)

 0101           1011           0011         1011          1100            1010         0111         1001                    (first bit)
 1010           1101           1100         1101          0011            0101         1110         1001                    (first bit reversed)  <<< --- Passt zum Jeelib Decoder
*/
bool OSV2Decoder::processMessage()
{
	if (!checkMessage()) return false;
	unsigned char cdata;
    uint8_t idx;

	/*
    if (mcdetector->ManchesterBits->valcount < 120) return false; // Message to short

    // Bytes are stored from left to right in our buffer. We reverse them for better readability and check first 4 Bytes for our Sync Signal
    unsigned char cdata;

    // Check sync sginal
    uint8_t idx=0;
    uint8_t twobit=0;
    uint8_t synccnt=0;

    // Check if we have 12 x 10 bitspairs
    do {
        twobit = mcdetector->ManchesterBits->getValue(idx) <<1 | mcdetector->ManchesterBits->getValue(idx+1);
        if ( twobit == 0x2)             // Check if the two bis are b10 / 0x2
        {
            synccnt+=2;                // Count the number of sync bit
        } else if (synccnt > 24)  {    // minimum of 24 sync bits must be reveived
            break;
        } else {
            return false;              // Not a valid OSV2 Sync sequence
        }
        idx+=2;
    } while (true);
    // Minimum 24 Sync bits received. Preamble beginns with 01
    //Serial.println(synccnt); -> 34! Preamble begins at pos 32! (Index is 0-31 = 32 Sync Bits)
    uint8_t datastart=synccnt;    // Starting point for furher inspection is one bit ahead!

    // Check the next 8 Manchester Bits
#ifdef DEBUGDECODE
    Serial.print("OSV2 Preamble at pos:");Serial.print(datastart);
#endif
    // Extract the Preamble in right order
    cdata=getNibble(datastart);
#ifdef DEBUGDECODE
    Serial.print(" hex:");  Serial.println(cdata,HEX);
#endif
    if (cdata != 0xA) return false;     // Exit if our Preamble is not 0xA / b1010

	uint8_t numbits = int((mcdetector->ManchesterBits->valcount-datastart)/16)*8;  // Stores number of bits
	*/
	uint8_t numbits = int((mcdetector->ManchesterBits->valcount-syncend)/16)*8;  // Stores number of bits
#ifdef DEBUGDECODE
    // Now exract all of the message  // Todo: Check for a new sync signal, because we may have no delay between two transmissions of the messages may easy possible to check for occurence of 10101010 or 01010101
    Serial.print("OSV2 protocol len(0x");  // Print the OSV2 hex message
    Serial.print(numbits,HEX);   // Length of full bytes after Sync
    Serial.print(" ) Message: ");    // Print the OSV2 hex message
#endif
	this->protomessage.reserve((numbits/4)+2); 							 	 // Reserve buffer for Hex String
	this->protomessage= String(numbits,HEX);
    for (idx=syncend; idx<mcdetector->ManchesterBits->valcount; idx+=16) 	 // Iterate over every byte which uses 16 bits
    {
        if (mcdetector->ManchesterBits->valcount-idx<16) break;			  	 // Break if we do not have a full byte data left
  		cdata = getByte(idx);
  		this->protomessage.concat(String(cdata,HEX));
#ifdef DEBUGDECODE
        Serial.print(cdata,HEX);
#endif
	}
#ifdef DEBUGDECODE
	Serial.println();
#endif
	return true;
}
/*
returns the 4 bits one nibble, begins at position deliverd via startingPos
skips every second bit and returns the nibble in reversed (correct) order
*/
unsigned char OSV2Decoder::getNibble(uint8_t startingPos)
{
    // Todo: Use getNibble from baseclass and reorder the return value here
    uint8_t bcnt=0;
    unsigned char cdata=0;
	for (uint8_t idx=startingPos; idx<startingPos+8; idx=idx+2, bcnt++){
         //Serial.print(mcdetector->ManchesterBits->getValue(idx));
         cdata = cdata | (mcdetector->ManchesterBits->getValue(idx) << (bcnt)); // add bits in reversed order
	}
    return cdata;
}

/*
returns the 8 bits / one byte, begins at position deliverd via startingPos
skips every second bit and returns the byte in reversed (correct) order.
// Todo use getNibble to return the byte
*/
unsigned char OSV2Decoder::getByte(uint8_t startingPos)
{
    // Todo: Use getNibble to build a byte
    uint8_t bcnt=0;
    unsigned char cdata=0;
	for (uint8_t idx=startingPos; idx<startingPos+16; idx=idx+2, bcnt++){
         //Serial.print(mcdetector->ManchesterBits->getValue(idx));
         cdata = cdata | (mcdetector->ManchesterBits->getValue(idx) << (bcnt)); // add bits in reversed order
	}
    return cdata;
}



ASDecoder::ASDecoder(ManchesterpatternDetector *detector) :decoderBacis()
{
    //*mcdetector = new ManchesterpatternDetector(true);
    mcdetector = detector;
}

/*
	Function to check if it's a valid AS Protocol. Checks clock, Sync and preamble
*/
bool ASDecoder::checkMessage()
{

	bool valid=true;
#ifdef DEBUGDECODE
	Serial.print("Check AS:");
#endif
	valid = decoderBacis::checkMessage(380,405,38,56);							// Valid clock and length
	valid &= decoderBacis::checkSync(0x2,0,8,12,&syncend);						// Searching for sync bits, 8-12 bits must be sync pattern 0x2 = 10
#ifdef DEBUGDECODE
	Serial.print(valid);
#endif
	valid &= (getNibble(syncend) == 0xC);  										// Valid preamble for Sensor
#ifdef DEBUGDECODE
	Serial.print(valid);
	Serial.println();
#endif
	return  valid;
}


bool ASDecoder::processMessage()
{
  	if (!checkMessage()) return false;
	uint8_t idx =0;
	uint8_t numbits = (mcdetector->ManchesterBits->valcount-syncend-4);     // Stores number of bits in our message


/*
	if (mcdetector->ManchesterBits->bytecount < 4 && mcdetector->ManchesterBits->bytecount > 6) return false; // Message to short or to long
    // Bytes are stored from left to right in our buffer. We reverse them for better readability and check first Bytes for our Sync Signal
	// Check the sync Bit
	for (; idx<1; ++idx)
    {
        if (mcdetector->getMCByte(idx) != 0xB3) return false;
	}
    // Bytes are stored from left to right in our buffer. We reverse them for better readability
*/
	this->protomessage.reserve((numbits/4)+3); 							 	 // Reserve buffer for Hex String
	this->protomessage= String("AS:");
#ifdef DEBUGDECODE
	Serial.print("AS detected with len("); Serial.print(numbits); Serial.print(") :");
#endif
	char hexStr[] ="00";
	byte crcv=0x00;
	byte b=0;
	// Check the Data Bits
	for (idx=syncend+4; idx<mcdetector->ManchesterBits->valcount-8; idx=idx+8){
		b=getNibble(idx)<<4|getNibble(idx+4);
		sprintf(hexStr, "%02X",b);
		crcv = _crc_ibutton_update(crcv,b);
#ifdef DEBUGDECODE
		Serial.print(String(b,HEX));
#endif
		this->protomessage.concat(hexStr);
	}

	if (crcv == (getNibble(idx)<<4 | getNibble(idx+4))) {
#ifdef DEBUGDECODE
		Serial.print("  CRC Valid");
#endif
	} else {
#ifdef DEBUGDECODE
		Serial.print("  CRC: ");
		Serial.println(crcv,HEX);
#endif
		return false;
	}
#ifdef DEBUGDECODE

	Serial.println();
#endif
	return true;
}

