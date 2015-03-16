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
    tol=0;
}

bool patternBasic::detect(int* pulse){
	success = false;
	//swap buffer
	swap(first, last);
	*last = *pulse;
}

/* Returns the index of the searched pattern or -1 if not found */
int8_t patternBasic::findpatt(int *seq) {
	//seq[0] = Länge  //seq[1] = 1. Eintrag //seq[2] = 2. Eintrag ...
	// Iterate over patterns (1 dimension of array)
	for (uint8_t idx=0; idx<patternLen; ++idx)
    {
        uint8_t x;
        // Iterate over sequence in pattern (2 dimension of array)
        for (x=1; x<=seq[0]; x++)
        {
            //if (!(seq[x] ^ pattern[idx][x-1]) >= 0)  break;  // Not same sign, we can skip
            if (!inTol(seq[x],pattern[idx][x-1]))  // Skip this iteration, if seq[x] <> pattern[idx][x]
            //if (!inTol(seq[x],pattern[idx][x-1]))  // Skip this iteration, if seq[x] <> pattern[idx][x]
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
	return inTol(value, set, tol);
}

bool patternBasic::inTol(int value, int set, int tolerance){
	return (abs(value-set)<=tolerance);
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
*************** Puls Pause detector ********************
********************************************************
*/


//	constructor
patternDetector::patternDetector():patternBasic() {
	buffer[0] = 0; buffer[1] = 0;
	first = buffer;
	last = first+1;
	reset();
}

void patternDetector::reset() {
	patternBasic::reset();
	messageLen = 0;
	patternLen = 0;
	bitcnt = 0;
	state = searching;
	clock = sync=0;
	for (uint8_t i=0; i<maxNumPattern;++i)
		histo[0]=pattern[i][0]=0;
	success = false;
	tol = 150; //
	tolFact = 0.3;

	//Serial.println("reset");
}

bool patternDetector::detect(int* pulse){
	patternBasic::detect(pulse);   // Call function from base class, to do some basic stuff
    doDetectwoSync();			   // Search for signal

	return success;
}

void patternDetector::ArraySort(int arr[maxNumPattern][PATTERNSIZE], int n)
{
	int min[2], i, j, pos;
	for (i=0; i < n; ++i) // Das Array an Position i teilen
	{ // mit leerem linken Teil beginnen
		pos = i; // suche in unsortiertem Teil
		*min = *arr[i]; // das kleinste Element
		for( j = i+1; j < n; j++)
			if ( min[0] > arr[j][0])
			{
				pos = j;
				*min = *arr[j];
			}
		*arr[pos] = *arr[i]; // Austausche
		*arr[i] = *min;
	}

}

bool patternDetector::getSync(){
    // Durchsuchen aller Musterpulse und prüft ob darin ein Sync Faktor enthalten ist. Anschließend wird verifiziert ob dieser Syncpuls auch im Signal nacheinander übertragen wurde
	#if DEBUGDETECT > 3
	Serial.println("  --  Searching Sync  -- ");
	#endif
/*
	int pattern_up[maxNumPattern][PATTERNSIZE] ={};
	memcpy(pattern_up, pattern, sizeof(int)*maxNumPattern*PATTERNSIZE);
	ArraySort(pattern_up,maxNumPattern);
*/
    for (uint8_t i=0;i<patternLen;++i) 		  // Schleife für Clock
    {
		if (pattern[i][0]<0 || pattern[i][0] > 3276)  continue;  // Werte <0 / >3276 sind keine Clockpulse
        for (int8_t p=patternLen-1;p>=0;--p)  // Schleife für langen Syncpuls
        {
           	if (pattern[p][0] > 0 || pattern[p][0] == -maxPulse)  continue;  // Werte >0 sind keine Sync Pulse
           	if (!validSequence(&pattern[i][0],&pattern[p][0])) continue;
           	if ((syncMinFact* (pattern[i][0]) <= -1* (pattern[p][0]))) {//n>10 => langer Syncpulse (als 10*int16 darstellbar
                // Prüfe ob Sync und Clock valide sein können
                if (histo[p] > 6) continue;    // Maximal 6 Sync Pulse
                if (histo[i] < 10) continue;   // Mindestens 10 Clock Pulse

				// Prüfen ob der gefundene Sync auch als message [i, p] vorkommt
				uint8_t c = 0;
				while (c < messageLen)
				{
					if (message[c] == i && message[c+1] == p) break;
					c++;
				}

				if (c==messageLen) continue;	// nichts gefunden, also Sync weitersuchen

				// Sync wurde gefunden, Variablen setzen
                //clock = pattern[i][0];
                //sync = pattern[p][0];
                clock=i;
                sync=p;
                state = detecting;

                mstart=c;
                //tol = (int)round(clock*tolFact);

				// Delete Messagebits bevore detected Sync. because they are trash
				// Löschen deaktiviert, da eventuell zu viel fehlt

				//messageLen-=c;
				//memmove(message,message+c,sizeof(*message)*(messageLen));
				//calcHisto();  // Da etwas von der Nachricht entfernt wird, brauchen wir eine neue Histogramm berechnung

				#if DEBUGDETECT > 1
                //debug
                Serial.println();
                Serial.print("PD sync: ");
                Serial.print(pattern[i][0]); Serial.print(", ");Serial.print(pattern[p][0]);
                Serial.print(", TOL: ");Serial.print(tol);
                Serial.print(", sFACT: ");Serial.println(pattern[sync][0]/(float)pattern[clock][0]);
                #endif
                return true;
                //break;
           	}
        }
    }
    return false;


}


/* Detect without a Sync */
void patternDetector::doDetectwoSync() {
	//Serial.print("bitcnt:");Serial.println(bitcnt);
	#if DEBUGDETECT>0
	if (messageLen > maxMsgSize*8)
		Serial.println("Error, overflow in message Array");
        processMessage();
	#endif
	if (bitcnt >= 0) {//nächster Satz Werte (je 2 Neue) vollständig
		//Serial.println("doDetect");
		//Serial.print(*first); Serial.print(", ");Serial.println(*last);
		static uint8_t pattern_pos;
        bool valid;
        bool add_new_pattern=true;

		bitcnt = 0;

        valid=validSequence(first,last);
        valid&=!(messageLen+1 == maxMsgSize*8);
		if (pattern_pos > patternLen) patternLen=pattern_pos;
		if (messageLen ==0) valid=pattern_pos=patternLen=0;


	    int seq[2] = {1,0};
	    seq[1]=*first;
		int8_t fidx = findpatt(seq);

		#if DEBUGDETECT>3
		Serial.print(F("Pulse: "));Serial.print(*first); Serial.print(F(", "));Serial.print(*last);
		Serial.print(F(", TOL: ")); Serial.print(tol); Serial.print(F(", Found: ")); Serial.print(fidx);
		Serial.print(F(", pSeq: ")); Serial.print(seq[1]); Serial.print(F(", Vld: ")); Serial.print(valid);
		Serial.print(F(", mLen: ")); Serial.print(messageLen);
		#endif




		if (0<=fidx){
			//gefunden
			message[messageLen]=fidx;
			if (messageLen>1 && message[messageLen-1] == message[messageLen]) reset();  // haut Rauschen weg.
			//pattern[fidx][0] = (pattern[fidx][0]+seq[1])/2;
			messageLen++;
			add_new_pattern=false;
        }   else {
            // Prüfen ob wir noch Muster in den Puffer aufnehmen können oder ob wir Muster überschreiben würden
			calcHisto();
            if (patternLen==maxNumPattern && histo[pattern_pos] > 1)
            {
                valid=false;
                add_new_pattern=false;
            }
        }


        if (!valid && messageLen>=minMessageLen){

          success=true;
          processMessage();
          reset();  // GGF hier nicht ausführen.
          pattern_pos=0;
          //doDetectwoSync(); //Sichert den aktuellen Puls nach dem Reset, da wir ihn ggf. noch benötigen
          return;
        }

		/*else {
			if (messageLen>=minMessageLen){
				// Annahme, wir haben eine Nachricht empfangen, jetzt kommt rauschen, welches nicht zum Muster passt
				//printOut();
				//processMessage();
				//reset();pattern_pos=0;
			}
        */
        if (add_new_pattern)
        {
			// Löscht alle Einträge in dem Nachrichten Array die durch das hinzugügen eines neuen Pattern überschrieben werden
			// Array wird sozusagen nach links geschoben
			for (int16_t i=messageLen-1;(i>=0) ;--i)
			{
				if (message[i] == pattern_pos) // Finde den letzten Verweis im Array auf den Index der gleich überschrieben wird
				{
					i++; // i um eins erhöhen, damit zukünftigen Berechnungen darauf aufbauen können
					messageLen=messageLen-i; // Berechnung der neuen Nachrichtenlänge nach dem Löschen
					memmove(message,message+i,sizeof(*message)*(messageLen+1));
                    break;
                }
			}
			pattern[pattern_pos][0] = seq[1];						//Store pulse in pattern array
			message[messageLen]=pattern_pos;
            #if DEBUGDETECT>3
            Serial.print(F(", pattPos: ")); Serial.print(pattern_pos);
            #endif // DEBUGDETECT
			//*(message+messageLen) = patternLen; 					//Index des letzten Elements in die Nachricht schreiben

			messageLen++;
			pattern_pos++;

			//printOut();
			if (pattern_pos==maxNumPattern)
			{
				pattern_pos=0;  // Wenn der Positions Index am Ende angelegt ist, gehts wieder bei 0 los und wir überschreiben alte pattern
				patternLen=maxNumPattern;
			}
			/*
			if (pattern_pos==maxNumPattern)
			{
				pattern_pos=0;  // Wenn der Positions Index am Ende angelegt ist, gehts wieder bei 0 los und wir überschreiben alte pattern
				patternLen=maxNumPattern;
			} else {

				patternLen++;
			}
			*/
			/*
			DEBUG_BEGIN(2)
			printOut();
			DEBUG_END
			*/
		}
		#if DEBUGDETECT>3
		Serial.println();
		#endif // DEBUGDETECT
	} else { //zweiten Wert für Muster sammeln
		bitcnt++;
	}





}

/* Berechnet Werte für ein Histogramm aus den Daten des arrays message */
void patternDetector::calcHisto()
{
    for (uint8_t i=0;i<maxNumPattern;++i)
    {
        histo[i]=0;
    }

    //Serial.print(F("Calc histo"));
    for (uint8_t i=0;i<messageLen;++i)
    {
        histo[message[i]]++;
    }
}

void patternDetector::printOut() {
    Serial.println();
	Serial.print("Sync: ");Serial.print(pattern[sync][0]);
	Serial.print(" -> SyncFact: ");Serial.print(pattern[sync][0]/(float)pattern[clock][0]);
	Serial.print(", Clock: "); Serial.print(pattern[clock][0]);
	Serial.print(", Tol: "); Serial.print(tol);
	Serial.print(", PattLen: "); Serial.print(patternLen); Serial.print(" ");
	Serial.print(", Pulse: "); Serial.print(*first); Serial.print(", "); Serial.print(*last);
	Serial.print(", mStart: "); Serial.print(mstart);

	Serial.println();Serial.print("Signal: ");
	for (uint8_t idx=0; idx<messageLen; ++idx){
		Serial.print(*(message+idx));
	}
	Serial.print(". ");Serial.print(" [");Serial.print(messageLen);Serial.println("]");
	Serial.print("Pattern: ");
	for (uint8_t idx=0; idx<patternLen; ++idx){
        Serial.print(" P");Serial.print(idx);
        Serial.print(": "); Serial.print(histo[idx]);  Serial.print("*[");
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
}

void patternDetector::processMessage()
{
        calcHisto();
        if (messageLen >= minMessageLen)
        {
            getSync();
            #ifdef DEBUGDETECT >1
            printOut();
            #endif
        }
}

/* Searches a pattern in the detected message
key[0] = Anzahl der Elemente in key
key[1..n] = Zu suchende Elemente
*/

bool patternDetector::isPatternInMsg(int *key)
{
    bool valid = true;
    //bool found = false;

    uint8_t i,p=0;
    for(i=1; i<=key[0];++i)
    {
        //found=false;
        if (0<=getPatternIndex(key[i]))
			valid &= true;
		else
			valid &= false;
        /*
        for(p=0; p<maxNumPattern;++p)
        {

            if (found=inTol(key[i],pattern[p][0]))  break;
        }
        valid &=found;
		*/
    }
    return valid;
}

/* Searches a pattern in the detected message and returns the index where the pattern is found. returns -1 if it's not found */
int8_t patternDetector::getPatternIndex(int key)
{
	//const int tolerance= abs(key)>abs(clock)*13 ? abs(key*0.1) : abs(key*0.3);
	//const uint16_t tolerance= abs(key)>abs(clock)*13 ? tol*10 : tol;
	const uint16_t tolerance= key>10 ? 3: 1;
	//Serial.print("  tol: "); Serial.println(tolerance);
//valid &= inTol(round(sync/(float)clock), match.syncFact, 3); //sync in tolerance


	for(uint8_t p=0; p<patternLen;p++)
    {
		//if (inTol(key,pattern[p][0]),tolerance) return p;
		if (inTol(key,pattern[p][0]/(float)(pattern[clock][0]),tolerance)) return p;
	}
	return -1;
}



//-------------------------- Decoder -------------------------------

patternDecoder::patternDecoder(): patternDetector() {
/*
	tol = 200;
	tolFact = 0.5;
*/
}

bool patternDecoder::decode(int* pulse) {
	return detect(pulse);
}

void patternDecoder::processMessage()
{
	if (messageLen < minMessageLen) return; //mindestlänge der Message prüfen
	//Serial.println("Message decoded:");
	patternDetector::processMessage();//printOut();


	if (clock || sync)// Wenn sync oder clock <> index 0 ist, dann haben wir ein brauchbares Signal mit Sync.
	{
		// Setup of some protocol identifiers, should be retrieved via fhem in future
		protoID[0]=(s_sigid){-4,-8,-18,500,0,twostate}; // Logi
		protoID[1]=(s_sigid){-4,-8,-18,500,0,twostate}; // TCM 97001
		protoID[2]=(s_sigid){-1,-2,-18,500,0,twostate}; // AS
		protoID[3]=(s_sigid){-1,3,-30,pattern[clock][0],0,tristate}; // IT old


		uint8_t mend=mstart+2;   // GGf. kann man die Mindestlänge von x Signalen vorspringen
		bool m_endfound=false;

		uint8_t repeat;
		do {
			if (message[mend]==clock  && message[mend+1]==sync) {
				mend-=2;
				m_endfound=true;
				break;
			}
			mend+=2;
		} while ( mend<(messageLen));


		#if DEBUGDECODE > 1
		Serial.print("Index: ");
		Serial.print("SC: "); Serial.print(sync);
		Serial.print(", CP: "); Serial.print(clock);
		Serial.print(" - MEnd: "); Serial.println(mend);
		#endif // DEBUGDECODE

		if (m_endfound)
		{
			for (uint8_t i=0; i<4;i++)
			{
				#ifdef DEBUGDECODE
				Serial.print("ID:");Serial.print(i);Serial.print(" - ");;
				#endif
				if (checkSignal(protoID[i]))
				{
					/*				Output raw message Data				*/
					String preamble;

					preamble.concat('\n');

					for (uint8_t idx=0;idx<=patternLen;idx++)
					{
                        preamble.concat('P');preamble.concat(idx);preamble.concat(SERIAL_DELIMITER);
                        preamble.concat(pattern[idx][0]);preamble.concat(SERIAL_DELIMITER);
					}
					preamble.concat('M'); preamble.concat(i); preamble.concat(SERIAL_DELIMITER);

					String postamble;
					postamble.concat(SERIAL_DELIMITER); postamble.concat('\n');

					printMsgRaw(mstart,mend,preamble,postamble);
					success = true;
				} else
					Serial.println();
			}
		}
	} else {
		// Signale ohne Sync

		//ITTX

		// Manchster

		// ETC

	}
}

void patternDecoder::printMsgRaw(uint8_t m_start, uint8_t m_end, String preamble,String postamble)
{
	Serial.print(preamble);
	for (;m_start<m_end;m_start++)
	{
		Serial.print(message[m_start]);
	}
	Serial.print(postamble);
}


/*
// Function needs to be renamed
Checks Clock, Sync, low and high puls fact

Returns true if all values match
*/
bool patternDecoder::checkEV1527type(s_sigid match){
	bool valid = true;
	/*valid &= messageLen==Length;
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	*/
	if (match.clock !=0 )
		valid &= inTol(pattern[clock][0], match.clock);//clock in tolerance
	#ifdef DEBUGDECODE
	Serial.print(valid);
	#endif
	valid &= inTol(round(pattern[sync][0]/(float)pattern[clock][0]), match.syncFact, 3); //sync in tolerance
	#ifdef DEBUGDECODE
	Serial.print(valid);
	#endif

	int check_vals[3] = {2,0,0};
	/*
	check_vals[1]= match.clock;
	check_vals[2]= match.lowFact* match.clock;
	*/
	check_vals[1]= 1; // Clock fact
	check_vals[2]= match.lowFact;

	valid &= isPatternInMsg(check_vals);
//	valid &= inTol(*(pattern+1), -lowFact* clockTst); //p0=[ ,-nc]
	#ifdef DEBUGDECODE
	Serial.print(valid);
	#endif
	check_vals[2]=  match.highFact; //* match.clock;
	valid &= isPatternInMsg(check_vals);
//	valid &= inTol(*(pattern+3), -highFact* clockTst); //p1=[ ,-mc]
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	return valid;
}

bool patternDecoder::checkSignal(const s_sigid s_signal)
{
	bool valid = checkEV1527type(s_signal);
	return valid;  // Exit here, let's do all decoding Stuff outside


	if (valid && s_signal.messageType == twostate)
	{
		// Get Index for the message Pattern
		const s_pidx p_idx = {getPatternIndex(s_signal.lowFact),getPatternIndex(s_signal.highFact),clock};

		#if DEBUGDECODE > 1
		Serial.print(F("Index: ")); Serial.print("CP: "); Serial.print(clock);
		Serial.print(F(", SP: ")); Serial.print(p_idx.lf_idx);
		Serial.print(F(", LP: ")); Serial.println(p_idx.hf_idx);
		#endif // DEBUGDECODE

		/*
		valid &= (twoStateMessageBytes(p_idx) == s_signal.len);
		#ifdef DEBUGDECODE
			Serial.print(valid);
		#endif
		*/
		//valid &= ((byteMessage[0]&0b11110000)==0b01010000); //first bits has to be 0101
	}
	else if (valid && s_signal.messageType == tristate)
	{
		// Get Index for the message Pattern
		//const s_pidx p_idx = {getPatternIndex(-s_signal.lowFact*s_signal.clock),getPatternIndex(-s_signal.highFact*s_signal.clock),getPatternIndex(s_signal.clock)};
		/*
		const s_pidx p_idx = {getPatternIndex(s_signal.lowFact*s_signal.clock),
							  getPatternIndex(s_signal.highFact*s_signal.clock),
							  getPatternIndex(s_signal.clock),
							  getPatternIndex(s_signal.syncFact*s_signal.clock)  //Todo:  Toleranz ist hierfür zu gering
							  };
		*/
		const s_pidx p_idx = {getPatternIndex(s_signal.lowFact),
							  getPatternIndex(s_signal.highFact),
							  clock,
							  sync  //Todo:  Toleranz ist hierfür zu gering
							  };


		#if DEBUGDECODE > 1
		Serial.print(F("Index: "));
		Serial.print("SC: "); Serial.print(p_idx.sc_idx);
		Serial.print(", CP: "); Serial.print(p_idx.ck_idx);
		Serial.print(F(", SP: ")); Serial.print(p_idx.lf_idx);
		Serial.print(F(", LP: ")); Serial.println(p_idx.hf_idx);
		#endif // DEBUGDECODE
/*
		valid &= (triStateMessageBytes(p_idx) == s_signal.len);
		valid &= (printTristateMessage(p_idx) == s_signal.len);
		#ifdef DEBUGDECODE
			Serial.print(valid);
		#endif
*/
		//valid &= ((byteMessage[0]&0b11110000)==0b01010000); //first bits has to be 0101
	}

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
/*
	bool valid = checkEV1527type(500, 18, 1, 2, 32); //14 kommt hier nicht durch =>12
	twoStateMessageBytes();
	if (valid) {//ok, it's AS
		Serial.print("AS");
		printMessageHexStr();
		success = true;
	}
*/
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
	bool valid = checkSignal(protoID[0]);
	//valid &= ((byteMessage[0]&0b11110000)==0b01010000); //first bits has to be 0101
	#ifdef DEBUGDECODE
		Serial.println(valid);
	#endif

	if (valid) {//ok, it's Logilink
		//byteMessage[byteMessageLen-1] <<=3; //shift last bits to align left in bitsequence
		Serial.print("W03");
		printMessageHexStr();
		success = true;
	}
}


void patternDecoder::checkTCM97001(){
/*
TCM Art. Nr. 97001:
	clock:	 		500
	Sync factor: 	18
	start sequence: [500, -9000] => [1, -18]
	high pattern:	[500, -4000] => [1, -8]
	low pattern:	[500, -2000] => [1, -4]
	message length:	24 bit
	characteristic:	first 8 bits are sensor id, which is resettet every time the battery is changed
					Bit 9  			Battery
					Bit 10			unknown
					Bit 11 & 12	 	positiv (00) or negativ (11) temperature
					Bit 13-22 	 	Temperature / 10 if positiv or Temp^0x3fe/10 if negativ
					Bit 23			unknown may be a checksum?
					Bit 24 			transmitting auto =0 or manual =1
*/

	const s_sigid tcmSignal = {4,8,18,500};
	bool valid = checkEV1527type(tcmSignal);
	if (valid)
	{
		// Get Index for the message Pattern
		const s_pidx tcmPattern = {getPatternIndex(-tcmSignal.lowFact*clock),getPatternIndex(-tcmSignal.highFact*clock),getPatternIndex(clock)};
		#if DEBUGDECODE > 1
	//	Serial.print(F("Index: ")); Serial.print("CP: "); Serial.print(ck_idx);
	//	Serial.print(F(", SP: ")); Serial.print(sp_idx);
	//	Serial.print(F(", LP: ")); Serial.println(lp_idx);
		#endif // DEBUGDECODE
		valid &= (twoStateMessageBytes(tcmPattern) == 24);
	}
	if (valid) {//ok, it's TCM97001
		Serial.print("W08");
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
/*
	bool valid = checkEV1527type(270, 10, 1, 5, 64);
	// two bits in Message give one final bit:
	// 01 => 0, 10 => 1
	if (valid) {//ok, it's new IT selflearning
		Serial.print("AR");
		printNewITMessage();
		Serial.println();
		success = true;
	}
	*/
}

void patternDecoder::checkITold() {
/*
IT old with selector:
	clock: 		400 -> Variable Clock
	Sync factor: 	(31)  27-33
	start sequence: [400, -13200] => [1, -31]
	high pattern:	[1200, -400][1200, -400] => [3, -1][3, -1]
	low pattern:	[400, -1200][400, -1200] => [1, -3][1, -3]
	float pattern:	[400, -1200][1200, -400] => [1, -3][3, -1]
	message length:	12 bit
	characteristic: IT only uses 2 states: low, float
					clock varies between controls from 340 to 400 => using measured clock leads to lower (required) tolerances
*/
	bool valid = true;
	/*valid &= messageLen==24;
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	*/
	/*
	int clockTst = 360;
	valid &= inTol(clock, clockTst);
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
	*/

	valid &= inTol(abs(sync)/clock, 30, 3); //syncValues
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
//	valid &= (inTol(*(pattern+0),3*clock) & inTol(*(pattern+1),-1*clock));//p0=[3, -1]
	#ifdef DEBUGDECODE
		Serial.print(valid);
	#endif
//	valid &= (inTol(*(pattern+2),1*clock) & inTol(*(pattern+3),-3*clock));//p1=[1, -3]
	#ifdef DEBUGDECODE
		Serial.println(valid);
	#endif
	if (valid){
		//printOut();
		triStateMessageBytes();
		byteMessage[byteMessageLen-1] <<=4; //shift last bits to align left in bitsequence
		Serial.print("IR");
	//	printTristateMessage();
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

uint8_t patternDecoder::printTristateMessage(const s_pidx s_patt){
	/* Message Code
		0: [1, -3], [1, -3] => message 1,1
		1: [3, -1], [3, -1] => message 0,0
		F: [1, -3], [3, -1] => message 1,0
	*/
	char msgChar;
	uint8_t chrcnt=0;
	//for (byte idx=0; idx<messageLen; idx +=2){
	for(uint8_t idx=mstart+2; idx<messageLen; idx+=4) {

		if (message[idx] == s_patt.ck_idx && message[idx+1] == s_patt.sc_idx)  // Ende neuer Sync oder sonst was
			break;
		else if (message[idx] == s_patt.hf_idx && message[idx]==message[idx+2] && message[idx+1] == message[idx+3] )
			msgChar = '1';
		else if (message[idx] == s_patt.ck_idx && message[idx]==message[idx+2] && message[idx+1] == message[idx+3] )
			msgChar = '0';
		else if (message[idx] == s_patt.ck_idx && message[idx] != message[idx+2] && message[idx+1] != message[idx+3] )
			msgChar = 'F';
		Serial.print(msgChar);
		chrcnt++;
	}
	return chrcnt;
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
uint8_t patternDecoder::twoStateMessageBytes(const s_pidx s_patt){
	byte byteCode = 0;
	byte byteCnt = 0;
	uint8_t idx=0;
	uint8_t bitcnt=0;
	// First and second message Index is sync
	for(idx=mstart+2; idx<messageLen; ++idx) {
		byteCode <<= 1;
		if (message[idx]!=s_patt.ck_idx) break;			// no Clock, abort

		idx++;
		if (message[idx]==s_patt.hf_idx){
			byteCode |=1;						// Highpuls = 1
			#if DEBUGDECODE >1
			Serial.print(1);
			#endif
		} else if (message[idx]!=s_patt.lf_idx){
			break;								// No low Puls, abort
		} else {
			#if DEBUGDECODE >1
			Serial.print(0);
			#endif
		}
        bitcnt++;

		if (bitcnt==8){ //byte full
            bitcnt=0;
			//Serial.println(byteCode,HEX);
			#if DEBUGDECODE >1
			Serial.print(" ");
			#endif
			byteMessage[byteCnt]=byteCode;
			byteCnt++;
			byteCode = 0;
 		}
	}

	if (bitcnt != 0) {//non full byte
		//Serial.println(byteCode,HEX);
		byteCode <<= 7-bitcnt; // Align Bits left
		byteMessage[byteCnt]=byteCode;
		byteMessageLen = byteCnt+1;
	} else
		byteMessageLen = byteCnt;
	#if DEBUGDECODE >1
	Serial.print(" IDX: "); Serial.print(idx);
	Serial.print(", bitcnt: "); Serial.print(bitcnt + (byteCnt*8));
	Serial.println();
	#endif
	return (bitcnt + (byteCnt*8) );  // Return number of detected bits
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


ManchesterpatternDetector::ManchesterpatternDetector(bool silentstate):patternBasic(){
	//tol = 200; //
	tolFact = 5.0;
	//patternStore = new BitStore(2,0); // Init our Patternstore, with 0 bytes, because we do not use it
	free(patternStore);
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
#if DEBUGDETECT==255
        Serial.print("reinit clock math...");
#endif // DEBUGDETECT
        reset();
        clock=abs(*pulse)/2;
        sample=1;
        average=clock;
    }
    else if (abs(*pulse) < int(clock*0.5))
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



/*
  Updates the Mancheser Clock witch given clock
*/
void ManchesterpatternDetector::updateClock(int newClock)
{
    /* Todo Auswetung der Pulse kann in die Schleife doDetect verlagert werden */
    static uint8_t sample=0;
    static uint32_t average=0;      // Need 32 Bit, because 16 bit it will overflow shortly.



    if (clock == 0) // Reset the counter
    {
#if DEBUGDETECT==255
        Serial.print("reinit clock to 0...");
#endif // DEBUGDETECT
        //reset();
        sample=0;
        average=clock=0;
	}

	if (sample > 48) return;		// Use max 48 Samples for the clock calculation. Then save cpu cycles

    average=average+abs(newClock);
    sample++;
    clock=average/sample;
#if DEBUGDETECT==255
    Serial.print("  recalc Clock with :");
    Serial.print(newClock);
    Serial.print(" to clock :");
#endif // DEBUGDETECT
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

	// Assume that the first pulse is a long one, clock must be half of the pulse
    updateClock((*last)/2);
    tol = (int)round(clock*tolFact/10);

    // If sencond pulse does not fit to out assumption, recalc our clock
    if ((!isLong(first)) && (!isShort(first)))
    {
		clock=clock*2;
		tol = (int)round(clock*tolFact/10);
    }

    if (isLong(first) || isShort(first))
    {
      // Valid
        state = detecting;
        //updateClock(first);
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
    //updateClock(first);


    // May a valid manchester pulse
    int seq[3] = {0,0,0};
    if (isLong(first))
    {
        // valid it is a long pulse
        updateClock(*first/2);
        seq[0] = 1;
        seq[1] = *first;
        //ManchesterBits->addValue(!(*first & (1<<7))); // Check if bit 7 is set
        //Serial.println(*first,BIN);
        //ManchesterBits->addValue((*first >>15)? 0 :1 ); // Check if bit 7 is set
        ManchesterBits->addValue(!(*first >>15)); // Check if bit 7 is set
    }
    else if(isShort(first) && isShort(last) )
    {
		updateClock(*first);
		updateClock(*last);
         // valid
        seq[0] = 2;
        seq[1] = *first;
        seq[2] = *last;
        //updateClock(last); // Update Clock also with last pulse, because we will skip this in the next iteration
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

decoderBasic::decoderBasic()
{
}

String decoderBasic::getMessageHexStr()  const                    // Returns a Pointer the hex message on serial
{
	return protomessage;
}

String decoderBasic::valToHex(unsigned char Value, uint8_t desiredStringLength) {


  String hexStr = String(Value, HEX);
  while (hexStr.length() < desiredStringLength) hexStr = "0" + hexStr;
  return hexStr;
}

bool decoderBasic::decode(){
    if (mcdetector->manchesterfound()) {
        message_decoded= processMessage();
        return message_decoded;
    }
    return false;
}

/*
	Check if the clock and the bitlength fit's our protocol
*/
bool decoderBasic::checkMessage(uint16_t min_clock, uint16_t max_clock, uint8_t min_Length,uint8_t max_Length)
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
bool decoderBasic::checkSync(unsigned char pattern, uint8_t startpos, uint8_t mincount,uint8_t maxcount,uint8_t *syncend)
{
	bool valid=true;

    uint8_t twobit=0; // Check two bits of the sync signal
    const uint8_t endcount = startpos+maxcount;
    uint8_t idx;
	for (idx=startpos; idx<endcount;idx+=2)
	{
		twobit = getDataBits(idx,2);
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
unsigned char decoderBasic::getNibble(uint8_t startingPos)
{
    const uint8_t nibsize=4;
    return getDataBits(startingPos,nibsize);
}
/*
returns n bits , begins at position deliverd via startingPos, returns numbits Maximum is 8 bits.
returns the nibble in received order
*/
unsigned char decoderBasic::getDataBits(uint8_t startingPos,uint8_t numbits)
{
    if (numbits>8) numbits=8;					//Max is 8 bits
    uint8_t bcnt=numbits-1;						//Calculates the bitposition for the first bit
    unsigned char cdata=0;
	for (uint8_t idx=startingPos; idx<startingPos+numbits; idx++, bcnt--){
       cdata = cdata | (mcdetector->ManchesterBits->getValue(idx) << (bcnt)); // add bits in received order
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
	valid = decoderBasic::checkMessage(440,540,150,220);							// Valid clock and length
	//valid &= checkSync(uint8_t (0),uint8_t (24),uint8_t (33),&syncend);			// Valid sync sequence
	valid &= decoderBasic::checkSync(0x2,0,24,33,&syncend);							// Valid sync sequence ia 24-33 bits like 10
#ifdef DEBUGDECODE
	Serial.print(valid);
#endif
	valid &= (getNibble(syncend) == 0xA);  											// Valid preamble
#ifdef DEBUGDECODE
	Serial.print(valid);
	Serial.println();
#endif
	return  valid;

	// We may have started detecting the signal after the sync. There is an option, that there is another sync in our message, but may not the complete signal.
	// Todo Die ganze Nachricht anch einem Sync absuchen
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

  		this->protomessage.concat(valToHex(cdata));
  		//this->protomessage.concat(String(cdata,HEX));
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
returns numbit bits one , begins at position deliverd via startingPos
skips every second bit and returns the bits in reversed (correct for OSV2) order
*/
unsigned char OSV2Decoder::getDataBits(uint8_t startingPos,uint8_t numbits)
{
    uint8_t bcnt=0;
    unsigned char cdata=0;

    if (numbits>8) numbits=8;				// Max 8 bits can be retuned (one byte)
    numbits=numbits*2;

	for (uint8_t idx=startingPos; idx<startingPos+numbits; idx=idx+2, bcnt++){
         cdata = cdata | (mcdetector->ManchesterBits->getValue(idx) << (bcnt)); // add bits in reversed order
	}
    return cdata;
}



/*
returns the 4 bits one nibble, begins at position deliverd via startingPos
skips every second bit and returns the nibble in reversed (correct) order
*/
unsigned char OSV2Decoder::getNibble(uint8_t startingPos)
{
    return getDataBits(startingPos,4);
}

/*
returns the 8 bits / one byte, begins at position deliverd via startingPos
skips every second bit and returns the byte in reversed (correct) order.
// Todo use getNibble to return the byte
*/
unsigned char OSV2Decoder::getByte(uint8_t startingPos)
{
     return getDataBits(startingPos,8);
}



ASDecoder::ASDecoder(ManchesterpatternDetector *detector) :decoderBasic()
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
	valid = decoderBasic::checkMessage(380,425,52,56);							// Valid clock and length
	valid &= decoderBasic::checkSync(0x2,0,8,12,&syncend);						// Searching for sync bits, 8-12 bits must be sync pattern 0x2 = 10
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
	this->protomessage.reserve((numbits/4)+3); 							 	 // Reserve buffer for Hex String
	this->protomessage= String("AS");
#ifdef DEBUGDECODE
	Serial.print("AS detected with len("); Serial.print(numbits); Serial.print(") :");
#endif
	char hexStr[] ="00";
	byte crcv=0x00;
	byte b=0;
	// Check the Data Bits
	for (idx=syncend+4; idx<mcdetector->ManchesterBits->valcount-8; idx=idx+8){
		b=getDataBits(idx,8);
		sprintf(hexStr, "%02X",b);						// Migrate to valToHex to save some memory
		#ifndef ARDUSIM
		crcv = _crc_ibutton_update(crcv,b);
		#endif
#ifdef DEBUGDECODE
		Serial.print(String(b,HEX));
#endif
		this->protomessage.concat(hexStr);
	}

	if (crcv == (getDataBits(idx,8))) {
#ifdef DEBUGDECODE
		Serial.print("  CRC valid: ");
		Serial.print(crcv,HEX);
#endif
	} else {
#ifdef DEBUGDECODE
		Serial.print("  CRC not valid: ");
		Serial.println(crcv,HEX);
#endif
		return false;
	}
#ifdef DEBUGDECODE

	Serial.println();
#endif
	return true;
}

