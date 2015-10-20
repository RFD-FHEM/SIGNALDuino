/*
*   Pattern Decoder Library V1
*   Library to decode radio signals based on patternd detection
*   2014-2015  N.Butzek, S.Butzek

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
	patternStore = new BitStore(2,1); // Init our Patternstore, default is 10 bytes. So we can save 40 Values.

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
	clock = -1;
	patternStore->reset();
    tol=0;
	for (int i=0;i<maxNumPattern;i++)
        pattern[i][0]=0;

}


bool patternBasic::detect(const int* pulse){
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



bool patternBasic::inTol(const int value, const int set){
	return inTol(value, set, tol);
}

bool patternBasic::inTol(const int value, int set, const int tolerance){
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
bool patternBasic::validSequence(const int *a, const int *b)
{

    //return ((*a> 0 and 0 > *b) or (*b > 0  and 0 > *a));
    //return ((*a)*(*b)<0);
    return ((*a ^ *b) < 0); // true if a and b have opposite signs

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
	sync=-1;
	reset();
}

void patternDetector::reset() {
	patternBasic::reset();
	messageLen = 0;
	patternLen = 0;
	bitcnt = 0;
	state = searching;
	clock = sync=-1;
	for (uint8_t i=0; i<maxNumPattern;++i)
		histo[i]=pattern[i][0]=0;
	success = false;
	tol = 150; //
	tolFact = 0.3;
	mstart=0;
	m_truncated=false;
	m_overflow=false;
	//Serial.println("reset");
}

const status patternDetector::getState()
{
	return state;
}




bool patternDetector::detect(const int* pulse){
	patternBasic::detect(pulse);   // Call function from base class, to do some basic stuff
    doDetectwoSync();			   // Search for signal

	return success;
}

/** @brief (Checks is stored message can be a valid signal)
  *
  * (documentation goes here)
  */
bool patternDetector::validSignal()
{
	if (messageLen < minMessageLen) return false;

	//if (patternLen < 3)	return false;
	/*uint8_t unequal_pulse_cnt=0;
	for (uint8_t i=0;i<patternLen;i++)
	{
		if (histo[i] > 10) unequal_pulse_cnt++;		// Count all signaltypes in hisogram with more than 10 pulses.
	}
	if (unequal_pulse_cnt <3)	return false;
	*/


	return true;									// Message seems to be valid
}

bool patternDetector::getSync(){
    // Durchsuchen aller Musterpulse und prüft ob darin ein Sync Faktor enthalten ist. Anschließend wird verifiziert ob dieser Syncpuls auch im Signal nacheinander übertragen wurde
    //
	#if DEBUGDETECT > 3
	Serial.println("  --  Searching Sync  -- ");
	#endif

	if (state == clockfound)		// we need a clock to find this type of sync
	{
		// clock wurde bereits durch getclock bestimmt.
		for (int8_t p=patternLen-1;p>=0;--p)  // Schleife für langen Syncpuls
		{
			if (pattern[p][0] > 0 || (abs(pattern[p][0]) > syncMaxMicros && abs(pattern[p][0])/pattern[clock][0] > syncMaxFact))  continue;  // Werte >0 oder länger maxfact sind keine Sync Pulse
			//if (pattern[p][0] > 0 || pattern[p][0] == -1*maxPulse)  continue;  // Werte >0 sind keine Sync Pulse
			if (!validSequence(&pattern[clock][0],&pattern[p][0])) continue;
			if ((syncMinFact* (pattern[clock][0]) <= -1*pattern[p][0])) {//n>9 => langer Syncpulse (als 10*int16 darstellbar
				// Prüfe ob Sync und Clock valide sein können
				if (histo[p] > 6) continue;    // Maximal 6 Sync Pulse  Todo: 6 Durch Formel relativ zu messageLen ersetzen

				// Prüfen ob der gefundene Sync auch als message [clock, p] vorkommt
				uint8_t c = 0;
				while (c < messageLen)		// Todo: Abstand zum Ende berechnen, da wir eine mindest Nachrichtenlänge nach dem sync erwarten, brauchen wir nicht bis zum Ende suchen.
				{
					//if (message[c] == clock && message[c+1] == p) break;
					//if (message[c] == p && message[c-1] == clock) break;   // Faster version as bevore, but does not work
					if (message[c+1] == p && message[c] == clock ) break;	// Fast as before and works.
					c++;
				}

				if (c==messageLen) continue;	// nichts gefunden, also Sync weitersuchen

				// Sync wurde gefunden, Variablen setzen
				//clock = pattern[i][0];
				//sync = pattern[p][0];
				sync=p;
				state = syncfound;

				mstart=c;

				#ifdef DEBUGDECODE
				//debug
				Serial.println();
				Serial.print("PD sync: ");
				Serial.print(pattern[clock][0]); Serial.print(", ");Serial.print(pattern[p][0]);
				Serial.print(", TOL: ");Serial.print(tol);
				Serial.print(", sFACT: ");Serial.println(pattern[sync][0]/(float)pattern[clock][0]);
				#endif
				return true;
				//break;
			}
		}
	}

/*
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
    */
	sync=-1; // Workaround for sync detection bug.
    return false;


}


bool patternDetector::getClock(){
    // Durchsuchen aller Musterpulse und prüft ob darin eine clock vorhanden ist
	#if DEBUGDETECT > 3
	Serial.println("  --  Searching Clock in signal -- ");
	#endif
	int tstclock=-1;

	clock=-1; // Workaround for sync detection bug.

    for (uint8_t i=0;i<patternLen;++i) 		  // Schleife für Clock
    {
		//if (pattern[i][0]<=0 || pattern[i][0] > 3276)  continue;  // Annahme Werte <0 / >3276 sind keine Clockpulse
		if (tstclock==-1 && (pattern[i][0]>=0) && (histo[i] > messageLen*0.17) )
		{
			tstclock = i;
			continue;
		}

		if ((pattern[i][0]>=0) && (pattern[i][0] < pattern[tstclock][0]) && (histo[i] > messageLen*0.17)){
			tstclock = i;
		}
    }


	// Check Anzahl der Clockpulse von der Nachrichtenlänge
	//if ((tstclock == 3276) || (maxcnt < (messageLen /7*2))) return false;
	if (tstclock == -1) return false;

	clock=tstclock;

	// Todo: GGf. andere Pulse gegen die ermittelte Clock verifizieren

	state=clockfound;
	return true;
}





/* Detect without a Sync */
void patternDetector::doDetectwoSync() {
	//Serial.print("bitcnt:");Serial.println(bitcnt);

	if (messageLen+1 >= maxMsgSize*8) {
		#if DEBUGDETECT>0
		Serial.println("Error, overflow in message Array");
		#endif
		m_overflow=true;
	    processMessage();
        m_truncated&=false;
	    //reset(); // Moved reset to processMessage function if needed
	}
	if (bitcnt >= 0) {//nächster Satz Werte (je 2 Neue) vollständig
		//Serial.println("doDetect");
		//Serial.print(*first); Serial.print(", ");Serial.println(*last);
		static uint8_t pattern_pos;
        bool valid;
        bool add_new_pattern=true;
		bitcnt = 0;
        valid=validSequence(first,last);
        valid &= (messageLen+1 == maxMsgSize*8) ? false : true;
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
			pattern[fidx][0] = (pattern[fidx][0]+seq[1])/2; // Moving average
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
		  //Serial.println("not valid, processing");

          success=true;
          processMessage();
         // reset();  // GGF hier nicht ausführen.
          pattern_pos=0;
          //doDetectwoSync(); //Sichert den aktuellen Puls nach dem Reset, da wir ihn ggf. noch benötigen
          return;
        } else if (!valid) {
			reset();
			success=false;
			pattern_pos=0;
        } else if (valid && messageLen>=minMessageLen){
			state=detecting;  // Set state to detecting, because we have more than minMessageLen data gathered, so this is no noise
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
void patternDetector::calcHisto(const uint8_t startpos, uint8_t endpos)
{
    for (uint8_t i=0;i<maxNumPattern;++i)
    {
        histo[i]=0;
    }

	if (endpos==0) endpos=messageLen;

    for (uint8_t i=startpos;i<endpos;++i)
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
        compress_pattern();
        calcHisto();
        if (messageLen >= minMessageLen)
        {
            getClock();
            getSync();
            #if DEBUGDETECT >= 1
            printOut();
            #endif
        }
}


/** @brief (compares pattern if they are equal after moving average calculation and reduces within tolerance)
  *
  * (documentation goes here)
  */
void patternDetector::compress_pattern()
{

	calcHisto();
	for (uint8_t idx=0; idx<patternLen;idx++)
	{
		for (uint8_t idx2=idx+1; idx2<patternLen;idx2++)
		{
			if (inTol(pattern[idx2][0],pattern[idx][0]))  // Pattern are very equal, so we can combine them
			{
				// Change val -> ref_val in message array
				for (uint8_t i=0;i<messageLen;i++)
				{
                    if (message[i]==idx2)
					{
						message[i]=idx;
                    }
				}

				#if DEBUGDETECT>2
				Serial.print("compr: ");Serial.print(idx2);Serial.print("->");Serial.print(idx);Serial.print(";");
				Serial.print(histo[idx2]);Serial.print("*");Serial.print(pattern[idx2][0]);
				Serial.print("->");
				Serial.print(histo[idx]);Serial.print("*");Serial.print(pattern[idx][0]);
				#endif // DEBUGDETECT


				int  sum = histo[idx] + histo[idx2];

				pattern[idx][0] = (pattern[idx][0]*histo[idx]/ sum)+(pattern[idx2][0]*histo[idx2]/ sum);
				//pattern[idx][0] = (pattern[idx][0]*float(histo[idx]/ sum))+(pattern[idx2][0]*float(histo[idx2]/ sum)); // Store the average of both pattern, may better to calculate the number of stored pattern in message
				//pattern[idx][0] = (pattern[idx][0]+pattern[idx2][0])/2;
				pattern[idx2][0]=0;

				#if DEBUGDETECT>2
				Serial.print(" idx:");Serial.print(pattern[idx][0]);
				Serial.print(" idx2:");Serial.print(pattern[idx2][0]);
				Serial.println(";");
				#endif // DEBUGDETECT

			}
		}
	}
	//calcHisto(); // recalc histogram
}


//-------------------------- Decoder -------------------------------

patternDecoder::patternDecoder(): patternDetector() {
/*
	tol = 200;
	tolFact = 0.5;
*/
	/*
	protoID[0]=(s_sigid){-4,-8,-18,500,0,twostate}; // Logi, TCM 97001 etc.
	protoID[1]=(s_sigid){0,0,-10,650,0,twostate}; // RSL
	protoID[2]=(s_sigid){-1,-2,-18,500,0,twostate}; // AS
	protoID[3]=(s_sigid){-1,3,-30,pattern[clock][0],0,tristate}; // IT old
	*/
	preamble.reserve(70);
	postamble.reserve(20);

	numprotos=0;
}
void patternDecoder::reset() {
    patternDetector::reset();
}

bool patternDecoder::decode(const int* pulse) {
	return detect(pulse);
}

void patternDecoder::processMessage()
{
	if (messageLen < minMessageLen ) return; //mindestlänge der Message prüfen
	//Serial.println("Message decoded:");
	patternDetector::processMessage();//printOut();


	if (state == syncfound)// Messages mit clock / Sync Verhältnis prüfen
	{

		// Setup of some protocol identifiers, should be retrieved via fhem in future

		uint8_t mend=mstart+2;   // GGf. kann man die Mindestlänge von x Signalen vorspringen
		bool m_endfound=false;

		//uint8_t repeat;
		do {
			if (message[mend]==clock  && message[mend+1]==sync) {
				mend-=1;					// Previus signal is last from message
				m_endfound=true;
				break;
			}
			mend+=2;
		} while ( mend<(messageLen));
		if (mend > messageLen) mend=messageLen;  // Reduce mend if we are behind messageLen
		calcHisto(mstart,mend);	// Recalc histogram due to shortened message


		#if DEBUGDECODE > 1
		Serial.print("Index: ");
		Serial.print(" MStart: "); Serial.print(mstart);
		Serial.print(" SYNC: "); Serial.print(sync);
		Serial.print(", CP: "); Serial.print(clock);
		Serial.print(" - MEFound: "); Serial.println(m_endfound);
		Serial.print(" - MEnd: "); Serial.println(mend);
		#endif // DEBUGDECODE
		if ((m_endfound && (mend - mstart) >= minMessageLen) || (!m_endfound && messageLen < (maxMsgSize*8)))//(!m_endfound && messageLen  >= minMessageLen))	// Check if message Length is long enough
		{
            #ifdef DEBUGDECODE
            #Serial.println("Filter Match: ");;
            #endif


			preamble="";
			postamble="";

			/*				Output raw message Data				*/
			preamble.concat(MSG_START);
			//preamble.concat('\n');
			preamble.concat("MS");   // Message Index
			//preamble.concat(int(pattern[sync][0]/(float)pattern[clock][0]));
			preamble.concat(SERIAL_DELIMITER);  // Message Index
			for (uint8_t idx=0;idx<patternLen;idx++)
			{
				if (histo[idx] == 0) continue;
				preamble.concat('P');preamble.concat(idx);preamble.concat("=");preamble.concat(pattern[idx][0]);preamble.concat(SERIAL_DELIMITER);  // Patternidx=Value
			}
			preamble.concat("D=");

			postamble.concat(SERIAL_DELIMITER);
			postamble.concat("CP=");postamble.concat(clock);postamble.concat(SERIAL_DELIMITER);    // ClockPulse
			postamble.concat("SP=");postamble.concat(sync);postamble.concat(SERIAL_DELIMITER);     // SyncPuöse
			if (m_overflow) {
				postamble.concat("O");
				postamble.concat(SERIAL_DELIMITER);
			}
			postamble.concat(MSG_END);
			postamble.concat('\n');

			printMsgRaw(mstart,mend,&preamble,&postamble);
			success = true;

			#ifdef mp_crc
			const int8_t crco = printMsgRaw(mstart,mend,&preamble,&postamble);

			if ((mend<messageLen-minMessageLen) && (message[mend+1] == message[mend-mstart+mend+1])) {
				mstart=mend+1;
				byte crcs=0x00;
				#ifndef ARDUSIM
				for (uint8_t i=mstart+1;i<=mend-mstart+mend;i++)
				{
					crcs  = _crc_ibutton_update(crcs,message[i]);
				}
				#endif
				if (crcs == crco)
				{
					// repeat found
				}
				//processMessage(); // Todo: needs to be optimized
			}
			#endif


		} else if (m_endfound == false && mstart > 1 && mend+1 >= maxMsgSize*8) // Start found, but no end. We remove everything bevore start and hope to find the end later
        {
        	Serial.print("copy");
            messageLen=messageLen-mstart; // Berechnung der neuen Nachrichtenlänge nach dem Löschen
			memmove(message,message+mstart,sizeof(*message)*(messageLen+1));
			m_truncated=true;  // Flag that we truncated the message array and want to receiver some more data
		} else {
            #ifdef DEBUGDECODE
			Serial.println(" Buffer overflow, flushing message array");
			#endif
			//Serial.print(MSG_START);
			//Serial.print("Buffer overflow while processing signal");
			//Serial.print(MSG_END);
            reset(); // Our Messagebuffer is not big enough, no chance to get complete Message

		}
	} else if (state == clockfound && messageLen >= minMessageLen) {


		// Message has a clock puls, but no sync. Try to decode this

		preamble="";
		postamble="";

		#if DEBUGDECODE > 1
		Serial.print("Clock found: ");
		Serial.print(", CP: "); Serial.print(clock);
		Serial.println("");
		#endif // DEBUGDECODE


		//String preamble;

		preamble.concat(MSG_START);

		ManchesterpatternDecoder mcdecoder(this);			// Init Manchester Decoder class

		mcdecoder.reset();

		if (mcdecoder.isManchester() && mcdecoder.doDecode())	// Check if valid manchester pattern and try to decode
		{
			String mcbitmsg;
			//Serial.println("MC");
			mcbitmsg ="D=";
			mcdecoder.getMessageHexStr(&mcbitmsg);
   		    //Serial.println("f");


			preamble.concat("MC");
			preamble.concat(SERIAL_DELIMITER);
			mcdecoder.getMessagePulseStr(&preamble);

			postamble.concat(SERIAL_DELIMITER);
			mcdecoder.getMessageClockStr(&postamble);

			postamble.concat(MSG_END);
			postamble.concat('\n');

			//preamble = String(MSG_START)+String("MC")+String(SERIAL_DELIMITER)+preamble;
			//printMsgRaw(0,messageLen,&preamble,&postamble);

			//preamble.concat("MC"); ; preamble.concat(SERIAL_DELIMITER);  // Message Index

			// Output Manchester Bits
			printMsgStr(&preamble,&mcbitmsg,&postamble);

			#if DEBUGDECODE == 1
			preamble = "MC";
			preamble.concat(SERIAL_DELIMITER);

			for (uint8_t idx=0;idx<patternLen;idx++)
			{
				if (histo[idx] == 0) continue;

				preamble.concat("P");preamble.concat(idx);preamble.concat("=");preamble.concat(pattern[idx][0]);preamble.concat(SERIAL_DELIMITER);  // Patternidx=Value
			}
			preamble.concat("D=");

			//String postamble;
			postamble = String(SERIAL_DELIMITER);
			postamble.concat("CP=");postamble.concat(clock);postamble.concat(SERIAL_DELIMITER);    // ClockPulse, (not valid for manchester)
			if (m_overflow) {
				postamble.concat("O");
				postamble.concat(SERIAL_DELIMITER);
			}

			postamble.concat(MSG_END);
			postamble.concat('\n');

			printMsgRaw(0,messageLen,&preamble,&postamble);
			#endif

		} else {

			//preamble = String(MSG_START)+String("MU")+String(SERIAL_DELIMITER)+preamble;

			preamble.concat("MU");
			preamble.concat(SERIAL_DELIMITER);

			for (uint8_t idx=0;idx<patternLen;idx++)
			{
				if (histo[idx] == 0) continue;

				preamble.concat("P");preamble.concat(idx);preamble.concat("=");preamble.concat(pattern[idx][0]);preamble.concat(SERIAL_DELIMITER);  // Patternidx=Value
			}
			preamble.concat("D=");

			//String postamble;
			postamble.concat(SERIAL_DELIMITER);
			postamble.concat("CP=");postamble.concat(clock);postamble.concat(SERIAL_DELIMITER);    // ClockPulse, (not valid for manchester)
			if (m_overflow) {
				postamble.concat("O");
				postamble.concat(SERIAL_DELIMITER);
			}
			postamble.concat(MSG_END);
			postamble.concat('\n');

			printMsgRaw(0,messageLen,&preamble,&postamble);
		}

		success = true;


	} else {
		success=false;
		//reset();
	}

	if (!m_truncated)
    {
        reset();
    }

}
/** @brief (Prints given strings in provided order)
  *
  * (documentation goes here)
  */
void patternDecoder::printMsgStr(const String *first, const String *second, const String *third)
{
	Serial.print(*first);
	Serial.print(*second);
	Serial.print(*third);
}

/** @brief (Convertes message array into string for serial output)
  *
  * (returns the crc value of the message)
  */

int8_t patternDecoder::printMsgRaw(uint8_t m_start, const uint8_t m_end, const String *preamble,const String *postamble)
{

	Serial.print(*preamble);
	//String msg;
	//msg.reserve(m_end-mstart);
	byte crcv=0x00;
	for (;m_start<=m_end;m_start++)
	{
		//msg + =message[m_start];
		//Serial.print((100*message[m_start])+(10*message[m_start])+message[m_start]);
		Serial.print(message[m_start]);
		#ifndef ARDUSIM
		crcv = _crc_ibutton_update(crcv,message[m_start]);
		#endif
	}
	//Serial.print(msg);
	Serial.print(*postamble);
	return crcv;
	//printMsgStr(preamble,&msg,postamble);
}




/*
********************************************************
************* Manchester DECODER class ***************
********************************************************
*/
/** @brief (Constructor for Manchester decoder. ref= object of type patternDetecor which is calling the manchester decoder)
  *
  * Initialisation of class MancheserpatternDecoder
  */

ManchesterpatternDecoder::ManchesterpatternDecoder(patternDecoder *ref_dec)
{
	pdec = ref_dec;
	ManchesterBits=new BitStore(1,30); // use 1 Bit for every value stored, reserve 30 Bytes = 240 Bits
	reset();
}
/** @brief (one liner)
  *
  * (documentation goes here)
  */
ManchesterpatternDecoder::~ManchesterpatternDecoder()
{
	delete ManchesterBits;

}



/** @brief (Resets internal vars to defaults)
  *
  * Reset internal vars to defaults. Called after error or when finished
  */
void ManchesterpatternDecoder::reset()
{
	longlow =   -1;
	longhigh =  -1;
	shortlow =  -1;
	shorthigh = -1;
	minbitlen = 20; // Set defaults
	ManchesterBits->reset();
}
/** @brief (Sets internal minbitlen to new value)
  *
  * (documentation goes here)
  */
void ManchesterpatternDecoder::setMinBitLen(uint8_t len)
{
		minbitlen = len;
}


  /** @brief (Returns true if given pattern index matches a long puls index)
    *
    * (documentation goes here)
    */
bool ManchesterpatternDecoder::isLong(uint8_t pulse_idx)
{
	return (pulse_idx == longlow || pulse_idx == longhigh);
}

  /** @brief (Returns true if given pattern index matches a short puls index)
    *
    * (documentation goes here)
    */

bool ManchesterpatternDecoder::isShort(uint8_t pulse_idx)
{
	return (pulse_idx == shortlow || pulse_idx == shorthigh);
}

  /** @brief (Converts decoded manchester bits in a provided string as hex)
    *
    * ()
    */
void ManchesterpatternDecoder::getMessageHexStr(String *message)
{
	char hexStr[] ="00"; // Not really needed
	message->reserve(ManchesterBits->bytecount*3); // Todo: Reduce to exact needed size
	if (!message)
		return ;

    // Bytes are stored from left to right in our buffer. We reverse them for better readability
	for (uint8_t idx=0; idx<=ManchesterBits->bytecount; ++idx){
        //Serial.print(getMCByte(idx),HEX);
        //sprintf(hexStr, "%02X",reverseByte(ManchesterBits->getByte(idx)));
        //Serial.print(".");
		sprintf(hexStr, "%02X",getMCByte(idx));
        message->concat(hexStr);

		//Serial.print(hexStr);
	}
	//Serial.println();

}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
void ManchesterpatternDecoder::getMessagePulseStr(String* str)
{
	str->reserve(32);
	if (!str)
			return;

	str->concat("LL=");str->concat(pdec->pattern[longlow][0]);str->concat(SERIAL_DELIMITER);
	str->concat("LH=");str->concat(pdec->pattern[longhigh][0]);str->concat(SERIAL_DELIMITER);
	str->concat("SL=");str->concat(pdec->pattern[shortlow][0]);str->concat(SERIAL_DELIMITER);
	str->concat("SH=");str->concat(pdec->pattern[shorthigh][0]);str->concat(SERIAL_DELIMITER);
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
void ManchesterpatternDecoder::getMessageClockStr(String* str)
{
	str->reserve(7);
	if (!str)
			return;

	str->concat("C=");str->concat(clock);str->concat(SERIAL_DELIMITER);
}



  /** @brief (retieves one Byte out of the Bitstore for manchester decoded bits)
    *
    * (Returns a comlete byte from the pattern store)
    */

unsigned char ManchesterpatternDecoder::getMCByte(uint8_t idx){

    return ManchesterBits->getByte(idx);
}


  /** @brief (Decodes the manchester pattern to bits. Returns true on success and false on error )
    *
    * (Call only after ismanchester returned true)
    */

bool ManchesterpatternDecoder::doDecode() {
    //Serial.print("bitcnt:");Serial.println(bitcnt);

	uint8_t i=0;

	bool mc_start_found=false;
        #ifdef DEBUGDECODE
        Serial.print("mlen: ");
        Serial.print(pdec->messageLen);
        Serial.print(" mstart: ");
        Serial.print(pdec->mstart);

        #endif
	while (i < pdec->messageLen)
	{
		if ( mc_start_found==false && (isLong(pdec->message[i]) ||  isShort(pdec->message[i]))  )
		{
			pdec->mstart=i;
			mc_start_found=true;
		}
		if (mc_start_found)
		{

            if (isLong(pdec->message[i])) {
		          ManchesterBits->addValue(!(pdec->pattern[pdec->message[i]][0] >>15)); // Check if bit 15 is set
				  #ifdef DEBUGDECODE
		          Serial.print("L");
		          #endif
            }
			else if(isShort(pdec->message[i]) && i < pdec->messageLen-1 && isShort(pdec->message[i+1])  )
			{
				  ManchesterBits->addValue(!(pdec->pattern[pdec->message[i+1]][0] >>15)); // Check if bit 15 is set
				  #ifdef DEBUGDECODE
				  Serial.print("SS");
				  #endif

				  i++;
			}
			else { // Found something that fits not to our manchester signal
				if (i < pdec->messageLen-minbitlen)
				{
					ManchesterBits->reset();
					pdec->mstart=i;
					#ifdef DEBUGDECODE
					Serial.print("RES");
					#endif

				} else {
					#ifdef DEBUGDECODE
					Serial.print("END");
					#endif

                    #ifdef DEBUGDECODE
                    Serial.print(" mpos: ");
                    Serial.print(i);
                    Serial.print(" mstart: ");
                    Serial.print(pdec->mstart);
                    #endif

					return (ManchesterBits->valcount >= minbitlen);  // Min 20 Bits needed
				}
			}

		}
		//Serial.print(" S MC ");
		i++;
	}
        #ifdef DEBUGDECODE
        Serial.print("mlen: ");
        Serial.print(pdec->messageLen);
        Serial.print(" mstart: ");
        Serial.print(pdec->mstart);

        #endif

    // Check if last entry in our message array belongs to our manchester signal
	if (i==pdec->messageLen && pdec->mstart > 1)
    {
        #ifdef DEBUGDECODE
        Serial.print("Message truncated");
        #endif

        pdec->messageLen=pdec->messageLen-pdec->mstart; // Berechnung der neuen Nachrichtenlänge nach dem Löschen
		memmove(pdec->message,pdec->message+pdec->mstart,sizeof(*pdec->message)*(pdec->messageLen+1));
		pdec->m_truncated=true;  // Flag that we truncated the message array and want to receiver some more data
    }

	return (ManchesterBits->valcount >= minbitlen);  // Min 20 Bits needed, then return true, otherwise false

	//Serial.print(" ES MC ");
}

  /** @brief (Verifies if found signal data is a valid manchester signal, returns true or false)
    *
    * (Check signal based on patternLen, histogram and pattern store for valid manchester style.Provides key indexes for the 4 signal states for later decoding)
    */

bool ManchesterpatternDecoder::isManchester()
{
    // Durchsuchen aller Musterpulse und prüft ob darin eine clock vorhanden ist
	#if DEBUGDETECT >= 1
	Serial.print("  --  chk MC -- ");
	#endif
    if (pdec->patternLen < 4)	return false;

	int tstclock=-1;

	uint8_t pos_cnt=0;
	uint8_t neg_cnt=0;
	uint8_t equal_cnt=0;
	const uint8_t minHistocnt=pdec->messageLen*0.14;


    for (uint8_t i=0;i< pdec->patternLen;i++)
    {
		#if DEBUGDETECT >= 1
		Serial.print(i);Serial.print(" ");
		#endif

		if (pdec->histo[i] < minHistocnt) continue;		// Skip this pattern, due to less occurence in our message
		int aktpulse = pdec->pattern[i][0];
		//if (longlow == -1)
        //    longlow=longhigh=shortlow=shorthigh=i;  // Init to first valid mc index to allow further ajustment


		if (aktpulse > 0 )
		{
			equal_cnt += pdec->histo[i];
			pos_cnt++;
			tstclock += aktpulse;

			longhigh = longhigh == -1 || pdec->pattern[longhigh][0] < aktpulse ? i  : longhigh;
			shorthigh = shorthigh == -1 ||  pdec->pattern[shorthigh][0] > aktpulse ? i  : shorthigh;

		} else {
			equal_cnt -= pdec->histo[i];
			neg_cnt++;

			longlow = longlow == -1 || pdec->pattern[longlow][0] > aktpulse ? i  : longlow;
			shortlow = shortlow == -1 || pdec->pattern[shortlow][0] < aktpulse ? i : shortlow;
		}

    }

    if (equal_cnt > pdec->messageLen*0.02) return false;
	#if DEBUGDETECT >= 1
	Serial.print("  MC equalcnt matched");
	#endif

	if (neg_cnt != pos_cnt ) return false;  // Both must be 2
	#if DEBUGDETECT >= 1
	Serial.print("  MC neg and pos pattern cnt is equal");
	#endif

	tstclock=tstclock/3;
	#if DEBUGDETECT >= 1
	Serial.print("  tstclock: ");Serial.print(tstclock);
	#endif
	clock =tstclock;
//	dclock=clock*2;

	#if DEBUGDETECT >= 1
	Serial.print(" MC LL:"); Serial.print(longlow);
	Serial.print(", MC LH:"); Serial.print(longhigh);

	Serial.print(", MC SL:"); Serial.print(shortlow);
	Serial.print(", MC SH:"); Serial.print(shorthigh);
	Serial.println("");
	#endif
	if ( (longlow == -1) || (shortlow == -1) || (longlow == shortlow) || (longhigh ==-1) || (shorthigh == -1) ||(longhigh == shorthigh) ) return false; //Check if the indexes are valid

	#if DEBUGDETECT >= 1
	Serial.println("  -- MC found -- ");
	#endif

	return true;
}




/** @brief (sets a pattern to the store)
  *
  * (documentation goes here)
  */
int8_t patternO::setPattern(const uint8_t idx, const int16_t val)
{
	patternStore[idx]=val;
}

/** @brief (gets a pattern from the store)
  *
  * (documentation goes here)
  */
int8_t patternO::getPattern(const uint8_t idx)
{
	return patternStore[idx];
}

/** @brief (one liner)
  *
  * (Finds a pattern in our pattern store. returns -1 if te pattern is not found)
  */
int8_t patternO::patternExists(int* val)
{
	//patternStore[idx]=patternval

	// Iterate over patterns (1 dimension of array)
	for (uint8_t idx=0; idx<patternPos; ++idx)
    {
		if (patternBasic::inTol(*val,patternStore[idx],*val*0.3))  // Skip this iteration, if pattern is not in tolerance
		{
			return idx;
		}
    }
    // sequence was not found in pattern
    return -1;
}








/*

********************************************



********************************************



********************************************



  Stuff behind this, is not used anymore




********************************************



********************************************



********************************************

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
/*
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

*/
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

/*
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

*/

/*
returns numbit bits one , begins at position deliverd via startingPos
skips every second bit and returns the bits in reversed (correct for OSV2) order
*/

/*
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
*/


/*
returns the 4 bits one nibble, begins at position deliverd via startingPos
skips every second bit and returns the nibble in reversed (correct) order
*/

/*
unsigned char OSV2Decoder::getNibble(uint8_t startingPos)
{
    return getDataBits(startingPos,4);
}
*/

/*
returns the 8 bits / one byte, begins at position deliverd via startingPos
skips every second bit and returns the byte in reversed (correct) order.
// Todo use getNibble to return the byte
*/

/*
unsigned char OSV2Decoder::getByte(uint8_t startingPos)
{
     return getDataBits(startingPos,8);
}
*/


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

