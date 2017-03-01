/*
*   Pattern Decoder Library V3
*   Library to decode radio signals based on patternd detection
*   2014-2015  N.Butzek, S.Butzek
*   2015-2017  S.Butzek

*   This library contains classes to perform decoding of digital signals
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
#include "signalDecoder.h"


void SignalDetectorClass::bufferMove(const uint8_t start)
{
	m_truncated = false;
	DBG_PRINTLN("");
	if (start > messageLen - 1 || start == 0) {
		//DBG_PRINT(__FUNCTION__); DBG_PRINT(" start oor "); 	DBG_PRINT(start); DBG_PRINT(" "); DBG_PRINT(messageLen);
	}
	else if (message.moveLeft(start))
	{
		m_truncated = true; 
		//DBG_PRINT(__FUNCTION__); DBG_PRINT(" -> "); 	DBG_PRINT(start);  DBG_PRINT(" "); DBG_PRINT(messageLen);
		//DBG_PRINT(" "); DBG_PRINT(message.bytecount);
		messageLen = messageLen - start;
		
	} else {
		//DBG_PRINT(__FUNCTION__); DBG_PRINT(" unsup "); 	DBG_PRINT(start);
		//printOut();
	}
}


inline void SignalDetectorClass::addData(const uint8_t value)
{
	//message += value;
	/*if (message.valcount >= 254)
	{
		DBG_PRINTLN(""); 	DBG_PRINT(__FUNCTION__); DBG_PRINT(" msglen: "); DBG_PRINT(messageLen);
	}*/
	if (message.addValue(value))
	{
		messageLen++;
	}
}

inline void SignalDetectorClass::addPattern()
{
	pattern[pattern_pos] = *first;						//Store pulse in pattern array
	pattern_pos++;
}

inline void SignalDetectorClass::updPattern( const uint8_t ppos)
{
	pattern[ppos] = (long(pattern[ppos]) + *first) / 2; // Moving average
}


inline void SignalDetectorClass::doDetect()
{

	//printOut();

	bool valid;
	valid = (messageLen == 0 || last == NULL || (*first ^ *last) < 0); // true if a and b have opposite signs
	valid &= (messageLen == maxMsgSize) ? false : true;
	valid &= (*first > -maxPulse);  // if low maxPulse detected, start processMessage()

//		if (messageLen == 0) pattern_pos = patternLen = 0;
		//if (messageLen == 0) valid = true;
	if (!valid) {
		if (mcDetected == true || messageLen >= minMessageLen) {
			// Try output
			m_overflow = (messageLen == maxMsgSize) ? true : false;
			processMessage();
		} else {
			MsMoveCount = 3;
			reset();
		}
		
	}
	else if (messageLen == minMessageLen) {
		state = detecting;  // Set state to detecting, because we have more than minMessageLen data gathered, so this is no noise
		rssiValue = _rssiCallback();
	}

	int8_t fidx = findpatt(*first);
	if (fidx >= 0) {
		// Upd pattern

		updPattern(fidx);
	}
	else {

		// Add pattern
		if (patternLen == maxNumPattern)
		{
			calcHisto();
			if (histo[patternLen] > 2) processMessage();
			for (int16_t i = messageLen - 1; i > 0; --i)
			{
				if (message[i] == pattern_pos) // Finde den letzten Verweis im Array auf den Index der gleich ueberschrieben wird
				{
					i++; // i um eins erhoehen, damit zukuenftigen Berechnungen darauf aufbauen koennen
					bufferMove(i);
					
					break;
				}
			}
		}

		fidx = pattern_pos;
		addPattern();

		if (pattern_pos == maxNumPattern)
		{
			pattern_pos = 0;  // Wenn der Positions Index am Ende angelegt ist, gehts wieder bei 0 los und wir ueberschreiben alte pattern
			patternLen = maxNumPattern;
			mcDetected = false;  // When changing a pattern, we need to redetect a manchester signal and we are not in a buffer full mode scenario

		}
		if (pattern_pos > patternLen) patternLen = pattern_pos;

	}

	// Add data to buffer
	addData(fidx);


#if DEBUGDETCT > 3
		DBG_PRINT("Pulse: "); DBG_PRINT(*first);
		DBG_PRINT(", "); DBG_PRINT(*last);
		DBG_PRINT(", TOL: "); DBG_PRINT(tol); DBG_PRINT(", fidx: "); DBG_PRINT(fidx);
		DBG_PRINT(", Vld: "); DBG_PRINT(valid);
		DBG_PRINT(", pattPos: "); DBG_PRINT(pattern_pos);
		DBG_PRINT(", mLen: "); DBG_PRINT(messageLen);
		DBG_PRINT(", BC:");	DBG_PRINT(message.bytecount); 
		DBG_PRINT(", vcnt:");	DBG_PRINT(message.valcount);
		DBG_PRINTLN(" ");
#endif


}

bool SignalDetectorClass::decode(const int * pulse)
{
	success = false;
	if (messageLen > 0)
		last = &pattern[message[messageLen - 1]];
	*first = *pulse;
	
	doDetect();
	return success;
}


void SignalDetectorClass::compress_pattern()
{
	calcHisto();
	for (uint8_t idx = 0; idx<patternLen-1; idx++)
	{
		if (histo[idx] == 0)
			continue;

		for (uint8_t idx2 = idx + 1; idx2<patternLen; idx2++)
		{
			if (histo[idx2] == 0 || (pattern[idx] ^ pattern[idx2]) >> 15)
				continue;
			const int16_t tol = int((abs(pattern[idx2])*tolFact) + (abs(pattern[idx2])*tolFact) / 2);
			if (inTol(pattern[idx2], pattern[idx], tol))  // Pattern are very equal, so we can combine them
			{
				// Change val -> ref_val in message array
				for (uint8_t i = 0; i<messageLen; i++)
				{
					if (message[i] == idx2)
					{
						message.changeValue(i, idx);
					}
				}

#if DEBUGDETECT>2
				DBG_PRINT("compr: "); DBG_PRINT(idx2); DBG_PRINT("->"); DBG_PRINT(idx); DBG_PRINT(";");
				DBG_PRINT(histo[idx2]); DBG_PRINT("*"); DBG_PRINT(pattern[idx2]);
				DBG_PRINT("->");
				DBG_PRINT(histo[idx]); DBG_PRINT("*"); DBG_PRINT(pattern[idx]);
				
#endif // DEBUGDETECT


				int  sum = histo[idx] + histo[idx2];
				pattern[idx] = ((long(pattern[idx]) * histo[idx]) + (pattern[idx2] * histo[idx2])) / sum;
				histo[idx] += histo[idx2];
				pattern[idx2] = histo[idx2]= 0;

#if DEBUGDETECT>2
				DBG_PRINT(" idx:"); DBG_PRINT(pattern[idx]);
				DBG_PRINT(" idx2:"); DBG_PRINT(pattern[idx2]);
				DBG_PRINTLN(";");
#endif // DEBUGDETECT

			}
		}
	}
}

void SignalDetectorClass::processMessage()
{
	yield();

	if (mcDetected == true || messageLen >= minMessageLen) {
		success = false;

#if DEBUGDETECT >= 1
		DBG_PRINTLN("Message received:");
#endif

		compress_pattern();
		calcHisto();
		getClock();
		if (state == clockfound) getSync();

#if DEBUGDETECT >= 1
		printOut();
#endif

		if (MSenabled && state == syncfound && messageLen >= minMessageLen)// Messages mit clock / Sync Verhaeltnis pruefen
		{
#if DEBUGDECODE >0
			MSG_PRINT(" MS check: ");

			//printOut();
#endif	

			// Setup of some protocol identifiers, should be retrieved via fhem in future

			mend = mstart + 2;   // GGf. kann man die Mindestlaenge von x Signalen vorspringen
			bool m_endfound = false;

			//uint8_t repeat;
			while (mend < messageLen - 1)
			{
				if (message[mend + 1] == sync && message[mend] == clock) {
					mend -= 1;					// Previus signal is last from message
					m_endfound = true;
					break;
				}
				mend += 2;
			}
			if (mend > messageLen) mend = messageLen;  // Reduce mend if we are behind messageLen
													   //if (!m_endfound) mend=messageLen;  // Reduce mend if we are behind messageLen

			calcHisto(mstart, mend);	// Recalc histogram due to shortened message


#if DEBUGDECODE > 1
			DBG_PRINT("Index: ");
			DBG_PRINT(" MStart: "); DBG_PRINT(mstart);
			DBG_PRINT(" SYNC: "); DBG_PRINT(sync);
			DBG_PRINT(", CP: "); DBG_PRINT(clock);
			DBG_PRINT(" - MEFound: "); DBG_PRINTLN(m_endfound);
			DBG_PRINT(" - MEnd: "); DBG_PRINTLN(mend);
#endif // DEBUGDECODE
			if ((m_endfound && (mend - mstart) >= minMessageLen) || (!m_endfound && messageLen < (maxMsgSize)))//(!m_endfound && messageLen  >= minMessageLen))	// Check if message Length is long enough
			{
#ifdef DEBUGDECODE
				MSG_PRINTLN("Filter Match: ");;
#endif


				//preamble = "";
				//postamble = "";

				/*				Output raw message Data				*/
				if (MredEnabled) {
					int patternInt;
					uint8_t patternLow;
					uint8_t patternIdx;
					
					MSG_PRINT(MSG_START);  MSG_PRINT("Ms");  MSG_PRINT(SERIAL_DELIMITER);
					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue;
						patternIdx = idx;
						patternInt = pattern[idx];

						if (patternInt < 0) {
							patternIdx = idx | B10100000;    // Bit5 = 1 (Vorzeichen negativ)
							patternInt = -patternInt;
						}
						else {
							patternIdx = idx | B10000000;    // Bit5 = 0 (Vorzeichen positiv)
						}
						
						patternLow = lowByte(patternInt);
						if (bitRead(patternLow,7) == 0) {
							bitSet(patternLow,7);
						}
						else {
							bitSet(patternIdx, 4);   // wenn bei patternLow Bit7 gesetzt ist, dann bei patternIdx Bit4 = 1
						}
						Serial.write(patternIdx);
						Serial.write(patternLow);
						Serial.write(highByte(patternInt) | B10000000);
						MSG_PRINT(SERIAL_DELIMITER);
					}

					uint8_t n;

					if ((mend & 1) == 1) {   // ungerade
						MSG_PRINT("D");
					}
					else {
						MSG_PRINT("d");
					}
					if ((mstart & 1) == 1) {  // ungerade
						mstart--;
						n = (message.getByte(mstart/2) & 15) | 128;    // high nibble = 8 als Kennzeichen für ungeraden mstart
						Serial.write(n);
						//MSG_WRITE(n);
 						mstart += 2;
					}
					for (uint8_t i = mstart; i <= mend; i=i+2) {					
						n = message.getByte(i/2);
						Serial.write(n);
						//MSG_WRITE(n);
					}

					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("C");  MSG_PRINT(clock, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("S");  MSG_PRINT(sync, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("R");  MSG_PRINT(rssiValue, HEX);  MSG_PRINT(SERIAL_DELIMITER);				}
				else {
					MSG_PRINT(MSG_START); MSG_PRINT("MS");  MSG_PRINT(SERIAL_DELIMITER);		
					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue;
						MSG_PRINT('P'); MSG_PRINT(idx); MSG_PRINT('='); MSG_PRINT(pattern[idx]); MSG_PRINT(SERIAL_DELIMITER);
					}
					MSG_PRINT("D=");

					for (uint8_t i = mstart; i <= mend; ++i)
					{
						MSG_PRINT(message[i]);
					}
					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("CP="); MSG_PRINT(clock);     MSG_PRINT(SERIAL_DELIMITER);     // ClockPulse
					MSG_PRINT("SP="); MSG_PRINT(sync);      MSG_PRINT(SERIAL_DELIMITER);     // SyncPulse
					MSG_PRINT("R=");  MSG_PRINT(rssiValue); MSG_PRINT(SERIAL_DELIMITER);     // Signal Level (RSSI)					
				}
				
				if (m_overflow) {
					MSG_PRINT("O");  MSG_PRINT(SERIAL_DELIMITER);
				}
				
				if ((messageLen - mend) >= minMessageLen && MsMoveCount > 0) {
					//MSG_PRINT(F("MS move. messageLen ")); MSG_PRINT(messageLen); MSG_PRINT(" "); MSG_PRINTLN(MsMoveCount)
					MsMoveCount--;
					bufferMove(mend+1);
					//MSG_PRINT(F("MS move. messageLen ")); MSG_PRINTLN(messageLen);
					mstart = 0;
					MSG_PRINT("m"); MSG_PRINT(MsMoveCount) MSG_PRINT(SERIAL_DELIMITER);
				}
				
				MSG_PRINT(MSG_END);
				MSG_PRINT("\n");
				
				success = true;
				m_truncated = false;
#ifdef mp_crc
				const int8_t crco = printMsgRaw(mstart, mend, &preamble, &postamble);

				if ((mend < messageLen - minMessageLen) && (message[mend + 1] == message[mend - mstart + mend + 1])) {
					mstart = mend + 1;
					byte crcs = 0x00;
#ifndef ARDUSIM
					for (uint8_t i = mstart + 1; i <= mend - mstart + mend; i++)
					{
						crcs = _crc_ibutton_update(crcs, message[i]);
					}
#endif
					if (crcs == crco)
					{
						// repeat found
					}
					//processMessage(); // Todo: needs to be optimized
				}
#endif


			}
			else if (m_endfound == false && mstart > 0 && mend + 1 >= maxMsgSize) // Start found, but no end. We remove everything bevore start and hope to find the end later
			{
				//MSG_PRINT("copy");
#ifdef DEBUGDECODE
				DBG_PRINT(" move msg ");;
#endif
				bufferMove(mstart);
				//m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
			} 
			else if (m_endfound && mend < maxMsgSize) {  // Start and end found, but end is not at end of buffer, so we remove only what was checked
#ifdef DEBUGDECODE
				DBG_PRINT(" move msg ");;
#endif
				bufferMove(mend);
				//m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
				success = true;	// don't process other message types
			}
			else {
#ifdef DEBUGDECODE
				MSG_PRINTLN(" Buffer overflow, flushing message array");
#endif
				//MSG_PRINT(MSG_START);
				//MSG_PRINT("Buffer overflow while processing signal");
				//MSG_PRINT(MSG_END);
				reset(); // Our Messagebuffer is not big enough, no chance to get complete Message
				
				success = true;	// don't process other message types
			}
		}
		if (success == false && (MUenabled || MCenabled)) {

#if DEBUGDECODE >0
			DBG_PRINT(" check:");

			//printOut();
#endif	
// Message has a clock puls, but no sync. Try to decode this

			//preamble = "";
			//postamble = "";

			if (MCenabled)
			{
				//DBG_PRINT(" mc: ");

				static ManchesterpatternDecoder mcdecoder(this);			// Init Manchester Decoder class

				if (mcDetected == false)
				{
					mcdecoder.reset();
					mcdecoder.setMinBitLen(mcMinBitLen);								
				}
#if DEBUGDETECT>3
				MSG_PRINT("vcnt: "); MSG_PRINT(mcdecoder.ManchesterBits.valcount);
#endif

				if ((mcDetected || mcdecoder.isManchester()) && mcdecoder.doDecode())	// Check if valid manchester pattern and try to decode
				{
#if DEBUGDECODE > 1
					MSG_PRINT(" MC found: ");
#endif // DEBUGDECODE

#if DEBUGDECODE == 1
					MSG_PRINT(MSG_START);
					MSG_PRINT("MC");
					MSG_PRINT(SERIAL_DELIMITER);

					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (histo[idx] == 0) continue;
						MSG_PRINT('P'); MSG_PRINT(idx); MSG_PRINT('='); MSG_PRINT(pattern[idx]); MSG_PRINT(SERIAL_DELIMITER);
					}
					MSG_PRINT("D=");


					mend = min(mend, messageLen); // Workaround if mend=255
					for (uint8_t i = mstart; i <= mend; ++i)
					{
						MSG_PRINT(message[i]);
					}
					MSG_PRINT(SERIAL_DELIMITER);

					if (m_overflow) {
						MSG_PRINT("O");
						MSG_PRINT(SERIAL_DELIMITER);
					}
					MSG_PRINTLN(MSG_END);
#endif
					MSG_PRINT("MC");
					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("LL="); MSG_PRINT(pattern[mcdecoder.longlow]); MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("LH="); MSG_PRINT(pattern[mcdecoder.longhigh]); MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("SL="); MSG_PRINT(pattern[mcdecoder.shortlow]); MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("SH="); MSG_PRINT(pattern[mcdecoder.shorthigh]); MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("D=");  mcdecoder.printMessageHexStr();
					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("C="); MSG_PRINT(mcdecoder.clock); MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("L="); MSG_PRINT(mcdecoder.ManchesterBits.valcount); MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("R=");  MSG_PRINT(rssiValue); MSG_PRINT(SERIAL_DELIMITER);     // Signal Level (RSSI)
#ifdef DEBUGDECODE
					DBG_PRINTLN("");
#endif

//					printMsgStr(&preamble, &mcbitmsg, &postamble);
					mcDetected = false;
					success = true;



				}
				else if (mcDetected == true && m_truncated == true) {

					success = true;   // Prevents MU Processing
				}

			}
			if (MUenabled && state == clockfound && success == false && messageLen >= minMessageLen) {

#if DEBUGDECODE > 1
				DBG_PRINT(" MU found: ");
#endif // DEBUGDECODE

				//preamble = String(MSG_START)+String("MU")+String(SERIAL_DELIMITER)+preamble;

				MSG_PRINT(MSG_START);
				MSG_PRINT("MU");
				MSG_PRINT(SERIAL_DELIMITER);

				for (uint8_t idx = 0; idx < patternLen; idx++)
				{
					if (pattern[idx] == 0 || histo[idx] == 0) continue; 
					MSG_PRINT("P"); MSG_PRINT(idx); MSG_PRINT("="); MSG_PRINT(pattern[idx]); MSG_PRINT(SERIAL_DELIMITER);  // Patternidx=Value
				}
				MSG_PRINT("D=");
				for (uint8_t i = 0; i < messageLen; ++i)
				{
					MSG_PRINT(message[i]);
				}
				//String postamble;
				MSG_PRINT(SERIAL_DELIMITER);
				MSG_PRINT("CP="); MSG_PRINT(clock);     MSG_PRINT(SERIAL_DELIMITER);    // ClockPulse, (not valid for manchester)
				MSG_PRINT("R=");  MSG_PRINT(rssiValue); MSG_PRINT(SERIAL_DELIMITER);     // Signal Level (RSSI)

				if (m_overflow) {
					MSG_PRINT("O");
					MSG_PRINT(SERIAL_DELIMITER);
				}
				MSG_PRINTLN(MSG_END);
				//postamble.concat('\n');

				//printMsgRaw(0, messageLen - 1, &preamble, &postamble);
				m_truncated = false;
				success = true;
			}

		}
		
		if (success == false) 
		{
#if DEBUGDETECT >= 1
			DBG_PRINTLN("nothing to to");
#endif
		}
	}
	if (!m_truncated)  // Todo: Eventuell auf vollen Puffer prüfen
	{
		reset();
	}

	//MSG_PRINTLN("process finished");
}






void SignalDetectorClass::reset()
{
	messageLen = 0;
	patternLen = 0;
	pattern_pos = 0;
	message.reset();
//	bitcnt = 0;
	state = searching;
	clock = sync = -1;
	for (uint8_t i = 0; i<maxNumPattern; ++i)
	  histo[i] = pattern[i] = 0;
	success = false;
	tol = 150; //
	tolFact = 0.2;
	mstart = 0;
	m_truncated = false;
	m_overflow = false;
	mcDetected = false;
	//MSG_PRINTLN("reset");
	mend = 0;
	//DBG_PRINT(":sdres:");
	//last = NULL;
}

const status SignalDetectorClass::getState()
{
	return status();
}


const bool SignalDetectorClass::inTol(const int val, const int set, const int tolerance)
{
	
	// tolerance = tolerance == 0 ? tol : tolerance;
	//return (abs(val - set) <= tolerance == 0 ? tol: tolerance);
	return (abs(val - set) <= tolerance);
}

void SignalDetectorClass::printOut()
{
	DBG_PRINTLN("");
	if (sync > -1) {
		DBG_PRINT("Sync: ");DBG_PRINT(pattern[sync]);
		DBG_PRINT(" -> SyncFact: "); DBG_PRINT(pattern[sync] / (float)pattern[clock]);
		DBG_PRINT(",");
	}
	DBG_PRINT(" Clock: "); DBG_PRINT(pattern[clock]);
	DBG_PRINT(", Tol: "); DBG_PRINT(tol);
	DBG_PRINT(", PattLen: "); DBG_PRINT(patternLen); DBG_PRINT(" ("); DBG_PRINT(pattern_pos); DBG_PRINT(")");
	DBG_PRINT(", Pulse: "); DBG_PRINT(*first); DBG_PRINT(", "); DBG_PRINT(*last);
	DBG_PRINT(", mStart: "); DBG_PRINT(mstart);
	DBG_PRINT(", MCD: "); DBG_PRINT(mcDetected,DEC);


	DBG_PRINTLN(); DBG_PRINT("Signal: ");
	uint8_t idx;
	for (idx = 0; idx<messageLen; ++idx) {
		DBG_PRINT((int8_t)message[idx],DEC);
		//DBG_PRINT(message.getValue(idx));

	}
	DBG_PRINT(". "); DBG_PRINT(" ["); DBG_PRINT(messageLen); DBG_PRINTLN("]");
	DBG_PRINT("Pattern: ");
	for (uint8_t idx = 0; idx<patternLen; ++idx) {
		DBG_PRINT(" P"); DBG_PRINT(idx);
		DBG_PRINT(": "); DBG_PRINT(histo[idx]);  DBG_PRINT("*[");
		if (pattern[idx] != 0)
		{
			//DBG_PRINT(",");
			DBG_PRINT(pattern[idx]);
		}

		DBG_PRINT("]");
	}
	DBG_PRINTLN();
}

int8_t SignalDetectorClass::findpatt(const int val)
{
	//seq[0] = Laenge  //seq[1] = 1. Eintrag //seq[2] = 2. Eintrag ...
	// Iterate over patterns (1 dimension of array)
	tol = abs(val)*0.2;
	for (uint8_t idx = 0; idx<patternLen; ++idx)
	{
		if ((val ^ pattern[idx]) >> 15)
			continue;
		if (pattern[idx] != 0 && inTol(val, pattern[idx],tol))  // Skip this iteration, if seq[x] <> pattern[idx][x]
									   //if (!inTol(seq[x],pattern[idx][x-1]))  // Skip this iteration, if seq[x] <> pattern[idx][x]
		{
			return idx;
		}
	}
	// sequence was not found in pattern
	return -1;
}
/*
bool SignalDecoderClass::validSequence(const int * a, const int * b)
{
	return ((*a ^ *b) < 0); // true if a and b have opposite signs
}
*/

void SignalDetectorClass::calcHisto(const uint8_t startpos, uint8_t endpos)
{
	for (uint8_t i = 0; i<maxNumPattern; ++i)
	{
		histo[i] = 0;
	}

	if (endpos == 0) endpos = messageLen;
	/*for (uint8_t i = startpos; i < endpos; i++) 
		histo[message.getValue(i)]++;
	*/
	uint16_t bstartpos = startpos *4/8;
	uint16_t bendpos = endpos*4 / 8;
	unsigned char bval;
	for (uint8_t i = bstartpos; i<bendpos; ++i)
	{
		bval = message.getByte(i);
		histo[bval >> 4]++;
		histo[bval & B00001111]++;
	}
	if (endpos % 2==1)
		histo[bval & B00001111]--;

	
}

bool SignalDetectorClass::getClock()
{
	// Durchsuchen aller Musterpulse und prueft ob darin eine clock vorhanden ist
#if DEBUGDETECT > 3
	MSG_PRINTLN("  --  Searching Clock in signal -- ");
#endif
	int tstclock = -1;

	clock = -1; // Workaround for sync detection bug.

	for (uint8_t i = 0; i<patternLen; ++i) 		  // Schleife fuer Clock
	{
		//if (pattern[i][0]<=0 || pattern[i][0] > 3276)  continue;  // Annahme Werte <0 / >3276 sind keine Clockpulse
		if (tstclock == -1 && (pattern[i] >= 0) && (histo[i] > messageLen*0.17))
		{
			tstclock = i;
			continue;
		}

		if ((pattern[i] >= 0) && (pattern[i] < pattern[tstclock]) && (histo[i] > messageLen*0.17)) {
			tstclock = i;
		}
	}


	// Check Anzahl der Clockpulse von der Nachrichtenlaenge
	//if ((tstclock == 3276) || (maxcnt < (messageLen /7*2))) return false;
	if (tstclock == -1) return false;

	clock = tstclock;

	// Todo: GGf. andere Pulse gegen die ermittelte Clock verifizieren

	state = clockfound;
	return true;
}

bool SignalDetectorClass::getSync()
{
	// Durchsuchen aller Musterpulse und prueft ob darin ein Sync Faktor enthalten ist. Anschließend wird verifiziert ob dieser Syncpuls auch im Signal nacheinander uebertragen wurde
	//
#if DEBUGDETECT > 3
	DBG_PRINTLN("  --  Searching Sync  -- ");
#endif

	if (state == clockfound)		// we need a clock to find this type of sync
	{
		// clock wurde bereits durch getclock bestimmt.
		for (int8_t p = patternLen - 1; p >= 0; --p)  // Schleife fuer langen Syncpuls
		{
			//if (pattern[p] > 0 || (abs(pattern[p]) > syncMaxMicros && abs(pattern[p])/pattern[clock] > syncMaxFact))  continue;  // Werte >0 oder laenger maxfact sind keine Sync Pulse
			//if (pattern[p] == -1*maxPulse)  continue;  // Werte >0 sind keine Sync Pulse
			//if (!validSequence(&pattern[clock],&pattern[p])) continue;
			/*
			if ( (pattern[p] > 0) ||
			((abs(pattern[p]) > syncMaxMicros && abs(pattern[p])/pattern[clock] > syncMaxFact)) ||
			(pattern[p] == -maxPulse) ||
			(!validSequence(&pattern[clock],&pattern[p])) ||
			(histo[p] > 6)
			) continue;
			*/
			uint16_t syncabs = abs(pattern[p]);
			if ((pattern[p] < 0) &&
				//((abs(pattern[p]) <= syncMaxMicros && abs(pattern[p])/pattern[clock] <= syncMaxFact)) &&
				(syncabs < syncMaxMicros && syncabs / pattern[clock] <= syncMaxFact) &&
				(syncabs > syncMinFact*pattern[clock]) &&
				// (syncabs < maxPulse) &&
				//	 (validSequence(&pattern[clock],&pattern[p])) &&
				(histo[p] < messageLen*0.08) && (histo[p] > 1)
				//(histo[p] < 8) && (histo[p] > 1)

				//(syncMinFact*pattern[clock] <= syncabs)
				)
			{
				//if ((syncMinFact* (pattern[clock]) <= -1*pattern[p])) {//n>9 => langer Syncpulse (als 10*int16 darstellbar
				// Pruefe ob Sync und Clock valide sein koennen
				//	if (histo[p] > 6) continue;    // Maximal 6 Sync Pulse  Todo: 6 Durch Formel relativ zu messageLen ersetzen

				// Pruefen ob der gefundene Sync auch als message [clock, p] vorkommt
				uint8_t c = 0;

				//while (c < messageLen-1 && message[c+1] != p && message[c] != clock)		// Todo: Abstand zum Ende berechnen, da wir eine mindest Nachrichtenlaenge nach dem sync erwarten, brauchen wir nicht bis zum Ende suchen.

				while (c < messageLen - 1)		// Todo: Abstand zum Ende berechnen, da wir eine mindest Nachrichtenlaenge nach dem sync erwarten, brauchen wir nicht bis zum Ende suchen.
				{
					if (message[c + 1] == p && message[c] == clock) break;
					c++;
				}

				//if (c==messageLen) continue;	// nichts gefunden, also Sync weitersuchen
				if (c<messageLen - minMessageLen)
				{
					sync = p;
					state = syncfound;
					mstart = c;

#ifdef DEBUGDECODE
					//debug
					DBG_PRINTLN();
					DBG_PRINT("PD sync: ");
					DBG_PRINT(pattern[clock]); DBG_PRINT(", "); DBG_PRINT(pattern[p]);
					DBG_PRINT(", TOL: "); DBG_PRINT(tol);
					DBG_PRINT(", sFACT: "); DBG_PRINTLN(pattern[sync] / (float)pattern[clock]);
#endif
					return true;
				}
			}
		}
	}
	sync = -1; // Workaround for sync detection bug.
	return false;
}

void SignalDetectorClass::printMsgStr(const String * first, const String * second, const String * third)
{
	MSG_PRINT(*first);
	MSG_PRINT(*second);
	MSG_PRINT(*third);

}

int8_t SignalDetectorClass::printMsgRaw(uint8_t m_start, const uint8_t m_end, const String * preamble, const String * postamble)
{
	MSG_PRINT(*preamble);
	//String msg;
	//msg.reserve(m_end-mstart);
	byte crcv = 0x00;
	for (; m_start <= m_end; m_start++)
	{
		//msg + =message[m_start];
		//MSG_PRINT((100*message[m_start])+(10*message[m_start])+message[m_start]);
		MSG_PRINT(message[m_start]);
#ifndef ARDUSIM
		//crcv = _crc_ibutton_update(crcv, message[m_start]);
#endif
	}
	//MSG_PRINT(msg);
	MSG_PRINT(*postamble);
	return crcv;
	//printMsgStr(preamble,&msg,postamble);}
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
/*
ManchesterpatternDecoder::ManchesterpatternDecoder(signalDecoder *ref_dec)
{
pdec = ref_dec;
//ManchesterBits->new BitStore(1); // use 1 Bit for every value stored, reserve 30 Bytes = 240 Bits
reset();
}*/
/** @brief (one liner)
*
* (documentation goes here)
*/
ManchesterpatternDecoder::~ManchesterpatternDecoder()
{
	//delete ManchesterBits->

}



/** @brief (Resets internal vars to defaults)
*
* Reset internal vars to defaults. Called after error or when finished
*/
void ManchesterpatternDecoder::reset()
{
#ifdef DEBUGDECODE
	DBG_PRINT("mcrst:");
#endif
	longlow =   -1;
	longhigh =  -1;
	shortlow =  -1;
	shorthigh = -1;
	
	mc_start_found = false;
	mc_sync = false;

	clock = 0;
	minbitlen = 20; // Set defaults
	ManchesterBits.reset();


}
/** @brief (Sets internal minbitlen to new value)
*
* (documentation goes here)
*/
void ManchesterpatternDecoder::setMinBitLen(const uint8_t len)
{
	minbitlen = len;
}


/** @brief (Returns true if given pattern index matches a long puls index)
*
* (documentation goes here)
*/
const bool ManchesterpatternDecoder::isLong(const uint8_t pulse_idx)
{
	return (pulse_idx == longlow || pulse_idx == longhigh);
}

/** @brief (Returns true if given pattern index matches a short puls index)
*
* (documentation goes here)
*/

const bool ManchesterpatternDecoder::isShort(const uint8_t pulse_idx)
{
	return (pulse_idx == shortlow || pulse_idx == shorthigh);
}

/** @brief (Converts decoded manchester bits in a provided string as hex)
*
* ()
*/
void ManchesterpatternDecoder::getMessageHexStr(String *message)
{
	char hexStr[] = "00" ; // Not really needed

	message->reserve((ManchesterBits.valcount /4)+2);
	if (!message)
		return;
	uint8_t idx;
	// Bytes are stored from left to right in our buffer. We reverse them for better readability
	for ( idx = 0; idx <= ManchesterBits.bytecount-1; ++idx) {
		//MSG_PRINT(getMCByte(idx),HEX);
		//sprintf(hexStr, "%02X",reverseByte(ManchesterBits->>getByte(idx)));
		//MSG_PRINT(".");
		sprintf(hexStr, "%02X", getMCByte(idx));
		message->concat(hexStr);
		//MSG_PRINT(hexStr);
	}
	
	sprintf(hexStr, "%01X", getMCByte(idx) >> 4 & 0xf);
	message->concat(hexStr);
	if (ManchesterBits.valcount % 8 > 4 || ManchesterBits.valcount % 8 == 0)
	{
		sprintf(hexStr, "%01X", getMCByte(idx) & 0xF);
		message->concat(hexStr);
	}

	//MSG_PRINTLN();

}

/** @brief (Converts decoded manchester bits in a provided string as hex)
*
* ()
*/
void ManchesterpatternDecoder::printMessageHexStr()
{
	char hexStr[] = "00"; // Not really needed

	uint8_t idx;
	// Bytes are stored from left to right in our buffer. We reverse them for better readability
	for (idx = 0; idx <= ManchesterBits.bytecount - 1; ++idx) {
		sprintf(hexStr, "%02X", getMCByte(idx));
		MSG_PRINT(hexStr);
	}

	sprintf(hexStr, "%01X", getMCByte(idx) >> 4 & 0xf);
	MSG_PRINT(hexStr);
	if (ManchesterBits.valcount % 8 > 4 || ManchesterBits.valcount % 8 == 0)
	{
		sprintf(hexStr, "%01X", getMCByte(idx) & 0xF);
		MSG_PRINT(hexStr);
	}
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

	str->concat("LL="); str->concat(pdec->pattern[longlow]); str->concat(SERIAL_DELIMITER);
	str->concat("LH="); str->concat(pdec->pattern[longhigh]); str->concat(SERIAL_DELIMITER);
	str->concat("SL="); str->concat(pdec->pattern[shortlow]); str->concat(SERIAL_DELIMITER);
	str->concat("SH="); str->concat(pdec->pattern[shorthigh]); str->concat(SERIAL_DELIMITER);
}

/** @brief (one liner)
*
* (documentation goes here)
*/
void ManchesterpatternDecoder::printMessagePulseStr()
{
	MSG_PRINT("LL="); MSG_PRINT(pdec->pattern[longlow]); MSG_PRINT(SERIAL_DELIMITER);
	MSG_PRINT("LH="); MSG_PRINT(pdec->pattern[longhigh]); MSG_PRINT(SERIAL_DELIMITER);
	MSG_PRINT("SL="); MSG_PRINT(pdec->pattern[shortlow]); MSG_PRINT(SERIAL_DELIMITER);
	MSG_PRINT("SH="); MSG_PRINT(pdec->pattern[shorthigh]); MSG_PRINT(SERIAL_DELIMITER);
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

	str->concat("C="); str->concat(clock); str->concat(SERIAL_DELIMITER);
}

void ManchesterpatternDecoder::getMessageLenStr(String* str)
{

	str->concat("L="); str->concat(ManchesterBits.valcount); str->concat(SERIAL_DELIMITER);
}


/** @brief (retieves one Byte out of the Bitstore for manchester decoded bits)
*
* (Returns a comlete byte from the pattern store)
*/

unsigned char ManchesterpatternDecoder::getMCByte(const uint8_t idx) {

	return ManchesterBits.getByte(idx);
}




/** @brief (Decodes the manchester pattern to bits. Returns true on success and false on error )
*
* (Call only after ismanchester returned true)
*/

const bool ManchesterpatternDecoder::doDecode() {
	//MSG_PRINT("bitcnt:");MSG_PRINTLN(bitcnt);

	uint8_t i = 0;
	pdec->m_truncated = false;
//	bool mc_start_found = false;
//	bool mc_sync = false;
	pdec->mstart = 0; // Todo: pruefen ob start aus isManchester uebernommen werden kann
#ifdef DEBUGDECODE
	DBG_PRINT("mlen:");
	DBG_PRINT(pdec->messageLen);
	DBG_PRINT(":mstart: ");
	DBG_PRINT(pdec->mstart);
	DBG_PRINTLN("");

#endif
//	char  lastbit;
	bool ht = false;
	bool hasbit = false;

	while (i < pdec->messageLen)
	{
		// Start vom MC Signal suchen
		if (mc_start_found == false && (isLong(pdec->message[i]) || isShort(pdec->message[i])))
		{
			pdec->mstart = i;
			mc_start_found = true;
			mc_sync = true;

			// lookup for first long
			int pulseCnt = 0; 
			bool preamble = false;
			for (uint8_t l=i; l<pdec->messageLen; l++) {
				bool pulseIsLong = isLong(pdec->message[l]);
				
				// no manchester
				if (!(pulseIsLong || isShort(pdec->message[l]))) {
					break;
				}
				
				pulseCnt += (pulseIsLong ? 2 : 1);
				// probe signal to match manchester
				if (pulseIsLong) {
					// probe clock based preamble
					if (l == i && i > 0) {
						int pClock = abs(pdec->pattern[pdec->message[l - 1]]);

						if (pClock < maxPulse && (pdec->pattern[pdec->message[l - 1]] ^ pdec->pattern[pdec->message[l]] )>>15)
						{
							int pClocks = round(pClock / (float)clock);
							
							if (pClocks > 1 && abs(1 - (pClock / (pClocks * (float)clock))) <= 0.07) {
#ifdef DEBUGDECODE
								DBG_PRINT(F("preamble:")); DBG_PRINT(pClocks); DBG_PRINT(F("C"));
#endif
								pdec->mstart--;
								preamble = true;
								break;
							}
						}
					}
					
					ht=((pulseCnt & 0x1) == 0);
#ifdef DEBUGDECODE
					if (ht) {
						DBG_PRINT(F("pulseShift:")); DBG_PRINT(l); DBG_PRINT(";");
					}
#endif
					break;
				}
			}
			
			// interpret first long as short if preamble was found 
			if (preamble) {
				ht = true;
				i++;
				continue;
			}
			
		}
		// Sync to a long or short pulse 
		/*
		if (mc_start_found && !mc_sync)
		{
			while ( (!isShort(pdec->message[i]  || !isLong(pdec->message[i])) && i < pdec->messageLen) {
				i++;
			}
			if (i < pdec->messageLen) {
				lastbit = (char)((unsigned int)pdec->pattern[pdec->message[i]] >> 15); // 1, wenn Pegel Low war, 0 bei einem High Pegel.
				//lastbit = ~lastbit;  //TODO: Pruefen ob negiert korrekt ist.

				uint8_t z = i - pdec->mstart;
				if ((z < 1) or ((z % 2) == 0))
					i = pdec->mstart;
				else
					i = pdec->mstart + 1;
				//ManchesterBits->addValue((lastbit));
				mc_sync = true;
				//i++;
				//MSG_PRINT("lb:"); MSG_PRINT(lastbit,DEC);
			}
		}
		*/
		// Decoding occures here
		if (mc_sync && mc_start_found)
		{
			#ifdef DEBUGDECODE
			char value;
			#endif
			if (isShort(pdec->message[i])) //Todo: Check for second short
			{
				hasbit = ht;
				ht = !ht;
				#ifdef DEBUGDECODE
				value = 'S';
				#endif
				
			}
			else if (isLong(pdec->message[i])) {
				hasbit = true;
				ht = true;
				#ifdef DEBUGDECODE
				value = 'L';
				#endif
			}
			else { // Found something that fits not to our manchester signal
#ifdef DEBUGDECODE
				DBG_PRINT("H(");
				DBG_PRINT("vcnt:");
				DBG_PRINT(ManchesterBits.valcount);
#endif

				   //if (i < pdec->messageLen-minbitlen)
				if (ManchesterBits.valcount < minbitlen)
				{
//					if (isShort(pdec->message[i]) && i < pdec->messageLen - 1 && !isShort(pdec->message[i + 1])) {
//						// unequal number of short pulses. Restart, but one pulse ahead i is incremented at end of while loop
//						i = pdec->mstart;
//					}
					//pdec->mstart=i;
					mc_start_found = false; // Reset to find new starting position
					mc_sync = false;
					ht = false; // reset short count too
#ifdef DEBUGDECODE
					MSG_PRINT(":RES:");
#endif
					ManchesterBits.reset();

				} else {
					pdec->mend = i - (ht ? 0 : 1); // keep short in buffer
//					if (isShort(pdec->message[i]) && i == maxMsgSize - 1)) {
//						pdec->mend--;
//					}
#ifdef DEBUGDECODE
					DBG_PRINT(":mpos=");
					DBG_PRINT(i);
					DBG_PRINT(":mstart=");
					DBG_PRINT(pdec->mstart);
					DBG_PRINT(":mend:");
					DBG_PRINT(pdec->mend);
					DBG_PRINT(":mlen:");
					DBG_PRINT(pdec->messageLen);
					DBG_PRINT(":found:");
					DBG_PRINT(":pidx=");
					DBG_PRINT(pdec->message[i]);
					//DBG_PRINT(pdec->pattern[pdec->message[i]]);

#endif
					//pdec->printOut();
					pdec->bufferMove(i);   // Todo: BufferMove könnte in die Serielle Ausgabe verschoben werden, das würde ein paar Mikrosekunden Zeit sparen
					//pdec->m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
					mc_start_found = false;  // This will break serval unit tests. Normaly setting this to false shoud be done by reset, needs to be checked if reset shoud be called after hex string is printed out
			
					//if (i+minbitlen > pdec->messageLen)
					/*
					if ( isShort(pdec->message[pdec->messageLen]) )
					{

						pdec->mcDetected = true;
						return false;
					}
					*/
					return (ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed
				}
#ifdef DEBUGDECODE
				MSG_PRINT(")");
#endif
			}


			if (mc_start_found) { // don't write if manchester processing was canceled
#ifdef DEBUGDECODE
				if (pdec->pattern[pdec->message[i]] < 0)
					value = (value + 0x20); //lowwecase
				DBG_PRINT(value);
#endif
	
				if (hasbit) {
					ManchesterBits.addValue((pdec->pattern[pdec->message[i]] > 0 ? 1 : 0));
#ifdef DEBUGDECODE
					DBG_PRINT(ManchesterBits.getValue(ManchesterBits.valcount-1));
#endif
					hasbit = false;
				} else {
#ifdef DEBUGDECODE
					DBG_PRINT("_");
#endif
				}
			}


		}
		//MSG_PRINT(" S MC ");
		i++;
	}
	pdec->mend = i - (ht ? 0 : 1); // keep short in buffer;

#ifdef DEBUGDECODE
	DBG_PRINT(":mpos=");
	DBG_PRINT(i);
	DBG_PRINT(":mstart=");
	DBG_PRINT(pdec->mstart);
	DBG_PRINT(":mend=");
	DBG_PRINT(pdec->mend);
	DBG_PRINT(":vcnt=");
	DBG_PRINT(ManchesterBits.valcount-1);
	DBG_PRINT(":bfin:");
#endif


	// Check if last entry in our message array belongs to our manchester signal
	if (i == maxMsgSize && i == pdec->messageLen  && pdec->mstart > 0 && ManchesterBits.valcount > minbitlen / 2)
	{
#ifdef DEBUGDECODE
		DBG_PRINT(":bmove:");
#endif

		pdec->bufferMove(pdec->mstart);
		//pdec->m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
		
		pdec->mcDetected = true;
		return false;
	}
	// Buffer is full with mc signal, so we clear the buffer and caputre additional signaldata
	else if (i == maxMsgSize && i == pdec->messageLen && pdec->mstart == 0)
	{
		pdec->mcDetected = true;
		pdec->messageLen = 0;
		pdec->message.reset();
		//pdec->m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
#ifdef DEBUGDECODE
		DBG_PRINT(":bflush:");

#endif
		return false;
	}

	return (ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed, then return true, otherwise false

													//MSG_PRINT(" ES MC ");
}

/** @brief (Verifies if found signal data is a valid manchester signal, returns true or false)
*
* (Check signal based on patternLen, histogram and pattern store for valid manchester style.Provides key indexes for the 4 signal states for later decoding)
*/

const bool ManchesterpatternDecoder::isManchester()
{
	// Durchsuchen aller Musterpulse und prueft ob darin eine clock vorhanden ist
#if DEBUGDETECT >= 1
	DBG_PRINTLN("");
	DBG_PRINTLN("  --  chk MC -- ");
	DBG_PRINT("mstart:");
	DBG_PRINTLN(pdec->mstart);
#endif
	if (pdec->patternLen < 4)	return false;

	int tstclock = -1;

	uint8_t pos_cnt = 0;
	uint8_t neg_cnt = 0;
	int equal_cnt = 0;
	const uint8_t minHistocnt = round(pdec->messageLen*0.04);
				                          //     3     1    0     2
	uint8_t sortedPattern[maxNumPattern]; // 1300,-1300,-734,..800
	uint8_t p=0;

	for (uint8_t i = 0; i < pdec->patternLen; i++)
	{
		if (pdec->histo[i] < minHistocnt) continue;		// Skip this pattern, due to less occurence in our message
		#if DEBUGDETECT >= 1
				MSG_PRINT(p);
		#endif		

		uint8_t ptmp = p;

		while ( p!= 0 && pdec->pattern[i] < pdec->pattern[sortedPattern[p-1]])
		{
			sortedPattern[p] = sortedPattern[p-1];
			p--;
		}
#if DEBUGDETECT >= 1
		DBG_PRINT("="); DBG_PRINT(i); DBG_PRINT(",");
#endif
		sortedPattern[p] = i;
		p = ptmp+1;
	}
#if DEBUGDETECT >= 3
	DBG_PRINT("Sorted:");
	for (uint8_t i = 0; i < p; i++)
	{
		DBG_PRINT(sortedPattern[i]); DBG_PRINT(",");
	}
	DBG_PRINT(";");
#endif


	for (uint8_t i = 0; i<p ; i++)
	{
		if (pdec->pattern[sortedPattern[i]] <=0) continue;
#if DEBUGDETECT >= 2
		DBG_PRINT("CLK="); DBG_PRINT(sortedPattern[i]); DBG_PRINT(":");
#endif
		longlow = -1;
		longhigh = -1;
		shortlow = -1;
		shorthigh = -1;
		pos_cnt=0;
		neg_cnt = 0;
		tstclock = -1;
		equal_cnt = 0;

		const int clockpulse = pdec->pattern[sortedPattern[i]];
		for (uint8_t x = 0; x < p; x++)
		{
#if DEBUGDETECT >= 1
			DBG_PRINT(sortedPattern[x]); DBG_PRINT("^=(");
#endif

			const int aktpulse = pdec->pattern[sortedPattern[x]];
			bool pshort = false;
			bool plong = false;

			if (pdec->inTol(clockpulse, abs(aktpulse), clockpulse*0.5))
				pshort = true;
			else if (pdec->inTol(clockpulse*2, abs(aktpulse), clockpulse*0.80))
				plong = true;

			#if DEBUGDETECT >= 3
			DBG_PRINT("PS="); DBG_PRINT(pshort); DBG_PRINT(";");
			DBG_PRINT("PL="); DBG_PRINT(plong); DBG_PRINT(";)");
			#endif


			if (aktpulse > 0)
			{
				if (pshort) shorthigh = sortedPattern[x];
				else if (plong) longhigh = sortedPattern[x];
				else continue;
				//equal_cnt += pdec->histo[sortedPattern[x]]; 

				pos_cnt++;
				tstclock += aktpulse;
			}
			else {
				if (pshort) shortlow = sortedPattern[x];
				else if (plong) longlow = sortedPattern[x];
				else continue;

				//equal_cnt -= pdec->histo[sortedPattern[x]];
				neg_cnt++;
				tstclock -= aktpulse;

			}
		
			//TODO: equal_cnt sollte nur ueber die validen Pulse errechnet werden Signale nur aus 3 Pulsen sind auch valide (FFFF)...

			if ((longlow != -1) && (shortlow != -1) && (longhigh != -1) && (shorthigh != -1))
			{
#if DEBUGDETECT >= 1
				DBG_PRINT("vfy ");
#endif

				int z = 0;
				while (z < pdec->messageLen)
				{

					if ( ((isLong(pdec->message[z]) == false) && (isShort(pdec->message[z]) == false)) || (z == (pdec->messageLen-1)))
					{  
#if DEBUGDETECT >= 1
						DBG_PRINT(z); DBG_PRINT("=")DBG_PRINT(pdec->message[i]); DBG_PRINT(";")

						DBG_PRINT("Long"); DBG_PRINT(isLong(pdec->message[i])); DBG_PRINT(";");
						DBG_PRINT("Short"); DBG_PRINT(isShort(pdec->message[i])); DBG_PRINTLN(";");

#endif
						if ((z - pdec->mstart) > minbitlen)  // Todo: Hier wird auf minbitlen geprueft. Die Differenz zwischen mstart und mend sind aber Pulse und keine bits
						{
							pdec->mend = z;

							pdec->calcHisto(pdec->mstart, pdec->mend);
							equal_cnt = pdec->histo[shorthigh] + pdec->histo[longhigh] - pdec->histo[shortlow] - pdec->histo[longlow];

#if DEBUGDETECT >= 1
							DBG_PRINT("equalcnt: pos "); DBG_PRINT(pdec->mstart); DBG_PRINT(" to ") DBG_PRINT(pdec->mend); DBG_PRINT("count=");  DBG_PRINT(equal_cnt); DBG_PRINT(" ");
#endif
							mc_start_found = false;
							if (abs(equal_cnt) > round(pdec->messageLen*0.04))  break; //Next loop
#if DEBUGDETECT >= 1
							DBG_PRINT(" MC equalcnt matched");
#endif
							if (neg_cnt != pos_cnt) break;  // Both must be 2   //TODO: For FFFF we have only 3 valid pulses!
#if DEBUGDETECT >= 1
							DBG_PRINT("  MC neg and pos pattern cnt is equal");
#endif

							if ((longlow == longhigh) || (shortlow == shorthigh) || (longlow == shortlow) || (longhigh == shorthigh) || (longlow == shorthigh) || (longhigh == shortlow)) break; //Check if the indexes are valid

							tstclock = tstclock / 6;
#if DEBUGDETECT >= 1
							MSG_PRINT("  tstclock: "); DBG_PRINT(tstclock);
#endif
							clock = tstclock;

#if DEBUGDETECT >= 1
							DBG_PRINT(" MC LL:"); DBG_PRINT(longlow);
							DBG_PRINT(", MC LH:"); DBG_PRINT(longhigh);

							DBG_PRINT(", MC SL:"); DBG_PRINT(shortlow);
							DBG_PRINT(", MC SH:"); DBG_PRINT(shorthigh);
							DBG_PRINTLN("");
#endif
							// TOdo: Bei FFFF passt diese Pruefung nicht.

#if DEBUGDETECT >= 1
							DBG_PRINTLN("  -- MC found -- ");
#endif

							return true;;

						}
						else {
							mc_start_found = false;
							mc_sync = false;
						}
					} else {
						if (mc_start_found == false)
						{
							pdec->mstart = z;
							mc_start_found = true;
						}
					}
					z++;
				}
			}
		}

	}
	return false;



}

