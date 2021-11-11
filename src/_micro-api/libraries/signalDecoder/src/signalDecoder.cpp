/*
*   Pattern Decoder Library V3
*   Library to decode radio signals based on patternd detection
*   2014-2015  N.Butzek, S.Butzek---
*   2015-2018  S.Butzek
*   2018-2020  S.Butzek, HomeAutoUser
*   
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

// TODO: Update lib with this content to to src\_micro-api\libraries\signalDecoder and use this one instead of a local copy for testing!

// Ony on ESP8266 needed:
#define sd_min(a,b) ((a)<(b)?(a):(b))
#define sd_max(a,b) ((a)>(b)?(a):(b))

#if defined(WIN32) || defined(__linux__) /* is required to run tests on the library - https://github.com/RFD-FHEM/SIGNALDuino/pull/145#discussion_r499140057 */
	#define ARDUINO 101
	#define NOSTRING
#endif



/* **********************************************************
 * SELFMADE - implementation of non standard function itoa()
 * "itoa() function is not defined in ANSI-C and is not part of C++, but is supported by some compilers."
 */

char* myitoa(int num, char *str) {
  static const uint16_t pows10[4] = {10000, 1000, 100, 10};
  const uint16_t *p = pows10;
  uint16_t pow10;
  bool not0 = 0;
  uint8_t idx = 0;
  if (num < 0) {
    str[idx++] = '-';
    num = -num;
  }
  do {
    char c = '0';
    pow10 = *p++;
    while ((uint16_t) num >= pow10) {
      not0 = 1;
      num -= pow10;
      c++;
    }
    if (not0) str[idx++] = c;
  } while (! (pow10 & 2)); // pow10 != 10
  str[idx++] = num + '0';
  str[idx++] = '\0';
  return str;
}

/* END - SELFMADE implementation section */


//Helper function to check buffer for bad data
const bool SignalDetectorClass::checkMBuffer(const uint8_t begin)
{
	for (uint8_t i = begin; i < messageLen-1; i++)
	{
		if ( (pattern[message[i]] ^ pattern[message[i+1]]) >= 0) 
		{
			return false;
		}
	}
	return true;
}


void SignalDetectorClass::bufferMove(const uint8_t start)
{
	m_truncated = false;
	if (start == 0 || messageLen == 0) 	return;

	if (start > messageLen - 1) {
		reset();
	}
	else if (start == 0)
	{
	
	}
	else if (message.moveLeft(start))
	{
		m_truncated = true; 
		//DBG_PRINT(__FUNCTION__); DBG_PRINT(" -> "); 	DBG_PRINT(start);  DBG_PRINT(" "); DBG_PRINT(messageLen);
		//DBG_PRINT(" "); DBG_PRINT(message.bytecount);
		//messageLen = messageLen - start;
		messageLen = message.valcount;
		if (messageLen > 0)
			last = &pattern[message[messageLen - 1]]; //Eventuell wird last auf einen nicht mehr vorhandenen Wert gesetzt, da der Puffer komplett gelöscht wurde
		else
			last = nullptr;
	} else {
		DBG_PRINT(__FUNCTION__); DBG_PRINT(" move error "); 	DBG_PRINT(start);
		//printOut();
	}
}


void SignalDetectorClass::addData(const int8_t value)
{
	//message += value;
	/*if (message.valcount >= 254)
	{
		DBG_PRINTLN(""); 	DBG_PRINT(__FUNCTION__); DBG_PRINT(" msglen: "); DBG_PRINT(messageLen);
	}*/

	if (message.addValue(value))
	{
		messageLen=message.valcount;
		m_truncated = false; // Clear truncated flag
		return;
/*		if (messageLen > 1 && checkMBuffer(messageLen-2))
		{
			return;
		} else {
			SDC_PRINT(" addValue ->");
			SDC_PRINT(" fir="); SDC_PRINT(*first);
			SDC_PRINT(" las="); SDC_PRINT(*last);
		}
		*/
	} else {
		printOut();
		SDC_PRINT(" addData oflow-> mstart="); SDC_PRINT(mstart);
		SDC_PRINT(" mend="); SDC_PRINT(mend);
	}
	SDC_PRINT(" val="); SDC_PRINT(value);
	SDC_PRINT(" msglen="); SDC_PRINT(messageLen);
	SDC_PRINT(" bytc="); SDC_PRINT(message.bytecount);
	SDC_PRINT(" valc="); SDC_PRINT(message.valcount);
	SDC_PRINT(" mTrunc="); SDC_PRINT(m_truncated);
	SDC_PRINT(" state="); SDC_PRINT(state);
	SDC_PRINT(" success="); 
	SDC_PRINTLN(success);
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
	valid = (messageLen == 0 || last == nullptr || (*first ^ *last) < 0); // true if a and b have opposite signs
	valid &= (messageLen == maxMsgSize) ? false : true;
	valid &= (*first > -maxPulse);  // if low maxPulse detected, start processMessage()

//		if (messageLen == 0) pattern_pos = patternLen = 0;
//      if (messageLen == 0) valid = true;
	if (!valid) {
		//DBG_PRINT(" not valid ");

		// Try output
		processMessage();

		// processMessage was not able to find anything useful in our buffer. As the pulses are not valid, we reset and start new buffering. Also a check if pattern has opposit sign is done here again to prevent failuer adding after a move
		if ((success == false && !mcDetected) || (messageLen > 0 && last != nullptr && (*first ^ *last) >= 0)) {
			if (last != nullptr && (*first ^ *last) >= 0 ) //&& *last != 0
			{
				/*
				SDC_PRINT(" nv reset");
				char buf[20];
				sprintf(buf, "%d,%d,%d,%d\n", *first, *last,messageLen,message.valcount);
				SDC_PRINT(buf);
				printOut();
				*/
			}
			reset();
			last = nullptr;
			valid = true;
		}
		
		if (messageLen == maxMsgSize )
		{
			DBG_PRINT(millis());
			DBG_PRINTLN(F(" mb f a t proc ")); // message buffer full after try proccessMessage
			printOut();
		}
	}
	else if (messageLen == minMessageLen) {
		state = detecting;  // Set state to detecting, because we have more than minMessageLen data gathered, so this is no noise
		if (_rssiCallback != nullptr) 
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
			if (histo[pattern_pos] > 2)
			{
				processMessage();
				calcHisto();
			}
			for (uint8_t i = messageLen - 1 ; i >= 0 && histo[pattern_pos] > 0 && messageLen>0; --i)
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
	//histo[fidx ]++;  // need changes in unittests

	#if DEBUGDETECT > 3
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
	else
		last = nullptr;

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
			if (histo[idx2] == 0 || (pattern[idx] ^ pattern[idx2]) < 0)
				continue;
			const int16_t tol = int(((abs(pattern[idx2])*tolFact) + (abs(pattern[idx])*tolFact)) / 2);

			#if DEBUGDETECT > 2
				DBG_PRINT("comptol: "); DBG_PRINT(tol); DBG_PRINT("  "); DBG_PRINT(idx2); DBG_PRINT("<->"); DBG_PRINT(idx); DBG_PRINT(';');
			#endif

			if (inTol(pattern[idx2], pattern[idx], tol))  // Pattern are very equal, so we can combine them
			{
				uint8_t change_count = 0;
				// Change val -> ref_val in message array
				for (uint8_t i = 0; i<messageLen && change_count < histo[idx2]; i++)
				{
					if (message[i] == idx2)
					{
						message.changeValue(i, idx);
						change_count++;
					}
				}

				#if DEBUGDETECT > 2
					DBG_PRINT("compr: "); DBG_PRINT(idx2); DBG_PRINT("->"); DBG_PRINT(idx); DBG_PRINT(';');
					DBG_PRINT(histo[idx2]); DBG_PRINT('*'); DBG_PRINT(pattern[idx2]);
					DBG_PRINT("->");
					DBG_PRINT(histo[idx]); DBG_PRINT('*'); DBG_PRINT(pattern[idx]);
				#endif

				int  sum = histo[idx] + histo[idx2];
				pattern[idx] = ((long(pattern[idx]) * histo[idx]) + (long(pattern[idx2]) * histo[idx2])) / sum;
				histo[idx] += histo[idx2];
				pattern[idx2] = histo[idx2]= 0;

				#if DEBUGDETECT > 2
					DBG_PRINT(" idx:"); DBG_PRINT(pattern[idx]);
					DBG_PRINT(" idx2:"); DBG_PRINT(pattern[idx2]);
					DBG_PRINTLN(';');
				#endif
			}
		}
	}
	/*
	if (!checkMBuffer())
	{
		SDC_PRINT("after compressPattern :");

		SDC_PRINT(" mstart="); SDC_PRINT(mstart);
		SDC_PRINT(" mend="); SDC_PRINT(mend);
		SDC_PRINT(" msglen="); SDC_PRINT(messageLen);
		SDC_PRINT(" bytecnt="); SDC_PRINT(message.bytecount);
		SDC_PRINT(" valcnt="); SDC_PRINT(message.valcount);
		SDC_PRINT(" mTrunc="); SDC_PRINT(m_truncated);
		SDC_PRINT(" state="); SDC_PRINT(state);
		SDC_PRINTLN(" wrong Data in Buffer");
		printOut();
	}
	*/
}


void SignalDetectorClass::processMessage()
{
	yield();
	char buf[22] = {};
	uint8_t n = 0;

	if (mcDetected == true || messageLen >= minMessageLen) {
		success = false;
		m_overflow = (messageLen == maxMsgSize) ? true : false;

		#if DEBUGDETECT >= 1
			DBG_PRINTLN("Message received:");
		#endif

		if (!mcDetected)
		{
			compress_pattern();
			//calcHisto();
			getClock();
			if (state == clockfound && MSenabled) getSync();
		}
		else {
			calcHisto();
		}

		#if DEBUGDETECT >= 1
			printOut();
		#endif

		if (state == syncfound && messageLen >= minMessageLen)// Messages mit clock / Sync Verhaeltnis pruefen
		{
			#if DEBUGDECODE > 0
				DBG_PRINT(" MS check: ");
				//printOut();
			#endif

			// Setup of some protocol identifiers, should be retrieved via fhem in future

			mend = mstart + 2;   // GGf. kann man die Mindestlaenge von x Signalen vorspringen
			bool m_endfound = false;
			bool syncend = false;
			//uint8_t repeat;

			while (mend < messageLen - 1)
			{
				if (!syncend && message[mend + 1] != sync)
				{
					syncend = true; // skip as long as there are sync pulses at start
					//mstart = mend;
				} else if (syncend && message[mend + 1] == sync && message[mend] == clock) {
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
			#endif

			if (m_endfound && (mend - mstart) < minMessageLen) {
				state = clockfound; // step back back to clockfound state, because it is to short for our ms signals
				goto MUOutput;
			}
			if ((m_endfound && (mend - mstart) >= minMessageLen) || (!m_endfound && messageLen < maxMsgSize && (messageLen - mstart) >= minMessageLen))
			{

				#ifdef DEBUGDECODE
					DBG_PRINTLN("Filter Match: ");
				#endif

				//preamble = "";
				//postamble = "";

				/* ### ### Output raw message Data ### ### */
				SDC_PRINT(MSG_START);

				if (MredEnabled) {
					int patternInt;
					uint8_t patternLow;
					uint8_t patternIdx;

					SDC_PRINT("Ms;");
					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue;
						patternIdx = idx;
						patternInt = pattern[idx];

						if (patternInt < 0) {
							patternIdx = idx | 0xA0;    // Bit5 = 1 (Vorzeichen negativ)
							patternInt = -patternInt;
						}
						else {
							patternIdx = idx | 0x80;    // Bit5 = 0 (Vorzeichen positiv)
						}

						patternLow = lowByte(patternInt);
						if (bitRead(patternLow, (uint8_t)7) == 0) {
							bitSet(patternLow, (uint8_t)7);
						}
						else {
							bitSet(patternIdx, (uint8_t)4);   // wenn bei patternLow Bit7 gesetzt ist, dann bei patternIdx Bit4 = 1
						}
						SDC_PRINT(patternIdx);
						SDC_PRINT(patternLow);
						SDC_PRINT(highByte(patternInt) | 0x80);
						SDC_PRINT(';');
					}

					//uint8_t n;
					if ((mend & 1) == 1) {   // zwei Nibble im letzten Byte übergeben
						SDC_PRINT('D');
					}
					else {
						SDC_PRINT('d');    // ein Nibble im letzten Byte übergeben
					}
					if ((mstart & 1) == 1) {  // ungerade Startposition
						mstart--;
						
						message.getByte(mstart / 2, &n);
						n = (n & 15) | 128;             // high nibble = 8 als Kennzeichen für ungeraden mstart

						SDC_PRINT(n);
 						mstart += 2;
					}
					for (uint8_t i = mstart; i <= mend; i=i+2) {					
						message.getByte(i/2,&n);
						SDC_PRINT(n);
					}

					//␂Ms;��;��;���;��;D$$!!!###!#!#!!!!#!##!##!!##!!!!!!!!!!!!!!#!!␂    ;C2;S4;    RF1;O;m2;␃
					SDC_PRINT(";C"); SDC_PRINT(clock + 48); SDC_PRINT(";S"); SDC_PRINT(sync + 48); SDC_PRINT(';');

					if (_rssiCallback != nullptr)
					{
						//␂Ms;��;��;���;��;D$$!!!###!#!#!!!!#!##!##!!##!!!!!!!!!!!!!!#!!␂;C2;S4;    RF1;    O;m2;␃
						SDC_PRINT('R'); SDC_PRINT_intToHex(rssiValue); SDC_PRINT(';');
					}
				}
				else {
					SDC_PRINT("MS;");

					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue;
						//␂MS;    P2=-14273;P3=371;P4=-1430;P5=1285;P6=-540;    D=32345634563456345634563456345634565656343434343434;CP=3;SP=2;R=35;m2;␃
						SDC_PRINT('P'); SDC_PRINT(idx + 48); SDC_PRINT('='); SDC_PRINT(myitoa(pattern[idx], buf)); SDC_PRINT(';');
					}

					SDC_PRINT("D=");

					for (uint8_t i = mstart; i <= mend; i++)
					{
						//␂MS;P2=-14273;P3=371;P4=-1430;P5=1285;P6=-540;D=    32345634563456345634563456345634565656343434343434    ;CP=3;SP=2;R=35;m2;␃
						SDC_PRINT(message[i] + 48);
					}

					//␂MS;P2=-14273;P3=371;P4=-1430;P5=1285;P6=-540;D=32345634563456345634563456345634565656343434343434    ;CP=3;SP=2;    R=35;m2;␃
					SDC_PRINT(";CP="); SDC_PRINT(clock + 48); SDC_PRINT(";SP="); SDC_PRINT(sync + 48); SDC_PRINT(';');
          
					if (_rssiCallback != nullptr)
					{
						//␂MS;P2=-14273;P3=371;P4=-1430;P5=1285;P6=-540;D=32345634563456345634563456345634565656343434343434;CP=3;SP=2;    R=35;    m2;␃
						SDC_PRINT("R="); SDC_PRINT(myitoa(rssiValue, buf)); SDC_PRINT(';');
					}
				}

				if (m_overflow) {
					//␂MS;P1=489;P4=-2045;P5=-3971;P6=-8054;D=1616141414151515141514151414141414141414141515151415151414141414141414141414141415141415;CP=1;SP=6;R=239;    O;    m2;␃
					SDC_PRINT("O;");
				}
				m_truncated = false;
				
				if ((messageLen - mend) >= minMessageLen && MsMoveCount > 0) {
					//SDC_PRINT(F("MS move. messageLen ")); SDC_PRINT(messageLen); SDC_PRINT(" "); SDC_PRINTLN(MsMoveCount)
					MsMoveCount--;
					bufferMove(mend+1);
					//SDC_PRINT(F("MS move. messageLen ")); SDC_PRINTLN(messageLen);
					mstart = 0;

					//␂MS;P1=489;P4=-2045;P5=-3971;P6=-8054;D=1616141414151515141514151414141414141414141515151415151414141414141414141414141415141415;CP=1;SP=6;R=239;O;    m2;␃
					SDC_PRINT('m'); SDC_PRINT(MsMoveCount + 48); SDC_PRINT(';');
				}
				SDC_PRINT(MSG_END);
				SDC_PRINT(char(0xA));
				success = true;
			}
			else if (m_endfound == false && mstart > 0 && mend + 1 >= maxMsgSize) // Start found, but no end. We remove everything bevore start and hope to find the end later
			{
				//SDC_PRINT("copy");
				#ifdef DEBUGDECODE
					DBG_PRINT(" move msg ");
				#endif

				bufferMove(mstart);
				mstart = 0;
				//m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
			} 
			else if (m_endfound && mend < maxMsgSize) {  // Start and end found, but end is not at end of buffer, so we remove only what was checked
				#ifdef DEBUGDECODE
					DBG_PRINT(" move msg ");
				#endif

				bufferMove(mend+1);
				mstart = 0;
				//m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
				success = true;	// don't process other message types
			}
			else {
				#ifdef DEBUGDECODE
					SDC_PRINTLN(" Buffer overflow, flushing message array");
				#endif

				//SDC_PRINT(MSG_START);
				//SDC_PRINT("Buffer overflow while processing signal");
				//SDC_PRINT(MSG_END);
				reset(); // Our Messagebuffer is not big enough, no chance to get complete Message

				success = true;	// don't process other message types
			}
		}
MUOutput:
		if (success == false && (MUenabled || MCenabled)) {

			#if DEBUGDECODE > 0
				DBG_PRINT(" check:");

				//printOut();
			#endif	
			// Message has a clock puls, but no sync. Try to decode this

			//preamble = "";
			//postamble = "";

			if (MCenabled)
			{
				//DBG_PRINT(" mc: ");
				//SDC_PRINT(" try mc ");

				//static ManchesterpatternDecoder mcdecoder(this);			// Init Manchester Decoder class
				if (mcdecoder == nullptr) { mcdecoder = new ManchesterpatternDecoder(this); }
				if (mcDetected == false)
				{
					mcdecoder->reset();
					mcdecoder->setMinBitLen(mcMinBitLen);
				}

				#if DEBUGDETECT > 3
					SDC_PRINT("vcnt: "); SDC_PRINT(mcdecoder->ManchesterBits.valcount);
				#endif

				if ((mcDetected || mcdecoder->isManchester()))	// Check if valid manchester pattern and try to decode
				{
					#if DEBUGDECODE > 1
						DBG_PRINTLN(" MC found: ");
					#endif

					#if DEBUGDECODE == 1
						SDC_WRITE(MSG_START);
						SDC_PRINT("DMC");
						SDC_WRITE(';');

						for (uint8_t idx = 0; idx < patternLen; idx++)
						{
							//calcHisto();
							if (histo[idx] == 0) continue;
							//SDC_PRINT('P'); SDC_PRINT(idx); SDC_PRINT('='); SDC_PRINT(itoa(pattern[idx], buf, 10)); SDC_PRINT(SERIAL_DELIMITER);
							n = sprintf(buf, "P%i=%i;", idx, pattern[idx]);
							//SDC_WRITE((const uint8_t *)buf, n);
							SDC_WRITE(buf);
						}
						SDC_PRINT("D=");

						for (uint8_t i = 0; i < messageLen; ++i)
						{
							SDC_PRINT(myitoa(message[i], buf));
						}
						SDC_PRINT(';');

						if (m_overflow) {
							SDC_PRINT("O;");
						}

						if (mcDetected) {
							SDC_PRINT("MD;");
						}

						SDC_PRINTLN(MSG_END);
					#endif

					if (mcdecoder->doDecode())
					{
						SDC_PRINT(MSG_START);

						//␂    MC;LL=-2926;LH=2935;SL=-1472;SH=1525    ;D=AFF1FFA1;C=1476;L=32;R=0;␃
						SDC_PRINT("MC;LL="); SDC_PRINT(myitoa(pattern[mcdecoder->longlow], buf));
						SDC_PRINT(";LH="); SDC_PRINT(myitoa(pattern[mcdecoder->longhigh], buf));
						SDC_PRINT(";SL="); SDC_PRINT(myitoa(pattern[mcdecoder->shortlow], buf));
						SDC_PRINT(";SH="); SDC_PRINT(myitoa(pattern[mcdecoder->shorthigh], buf));

						//␂MC;LL=-2926;LH=2935;SL=-1472;SH=1525    ;D=AFF1FFA1;C=1476;L=32;    R=0;␃
						SDC_PRINT(";D="); mcdecoder->printMessageHexStr();
						SDC_PRINT(";C="); SDC_PRINT(myitoa(mcdecoder->clock, buf));
						SDC_PRINT(";L="); SDC_PRINT(myitoa(mcdecoder->ManchesterBits.valcount, buf));
						SDC_PRINT(';');

						if (_rssiCallback != nullptr)
						{
							//␂MC;LL=-2926;LH=2935;SL=-1472;SH=1525;D=AFF1FFA1;C=1476;L=32;    R=0;    ␃
							SDC_PRINT("R="); SDC_PRINT(myitoa(rssiValue, buf)); SDC_PRINT(';');
						}

						//␂MC;LL=-2926;LH=2935;SL=-1472;SH=1525;D=AFF1FFA1;C=1476;L=32;R=0;    ␃
						SDC_PRINT(MSG_END);
						SDC_PRINT(char(0xA));

						#ifdef DEBUGDECODE
							DBG_PRINTLN("");
						#endif
						//					printMsgStr(&preamble, &mcbitmsg, &postamble);
						mcDetected = false;
						success = true;
					}
					else if (mcDetected == true) {
						if (m_truncated == true) {
							//SDC_PRINT(" mc true");
							success = true;   // Prevents MU Processing
						} else {
							// message buffer is untouched till now, but we will reset the the message buffer now to preserve anything else
							message.reset();
							messageLen = message.valcount;
							m_truncated = true; // Preserve anything else like pattern and so on.
						}
					}
				}
			}
		}
		if (MUenabled && !mcDetected && state == clockfound && success == false && messageLen >= minMessageLen) {
				//SDC_PRINT(" try mu");

				#if DEBUGDECODE > 1
					DBG_PRINT(" MU found: ");
				#endif

				SDC_PRINT(MSG_START);

				if (MredEnabled) {
					int patternInt;
					uint8_t patternLow;
					uint8_t patternIdx;

					SDC_PRINT("Mu;");
					calcHisto();
					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue;
						patternIdx = idx;
						patternInt = pattern[idx];

						if (patternInt < 0) {
							patternIdx = idx | 0xA0;    // Bit5 = 1 (Vorzeichen negativ)
							patternInt = -patternInt;
						}
						else {
							patternIdx = idx | 0x80;    // Bit5 = 0 (Vorzeichen positiv)
						}
						
						patternLow = lowByte(patternInt);
						if (bitRead(patternLow, (uint8_t)7) == 0) {
							bitSet(patternLow, (uint8_t)7);
						}
						else {
							bitSet(patternIdx, (uint8_t)4);   // wenn bei patternLow Bit7 gesetzt ist, dann bei patternIdx Bit4 = 1
						}
						SDC_PRINT(patternIdx);
						SDC_PRINT(patternLow);
						SDC_PRINT(highByte(patternInt) | 0x80);
						SDC_PRINT(';');
					}

					if ((messageLen & 1) == 1) {  // ein Nibble im letzten Byte übergeben ungerade 
						SDC_PRINT('d');
					}
					else {
						SDC_PRINT('D');			// zwei Nibble im letzten Byte übergeben ungerade 
					}

					for (uint8_t i = 0; i <= message.bytecount; i++) {
						message.getByte(i, &n);
						SDC_PRINT(n);
					}

					//␂Mu;���;�؁;���;���;���;�Å;���;�܅;D␁#$TgTTgggggggggggTgggT    ;C7;    R21;␃
					SDC_PRINT(";C"); SDC_PRINT(clock + 48); SDC_PRINT(';');

					if (_rssiCallback != nullptr)
					{
						//␂Mu;���;�؁;���;���;���;�Å;���;�܅;D␁#$TgTTgggggggggggTgggT;C7;    R21;    ␃
						SDC_PRINT('R'); SDC_PRINT_intToHex(rssiValue); SDC_PRINT(';');
					}
				}
				else {
				
					SDC_PRINT("MU;");
					calcHisto();

					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue;

						//␂MU;    P0=-32001;P3=373;P4=-1432;P5=1287;P6=-540;    D=34563456345634563456345634563456565634343434343430;CP=3;R=25;␃
						SDC_PRINT('P'); SDC_PRINT(idx + 48); SDC_PRINT('='); SDC_PRINT(myitoa(pattern[idx], buf)); SDC_PRINT(';');
					}

					SDC_PRINT("D=");

					for (uint8_t i = 0; i < messageLen; ++i) 
					{
						//␂MU;P0=-32001;P3=373;P4=-1432;P5=1287;P6=-540;D=    34563456345634563456345634563456565634343434343430    ;CP=3;R=25;␃
						SDC_PRINT(message[i] + 48);
					}
					//String postamble;

					//␂MU;P0=-32001;P3=373;P4=-1432;P5=1287;P6=-540;D=34563456345634563456345634563456565634343434343430    ;CP=3;    R=25;␃
					SDC_PRINT(";CP="); SDC_PRINT(clock + 48); SDC_PRINT(';');

					if (_rssiCallback != nullptr)
					{
						//␂MU;P0=-32001;P3=373;P4=-1432;P5=1287;P6=-540;D=34563456345634563456345634563456565634343434343430;CP=3;    R=25;    ␃
						SDC_PRINT("R="); SDC_PRINT(myitoa(rssiValue, buf)); SDC_PRINT(';');
					}
				}

				if (m_overflow) {
					SDC_PRINT("O;");
				}

				SDC_PRINT(MSG_END);
				SDC_PRINT(char(0xA));
				
				m_truncated = false;
				success = true;
			}
		}
		
	if (success == false) 
	{
		if (m_truncated)
		{
			// no clock, not mc but truncated buffer, so we are now here

			int dp = 2000;  // marked as max value, but outside buffer range
			for (uint8_t i = 0; i < patternLen; i++)
			{
				if (pattern[i] > 0)
				{
					if (sd_min(pattern[i], dp) < dp)  				// Todo wenn der Puffer nur aus negativen Werten besteh, dann müssen wir auch etwas machen
					{
						dp = i;
					}
				}
			}
			for (uint8_t i = 0; i < messageLen && dp <2000; i++)
			{
				if (message[i] == dp) // Finde den letzten Verweis im Array auf den Index der gleich ueberschrieben wird
				{
					i--; // i verringern, damit ein zusätzlicher Wert erhalten bleibt
					bufferMove(i);
					break;
				}
			}
		}
		else {
						#if DEBUGDETECT >= 1
							DBG_PRINTLN("nothing to to");
						#endif
					}
	}
	
	if (!m_truncated)  // Todo: Eventuell auf vollen Puffer prüfen
	{
		bool _success=success;  // save success because it is set to false in reset()
		reset();
		success=_success;  // restore success
	}

	//SDC_PRINTLN("process finished");
}


/* function to convert to HEX without a leading zero */
void SignalDetectorClass::SDC_PRINT_intToHex(unsigned int numberToPrint) {  // smaller memory variant for sprintf hex output ( sprintf(buf, "R%X;", value) )
  if (numberToPrint >= 16)
    SDC_PRINT_intToHex(numberToPrint / 16);
  /* line is needed, no line - no output !!! */
  SDC_PRINT("0123456789ABCDEF"[numberToPrint % 16]);
}


void SignalDetectorClass::reset()
{
	patternLen = 0;
	pattern_pos = 0;
	message.reset();
	messageLen = message.valcount;
	//	bitcnt = 0;
	state = searching;
	clock = sync = -1;
	for (uint8_t i = 0; i<maxNumPattern; ++i)
		histo[i] = pattern[i] = 0;
	success = false;
	tol = 150; //
	tolFact = 0.25;
	mstart = 0;
	m_truncated = false;
	m_overflow = false;
	mcDetected = false;
	//SDC_PRINTLN("reset");
	mend = 0;
	//DBG_PRINT(":sdres:");
	last = nullptr;
	MsMoveCount = 3;
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
#ifdef DEBUG
	DBG_PRINTLN("");
	DBG_PRINT("Sync: ");

	if (sync > -1) {
		if (sync > -1) {
			DBG_PRINT(pattern[sync]);
			DBG_PRINT(" -> SyncFact: "); DBG_PRINT(pattern[sync] / (float)pattern[clock]);
			DBG_PRINT(',');
		} else 
			DBG_PRINT("NULL");
	} 
	DBG_PRINT(" Clock: "); 
	if (clock > -1) {
		DBG_PRINT(pattern[clock]);
	} else
		DBG_PRINT("NULL");

	DBG_PRINT(", Tol: "); DBG_PRINT(tol);
	DBG_PRINT(", PattLen: "); DBG_PRINT(patternLen); DBG_PRINT(" ("); DBG_PRINT(pattern_pos); DBG_PRINT(')');
	DBG_PRINT(", Pulse: "); DBG_PRINT(*first); DBG_PRINT(", "); DBG_PRINT(*last);
	DBG_PRINT(", mStart: "); DBG_PRINT(mstart);
	DBG_PRINT(", MCD: "); DBG_PRINT(mcDetected,DEC);
	DBG_PRINT(", mtrunc: "); DBG_PRINT(m_truncated, DEC);

	DBG_PRINTLN(); DBG_PRINT("Signal: ");
	uint8_t idx;
	for (idx = 0; idx<messageLen; ++idx) {
		const char c = message.getValue(idx) + '0';
		DBG_PRINT(c);
	}

	DBG_PRINT(". "); DBG_PRINT(" ["); DBG_PRINT(messageLen); DBG_PRINTLN(']');
	DBG_PRINT("Pattern: ");
	for (uint8_t idx = 0; idx<patternLen; ++idx) {
		DBG_PRINT(" P"); DBG_PRINT(idx);
		DBG_PRINT(": "); DBG_PRINT(histo[idx]);  DBG_PRINT("*[");
		if (pattern[idx] != 0)
		{
			//DBG_PRINT(",");
			DBG_PRINT(pattern[idx]);
		}
		DBG_PRINT(']');
	}
	DBG_PRINTLN();
#else
	delay(0);
#endif
}


const size_t SignalDetectorClass::write(const uint8_t *buf, size_t size)
{
	if (_streamCallback == nullptr)
		return 0;
	return _streamCallback(buf, size);
}


const size_t SignalDetectorClass::write(const char *str) {
	if (str == nullptr)
		return 0;
	return write((const uint8_t*)str, strlen(str));
}


const size_t SignalDetectorClass::write(uint8_t b)
{
	return write(&b, 1);
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
	if (messageLen == 0) return;

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
	uint8_t bval=0;
	if (startpos % 2 == 1)  // ungerade
	{
		message.getByte(bstartpos, &bval);
		histo[bval & 0XF]++;
		bstartpos++;
	}
	for (uint8_t i = bstartpos; i<bendpos; ++i)
	{
		message.getByte(i,&bval);
		histo[bval >> 4]++;
		histo[bval & 0xF]++; //Todo: 0x7
	}
	if (endpos % 2 == 1)
	{
		message.getByte(bendpos, &bval);
		histo[bval >> 4]++;
	}
}


bool SignalDetectorClass::getClock()
{
	// Durchsuchen aller Musterpulse und prueft ob darin eine clock vorhanden ist
	#if DEBUGDETECT > 3
		SDC_PRINTLN("  --  Searching Clock in signal -- ");
	#endif

	int tstclock = -1;
	state = searching;

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
	{					// clock wurde bereits durch getclock bestimmt

		const uint8_t syncLenMax = 125; 		      //  wenn in den ersten ca 125 Pulsen kein Sync gefunden wird, dann ist es kein MS Signal
		const uint8_t max_search = sd_min(syncLenMax, messageLen - minMessageLen);

		for (int8_t p = patternLen - 1; p >= 0; --p)  // Schleife fuer langen Syncpuls
		{
			uint16_t syncabs = abs(pattern[p]);
			if ((pattern[p] < 0) &&
				(syncabs < syncMaxMicros && syncabs / pattern[clock] <= syncMaxFact) &&
				(syncabs > syncMinFact*pattern[clock]) &&
				(histo[p] < messageLen*0.08) && (histo[p] > 1)
				)
			{

				// Pruefen ob der gefundene Sync auch als message [clock, p] vorkommt
				uint8_t c = 0;
				while (c < max_search)
				{
					if (message[c + 1] == p && message[c] == clock) {
						sync = p;
						state = syncfound;
						mstart = c;

						#ifdef DEBUGDECODE
							DBG_PRINTLN();
							DBG_PRINT("PD sync: ");
							DBG_PRINT(pattern[clock]); DBG_PRINT(", "); DBG_PRINT(pattern[p]);
							DBG_PRINT(", TOL: "); DBG_PRINT(tol);
							DBG_PRINT(", sFACT: "); DBG_PRINT(pattern[sync] / (float)pattern[clock]);
							DBG_PRINT(", mstart: "); DBG_PRINTLN(mstart);
						#endif

						return true;
					};
					c++;
				}
				//if (c==messageLen) continue;	// nichts gefunden, also Sync weitersuchen
				c++;
			}
		}
	}
	sync = -1;		// Workaround for sync detection bug.
	return false;
}

/*
void SignalDetectorClass::printMsgStr(const String * first, const String * second, const String * third)
{
	SDC_PRINT(*first);
	SDC_PRINT(*second);
	SDC_PRINT(*third);

}
*/
/*
int8_t SignalDetectorClass::printMsgRaw(uint8_t m_start, const uint8_t m_end, const String * preamble, const String * postamble)
{
	SDC_PRINT(*preamble);
	//String msg;
	//msg.reserve(m_end-mstart);

	for (; m_start <= m_end; m_start++)
	{
		//msg + =message[m_start];
		//SDC_PRINT((100*message[m_start])+(10*message[m_start])+message[m_start]);
		SDC_PRINT(message[m_start]);

	}
	//SDC_PRINT(msg);
	SDC_PRINT(*postamble);
	yield();

}
*/






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
	//minbitlen = 20; // Set defaults
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
#ifdef NOSTRING
	const char* ManchesterpatternDecoder::getMessageHexStr()
#else
	void ManchesterpatternDecoder::getMessageHexStr(String *message)
#endif
{
	char hexStr[] = "00" ; // Not really needed
	#ifndef NOSTRING
		message->reserve((ManchesterBits.valcount /4)+2);
		if (!message)
			return;
	#else
		char *message = (char*)malloc((sizeof(char)*ManchesterBits.valcount / 4) + 2);
		char *mptr=message;
	#endif

	uint8_t idx;
	// Bytes are stored from left to right in our buffer. We reverse them for better readability
	for ( idx = 0; idx <= ManchesterBits.bytecount-1; ++idx) {
		//SDC_PRINT(getMCByte(idx),HEX);
		//sprintf(hexStr, "%02X",reverseByte(ManchesterBits->>getByte(idx)));
		//SDC_PRINT(".");
		sprintf(hexStr, "%02X", getMCByte(idx));

		#ifdef NOSTRING
			//uint8_t *buf = (uint8_t*)hexStr;
			//pdec->write(buf, 2);
			memcpy(mptr, hexStr, sizeof(*hexStr)*2);
			mptr = mptr + (sizeof(*mptr) * 2);
		#endif
		//SDC_PRINT(hexStr);
	}

	sprintf(hexStr, "%01X", getMCByte(idx) >> 4 & 0xf);
	#ifdef NOSTRING
		//pdec->write(hexStr[0]);
		memcpy(mptr, hexStr, sizeof(*hexStr) * 1);
		mptr = mptr + (sizeof(*mptr));
	#else
		message->concat(hexStr);
	#endif

	if (ManchesterBits.valcount % 8 > 4 || ManchesterBits.valcount % 8 == 0)
	{
		sprintf(hexStr, "%01X", getMCByte(idx) & 0xF);

		#ifdef NOSTRING
			//uint8_t *buf = (uint8_t*)hexStr;
			memcpy(mptr, hexStr, sizeof(*hexStr) * 1);
			mptr = mptr + (sizeof(*mptr));

			//pdec->write(hexStr[0]);
		#else
			message->concat(hexStr);
		#endif
	}

	#ifdef NOSTRING
		*mptr = '\0';
		return message;
	#endif
	//SDC_PRINTLN();
}


/* convert a 4-bit nibble to a hexadecimal character */
char ManchesterpatternDecoder::nibble_to_HEX(uint8_t nibble) {
  nibble &= 0xF;
  return nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
}


/* convert byte to 2 hexadecimal characters | sprintf(cbuffer +1, "%02X", getMCByte(idx) & 0xF); */
void ManchesterpatternDecoder::HEX_twoDigits(char* cbuffer, uint8_t val)
{
  cbuffer[0] = nibble_to_HEX(val >> 4);
  cbuffer[1] = nibble_to_HEX(val);
  cbuffer[2] = '\0';
}


/** @brief (Converts decoded manchester bits in a provided string as hex)
*
* ()
*/
void ManchesterpatternDecoder::printMessageHexStr()
{
	//char hexStr[] = "00"; // Not really needed

	char cbuffer[3];
	uint8_t idx;
	// Bytes are stored from left to right in our buffer. We reverse them for better readability
	for (idx = 0; idx <= ManchesterBits.bytecount - 1; ++idx) {
		HEX_twoDigits(cbuffer, getMCByte(idx));
		pdec->write(cbuffer);
	}

		//sprintf(cbuffer, "%01X", getMCByte(idx) >> 4 & 0xf);
		pdec->write(nibble_to_HEX(getMCByte(idx) >> 4 & 0xf));
		//pdec->write(hexStr);
	if (ManchesterBits.valcount % 8 > 4 || ManchesterBits.valcount % 8 == 0)
	{
		//sprintf(cbuffer +1, "%01X", getMCByte(idx) & 0xF);
		pdec->write(nibble_to_HEX(getMCByte(idx) & 0xF));
	}
	//pdec->msgPort->print(cbuffer);

	//pdec->write(cbuffer);
}


/** @brief (one liner)
*
* (documentation goes here)
*/

#ifdef NOSTRING
	const char * ManchesterpatternDecoder::getMessagePulseStr()
#else
	void ManchesterpatternDecoder::getMessagePulseStr(String* str)
#endif
{
#ifdef NOSTRING
	char *message = (char*)malloc(sizeof(char)*50);
	char *mptr = message;

	sprintf(message, ";LL=%i;LH=%i;SL=%i;SH=%i;", pdec->pattern[longlow], pdec->pattern[longhigh], pdec->pattern[shortlow], pdec->pattern[shorthigh]);

	return message;
#else
	str->reserve(32);
	if (!str)
		return;

	str->concat("LL="); str->concat(pdec->pattern[longlow]);
	str->concat(";LH="); str->concat(pdec->pattern[longhigh]);
	str->concat(";SL="); str->concat(pdec->pattern[shortlow]);
	str->concat(";SH="); str->concat(pdec->pattern[shorthigh]); str->concat(';');
#endif
}


/** @brief (one liner)
*
* (documentation goes here)
*/

#ifdef NOSTRING
	const char * ManchesterpatternDecoder::getMessageClockStr()
#else
	void ManchesterpatternDecoder::getMessageClockStr(String* str)
#endif
{
#ifdef NOSTRING

	char *message = (char*)malloc(sizeof(char) * 10);
	char *mptr = message;

	sprintf(message, ";C=%i;", clock);

	return message;
#endif

#ifndef NOSTRING
	str->reserve(7);
	if (!str)
		return;

	str->concat("C="); str->concat(clock); str->concat(';');
#endif
}


#ifdef NOSTRING
	const char* ManchesterpatternDecoder::getMessageLenStr()
#else
	void ManchesterpatternDecoder::getMessageLenStr(String* str)
#endif
{
#ifndef NOSTRING
	str->concat("L="); str->concat(ManchesterBits.valcount); str->concat(';');
#else
	char *buf = (char*)malloc(7);

	sprintf(buf, ";L=%i", ManchesterBits.valcount);
	return buf;
#endif
}


/** @brief (retieves one Byte out of the Bitstore for manchester decoded bits)
*
* (Returns a comlete byte from the pattern store)
*/
unsigned char ManchesterpatternDecoder::getMCByte(const uint8_t idx) {
	uint8_t c = 0;
	ManchesterBits.getByte(idx,&c);
	return  c;
}


/** @brief (Decodes the manchester pattern to bits. Returns true on success and false on error )
*
* (Call only after ismanchester returned true)
*/
const bool ManchesterpatternDecoder::doDecode() {
	//SDC_PRINT("bitcnt:");SDC_PRINTLN(bitcnt);
	uint8_t i = 0;
	pdec->m_truncated = false;
	pdec->mstart = 0; // Todo: pruefen ob start aus isManchester uebernommen werden kann

	#ifdef DEBUGDECODE
		DBG_PRINT("mlen:");
		DBG_PRINT(pdec->messageLen);
		DBG_PRINT(":mstart: ");
		DBG_PRINT(pdec->mstart);
		DBG_PRINTLN("");
	#endif

	static uint8_t bit = 0; // bit state must be preserved if message goes over the buffer 
	//bool prelongdecoding = false; // Flag that we are in a decoding bevore the 1. long pulse
	#ifdef DEBUGDECODE
		char value = NULL;
	#endif

	pdec->mcDetected = false; // Reset our flag, so we can set it again or not for second decoding

	while (i < pdec->messageLen)
	{
		// Start vom MC Signal suchen, dazu long suchen
		if (mc_sync == false && isLong(pdec->message[i]))
		{
			bit = pdec->message[i] == longhigh ? 0 : 1; // Bit welches an der 2. Hälfte des long beginnt
			const uint8_t mpiLessOne = pdec->message[i - 1]; // Store previois pattern for further processing

															 // Wen lh also bit = 1 dann haben wir einen slow->shigh davor. der davor empfangene short gehört zu dem Bit 1 und darf nicht weiter beachtet werden!
			mc_sync = true;
			pdec->mstart = i;  // Save sync position for later

			if (i >0)
			{
				if ((bit == 0 && (mpiLessOne == shortlow || pdec->pattern[mpiLessOne] < pdec->pattern[longlow])) || (bit == 1 && (mpiLessOne == shorthigh || pdec->pattern[mpiLessOne] > pdec->pattern[longhigh]))) {
					bit = bit ^ 1; // Vor dem long die Bits erkennen 
				}
				else {
					pdec->mstart++;
				}
				ManchesterBits.addValue(bit);

				#ifdef DEBUGDECODE
					value = bit == 1 ? 'P' : 'p';
					DBG_PRINT(value);
					DBG_PRINT((int)ManchesterBits.getValue(ManchesterBits.valcount - 1), DEC);
				#endif

				while (--i > 1 && isShort(pdec->message[i]) && isShort(pdec->message[i - 1]))
				{
					if ((bit == 1 && (pdec->message[i - 2] == shortlow || pdec->pattern[pdec->message[i - 2]] < pdec->pattern[longlow])) || (bit == 0 && (pdec->message[i - 2] == shorthigh || pdec->pattern[pdec->message[i - 2]] > pdec->pattern[longhigh])))
					{
						// Short puls or longer as longxxx puls detected which matches current bit
						ManchesterBits.addValue(bit);

						#ifdef DEBUGDECODE
							value = bit == 1 ? 'P' : 'p';
							DBG_PRINT(value);
							DBG_PRINT((int)ManchesterBits.getValue(ManchesterBits.valcount - 1), DEC);
						#endif
					}
					i--;
				}
				i = pdec->mstart; // recover i to mstart
			}
			else
				bit = bit ^ 1; // umdrehen, da es erneut beim dekodieren umgedreht wird 

		}

		// Decoding occures here
		if (mc_sync)
		{
			const int8_t mpi = pdec->message[i]; // get pattern for further processing

			if (i < pdec->messageLen - 1 && isLong(mpi) ) {
				bit = bit ^ (1);

				#ifdef DEBUGDECODE
					value = 'L';
					if (mpi == longlow)
						value = 'l';
				#endif
			} else {  // No long
				const int8_t mpiPlusOne = pdec->message[i + 1]; // get  next pattern for further processing
				if (    pdec->messageLen - 2 <= i ||
					(	bit == 0 && (mpi != shortlow  || mpiPlusOne != shorthigh) ) ||
					(	bit == 1 && (mpi != shorthigh || mpiPlusOne != shortlow) )
				   ) {
					// Found something that fits not to our manchester signal
					#ifdef DEBUGDECODE
						DBG_PRINT("H(");
						DBG_PRINT("vcnt:");
						DBG_PRINT(ManchesterBits.valcount - 1, DEC);
					#endif

					if (ManchesterBits.valcount < minbitlen)
					{
						mc_start_found = false; // Reset to find new starting position
						mc_sync = false;

						#ifdef DEBUGDECODE
							DBG_PRINT(":RES:");
						#endif

						ManchesterBits.reset();
					} else {
						pdec->mend = i;

						#ifdef DEBUGDECODE
							DBG_PRINT(":mpos=");
							DBG_PRINT(i, DEC);
							DBG_PRINT(":mstart=");
							DBG_PRINT(pdec->mstart, DEC);
							DBG_PRINT(":mend:");
							DBG_PRINT(pdec->mend, DEC);
							DBG_PRINT(":mlen:");
							DBG_PRINT(pdec->messageLen, DEC);
							DBG_PRINT(":found:");
							DBG_PRINT(":pidx=");
							DBG_PRINT((int)pdec->message[i], DEC);
							//DBG_PRINT(pdec->pattern[pdec->message[i]]);
						#endif

						if (i == maxMsgSize - 1 && i == pdec->messageLen - 1)
						{
							pdec->mcDetected = true;
							//i--; // Process short later again, do not remove it
							pdec->state = mcdecoding; // Try to prevent other processing
						}
						else if ((pdec->pattern[mpi] < pdec->pattern[longlow] && i < pdec->messageLen - 1 && (mpiPlusOne == longhigh || mpiPlusOne == shorthigh))
							|| (i<pdec->messageLen - 2 && isShort(mpi) && pdec->pattern[mpiPlusOne] < pdec->pattern[longlow] && (isLong(pdec->message[i + 2]) || isShort(pdec->message[i + 3])) && i++)
							)
						{
							i++;  // This will remove a gap between two transmissions of a message preventing the gap to be interpreded as part of the message itself
							pdec->state = mcdecoding; // Try to prevent other processing
						}

						pdec->bufferMove(i);   // Todo: BufferMove könnte in die Serielle Ausgabe verschoben werden, das würde ein paar Mikrosekunden Zeit sparen

						#ifdef DEBUGDECODE
							DBG_PRINT(":mpos=");
							DBG_PRINT(i, DEC);
							DBG_PRINT(":mstart=");
							DBG_PRINT(pdec->mstart, DEC);
							DBG_PRINT(":mend:");
							DBG_PRINT(pdec->mend, DEC);
							DBG_PRINT(":mlen:");
							DBG_PRINT(pdec->messageLen, DEC);
							DBG_PRINT(":found:pidx=");
							DBG_PRINT((int)pdec->message[i], DEC);
							DBG_PRINT(":minblen=");
							DBG_PRINTLN(ManchesterBits.valcount >= minbitlen, DEC);
						#endif

						return (!pdec->mcDetected && ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed
					}

					#ifdef DEBUGDECODE
						DBG_PRINT(')');
					#endif						
				} else {
					i++;
				} 
				#ifdef DEBUGDECODE
				if (bit == 0 && mpi == shortlow && mpiPlusOne == shorthigh)	{
					value = 's';
				} else if (bit == 1 && mpi == shorthigh && mpiPlusOne == shortlow)	{
					value = 'S';
				} 
				#endif
			}

			if (mc_sync) { // don't add bit if manchester processing was canceled
				ManchesterBits.addValue(bit);
				#ifdef DEBUGDECODE
					DBG_PRINT(value);
					DBG_PRINT((uint8_t)ManchesterBits.getValue(ManchesterBits.valcount - 1), DEC);
				#endif
			} else {
				#ifdef DEBUGDECODE
				  DBG_PRINT('_');
				#endif
			}
		} // 		endif (mc_sync)
		//if (i<255) i++;
		i++;
	}
	pdec->mend = i; // Todo: keep short in buffer;

	#ifdef DEBUGDECODE
		DBG_PRINT(":mpos=");
		DBG_PRINT(i);
		DBG_PRINT(":mstart=");
		DBG_PRINT(pdec->mstart, DEC);
		DBG_PRINT(":mend=");
		DBG_PRINT(pdec->mend, DEC);
		DBG_PRINT(":vcnt=");
		DBG_PRINT(ManchesterBits.valcount - 1, DEC);
		DBG_PRINT(":bfin:");
	#endif

	if (i == maxMsgSize && ManchesterBits.valcount > minbitlen / 2)
	{
		// We are at end of buffer but have half or more of the minbitlen, we need to catch some more data
		#ifdef DEBUGDECODE
			DBG_PRINT(":mcDet:");
		#endif

		pdec->mcDetected = true; // This will reset the message buffer in the processMessage method and preserve it till then
		pdec->state = mcdecoding; // Try to prevent other processing
		return false; // Prevents serial output of data we already have in the buffer
	}
	else if (i == maxMsgSize)
	{
		// We are at end of buffer, but we haven't much mcdata 
		pdec->mcDetected = false;
		pdec->m_truncated = false;

		// Return false will allow mu processing
	}

	return (ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed, then return true, otherwise false
													//SDC_PRINT(" ES MC ");
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
		DBG_PRINTLN(pdec->mstart, DEC);
	#endif

	if (pdec->patternLen < 4)	return false;

	int tstclock = -1;

	uint8_t pos_cnt = 0;
	uint8_t neg_cnt = 0;
	int equal_cnt = 0;
	const uint8_t minHistocnt = round(pdec->messageLen*0.04);
	//     3     1    0     2
	uint8_t sortedPattern[maxNumPattern]; // 1300,-1300,-734,..800
	uint8_t p = 0;

	for (uint8_t i = 0; i < pdec->patternLen; i++)
	{
		if (pdec->histo[i] < minHistocnt) continue;		// Skip this pattern, due to less occurence in our message

		#if DEBUGDETECT >= 1
			DBG_PRINT('p');
		#endif

		uint8_t ptmp = p;

		while (p != 0 && pdec->pattern[i] < pdec->pattern[sortedPattern[p - 1]])
		{
			sortedPattern[p] = sortedPattern[p - 1];
			p--;
		}

		#if DEBUGDETECT >= 1
			DBG_PRINT('='); DBG_PRINT(i, DEC); DBG_PRINT(',');
		#endif

		sortedPattern[p] = i;
		p = ptmp + 1;
	}

	#if DEBUGDETECT >= 3
		DBG_PRINT("Sorted:");
		for (uint8_t i = 0; i < p; i++)
		{
			DBG_PRINT(sortedPattern[i]); DBG_PRINT(',');
		}
		DBG_PRINT(';');
	#endif

	for (uint8_t i = 0; i < p; i++)
	{
		if (pdec->pattern[sortedPattern[i]] <= 0) continue;

		#if DEBUGDETECT >= 2
			DBG_PRINT("CLK="); DBG_PRINT(sortedPattern[i]); DBG_PRINT(':');
		#endif

		longlow = -1;
		longhigh = -1;
		shortlow = -1;
		shorthigh = -1;
		pos_cnt = 0;
		neg_cnt = 0;
		tstclock = -1;
		equal_cnt = 0;

		const int clockpulse = pdec->pattern[sortedPattern[i]]; // double clock!
		for (uint8_t x = 0; x < p; x++)
		{
			#if DEBUGDETECT >= 1
				DBG_PRINT(sortedPattern[x]);
			#endif

			const int aktpulse = pdec->pattern[sortedPattern[x]];
			bool pshort = false;
			bool plong = false;

			if (pdec->inTol(clockpulse / 2, abs(aktpulse), clockpulse*0.2))
				pshort = true;
			else if (pdec->inTol(clockpulse, abs(aktpulse), clockpulse*0.40))
				plong = true;

			#if DEBUGDETECT >= 3
				DBG_PRINT("^=(PS="); DBG_PRINT(pshort); DBG_PRINT(';');
				DBG_PRINT("PL="); DBG_PRINT(plong); DBG_PRINT(";)");
			#endif

			#if DEBUGDETECT >= 1
				DBG_PRINT(',');
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

				int8_t sequence_even[4] = { -1,-1,-1,-1 };
				int8_t sequence_odd[4] = { -1,-1,-1,-1 };

				int z = 0;
				while (z < pdec->messageLen)
				{
					const uint8_t mpz = pdec->message[z]; // Store pattern for further processing

					if (((isLong(mpz) == false) && (isShort(mpz) == false)) || (z == (pdec->messageLen - 1)))
					{
						#if DEBUGDETECT >= 1
							DBG_PRINT(z); DBG_PRINT('=')DBG_PRINT(mpz); DBG_PRINT(';')

							DBG_PRINT("Long"); DBG_PRINT(isLong(mpz)); DBG_PRINT(';');
							DBG_PRINT("Short"); DBG_PRINT(isShort(mpz)); DBG_PRINTLN(';');
						#endif

						if ((z - pdec->mstart) > minbitlen)  // Todo: Hier wird auf minbitlen geprueft. Die Differenz zwischen mstart und mend sind aber Pulse und keine bits
						{
							pdec->mend = z;
							pdec->calcHisto(pdec->mstart, pdec->mend);
							equal_cnt = pdec->histo[shorthigh] + pdec->histo[longhigh] - pdec->histo[shortlow] - pdec->histo[longlow];

							#if DEBUGDETECT >= 1
								DBG_PRINT("equalcnt: pos "); DBG_PRINT(pdec->mstart); DBG_PRINT(" to ") DBG_PRINT(pdec->mend); DBG_PRINT(" count=");  DBG_PRINT(equal_cnt); DBG_PRINT(" ");
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

							bool break_flag = false;
							for (uint8_t a = 0; a < 4 && break_flag == false; a++)
							{
								#ifdef DEBUGDETECT  //>=0
									char str[20];
									sprintf(str, " seq_even[%d]=%d", a, sequence_even[a]);
									DBG_PRINT(str);
									sprintf(str, " seq_odd[%d]=%d", a, sequence_odd[a]);
									DBG_PRINT(str);
									#if DEBUGDETECT == 0
										DBG_PRINTLN(" ");
									#endif
								#endif

								if ((sequence_even[a] - sequence_odd[a] != 0) && (sequence_odd[a] == -1 || sequence_even[a] == -1))
								{
									break_flag = true;
								}
							}

							if (break_flag == false) {
								break_flag = true; // Must be set false during loops
								uint8_t seq_a = (shorthigh * 10) + longlow;
								uint8_t seq_b = (longhigh * 10) + shortlow;
								if (seq_a < 10) seq_a += 100;
								if (seq_b < 10) seq_b += 100;

								for (uint8_t a = 0; a < 4 && break_flag == true; a++)  // check longlow longhigh and vice versa
								{
									if (sequence_even[a] == seq_a || sequence_odd[a] == seq_a)
									{
										for (uint8_t a2 = 0; a2 < 4 && break_flag == true; a2++)
										{
											if (sequence_even[a2] == seq_b || sequence_odd[a2] == seq_b)
											{
												break_flag = false;
											}
										}
									}
								}

								#if DEBUGDETECT >= 1
									if (break_flag == true)
										DBG_PRINT("  sequence dual long match failed ");
								#endif
							}
							else {
								#if DEBUGDETECT >= 1
									DBG_PRINT("  basic sequence not passed ");
								#endif
							}

							if (break_flag == true) {
								// Check if we can start a new calulation at a later position
								if (pdec->messageLen - z > minbitlen)
								{
									// clear sequence buffers
									for (uint8_t a = 0; a < 4; a++)
									{
										sequence_even[a] = -1;
										sequence_odd[a] = -1;
									}
									z++; // Increase z counter to start new check
									continue;
								}
								break;
							}

							#if DEBUGDETECT >= 1
								DBG_PRINT("  all check passed");
							#endif

							tstclock = tstclock / 6;

							#if DEBUGDETECT >= 1
								DBG_PRINT("  tstclock: "); DBG_PRINT(tstclock, DEC);
							#endif

							clock = tstclock;

							#if DEBUGDETECT >= 1
								char str[20];
								sprintf(str, " MC %s:%d", "LL", longlow);
								DBG_PRINT(str);
								sprintf(str, " MC %s:%d", "LH", longhigh);
								DBG_PRINT(str);
								sprintf(str, " MC %s:%d", "SL", shortlow);
								DBG_PRINT(str);
								sprintf(str, " MC %s:%d", "SH", shorthigh);
								DBG_PRINT(str);

								DBG_PRINTLN("");
							#endif

							// TOdo: Bei FFFF passt diese Pruefung nicht.

							#if DEBUGDETECT >= 1
								DBG_PRINTLN("  -- MC found -- ");
							#endif

							return true;
						}
						else {
							mc_start_found = false;
							mc_sync = false;
							for (uint8_t a = 0; a < 4; a++)
							{
								sequence_even[a] = sequence_odd[a] = -1;
							}
						}
					}
					else {
						if (mc_start_found == false)
						{
							pdec->mstart = z;
							mc_start_found = true;
						}

						int8_t seq_found = -1;
						uint8_t seq = (mpz * 10) + pdec->message[z + 1];

						if (seq < 10) seq += 100;

						int8_t *seqptr;
						if (z % 2 == 0)  //Every even value
							seqptr = sequence_even;
						else
							seqptr = sequence_odd;

						for (uint8_t a = 0; a < 4 && seq_found == -1; a++)
						{
							if (seqptr[a] == seq)
							{
								seq_found = a;
							}
							else if (seqptr[a] == -1)
							{
								//DBG_PRINT(" +seq[:"); DBG_PRINT(seq); DBG_PRINTLN("]");

								seqptr[a] = seq;
								seq_found = a;
							}
						}
					}
					z++;
				}
			}
		}
	}
	return false;
}
