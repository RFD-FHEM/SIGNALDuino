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

#ifdef DEBUGGLEICH
//Helper function to check buffer for bad data
bool SignalDetectorClass::checkMBuffer()
{
	for (uint8_t i = 0; i < messageLen-1; i++)
	{
		
		if ( (pattern[message[i]] ^ pattern[message[i+1]]) > 0)
		//if (message[i] == message[i+1])
		{
			return false;
		}
	}
	return true;
}
#endif

void SignalDetectorClass::bufferMove(const uint8_t start)
{
	m_truncated = false;

	if (start == 0 || messageLen == 0) {
		//MSG_PRINTLN(F("buffMove start||msgLen=0"));
		//printOut();
		return;
	}
	if (start > messageLen - 1) {
		if (start > messageLen) {
			DBG_PRINT(F(" buffMove overflow!"));
			DBG_PRINT(F(" start=")); MSG_PRINTLN(start);
			//printOut();
		}
		reset();
	}
	else if (message.moveLeft(start))
	{
		m_truncated = true; 
		
		//messageLen = messageLen - start;
		messageLen = message.valcount;
		
		if (messageLen > 0) {
			//last = &pattern[message[messageLen - 1]]; //Eventuell wird last auf einen nicht mehr vorhandenen Wert gesetzt, da der Puffer komplett gelöscht wurde
		} else {
			last = NULL;
		}
		
	} else {
		DBG_PRINT(" moveErr "); DBG_PRINTLN(start)
		//printOut();
	}

#if DEBUGGLEICH > 1
	//if (bMoveFlag == 1 || !checkMBuffer())
	if (!checkMBuffer())
	{
		bMoveFlag = 0;
		MSG_PRINT(F("after buffermove ->"));
		MSG_PRINT(F(" start=")); MSG_PRINT(start);
		MSG_PRINT(F(" bytecnt=")); MSG_PRINT(message.bytecount);
		MSG_PRINT(F(" valcnt=")); MSG_PRINT(message.valcount);
		MSG_PRINT(F(" bcnt=")); MSG_PRINT(message.bcnt);
		//MSG_PRINT(F(" mTrunc=")); MSG_PRINT(m_truncated);
		MSG_PRINTLN(F(" wrong Data in Buffer"));
		printOut();
	}
#endif
}


inline void SignalDetectorClass::addData(const uint8_t value)
{
	//message += value;
	/*if (message.valcount >= 254)
	{
		DBG_PRINTLN(""); 	DBG_PRINT(__FUNCTION__); DBG_PRINT(" msglen: "); DBG_PRINT(messageLen);
	}*/
	//uint8_t nb;

#if DEBUGGLEICH > 1
	bMoveFlag = 0;
	//if (messageLen > 0 && value == message[messageLen-1]) { // && state > 0) {
	if (messageLen > 0 && (pattern[message[messageLen-1]] ^ pattern[value]) > 0) {
		bMoveFlag = 1;
		MSG_PRINT(F("addDataGleich ->"));
		MSG_PRINT(F(" val=")); MSG_PRINT(value);
		printOut();
	}
#endif
	if (message.addValue(value))
	{
		messageLen++;
#if DEBUGGLEICH > 1
		if (!checkMBuffer())
		{
				bMoveFlag = 1;
				MSG_PRINT(F("addData->"));
				MSG_PRINT(F("val=")); MSG_PRINTLN(value);
				printOut();
		}
#endif
	}
	else {
		MSG_PRINT(F("val=")); MSG_PRINT(value);
		MSG_PRINT(F(" bytecnt=")); MSG_PRINT(message.bytecount); 
		MSG_PRINTLN(F(" addData overflow!!"));
		printOut();
	}
	firstLast = *first;   // zum debuggen
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
	valid &= (*first > -maxPulse);  // if low maxPulse detected, start processMessage()

	//if (abs(*first) < 90) {
	//	DBG_PRINT("first<90 ");DBG_PRINTLN(*first);
	//}
	
	int8_t fidx = 0;
	
	if (!valid) {			// Nachrichtenende erkannt -> alles bis zum Nachrichtenende wird ausgegeben
		
		for (uint8_t n = 10; n > 0; --n) {
			// Try output
			processMessage(0);
			if (messageLen == 0) {
				last = NULL;
				break;
			}
		}
		
		if (messageLen > 0) reset();	// nur zur Sicherheit, duerfte eigentlich nie vorkommen
		
		MsMoveCount = 0;
		MuMoveCount = 0;
		addPattern();
	}
	else {  					// valid
		if (messageLen >= maxMsgSize) {
			processMessage(1);   // message Puffer voll aber kein message Ende
			calcHisto();
		}
		else if (messageLen == minMessageLen) {
			state = detecting;  // Set state to detecting, because we have more than minMessageLen data gathered, so this is no noise
			if (_rssiCallback != NULL) 
				rssiValue = _rssiCallback();
		}

		fidx = findpatt(*first);
		if (fidx >= 0) {
			// Upd pattern
			updPattern(fidx);
		}
		else {
		
			// Add pattern
			if (patternLen == maxNumPattern)
			{
				//calcHisto();
				//bool gr2Flag = false;
				if (histo[pattern_pos] > 2 && state > 0)
				{
					//gr2Flag = true;
					//DBG_PRINTLN(F("addP_histop>2"));  // pattern buffer full after proccessMessage
					//printOut();
					bool saveMsgSuccess = false;
					for (uint8_t n = 10; n > 0; --n) {
						if (printMsgSuccess) {
							saveMsgSuccess = true;
							printMsgSuccess = false;
						}
						processMessage(2);
						if (messageLen < minMessageLen || printMsgSuccess == false) {
							break;
						}
					}
					MsMoveCount = 0;
					MuMoveCount = 0;
					printMsgSuccess = saveMsgSuccess;
					//printOut();
					calcHisto();

				}
				if (messageLen > 0 && histo[pattern_pos] > 0) {
					for (uint8_t i = messageLen; i > 0; --i)
					{
						if (message[i-1] == pattern_pos) // Finde den letzten Verweis im Array auf den Index der gleich ueberschrieben wird
						{
							/*  // Test ob es was bringen wuerde, wenn der letzte Wert in message[] nicht ueberschrieben wuerde -> keine Vorteile erkennbar.
							if (i == messageLen - 1) {
								printOut();
							} */
							bufferMove(i);  // i um eins erhoehen, damit zukuenftigen Berechnungen darauf aufbauen koennen
							calcHisto();
							break;
						}
						if (i == 1 && messageLen > 253)
						{
							//DBG_PRINT(millis());
							
							DBG_PRINTLN(F(" mb nm b proc "));  // message buffer not moved before  proccessMessage
						}
					}
					/*if (gr2Flag) {
						DBG_PRINTLN(F("addP_histop>2 nach buffmove"));  // pattern buffer full after proccessMessage
						printOut();
					}*/
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
	}

	// Add data to buffer
	addData(fidx);
	histo[fidx]++;


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
	if (messageLen > 0) {
		last = &pattern[message[messageLen - 1]];
		lastPulse = *first;  // zum debuggen
	} else {
		last = NULL;
	}
	*first = *pulse;
	
	doDetect();
	return success;
}


bool SignalDetectorClass::compress_pattern()
{
	bool ret = false;
	calcHisto();
	/*if (!checkMBuffer())
	{
		MSG_PRINTLN(F("befCompP->wrongDat inBuf"));
		printOut();
	}*/
	
	for (uint8_t idx = 0; idx<patternLen-1; idx++)
	{
		if (histo[idx] == 0)
			continue;

		for (uint8_t idx2 = idx + 1; idx2<patternLen; idx2++)
		{
			if (histo[idx2] == 0 || (pattern[idx] ^ pattern[idx2]) >> 15)
				continue;
			const int16_t tol = int(((abs(pattern[idx2])*tolFact) + (abs(pattern[idx])*tolFact)) / 2);
			if (inTol(pattern[idx2], pattern[idx], tol))  // Pattern are very equal, so we can combine them
			{
#if DEBUGDoDETECT >3
				DBG_PRINT(" compr:idx= "); DBG_PRINT(idx); DBG_PRINT("/"); DBG_PRINT(pattern[idx]);
				DBG_PRINT(" idx2= "); DBG_PRINT(idx2); DBG_PRINT("/"); DBG_PRINT(pattern[idx2]);
				DBG_PRINT(" tol="); DBG_PRINT(tol);
#endif
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

#if DEBUGDETECT>3
				DBG_PRINT("compr: "); DBG_PRINT(idx2); DBG_PRINT("->"); DBG_PRINT(idx); DBG_PRINT(";");
				DBG_PRINT(histo[idx2]); DBG_PRINT("*"); DBG_PRINT(pattern[idx2]);
				DBG_PRINT("->");
				DBG_PRINT(histo[idx]); DBG_PRINT("*"); DBG_PRINT(pattern[idx]);
				
#endif // DEBUGDETECT


				int  sum = histo[idx] + histo[idx2];
				//int lPatternIdx = pattern[idx];
				pattern[idx] = ((long(pattern[idx]) * histo[idx]) + (long(pattern[idx2]) * histo[idx2])) / sum;
				//if (abs(pattern[idx]) < 90) {
				//	DBG_PRINT("upfirst<90 ");DBG_PRINT(idx);DBG_PRINT(" ");DBG_PRINT(idx2);DBG_PRINT(" ");DBG_PRINTLN(lPatternIdx);
				//	printOut();
				//}
				histo[idx] += histo[idx2];
				pattern[idx2] = histo[idx2]= 0;
				ret = true;

#if DEBUGDETECT >3
				DBG_PRINT(" idx:"); DBG_PRINT(pattern[idx]);
				DBG_PRINT(" idx2:"); DBG_PRINT(pattern[idx2]);
				DBG_PRINTLN(";");
#endif // DEBUGDETECT

			}
		}
	}
	
#if DEBUGDoDETECT >3
	if (ret==true)
		DBG_PRINTLN(" ");
#endif

#ifdef DEBUGGLEICH
	if (!checkMBuffer())
	{
		MSG_PRINTLN(F("aftCompP->wrongD InBuf"));
		printOut();
	}
#endif
	return ret;
}

void SignalDetectorClass::processMessage(const uint8_t p_valid)
{
	//yield();

	if (p_valid > 0) {
		m_truncated = true;
	} else {			// nicht valid (message Ende) -> reset wenn kleiner minMessageLen
		m_truncated = false;	
	}
	/*DBG_PRINT("msgRec:");
	DBG_PRINT(*first);
	DBG_PRINT(" ");
	DBG_PRINTLN(*last);*/
	
	if (mcDetected == true || messageLen >= minMessageLen) {
		success = false;
		m_overflow = (messageLen == maxMsgSize) ? true : false;

#if DEBUGDETECT >= 1
		DBG_PRINTLN("msgRec:");
#endif
		//if (MsMoveCount == 0 && MuMoveCount == 0) {	// bei Wiederholungen wird kein compress_pattern benötigt
			compress_pattern();
		//}

		state = searching;
		if (MsMoveCount > 0) {
			mstart = 0;
			while (mstart < 10) {
				if (message[mstart + 1] == sync && message[mstart] == clock) {	// nach sync und clock von der 1.Nachricht suchen
					state = syncfound;	//	sync und clock gefunden -> es muss nicht erneut nach clock und sync gesucht werden
					break;
				}
				mstart++;
			}
			if (state == searching) {	// die sync und clock Werte stimmen nicht mit der 1.Nachricht ueberein
				MsMoveCount = 0;
				calcHisto();
			}
		} else if (MuMoveCount > 0) {
			calcHisto();
		}
		
		if (state == searching) {
			getClock();
		}
		if (state == clockfound && MSenabled && mcRepeat == false) getSync();  // wenn MC Wiederholungen ausgegeben werden, wird die MS Decodierung uebersprungen

#if DEBUGDECODE >1
		DBG_PRINT("msgRec state="); DBG_PRINTLN(state)
#endif
		
#if DEBUGDETECT >= 1
		printOut();
#endif
		if (state == syncfound && messageLen >= minMessageLen)// Messages mit clock / Sync Verhaeltnis pruefen
		{
#if DEBUGDECODE >1
			MSG_PRINTLN(" MScheck: ");

			//printOut();
#endif	
			// Setup of some protocol identifiers, should be retrieved via fhem in future

			mend = mstart + 2;
			bool m_endfound = false;

			while (mend < (mstart + 10))	// testen ob innerhalb 10 Zeichen nach dem sync ein weiterer sync folgt, falls ja diesen als neuen mstart verwenden 
			{
				if (message[mend + 1] == sync && message[mend] == clock) {
#ifdef DEBUGDECODE
					DBG_PRINT(F("MStart:")); DBG_PRINT(mstart);
					DBG_PRINT(F(" new MStart:")); DBG_PRINTLN(mend);
#endif
					mstart = mend;
					mend += 2;
					break;
				}
				mend++;
			}
			
			while (mend < messageLen - 1)
			{
				if (message[mend + 1] == sync && message[mend] == clock) {
					mend -= 1;					// Previus signal is last from message
					m_endfound = true;
					break;
				}
				mend += 2;
			}

			if (mend > messageLen || !m_endfound) mend = messageLen - 1;  // Reduce mend if we are behind messageLen
			
			calcHisto(mstart, mend);	// Recalc histogram due to shortened message
			//if (message[messageLen-1] == 7 || mstart > 10) {
			//if (mstart > 30) {
			//	DBG_PRINT("MStart>30 valid="); DBG_PRINTLN(p_valid,DEC);
			//	printOut();
			//}

#ifdef DEBUGDECODE
			DBG_PRINT("Index: ");
			DBG_PRINT(" MStart: "); DBG_PRINT(mstart);
			DBG_PRINT(" SYNC: "); DBG_PRINT(sync);
			DBG_PRINT(", CP: "); DBG_PRINT(clock);
			DBG_PRINT(" - MEnd: "); DBG_PRINT(mend);
			DBG_PRINT(F(" msglen=")); DBG_PRINTLN(messageLen);
			
#endif // DEBUGDECODE
		    if ((m_endfound && (mend - mstart) >= minMessageLen) || (!m_endfound && (messageLen - mstart) >= minMessageLen))
		    {
			if (m_endfound || (!m_endfound && messageLen < maxMsgSize && p_valid != 2))  // nicht ausgeben, wenn kein Ende gefunden und patternpuffer overflow
			{
#if DEBUGDECODE >1
				MSG_PRINTLN(F("Filter Match: "));;
#endif


				//preamble = "";
				//postamble = "";
			    if (MsMoveCount < MsMoveCountmax) {
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
						MSG_WRITE(patternIdx);
						MSG_WRITE(patternLow);
						MSG_WRITE(highByte(patternInt) | B10000000);
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
						(message.getByte(mstart/2,&n) & 15) | 128;    // high nibble = 8 als Kennzeichen für ungeraden mstart
						MSG_WRITE(n);
 						mstart += 2;
					}
					for (uint8_t i = mstart; i <= mend; i=i+2) {					
						message.getByte(i/2,&n);
						MSG_WRITE(n);
					}

					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("C");  MSG_PRINT(clock, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("S");  MSG_PRINT(sync, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					#ifdef CMP_CC1101
					  MSG_PRINT("R");  MSG_PRINT(rssiValue, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					#endif				
			        }
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
					#ifdef CMP_CC1101
					  MSG_PRINT("R=");  MSG_PRINT(rssiValue); MSG_PRINT(SERIAL_DELIMITER);     // Signal Level (RSSI)
					#endif					
				}
				
				if (m_overflow) {
					MSG_PRINT("O");  MSG_PRINT(SERIAL_DELIMITER);
				}
				if (MdebEnabled) {
					if (p_valid == 0) {					// Nachrichtenende erkannt
						MSG_PRINT("e"); MSG_PRINT(SERIAL_DELIMITER);
					} else if (p_valid == 2) {				// Patternpuffer overflow
						MSG_PRINT("p"); MSG_PRINT(SERIAL_DELIMITER);
					}
				}
			    }
				m_truncated = false;
				
				if ((m_endfound || MsMoveCount > 0)) {
				    if (MdebEnabled && MsMoveCount < MsMoveCountmax) {
					MSG_PRINT("m"); MSG_PRINT(MsMoveCount); MSG_PRINT(SERIAL_DELIMITER);
				    }
				    MsMoveCount++;
				}
				if (m_endfound) {
				   if ((p_valid > 0 || (p_valid == 0 && (messageLen - mend) >= minMessageLen)) && MsMoveCount > 0) {  // wenn Nachrichtenende erkannt wurde, dann muss der Rest laenger als minMessageLen sein
					//MSG_PRINT(F("MS move. messageLen ")); MSG_PRINT(messageLen); MSG_PRINT(" "); MSG_PRINTLN(MsMoveCount)
					bufferMove(mend+1);
					//MSG_PRINT(F("MS move. messageLen ")); MSG_PRINTLN(messageLen);
					mstart = 0;
				   }
				}
			    if (MsMoveCount <= MsMoveCountmax) {
				MSG_PRINT(MSG_END);
				MSG_PRINT("\n");
			    }
			    
			    printMsgSuccess = true;
			    //success = true;

			}
			else if (m_endfound == false && mstart > 0 && mend + 1 >= maxMsgSize) // Start found, but no end. We remove everything bevore start and hope to find the end later
			{
				if (p_valid == 0) {   // wenn ein Nachrichtenende erkannt wurde, kann alles geloescht werden
					m_truncated = false;
#if DEBUGDECODE >1
					DBG_PRINT(F(" MSmoveMsg,noEnd->reset "));
#endif
				}
				else {
#if DEBUGDECODE >1
					DBG_PRINT(F(" MSmoveMsg,noEnd "));
#endif
					bufferMove(mstart);
					mstart = 0;
					//m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
				}
			} 
			else if (m_endfound && mend < maxMsgSize) {  // Start and end found, but end is not at end of buffer, so we remove only what was checked
						// mir ist nicht klar in welchen Faellen dies aufgerufen wird
#if DEBUGDECODE >1
				DBG_PRINTLN(F(" MSmoveMsg "));
#endif
				bufferMove(mend+1);
				mstart = 0;
				//m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
				//success = true;	// don't process other message types
			}
			else {
#ifdef DEBUGDECODE
				MSG_PRINTLN(F(" Buffer overflow, flushing message array"));
#endif
				//MSG_PRINT(MSG_START);
				//MSG_PRINT("Buffer overflow while processing signal");
				//MSG_PRINT(MSG_END);
				if (p_valid != 2) {
					reset(); // Our Messagebuffer is not big enough, no chance to get complete Message
				}
				//success = true;	// don't process other message types
			}
			success = true;
		    }
		    else {		  // m_endfound && (mend - mstart) < minMessageLen)  -> weiter mit MU message verarbeitung
			success == false;
		    }
		}
		if (success == false && (MUenabled || MCenabled)) {

#if DEBUGDECODE >1
			DBG_PRINT(" check:");

			//printOut();
#endif	
// Message has a clock puls, but no sync. Try to decode this

			//preamble = "";
			//postamble = "";

			if (MCenabled && state != syncfound)
			{
				//DBG_PRINT(" mc: ");

				static ManchesterpatternDecoder mcdecoder(this);			// Init Manchester Decoder class

				if (mcDetected == false)
				{
					mcdecoder.reset();
					mcdecoder.setMinBitLen(mcMinBitLen);								
				}
#if DEBUGDETECT>3
				MSG_PRINT("vcnt: "); MSG_PRINTLN(mcdecoder.ManchesterBits.valcount);
#endif
#if DEBUGDECODE > 2
				DBG_PRINTLN("");
#endif
				if ((mcDetected || mcdecoder.isManchester()) && mcdecoder.doDecode())	// Check if valid manchester pattern and try to decode
				{
#if MCDEBUGDECODE > 1
					MSG_PRINT(MSG_START);
					MSG_PRINT("DMC");
					MSG_PRINT(SERIAL_DELIMITER);

					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (idx > 4 && histo[idx] == 0) continue;
						MSG_PRINT('P'); MSG_PRINT(idx); MSG_PRINT('='); MSG_PRINT(pattern[idx]); MSG_PRINT(SERIAL_DELIMITER);
					}
					MSG_PRINT("D=");


					//mend = min(mend, messageLen); // Workaround if mend=255
					for (uint8_t i = 0; i < messageLen; ++i)
					{
						if (i == mstart || i == mend) {
							MSG_PRINT("_");
						}
						MSG_PRINT(message[i]);
					}
					MSG_PRINT(SERIAL_DELIMITER);

					if (m_overflow) {
						MSG_PRINT("O"); MSG_PRINT(SERIAL_DELIMITER);
					}
					if (MdebEnabled) {
						if (p_valid == 0) {					// Nachrichtenende erkannt
							MSG_PRINT("e"); MSG_PRINT(SERIAL_DELIMITER);
						} else if (p_valid == 2) {				// Patternpuffer overflow
							MSG_PRINT("p"); MSG_PRINT(SERIAL_DELIMITER);
						}
						if (mcValid == false) {
							MSG_PRINT("i"); MSG_PRINT(SERIAL_DELIMITER);
						}
					}
					MSG_PRINTLN(MSG_END);
#endif
					MSG_PRINT(MSG_START);
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
					#ifdef CMP_CC1101
					  MSG_PRINT("R=");  MSG_PRINT(rssiValue); MSG_PRINT(SERIAL_DELIMITER);     // Signal Level (RSSI)
					#endif
					if (MdebEnabled) {
						if (mcValid == false) {
							MSG_PRINT("i"); MSG_PRINT(SERIAL_DELIMITER);
						}
						if (mcdecoder.mc_sync_pos > 0) {
							MSG_PRINT("s"); MSG_PRINT(mcdecoder.mc_sync_pos); MSG_PRINT(SERIAL_DELIMITER);
							MSG_PRINT("b"); MSG_PRINT(mstart); MSG_PRINT(SERIAL_DELIMITER);
#ifdef MCDEBUGDECODE
							uint8_t ims = mstart;
							if (mstart > 3) {
								ims = 3;
							} else {
								ims = mstart;
							}
							MSG_PRINT("s");
							uint8_t ic;
							uint8_t ip;
							for (ic = mstart-ims; ic < mstart+30; ic++) {
								if (ic == mstart) {
									MSG_PRINT("_");
								}
								ip = message[ic];
								if (ip == mcdecoder.longlow) {
									MSG_PRINT("l");
								} else if (ip == mcdecoder.longhigh) {
									MSG_PRINT("L");
								} else if (ip == mcdecoder.shortlow) {
									MSG_PRINT("s");
								} else if (ip == mcdecoder.shorthigh) {
									MSG_PRINT("S");
								} else {
									MSG_PRINT(ip);
								}
							}
							MSG_PRINT(SERIAL_DELIMITER);
#endif
						}
					}
					if (m_overflow) {
						MSG_PRINT("O"); MSG_PRINT(SERIAL_DELIMITER);
					}
					
					mcDetected = false;
					success = true;
					printMsgSuccess = true;
					
					if (p_valid > 0 || (p_valid == 0 && (messageLen - mend) >= minMessageLen)) {  // wenn Nachrichtenende erkannt wurde, dann muss der Rest laenger als minMessageLen sein
						bufferMove(mend-1);
						mstart = 0;
						mcRepeat = true;
						if (MdebEnabled) {
							MSG_PRINT("w"); MSG_PRINT(SERIAL_DELIMITER);
						}
					}
					
					MSG_PRINT(MSG_END);
					MSG_PRINT("\n");
				}
				else if (mcDetected == true && m_truncated == true) {

					success = true;   // Prevents MU Processing
				}

			}
			if (MUenabled && (state == clockfound || state == syncfound) && success == false && messageLen >= minMessageLen) {

#if DEBUGDECODE > 1
				DBG_PRINT(" MU found: ");
#endif // DEBUGDECODE
				bool m_endfound = false;
				
				bool isMuRepeat = false;
				if (MuSplitThresh > 0) {
					if (mstart > 0) {	// wurde in isManchester der mstart veraendert und calcHisto ausgefuehrt?
						calcHisto();
						mstart = 0;
					}
					isMuRepeat = isMuMessageRepeat();
				}
#ifdef DEBUGMUREPEAT
				DBG_PRINTLN(isMuRepeat, DEC);
#endif
				if (isMuRepeat) {
					mend = minMessageLen;
					if (mend <= messageLen) {
						for (uint8_t i = mend; i < messageLen-1; ++i) {
							if (abs(pattern[message[i]]) >= MuSplitThresh) {
								mend = i;
								m_endfound = true;
								//MSG_PRINT(F("MUend found="));
								//MSG_PRINTLN(mend);
								//printOut();
								break;
							}
						}
					}
				
					//printOut();
				}

				if (!m_endfound) {
					mend = messageLen;
					if (mstart > 0) {	// wurde in isManchester der mstart veraendert und calcHisto ausgefuehrt?
						calcHisto();
					}
				}
				else {
					calcHisto(0, mend);	// Recalc histogram due to shortened message
				}
				
				if (MredEnabled) {
					int patternInt;
					uint8_t patternLow;
					uint8_t patternIdx;
					
					MSG_PRINT(MSG_START);  MSG_PRINT("Mu");  MSG_PRINT(SERIAL_DELIMITER);
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
						MSG_WRITE(patternIdx);
						MSG_WRITE(patternLow);
						MSG_WRITE(highByte(patternInt) | B10000000);
						MSG_PRINT(SERIAL_DELIMITER);
					}

					uint8_t n;

					if ((mend & 1) == 1) {   // ungerade
						MSG_PRINT("D");
					}
					else {
						MSG_PRINT("d");
					}

					for (uint8_t i = 0; i < mend; i=i+2) {					
						message.getByte(i/2,&n);
						MSG_WRITE(n);
					}

					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("C");  MSG_PRINT(clock, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					#ifdef CMP_CC1101
					  MSG_PRINT("R");  MSG_PRINT(rssiValue, HEX);  MSG_PRINT(SERIAL_DELIMITER);
					#endif				
				}
				else {
				
					MSG_PRINT(MSG_START); MSG_PRINT("MU");  MSG_PRINT(SERIAL_DELIMITER);

					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (pattern[idx] == 0 || histo[idx] == 0) continue; 
						MSG_PRINT("P"); MSG_PRINT(idx); MSG_PRINT("="); MSG_PRINT(pattern[idx]); MSG_PRINT(SERIAL_DELIMITER);  // Patternidx=Value
					}
					MSG_PRINT("D=");
					for (uint8_t i = 0; i < mend; ++i)
					{
						MSG_PRINT(message[i]);
					}
					//String postamble;
					MSG_PRINT(SERIAL_DELIMITER);
					MSG_PRINT("CP="); MSG_PRINT(clock);     MSG_PRINT(SERIAL_DELIMITER);    // ClockPulse, (not valid for manchester)
					#ifdef CMP_CC1101
					  MSG_PRINT("R=");  MSG_PRINT(rssiValue); MSG_PRINT(SERIAL_DELIMITER);     // Signal Level (RSSI)
				        #endif
				}
				
				m_truncated = false;
				printMsgSuccess = true;
				success = true;
				
				if (m_overflow) {
					MSG_PRINT("O");  MSG_PRINT(SERIAL_DELIMITER);
				}
				if (MdebEnabled) {
					if (p_valid == 0) {					// Nachrichtenende erkannt
						MSG_PRINT("e"); MSG_PRINT(SERIAL_DELIMITER);
					} else if (p_valid == 2) {				// Patternpuffer overflow
						MSG_PRINT("p"); MSG_PRINT(SERIAL_DELIMITER);
					}
					if (mcValid == false) {
						MSG_PRINT("i"); MSG_PRINT(SERIAL_DELIMITER);
					}
				}
				
				if (m_endfound) {
					if (MdebEnabled) {
						MSG_PRINT("w"); MSG_PRINT(MuMoveCount); MSG_PRINT(SERIAL_DELIMITER);
					}
					MuMoveCount++;
					if (p_valid > 0 || (p_valid == 0 && (messageLen - mend) >= minMessageLen)) {  // wenn Nachrichtenende erkannt wurde, dann muss der Rest laenger als minMessageLen sein
						//MSG_PRINT(F("MS move. messageLen ")); MSG_PRINT(messageLen); MSG_PRINT(" "); MSG_PRINTLN(MsMoveCount)
						bufferMove(mend);
						//MSG_PRINT(F("MS move. messageLen ")); MSG_PRINTLN(messageLen);
						mstart = 0;
					}
				}
				
				MSG_PRINT(MSG_END);  MSG_PRINT("\n");
				//if (m_endfound) {
				//  MSG_PRINT(F("MUend="));
				//  MSG_PRINTLN(mend);
				//}
			}

		}
		
		if (success == false) 
		{
			m_truncated = false;     //  -> reset
#if DEBUGDETECT >= 1
			DBG_PRINTLN("nothing to to");
#endif		
		}
		
		if (messageLen >= maxMsgSize && state == searching) {
			DBG_PRINTLN(F("mOverflow&state=0!"));
#ifdef DEBUGDECODE
			printOut();
#endif
			m_truncated = false;     //  -> reset
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
	tolFact = 0.25;
	mstart = 0;
	m_truncated = false;
	m_overflow = false;
	mcDetected = false;
	mcValid = true;
	mcRepeat = false;
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
	//DBG_PRINTLN("");

	DBG_PRINT("Sync: ");
	if (sync > -1) {
		if (sync > -1) {
			DBG_PRINT(pattern[sync]);
			DBG_PRINT(" -> SyncFact: "); DBG_PRINT(pattern[sync] / (float)pattern[clock]);
			DBG_PRINT(",");
		} else 
			DBG_PRINT("NULL");
		
	}
	DBG_PRINT(" Clock: "); 
	if (clock > -1) {
		DBG_PRINT(pattern[clock]);
	} else
		DBG_PRINT("NULL");
	//DBG_PRINT(F(", Tol: ")); DBG_PRINT(tol);
	DBG_PRINT(F(", PattLen: ")); DBG_PRINT(patternLen); DBG_PRINT(" ("); DBG_PRINT(pattern_pos); DBG_PRINT(")");
	DBG_PRINT(F(", Pulse: F/L/FL/LP ")); DBG_PRINT(*first); DBG_PRINT(", "); DBG_PRINT(*last);DBG_PRINT(", "); DBG_PRINT(firstLast);DBG_PRINT(", ");DBG_PRINT(lastPulse);
	DBG_PRINT(F(", mStart: ")); DBG_PRINT(mstart);
	DBG_PRINT(F(", mend: ")); DBG_PRINT(mend);
	DBG_PRINT(F(", valcnt: ")); DBG_PRINT(message.valcount);
	DBG_PRINT(F(", MCD: ")); DBG_PRINT(mcDetected,DEC);
	DBG_PRINT(F(", mtrunc: ")); DBG_PRINT(m_truncated, DEC);
	DBG_PRINT(F(", state: ")); DBG_PRINT(state);

	DBG_PRINTLN(); DBG_PRINT(F("Signal: "));
	uint8_t idx;
	for (idx = 0; idx < messageLen; ++idx) {
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
	DBG_PRINTLN("");
	DBG_PRINTLN("");
	
}

int8_t SignalDetectorClass::findpatt(const int val)
{
	//seq[0] = Laenge  //seq[1] = 1. Eintrag //seq[2] = 2. Eintrag ...
	// Iterate over patterns (1 dimension of array)
	tol = abs(val)*0.2;  //*tolFact
	for (uint8_t idx = 0; idx<patternLen; ++idx)
	{
		if ((val ^ pattern[idx]) >> 15)
			continue;
		if (pattern[idx] != 0 && histo[idx] > 0 && inTol(val, pattern[idx],tol))  // Skip this iteration, if seq[x] <> pattern[idx][x]
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

	if (endpos == 0) endpos = messageLen-1;
	uint8_t bstartpos = startpos/2;       // *4/8;
	uint8_t bendpos = endpos/2;           // *4/8;
	uint8_t bval;
	if (startpos % 2 == 1)  // ungerade
	{
		message.getByte(bstartpos,&bval);
		histo[bval & B00001111]++;
		bstartpos++;
	}
	for (uint8_t i = bstartpos; i <= bendpos; ++i)
	{
		message.getByte(i,&bval);
		histo[bval >> 4]++;
		histo[bval & B00001111]++;
	}
	if (endpos % 2 == 0)  // gerade
	{
		message.getByte(bendpos, &bval);
		histo[bval & B00001111]--;
	}
	
	/*
	// patternLen reduzieren, wenn in der histo[] mindestens die letzten 3 Einträge 0 sind.  Bringt anscheinend keine merklichen Vorteile.
	if (patternLen == maxNumPattern && histo[maxNumPattern-1] == 0) {
		for (uint8_t i = maxNumPattern-2; i > 0; --i)
		{
			if (histo[i] > 0) {
				if (i < 5 && state == 0) {
					MSG_PRINT("chist i=");
					MSG_PRINT(i);
					MSG_PRINT(" ep=");
					MSG_PRINT(endpos);
					patternLen = i + 1;
					pattern_pos = patternLen;
					printOut();
				}
				break;
			}
			
		}
	}*/
}

bool SignalDetectorClass::getClock()
{
	// Durchsuchen aller Musterpulse und prueft ob darin eine clock vorhanden ist
#if DEBUGDETECT > 3
	MSG_PRINTLN("  --  Searching Clock in signal -- ");
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
	// Durchsuchen aller Musterpulse und prueft ob darin ein Sync Faktor enthalten ist. Anschliessend wird verifiziert ob dieser Syncpuls auch im Signal nacheinander uebertragen wurde
	//
#if DEBUGDETECT > 3
	DBG_PRINTLN("  --  Searching Sync  -- ");
#endif

	if (state == clockfound)		// we need a clock to find this type of sync
	{					// clock wurde bereits durch getclock bestimmt
		
		uint8_t syncLenMax = 120; 		      //  wenn in den ersten ca 120 Pulsen kein Sync gefunden wird, dann ist es kein MS Signal
				
		if (syncLenMax > messageLen - minMessageLen)  // der Abstand vom sync zum Ende sollte mindestens minMessageLen sein
		{
			syncLenMax = messageLen - minMessageLen;
		}
		
		for (int8_t p = patternLen - 1; p >= 0; --p)  // Schleife fuer langen Syncpuls
		{
			if (histo[p] == 0)
				continue;
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
				(histo[p] < messageLen*0.08) && (histo[p] > 1)	// histo[p] => 1)
				)
			{
				// if (histo[p] == 1 && messageLen == maxMsgSize)	// ist evtl notwendig, wenn histo[p] => 1
				// continue;
				
				//if ((syncMinFact* (pattern[clock]) <= -1*pattern[p])) {//n>9 => langer Syncpulse (als 10*int16 darstellbar
				// Pruefe ob Sync und Clock valide sein koennen
				//	if (histo[p] > 6) continue;    // Maximal 6 Sync Pulse  Todo: 6 Durch Formel relativ zu messageLen ersetzen

				// Pruefen ob der gefundene Sync auch als message [clock, p] vorkommt
				uint8_t c = 0;
				
				while (c < syncLenMax)
				{
					if (message[c + 1] == p && message[c] == clock) break;
					c++;
				}

				if (c < syncLenMax)
				{
					sync = p;
					state = syncfound;
					mstart = c;
#if DEBUGDECODE >1
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

bool SignalDetectorClass::isMuMessageRepeat()
{
#ifdef DEBUGMUREPEAT
	DBG_PRINT(F("WT: "));
#endif
	for (int8_t p = patternLen - 1; p >= 0; --p)
	{
		if (abs(pattern[p]) < MuSplitThresh || histo[p] == 0) {
			continue;
		}
#ifdef DEBUGMUREPEAT
		DBG_PRINT(F("P"));
		DBG_PRINT(p);
		DBG_PRINT(F(":"));
		DBG_PRINT(histo[p]);
		DBG_PRINT(F(" "));
#endif
		if (histo[p] == 1)
		{
			uint8_t i = 0;
			while (i < 10)				// todo ist 10 ok, oder ist es zu gross oder zu klein?	
			{
				if (message[i] == p) break;	// p ist kein Wiederholungstrenner, da er schon am Anfang vorkommt
				i++;
			}
			if (i >= 10)
			{
				return true;		// p ist ein Wiederholungstrenner
			}
		}
		else if (histo[p] < 8) return true;	// wenn p nicht zu oft vorkommt, dann ist es hoechstwahrscheinlich ein Wiederholungstrenner
	}
	return false;					// kein Wiederholungstrenner gefunden
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
#if MCDEBUGDECODE > 2
	DBG_PRINT("mcrst:");
#endif
	longlow =   -1;
	longhigh =  -1;
	shortlow =  -1;
	shorthigh = -1;
	
	mc_start_found = false;

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

	unsigned char c;
	ManchesterBits.getByte(idx,&c);
	return  c;
}




/** @brief (Decodes the manchester pattern to bits. Returns true on success and false on error )
*
* (Call only after ismanchester returned true)
*/

const bool ManchesterpatternDecoder::doDecode() {

	uint8_t i = 1;
	pdec->m_truncated = false;
	pdec->mstart = 0; // Todo: pruefen ob start aus isManchester uebernommen werden kann
#ifdef DEBUGDECODE
	DBG_PRINT("mlen:");
	DBG_PRINT(pdec->messageLen);
	DBG_PRINT(":mstart: ");
	DBG_PRINT(pdec->mstart);
	DBG_PRINTLN("");

#endif
	uint8_t bit = 0;
	uint8_t pulseid;
	#ifdef DEBUGDECODE
	char value = NULL;
	#endif
	while (i < pdec->messageLen-30)
	{
		pdec->mcValid = true;
		while (i < pdec->messageLen-1)
		{
			// Start vom MC Signal suchen, dazu long suchen. Vor dem longlow darf kein Puls groesser long sein 
			pulseid = pdec->message[i];
			if (pulseid == longhigh || (pulseid == longlow && pdec->pattern[pdec->message[i-1]] <= pdec->pattern[longhigh]))
			{
				bit = pulseid == longhigh ? 1 : 0;
				mc_sync_pos = i;  // Save sync position for later
				break;
			}
			i++;
		}
		
		if (i >= pdec->messageLen-1)	// kein long gefunden, duerfte eigentlich nicht vorkommen
		{
			pdec->mcValid = false;
			return false;
		}
			
		ManchesterBits.addValue(bit);
		
		pdec->mstart = 255;
		if (isShort(pdec->message[i - 1]) && --i > 1)
		{
			while (i > 1)
			{
				if (isShort(pdec->message[i - 2]) && isShort(pdec->message[i - 1]))
				{
					i = i - 2;
				} else {
					// Letzter Durchlauf
					if (pdec->message[i - 1] == shorthigh) 
					{
						pdec->mstart = i - 2;
						i = 0; // leave the while loop after adding the value
					} else { 
						break;  // Add no more value
					}
				}
				ManchesterBits.addValue(bit);
			}
		}
		if (pdec->mstart == 255)
		{
			pdec->mstart = i;
		}
		
		i = mc_sync_pos; 	// recover i to sync_pos
		mc_start_found = true;
		
		// Decoding
		while (i < pdec->messageLen)
		{
			const uint8_t mpi = pdec->message[i]; // Store pattern for further processing
			
			if (isLong(mpi))
			{
				bit = bit ^ (1);
			}
			else {
				if (i >= pdec->messageLen-1) break;
				
				if (bit == 0)
				{
					if (mpi != shortlow)
					{
						break;
					}
					else if (pdec->message[i+1] != shorthigh)
					{
						if (pdec->message[i+1] == longhigh) pdec->mcValid = false;     // invalid, nach shortlow muss shorthigh folgen
						break;
					}
				}
				else  // bit = 1
				{
					if (mpi != shorthigh)
					{
						break;
					}
					else if (pdec->message[i+1] != shortlow)
					{
						if (pdec->message[i+1] == longlow) pdec->mcValid = false;     // invalid, nach shorthigh muss shortlow folgen
						break;
					}
				}
				i++;
			}
			i++;
			ManchesterBits.addValue(bit);
		}
		
		if (i < pdec->messageLen) i++;		// wenn Abbruch durch break
		
		if (ManchesterBits.valcount < minbitlen)
		{
			if (pdec->mcRepeat == false && i > minMessageLen)	// es ist keine Wiederholung und es wurde nach minMessageLen nichts gefunden -> weiter mit MU-Nachricht
			{
				pdec->mstart = i;
				return false;
			}
			ManchesterBits.reset();		// weiter suchen
#if MCDEBUGDECODE > 1
			DBG_PRINT("mcResMpos=");
			DBG_PRINTLN(i);
#endif
		}
		else 
		{
			pdec->mend = i;
			return true;
		}
	}
}
/*
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
		if (i == maxMsgSize-1 && i == pdec->messageLen-1 && isShort(mpi))
		{
						pdec->mcDetected = true;
					}
		
					pdec->bufferMove(i);   // Todo: BufferMove könnte in die Serielle Ausgabe verschoben werden, das würde ein paar Mikrosekunden Zeit sparen
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
					DBG_PRINT(":minblen=");
					DBG_PRINTLN(ManchesterBits.valcount>=minbitlen);
#endif
					return (ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed
				}
#ifdef DEBUGDECODE
				MSG_PRINT(")");
#endif
			}


			if (mc_start_found) { // don't add bit if manchester processing was canceled
#ifdef DEBUGDECODE
				if (pdec->pattern[mpi] > 0)
					value = (value + 0x20); //lowwercase
				DBG_PRINT(value);
#endif
				ManchesterBits.addValue(bit);
#ifdef DEBUGDECODE
				DBG_PRINT(ManchesterBits.getValue(ManchesterBits.valcount-1));
#endif
			} else {
#ifdef DEBUGDECODE
				DBG_PRINT("_");
#endif
			}
		} // 		endif (mc_sync && mc_start_found)
		i++;
	}
	pdec->mend = i; // Todo: keep short in buffer;

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
/*

/** @brief (Verifies if found signal data is a valid manchester signal, returns true or false)
*
* (Check signal based on patternLen, histogram and pattern store for valid manchester style.Provides key indexes for the 4 signal states for later decoding)
*/

const bool ManchesterpatternDecoder::isManchester()
{
	// Durchsuchen aller Musterpulse und prueft ob darin eine clock vorhanden ist
#if MCDEBUGDETECT >= 1
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
		#if MCDEBUGDETECT >= 1
				MSG_PRINT(p);
		#endif		

		uint8_t ptmp = p;

		while ( p!= 0 && pdec->pattern[i] < pdec->pattern[sortedPattern[p-1]])
		{
			sortedPattern[p] = sortedPattern[p-1];
			p--;
		}
#if MCDEBUGDETECT >= 1
		DBG_PRINT("="); DBG_PRINT(i); DBG_PRINT(",");
#endif
		sortedPattern[p] = i;
		p = ptmp+1;
	}
#if MCDEBUGDETECT >= 3
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
#if MCDEBUGDETECT >= 2
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

		const int clockpulse = pdec->pattern[sortedPattern[i]]; // double clock!
		for (uint8_t x = 0; x < p; x++)
		{
#if MCDEBUGDETECT >= 1
			DBG_PRINT(sortedPattern[x]); 
#endif

			const int aktpulse = pdec->pattern[sortedPattern[x]];
			bool pshort = false;
			bool plong = false;

			if (pdec->inTol(clockpulse/2, abs(aktpulse), clockpulse*0.2))
			//if (pdec->inTol(clockpulse, abs(aktpulse), clockpulse*0.5))
				pshort = true;
			else if (pdec->inTol(clockpulse, abs(aktpulse), clockpulse*0.40))
			//else if (pdec->inTol(clockpulse*2, abs(aktpulse), clockpulse*0.80))
				plong = true;

			#if MCDEBUGDETECT >= 3
			DBG_PRINT("^=(PS="); DBG_PRINT(pshort); DBG_PRINT(";");
			DBG_PRINT("PL="); DBG_PRINT(plong); DBG_PRINT(";)");
			#endif
			#if MCDEBUGDETECT >= 1
			DBG_PRINT(",");
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
#if MCDEBUGDETECT >= 1
				DBG_PRINT("vfy ");
#endif

				int8_t sequence_even[4] = { -1,-1,-1,-1 };				
				int8_t sequence_odd[4] = { -1,-1,-1,-1 };

				int z = 0;
				while (z < pdec->messageLen)
				{
					const uint8_t mpz = pdec->message[z]; // Store pattern for further processing

					if (((isLong(mpz) == false) && (isShort(mpz) == false)) || (z == (pdec->messageLen-1)))
					{  
#if MCDEBUGDETECT >= 1
						DBG_PRINT(z); DBG_PRINT("=")DBG_PRINT(mpz); DBG_PRINT(";")

						DBG_PRINT("Long"); DBG_PRINT(isLong(mpz)); DBG_PRINT(";");
						DBG_PRINT("Short"); DBG_PRINT(isShort(mpz)); DBG_PRINTLN(";");

#endif
						if ((z - pdec->mstart) > minbitlen)  // Todo: Hier wird auf minbitlen geprueft. Die Differenz zwischen mstart und mend sind aber Pulse und keine bits
						{
							pdec->mend = z;

							pdec->calcHisto(pdec->mstart, pdec->mend);
							equal_cnt = pdec->histo[shorthigh] + pdec->histo[longhigh] - pdec->histo[shortlow] - pdec->histo[longlow];

#if MCDEBUGDETECT >= 1
							DBG_PRINT("equalcnt: pos "); DBG_PRINT(pdec->mstart); DBG_PRINT(" to ") DBG_PRINT(pdec->mend); DBG_PRINT(" count=");  DBG_PRINT(equal_cnt); DBG_PRINT(" ");
#endif
							mc_start_found = false;
							if (abs(equal_cnt) > round(pdec->messageLen*0.04))  break; //Next loop
#if MCDEBUGDETECT >= 1
							DBG_PRINT(" MC equalcnt matched");
#endif
							if (neg_cnt != pos_cnt) break;  // Both must be 2   //TODO: For FFFF we have only 3 valid pulses!
#if MCDEBUGDETECT >= 1
							DBG_PRINT("  MC neg and pos pattern cnt is equal");
#endif

							if ((longlow == longhigh) || (shortlow == shorthigh) || (longlow == shortlow) || (longhigh == shorthigh) || (longlow == shorthigh) || (longhigh == shortlow)) break; //Check if the indexes are valid

							bool break_flag = false;
							for (uint8_t a = 0; a < 4 && break_flag==false; a++)
							{
#ifdef MCDEBUGDETECT  //>=0
								DBG_PRINT("  seq_even["); DBG_PRINT(a); DBG_PRINT("]");
								DBG_PRINT("="); DBG_PRINT(sequence_even[a]);
								DBG_PRINT("  seq_odd["); DBG_PRINT(a); DBG_PRINT("]");
								DBG_PRINT("="); DBG_PRINT(sequence_odd[a]);
#if MCDEBUGDETECT == 0
								DBG_PRINTLN(" "); 
#endif
#endif
								if ( (sequence_even[a] - sequence_odd[a] != 0) && (sequence_odd[a] == -1 || sequence_even[a] == -1))
								{
									break_flag = true;
								}

							}
							if (break_flag == true) {
#if MCDEBUGDETECT >= 1
								DBG_PRINT("  sequence check not passed");
#endif									
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
#if MCDEBUGDETECT >= 1
							DBG_PRINT("  all check passed");
#endif



							tstclock = tstclock / 6;
#if MCDEBUGDETECT >= 1
							MSG_PRINT("  tstclock: "); DBG_PRINT(tstclock);
#endif
							clock = tstclock;

#if MCDEBUGDETECT >= 1
							DBG_PRINT(" MC LL:"); DBG_PRINT(longlow);
							DBG_PRINT(", MC LH:"); DBG_PRINT(longhigh);

							DBG_PRINT(", MC SL:"); DBG_PRINT(shortlow);
							DBG_PRINT(", MC SH:"); DBG_PRINT(shorthigh);
							DBG_PRINTLN("");
#endif
							// TOdo: Bei FFFF passt diese Pruefung nicht.
#if MCDEBUGDETECT >= 1
							DBG_PRINTLN("  -- MC found -- ");
#endif
							return true;
						}
						else {
							mc_start_found = false;
						}
					} else {
						if (mc_start_found == false)
						{
							pdec->mstart = z;
							mc_start_found = true;
						}
					}
					
					int8_t seq_found = -1;
					uint8_t seq = (mpz* 10) + pdec->message[z + 1];

					if (seq < 10) seq += 100;
					
					int8_t *seqptr;
					if (z % 2 == 0)  //Every even value
						seqptr = sequence_even;
					else
						seqptr = sequence_odd;

					for (uint8_t a = 0; a < 4 && seq_found==-1; a++)
					{
							if (seqptr[a] == seq)
							{
								seq_found = a;
							}	else if (seqptr[a] == -1)
							{
								//DBG_PRINT(" +seq[:"); DBG_PRINT(seq); DBG_PRINTLN("]");

								seqptr[a] = seq;
								seq_found = a;
							}
							
					} 
					z++;
				}
			}
		}

	}
	return false;



}

