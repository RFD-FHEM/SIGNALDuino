#pragma once

#ifndef _COMMANDS_h
#define _COMMANDS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//	#include "WProgram.h"
#endif

#include "compile_config.h"               // MSG_PRINTtoHEX / DBG_PRINTtoHEX - need for right options | ETHERNET_PRINT -> MSG_PRINTER Serial or serverClient

/* Help functions MSG_PRINTtoHEX & DBG_PRINTtoHEX
 * This position is the only one where you can compile and
 * no dependencies on libraries.
 * ( functions for less memory sketch consumption )
 */

void MSG_PRINTtoHEX(uint8_t a) { // this function is the alternative to sprintf(b, "%02x", xxx(i))
  if(a < 16) {
    MSG_PRINT(0);
  }
  MSG_PRINT(a , HEX);
}

void DBG_PRINTtoHEX(uint8_t b) {  // this function is the alternative to sprintf(b, "%02x", yyy(i));
#ifdef DEBUG
  if(b < 16) {
    DBG_PRINT(0);
  }
  DBG_PRINT(b , HEX);
#endif
}

/* Help functions END */

#include <EEPROM.h>
#include "output.h"
#include "cc1101.h"
#include "functions.h"
#include "signalDecoder.h"

extern char IB_1[14];
extern bool hasCC1101;
extern SignalDetectorClass musterDec;
extern volatile bool blinkLED;
extern uint8_t ccmode;;          // xFSK
#ifdef CMP_CC1101
	extern bool AfcEnabled;
	#if defined (ESP8266) || defined (ESP32)
		extern bool wmbus;
		extern bool wmbus_t;
	#endif
#endif

namespace commands {

	inline void getPing()
	{
		MSG_PRINTLN("OK");
		delayMicroseconds(500);
	}

	inline void changeReceiver() {
		if (IB_1[1] == 'Q')
		{
			disableReceive();
		}
		else if (IB_1[1] == 'E')
		{
			enableReceive();
		}
	}

	inline void getConfig()
	{
		#ifdef CMP_CC1101
			if (cc1101::ccmode == 3) { // ASK/OOK = 3 (default)
				MSG_PRINT(F("MS="));
				MSG_PRINT(musterDec.MSenabled, DEC);
				MSG_PRINT(F(";MU="));
				MSG_PRINT(musterDec.MUenabled, DEC);
				MSG_PRINT(F(";MC="));
				MSG_PRINT(musterDec.MCenabled, DEC);
				MSG_PRINT(F(";Mred="));
				MSG_PRINTLN(musterDec.MredEnabled, DEC);
			} else {                   // FSK
				MSG_PRINT(F("MN=1"));
				#if defined (ESP8266) || defined (ESP32)
					MSG_PRINT(F(";WMBus="));
					MSG_PRINT(wmbus, DEC);
					MSG_PRINT(F(";WMBus_T="));
					MSG_PRINT(wmbus_t, DEC);
				#endif
				MSG_PRINT(F(";AFC="));
				MSG_PRINTLN(AfcEnabled, DEC);
			}
		#else
			MSG_PRINT(F("MS="));
			MSG_PRINT(musterDec.MSenabled, DEC);
			MSG_PRINT(F(";MU="));
			MSG_PRINT(musterDec.MUenabled, DEC);
			MSG_PRINT(F(";MC="));
			MSG_PRINT(musterDec.MCenabled, DEC);
			MSG_PRINT(F(";Mred="));
			MSG_PRINTLN(musterDec.MredEnabled, DEC);
		#endif
	}

	inline void configCMD()
	{
		bool *bptr;

		switch (IB_1[2])
		{
			case 'S' : //MS
				bptr = &musterDec.MSenabled;
				break;
			case 'U' : //MU
				bptr = &musterDec.MUenabled;
				break;
			case 'C' : //MC
				bptr = &musterDec.MCenabled;
				break;
			case 'R' : //Mreduce
				bptr = &musterDec.MredEnabled;
				break;
			#ifdef CMP_CC1101
			case 'A' : //Afc
				bptr = &AfcEnabled;
				break;
			#if defined (ESP8266) || defined (ESP32)
			case 'W' : //WMBus
				bptr = &wmbus;
				break;
			case 'T' : //WMBus_T
				bptr = &wmbus_t;
				break;
			#endif
			#endif
			default:
				return;
		}

		switch (IB_1[1])
		{
			case 'E':
				*bptr = true;
				break;
			case 'D':
				*bptr = false;
				break;
			default:
				return;
		}
		#if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32))
			storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled, musterDec.MredEnabled, AfcEnabled, wmbus, wmbus_t);
		#else
			storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled, musterDec.MredEnabled, AfcEnabled);
		#endif
	}

	inline void configSET()
	{
		//MSG_PRINT(cmdstring.substring(2, 8));
		if (strstr(&IB_1[2],"mcmbl=") != NULL)   // mc min bit len
		{
			musterDec.mcMinBitLen = strtol(&IB_1[8], NULL,10);
			MSG_PRINT(musterDec.mcMinBitLen); MSG_PRINT(F(" bits set"));
		}
	}


	void HandleShortCommand()
	{
		#define  cmd_Version 'V'
		#define  cmd_freeRam 'R'
		#define  cmd_uptime 't'
		#define  cmd_changeReceiver 'X'
		#define  cmd_help '?'
		#define  cmd_ping 'P'
		#define  cmd_ccFactoryReset 'e'  // EEPROM / factory reset
		#define  cmd_config 'C'          // CG get config, set config, C<reg> get CC1101 register
		#define  cmd_patable 'x' 
		#define  cmd_write 'W'           // write EEPROM und write CC1101 register
		#define  cmd_read  'r'           // read EEPROM
		#define  cmd_space ' '
		#define  cmd_send 'S'
		#define  cmd_status 's'

		switch (IB_1[0])
		{
		case cmd_help:
			MSG_PRINT(cmd_help);	MSG_PRINT(F(" Use one of "));
			MSG_PRINT(cmd_Version); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_freeRam); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_uptime); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_changeReceiver); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_send); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_ping); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_config); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_read); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_write); MSG_PRINT(FPSTR(TXT_BLANK));
			MSG_PRINT(cmd_status); MSG_PRINT(FPSTR(TXT_BLANK));
#ifdef CMP_CC1101
			if (hasCC1101) {
				MSG_PRINT(cmd_patable); MSG_PRINT(FPSTR(TXT_BLANK));
				MSG_PRINT(cmd_ccFactoryReset); MSG_PRINT(FPSTR(TXT_BLANK));
			}
#endif
			MSG_PRINTLN("");
			break;
		case cmd_ping:
			getPing();
			break;
		case cmd_Version:
			MSG_PRINT(F("V " PROGVERS PROGNAME));
#ifdef CMP_CC1101
			if (hasCC1101) {
				MSG_PRINT(FPSTR(TXT_CC1101));
				MSG_PRINT('(');

#endif
#ifdef PIN_MARK433
				MSG_PRINT(FPSTR(isLow(PIN_MARK433) ? TXT_433 : TXT_868));
				MSG_PRINT(FPSTR(TXT_MHZ));
				MSG_PRINT(')');
#else
	#ifdef CMP_CC1101
				MSG_PRINT(FPSTR(TXT_CHIP)); MSG_PRINT(FPSTR(TXT_BLANK)); MSG_PRINT(FPSTR(TXT_CC110));
				switch (cc1101::chipVersion()) {
					case 0x03:
						 MSG_PRINT("0"); 
						break;
					case 0x14:
					case 0x04:
						 MSG_PRINT("1");
						break;
					case 0x05:
						 MSG_PRINT("0E");
						break;
					case 0x07:
 					case 0x17:
						 MSG_PRINT("L");
						break;
					default:
						MSG_PRINT(F(" unknown"));
						break;
				}
				MSG_PRINT(')');
	#endif
#endif
#ifdef CMP_CC1101
			}
#endif
#ifdef DEBUG
			MSG_PRINT(F(" DBG"));
#endif
#ifdef WATCHDOG_STM32
	if (watchRes) {
		MSG_PRINT(F(" wr"));
	}
#endif
			MSG_PRINTLN(F(" - compiled at " __DATE__ " " __TIME__));
			break;
		case cmd_freeRam:
			MSG_PRINTLN(freeRam());
			break;
		case cmd_uptime:
			MSG_PRINTLN(getUptime());
			break;
		case cmd_changeReceiver:
			changeReceiver();
			break;
		case cmd_config:
			switch (IB_1[1])
			{
				case 'G':
					getConfig();
					break;
				case 'E':
				case 'D':
					configCMD();
					break;
				case 'S':
					configSET();
					break;
	#ifdef CMP_CC1101
				default:
					if (isxdigit(IB_1[1]) && isxdigit(IB_1[2]) && hasCC1101) {
						uint8_t val = (uint8_t)strtol(IB_1+1, nullptr, 16);
						cc1101::readCCreg(val);
					}
	#endif
			}
			break;
#ifdef CMP_CC1101
		case cmd_ccFactoryReset:
			if (hasCC1101) {
				cc1101::ccFactoryReset();
				cc1101::CCinit();
				dumpEEPROM();
			}
			break;
		case cmd_patable:  // example: x12
			if (isHexadecimalDigit(IB_1[1]) && isHexadecimalDigit(IB_1[2]) && hasCC1101) {
				uint8_t val = (uint8_t)strtol(IB_1+1, nullptr, 16);
				cc1101::writeCCpatable(val);
				MSG_PRINT(FPSTR(TXT_WRITE));
				MSG_PRINTtoHEX(val);
				MSG_PRINTLN(FPSTR(TXT_TPATAB));
			}
			break;

#endif
		case cmd_read:  // r<adr> read EEPROM, example: r0a, r03n
			if (isHexadecimalDigit(IB_1[1]) && isHexadecimalDigit(IB_1[2]) && hasCC1101) {
				const uint8_t reg = (uint8_t)strtol(IB_1+1, nullptr, 16);
				MSG_PRINT(FPSTR(TXT_EEPROM));
				MSG_PRINT(FPSTR(TXT_BLANK));

				MSG_PRINTtoHEX(reg);

				if (IB_1[3] == 'n') {
					MSG_PRINT(F(" :"));
					for (uint8_t i = 0; i < 16; i++) {
						const uint8_t val = EEPROM.read(reg + i);
						MSG_PRINTtoHEX(val);
					}
				}
				else {
					MSG_PRINT(F(" = "));
					const uint8_t val = EEPROM.read(reg);
					MSG_PRINTtoHEX(val);
				}
				MSG_PRINTLN("");
			}
			break;
		case cmd_write:
			if (IB_1[1] == 'S' && IB_1[2] == '3')
			{
				#ifdef CMP_CC1101
				cc1101::commandStrobes();
				#endif
			} else if (isHexadecimalDigit(IB_1[1]) && isHexadecimalDigit(IB_1[2]) && isHexadecimalDigit(IB_1[3]) && isHexadecimalDigit(IB_1[4])) {
				char b[3];
				b[2] = '\0';

				memcpy(b, &IB_1[1], 2);
				uint8_t reg = strtol(b, nullptr, 16);
				memcpy(b, &IB_1[3], 2);
				uint8_t val = strtol(b, nullptr, 16);

				EEPROM.write(reg, val); //Todo pruefen ob reg hier um 1 erhoeht werden muss
				DBG_PRINT(reg);
				DBG_PRINT('=');

				DBG_PRINTLN(val);

#ifdef CMP_CC1101
				if (hasCC1101) {
					cc1101::writeCCreg(reg, val);
				}

				if (reg == 0x10 + 2) {       // 0x10 MDMCFG4 bwidth 325 kHz (EEPROM-Addresse + 2)

					if (EEPROM.read(2) == 13) {     // adr 0x00 is only on OOK/ASK 0D (GD0) | DN022 -- CC110x CC111x OOK ASK Register Settings (Rev. E) "... optimum register settings for OOK/ASK operation."
						DBG_PRINTLN(F("optimum register settings for OOK/ASK operation"));

						reg = 0x21 + 2;            // 0x21 FREND1 (EEPROM-Addresse + 2)
						// RX filter bandwidth > 101 kHz, FREND1 = 0xB6
						// RX filter bandwidth <= 101 kHz, FREND1 = 0x56
						if (val >= 0xC7) {    // 199 = 0xC7 = 101 kHz
							val = 0x56;          // FREND1 = 0x56
						}
						else {
							val = 0xB6;         // FREND1 = 0xB6
						}
						EEPROM.write(reg, val);
						if (hasCC1101) {
							cc1101::writeCCreg(reg, val);
						}
						reg = 0x03 + 2;             // 0x03 FIFOTHR (EEPROM-Addresse + 2)
						memcpy(b, &IB_1[3], 2);
						val = strtol(b, nullptr, 16);
						// RX filter bandwidth > 325 kHz, FIFOTHR = 0x07
						// RX filter bandwidth <= 325 kHz, FIFOTHR = 0x47
						if (val >= 0x57) {     // 87 = 0x57 = 325 kHz
							val = 0x47;          // FIFOTHR = 0x47
						}
						else {
							val = 0x07;           // FIFOTHR = 0x07
						}
						EEPROM.write(reg, val);
						if (hasCC1101) {
							cc1101::writeCCreg(reg, val);
						}
					}
				}
#endif
#if defined(ESP32) || defined(ESP8266)
				EEPROM.commit();
#endif
			}
		break;
		case cmd_status:
#ifdef CMP_CC1101
			if (hasCC1101 && !cc1101::regCheck())
			{
				MSG_PRINT(FPSTR(TXT_CC1101));
				MSG_PRINT(FPSTR(TXT_DOFRESET));
				MSG_PRINTLN(FPSTR(TXT_COMMAND));
			}
			else
			{
				MSG_PRINTLN("OK");
			}
#endif
		break;
		default:
			MSG_PRINTLN(FPSTR(TXT_UNSUPPORTED1));
			return;
		}
		blinkLED = true;
	}


}


#endif
