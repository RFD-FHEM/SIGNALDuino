// cc1101.h

#ifndef _CC1101_h
#define _CC1101_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "output.h"


#define csPin   10   // CSN  out
#define mosiPin 11   // MOSI out
#define misoPin 12   // MISO in
#define sckPin  13   // SCLK
#define gdo0Pin  3   // GDO0
#define int_gdo0 0

#define CC1101_SRES   0x30   // reset
#define CC1101_SRX    0x34   // enable RX. perform calibration first
#define CC1101_CONFIG 0x80 
#define CC1101_STATUS 0xC0

#define wait_Miso()       while(isHigh(misoPin))      // wait until SPI MISO line goes low
#define cc1101_Select()   digitalLow(csPin)          // select (SPI) CC1101
#define cc1101_Deselect() digitalHigh(csPin) 



namespace cc1101 {
	extern String cmdstring;

	bool ccenabled = false;

	static const uint8_t initVal[] PROGMEM = 
	{
		// IDX NAME     RESET   COMMENT
		0x0D, // 00 IOCFG2    29     GDO2 as serial output
		0x2E, // 01 IOCFG1           Tri-State
		0x2D, // 02 IOCFG0    3F     GDO0 for input
		0x07, // 03 FIFOTHR   
		0xD3, // 04 SYNC1     
		0x91, // 05 SYNC0     
		0x3D, // 06 PKTLEN    0F
		0x04, // 07 PKTCTRL1  
		0x32, // 08 PKTCTRL0  45     
		0x00, // 09 ADDR     
		0x00, // 0A CHANNR   
		0x06, // 0B FSCTRL1   0F     152kHz IF Frquency
		0x00, // 0C FSCTRL0
		0x10, // 0D FREQ2     1E     Freq
		0xB0, // 0E FREQ1     C4     
		0x71, // 0F FREQ0     EC     
		0x57, // 10 MDMCFG4   8C     bWidth 325kHz
		0xC4, // 11 MDMCFG3   22     DataRate
		0x30, // 12 MDMCFG2   02     Modulation: ASK
		0x23, // 13 MDMCFG1   22     
		0xb9, // 14 MDMCFG0   F8     ChannelSpace: 350kHz
		0x00, // 15 DEVIATN   47     
		0x07, // 16 MCSM2     07     
		0x00, // 17 MCSM1     30     
		0x18, // 18 MCSM0     04     Calibration: RX/TX->IDLE
		0x14, // 19 FOCCFG    36     
		0x6C, // 1A BSCFG
		0x07, // 1B AGCCTRL2  03     42 dB instead of 33dB
		0x00, // 1C AGCCTRL1  40     
		0x90, // 1D AGCCTRL0  91     4dB decision boundery
		0x87, // 1E WOREVT1
		0x6B, // 1F WOREVT0
		0xF8, // 20 WORCTRL
		0x56, // 21 FREND1
		0x11, // 22 FREND0    16     0x11 for no PA ramping
		0xE9, // 23 FSCAL3    A9    E9 ?? 
		0x2A, // 24 FSCAL2    0A    
		0x00, // 25 FSCAL1    20    19 ??
		0x1F, // 26 FSCAL0    0D     
		0x41, // 27 RCCTRL1
		0x00, // 28 RCCTRL0
	};

	const bool  HandleCommand();
	//byte hex2int(byte hex);

	byte hex2int(byte hex) {    // convert a hexdigit to int    // Todo: printf oder scanf nutzen
		if (hex >= '0' && hex <= '9') hex = hex - '0';
		else if (hex >= 'a' && hex <= 'f') hex = hex - 'a' + 10;
		else if (hex >= 'A' && hex <= 'F') hex = hex - 'A' + 10;
		return hex;
	}

	void printHex2(const byte hex);

	void readCCreg(void);
	void writeCCreg(void);
	void init(void);

	uint8_t sendSPI(const uint8_t val);
	void cmdStrobe(const uint8_t cmd);
	uint8_t readReg(const uint8_t regAddr, const uint8_t regType);
	void writeReg(const uint8_t regAddr, const uint8_t val);






	}

	const bool cc1101::HandleCommand()
	{
	#define  cmd_writeCC 'W'
	#define  cmd_space ' '
	#define  cmd_help '?'

		if (cmdstring.charAt(0) == cmd_help) {
			MSG_PRINT(cmd_writeCC); MSG_PRINT(cmd_space);
		}
		else if (cmdstring.charAt(0) == cmd_writeCC) {                          // write CC11001 register
																				// writeEeprom();  
			if (ccenabled) {
				writeCCreg();
			}
			return true;
		}
	}



	void cc1101::printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
		if (hex < 16) {
			MSG_PRINT("0");
		}
		MSG_PRINT(hex, HEX);
	}




	// CC1101 Routinen

	void cc1101::readCCreg(void) {   // read CC11001 register
		uint8_t var;
		uint8_t reg;
		uint8_t hex;
		uint8_t n;

		if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {
			hex = (byte)cmdstring.charAt(1);
			reg = hex2int(hex) * 16;
			hex = (byte)cmdstring.charAt(2);
			reg = hex2int(hex) + reg;
			if (cmdstring.charAt(3) == 'n' && isHexadecimalDigit(cmdstring.charAt(4))) {   // C<reg>n<anz>  gibt anz+2 fortlaufende register zurueck
				hex = (byte)cmdstring.charAt(4);
				n = hex2int(hex);
				if (reg < 0x2F) {
					MSG_PRINT("C");
					printHex2(reg);
					MSG_PRINT("n");
					n += 2;
					printHex2(n);
					MSG_PRINT("=");
					for (byte i = 0; i < n; i++) {
						var = readReg(reg + i, CC1101_CONFIG);
						printHex2(var);
					}
					MSG_PRINTLN("");
				}
			}
			else {
				if (reg < 0x40) {
					if (reg < 0x2F) {
						var = readReg(reg, CC1101_CONFIG);
					}
					else {
						var = readReg(reg, CC1101_STATUS);
					}
					MSG_PRINT("C");
					printHex2(reg);
					MSG_PRINT(" = ");
					printHex2(var);
					MSG_PRINTLN("");
				}
				else if (reg == 0x99) {                   // alle register
														  //MSG_PRINTLN("read all CC11001 reg");
					for (byte i = 0; i < 0x2f; i++) {
						if (i == 0 || i == 0x10 || i == 0x20) {
							if (i > 0) {
								MSG_PRINT(" ");
							}
							MSG_PRINT(F("ccreg "));
							printHex2(i);
							MSG_PRINT(": ");
						}
						var = readReg(i, CC1101_CONFIG);
						printHex2(var);
						MSG_PRINT(" ");
					}
					MSG_PRINTLN("");
				}
			}
		}
}


void cc1101::writeCCreg(void) {  // write CC11001 register
	uint8_t var;
	uint8_t reg;
	uint8_t hex;

	if (cmdstring.charAt(1) == 'S' && cmdstring.charAt(2) == '3') {       // WS<reg>  Command Strobes
		if (isHexadecimalDigit(cmdstring.charAt(3))) {
			hex = (byte)cmdstring.charAt(3);
			reg = hex2int(hex) + 0x30;
			if (reg < 0x3e) {
				cmdStrobe(reg);
				MSG_PRINT(F("cmdStrobeReg "));
				printHex2(reg);
				MSG_PRINTLN("");
			}
		}
	}
	else {    // W<reg><var>
		if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && isHexadecimalDigit(cmdstring.charAt(3)) && isHexadecimalDigit(cmdstring.charAt(4))) {
			hex = (byte)cmdstring.charAt(1);
			reg = hex2int(hex) * 16;
			hex = (byte)cmdstring.charAt(2);
			reg = hex2int(hex) + reg;
			if (reg > 1 && reg < 0x3e) {
				hex = (byte)cmdstring.charAt(3);
				var = hex2int(hex) * 16;
				hex = (byte)cmdstring.charAt(4);
				var = hex2int(hex) + var;
				writeReg(reg - 2, var);
				MSG_PRINT("W");
				printHex2(reg);
				printHex2(var);
				MSG_PRINTLN("");
			}
		}
	}
}



void cc1101::init(void) {                              // initialize CC1101

	pinAsOutput(csPin);                               // set pins for SPI communication
	pinAsOutput(mosiPin);
	pinAsInput(misoPin);
	pinAsOutput(sckPin);
	pinAsInput(gdo0Pin);                              // config GDO0 as input

	digitalHigh(csPin);                              // SPI init
	digitalHigh(sckPin);
	digitalLow(mosiPin);

	SPCR = _BV(SPE) | _BV(MSTR);                            // SPI speed = CLK/4

	cc1101_Deselect();                                  // some deselect and selects to init the cc1101
	delayMicroseconds(5);

	cc1101_Select();
	delayMicroseconds(10);

	cc1101_Deselect();
	delayMicroseconds(41);

	// todo: testen ob sich der cc1101 meldet
	ccenabled = true;

	cmdStrobe(CC1101_SRES);                               // send reset
	delay(10);

	// define init settings


	for (uint8_t i = 0; i<sizeof(initVal); i++) {              // write init value to cc11001
		writeReg(i, pgm_read_byte(&initVal[i]));
	}
	delay(1);
	cmdStrobe(CC1101_SRX);       // enable RX
}


uint8_t cc1101::sendSPI(const uint8_t val) {					     // send byte via SPI
	SPDR = val;                                      // transfer byte via SPI
	while (!(SPSR & _BV(SPIF)));                     // wait until SPI operation is terminated
	return SPDR;
}

void cc1101::cmdStrobe(const uint8_t cmd) {                     // send command strobe to the CC1101 IC via SPI
	cc1101_Select();                                // select CC1101
	wait_Miso();                                    // wait until MISO goes low
	sendSPI(cmd);                                   // send strobe command
	cc1101_Deselect();                              // deselect CC1101
}

uint8_t cc1101::readReg(const uint8_t regAddr, const uint8_t regType) {       // read CC1101 register via SPI
	cc1101_Select();                                // select CC1101
	wait_Miso();                                    // wait until MISO goes low
	sendSPI(regAddr | regType);                     // send register address
	uint8_t val = sendSPI(0x00);                    // read result
	cc1101_Deselect();                              // deselect CC1101
	return val;
}

void cc1101::writeReg(const uint8_t regAddr, const uint8_t val) {     // write single register into the CC1101 IC via SPI
	cc1101_Select();                                // select CC1101
	wait_Miso();                                    // wait until MISO goes low
	sendSPI(regAddr);                               // send register address
	sendSPI(val);                                   // send value
	cc1101_Deselect();                              // deselect CC1101
}

#endif

