
#include "cc1101.h"

uint8_t cc1101::revision = 0x01;
 const uint8_t cc1101::initVal[] PROGMEM =
{
	// IDX NAME     RESET   COMMENT
0x0D, // 00 IOCFG2    29     GDO2 as serial output
0x2E, // 01 IOCFG1           Tri-State
0x2D, // 02 IOCFG0    3F     GDO0 for input
//0x07, // 03 FIFOTHR          RX filter bandwidth > 325 kHz, FIFOTHR = 0x07
0x47, // 03 FIFOTHR          RX filter bandwidth = 325 kHz, FIFOTHR = 0x47
0xD3, // 04 SYNC1     
0x91, // 05 SYNC0     
0x3D, // 06 PKTLEN    0F
0x04, // 07 PKTCTRL1  
0x32, // 08 PKTCTRL0  45     
0x00, // 09 ADDR     
0x00, // 0A CHANNR   
0x06, // 0B FSCTRL1   0F     152kHz IF Frquency
0x00, // 0C FSCTRL0
0x10, // 0D FREQ2     1E     Freq   #12  Reg Pos 0C
0xB0, // 0E FREQ1     C4				 Reg Pos 0D
0x71, // 0F FREQ0     EC				 Reg Pos 0E
0x57, // 10 MDMCFG4   8C     bWidth 325kHz
0xC4, // 11 MDMCFG3   22     DataRate
0x30, // 12 MDMCFG2   02     Modulation: ASK
0x23, // 13 MDMCFG1   22     
0xb9, // 14 MDMCFG0   F8     ChannelSpace: 350kHz
0x00, // 15 DEVIATN   47     
0x07, // 16 MCSM2     07     
0x00, // 17 MCSM1     30     Bit 3:2  RXOFF_MODE:  Select what should happen when a packet has been received: 0 = IDLE  3 =  Stay in RX ####
0x18, // 18 MCSM0     04     Calibration: RX/TX->IDLE
0x14, // 19 FOCCFG    36     
0x6C, // 1A BSCFG
0x07, // 1B AGCCTRL2  03     42 dB instead of 33dB
0x00, // 1C AGCCTRL1  40     
//0x90, // 1D AGCCTRL0  90     4dB decision boundery
0x91, // 1D AGCCTRL0  91     8dB decision boundery
0x87, // 1E WOREVT1
0x6B, // 1F WOREVT0
0xF8, // 20 WORCTRL
//0x56, // 21 FREND1    56     RX filter bandwidth = 101 kHz, FREND1 = 0x56
0xB6, // 21 FREND1    B6     RX filter bandwidth > 101 kHz, FREND1 = 0xB6
0x11, // 22 FREND0    16     0x11 for no PA ramping
0xE9, // 23 FSCAL3    A9    E9 ?? 
0x2A, // 24 FSCAL2    0A    
0x00, // 25 FSCAL1    20    19 ??
0x1F, // 26 FSCAL0    0D     
0x41, // 27 RCCTRL1
0x00, // 28 RCCTRL0
};



 byte cc1101::hex2int(byte hex) {    // convert a hexdigit to int    // Todo: printf oder scanf nutzen
	 if (hex >= '0' && hex <= '9') hex = hex - '0';
	 else if (hex >= 'a' && hex <= 'f') hex = hex - 'a' + 10;
	 else if (hex >= 'A' && hex <= 'F') hex = hex - 'A' + 10;
	 return hex;
	 // printf ("%d\n",$hex) ??
 }

uint8_t cc1101::sendSPI(const uint8_t val) {				 // send byte via SPI
#ifndef ESP8266
	 SPDR = val;                                      // transfer byte via SPI
	 while (!(SPSR & _BV(SPIF)));                     // wait until SPI operation is terminated
	 return SPDR;
#else
	 return SPI.transfer(val);
#endif
 }

uint8_t cc1101::cmdStrobe(const uint8_t cmd) {              // send command strobe to the CC1101 IC via SPI
	 cc1101_Select();                                // select CC1101
	 wait_Miso_rf();                                 // wait until MISO goes low
	 uint8_t ret = sendSPI(cmd);                     // send strobe command
	 wait_Miso_rf();                                 // wait until MISO goes low
	 cc1101_Deselect();                              // deselect CC1101
	 return ret;										// Chip Status Byte
 }
uint8_t cc1101::readReg(const uint8_t regAddr, const uint8_t regType) {       // read CC1101 register via SPI
	cc1101_Select();                                // select CC1101
	wait_Miso_rf();                                    // wait until MISO goes low
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

void cc1101::readPatable(void) {
	uint8_t PatableArray[8];
	// das PatableArray wird zum zwischenspeichern der PATABLE verwendet,
	// da ich mir nicht sicher bin ob es timing maessig passt, wenn es nach jedem sendSPI(0x00) eine kurze Pause beim msgprint gibt.

	cc1101_Select();                                // select CC1101
	wait_Miso();                                    // wait until MISO goes low
	sendSPI(CC1100_PATABLE | CC1100_READ_BURST);    // send register address
	for (uint8_t i = 0; i < 8; i++) {
		PatableArray[i] = sendSPI(0x00);        // read result
	}
	cc1101_Deselect();
	char b[4];

	for (uint8_t i = 0; i < 8; i++) {
		sprintf_P(b, PSTR(" %02X"), PatableArray[i]);
		MSG_PRINT(b);
	}
	MSG_PRINTLN("");
}

void cc1101::writePatable(void) {
	cc1101_Select();                                // select CC1101
	wait_Miso();                                    // wait until MISO goes low
	sendSPI(CC1100_PATABLE | CC1100_WRITE_BURST);   // send register address
	for (uint8_t i = 0; i < 8; i++) {
		sendSPI(EEPROM.read(EE_CC1100_PA + i));                     // send value
	}
	cc1101_Deselect();
}


void cc1101::readCCreg(const uint8_t reg) {   // read CC11001 register
	uint8_t var;
	uint8_t n;
	char b[11];

	if (IB_1[3] == 'n' && isHexadecimalDigit(IB_1[4])) {   // C<reg>n<anz>  gibt anz+2 fortlaufende register zurueck
		n = (uint8_t)strtol((const char*)IB_1 + 4, NULL, 16);
		if (reg < 0x2F) {
			n += 2;
			sprintf(b, "C%02Xn%02X=", reg, n);
			MSG_PRINT(b);

			for (uint8_t i = reg; i < reg + n; i++) {
				var = readReg(i, CC1101_CONFIG);
				sprintf(b, "%02X", var);
				MSG_PRINT(b);
			}
			MSG_PRINTLN("");
		}
	}
	else {
		if (reg < 0x3E) {
			if (reg < 0x2F) {
				var = readReg(reg, CC1101_CONFIG);
			}
			else {
				var = readReg(reg, CC1101_STATUS);
			}
			sprintf_P(b, PSTR("C%02X = %02X"), reg, var);
			MSG_PRINTLN(b);
		}
		else if (reg == 0x3E) {                   // patable
			MSG_PRINT(F("C3E ="));
			readPatable();
		}
		else if (reg == 0x99) {                   // alle register
			for (uint8_t i = 0; i < 0x2f; i++) {
				if (i == 0 || i == 0x10 || i == 0x20) {
					if (i > 0) {
						MSG_PRINT(" ");
					}
					sprintf_P(b, PSTR("ccreg %02X: "), i);
					MSG_PRINT(b);
				}
				var = readReg(i, CC1101_CONFIG);
				sprintf_P(b, PSTR("%02X "), var);
				MSG_PRINT(b);
			}
			MSG_PRINTLN("");
		}
	}
}

void cc1101::commandStrobes(void) {

	uint8_t reg;
	uint8_t val;
	uint8_t val1;

	if (isHexadecimalDigit(IB_1[3])) {
		reg = (uint8_t)strtol(&IB_1[3], nullptr, 16);
		if (reg < 0x3e) {
			val = cmdStrobe(reg);
			delay(1);
			val1 = cmdStrobe(0x3D);        //  No operation. May be used to get access to the chip status byte.
			char b[41];
			sprintf_P(b, PSTR("cmdStrobeReg %02X chipStatus %02X delay1 %02X"), reg, val >> 4, val1 >> 4);
			MSG_PRINTLN(b);
		}
	}
}

void cc1101::writeCCreg(uint8_t reg, uint8_t var) {    // write CC11001 register
	if (reg > 1 && reg < 0x40) {
		writeReg(reg - EE_CC1100_CFG, var);
		char b[6];
		sprintf_P(b, PSTR("W%02X%02X"), reg, var);
		MSG_PRINTLN(b);
	}
}
void cc1101::writeCCpatable(uint8_t var) {           // write 8 byte to patable (kein pa ramping)
	for (uint8_t i = 0; i < 8; i++) {
		if (i == 1) {
			EEPROM.write(EE_CC1100_PA + i, var);
		}
		else {
			EEPROM.write(EE_CC1100_PA + i, 0);
		}
	}
	#ifdef ESP8266
	EEPROM.commit();
	#endif
	writePatable();
}

uint8_t cc1101::chipVersionRev()
{
	return readReg((revision == 0x01 ? CC1101_VERSION_REV01 : CC1101_VERSION_REV00), CC1101_READ_SINGLE);
};

uint8_t cc1101::chipVersion() {
	uint8_t version = chipVersionRev();

	if (cc1101::revision != 0x00 && (version == 0xFF || version == 0x00)) {
		revision = 0x00;
		version = chipVersionRev();
	}

	return version;
}

bool cc1101::checkCC1101() {

	uint8_t version = chipVersion();  // Version
#ifdef DEBUG
	uint8_t partnum = readReg((revision == 0x01 ? CC1101_PARTNUM_REV01 : CC1101_PARTNUM_REV00), CC1101_READ_SINGLE);  // Partnum
	DBG_PRINT(FPSTR(TXT_CCREVISION));	DBG_PRINTLN("0x" + String(version, HEX));
	DBG_PRINT(FPSTR(TXT_CCPARTNUM));	DBG_PRINTLN("0x" + String(partnum, HEX)); //TODO String Klasse entfernen
#endif
	//checks if valid Chip ID is found. Usualy 0x03 or 0x14. if not -> abort
	if (version == 0x00 || version == 0xFF)
	{
		DBG_PRINT(F("no "));  DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(FPSTR(TXT_FOUND));//  F("no CC11xx found!"));
		return false;  // Todo: power down SPI etc
	}
	return true;
}


void cc1101::setup()
{
#ifndef ESP8266
	pinAsOutput(sckPin);
	pinAsOutput(mosiPin);
	pinAsInput(misoPin);
#endif
	pinAsOutput(csPin);                    // set pins for SPI communication

#ifdef PIN_MARK433
	pinAsInputPullUp(PIN_MARK433);
#endif


#ifndef ESP8266
	SPCR = _BV(SPE) | _BV(MSTR);               // SPI speed = CLK/4
	digitalHigh(csPin);                 // SPI init
	digitalHigh(sckPin);
	digitalLow(mosiPin);
#else
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
	pinAsInput(PIN_RECEIVE);    // gdo2
	pinAsInput(PIN_SEND);      // gdo0Pi, sicherheitshalber bis zum CC1101 init erstmal input   
}

uint8_t cc1101::getRevision() 
{ return revision; }


uint8_t cc1101::getRSSI() {
	return readReg((revision == 0x01 ? CC1100_RSSI_REV01 : CC1100_RSSI_REV00), CC1101_STATUS);
}

void cc1101::setIdleMode()
{
	cmdStrobe(CC1100_SIDLE);                             // Idle mode
	delay(1);
}

uint8_t cc1101::currentMode() {
	return readReg((revision == 0x01 ? CC1100_MARCSTATE_REV01 : CC1100_MARCSTATE_REV00), CC1100_READ_BURST);
}

void cc1101::setReceiveMode()
{
	setIdleMode();
	uint8_t maxloop = 0xff;

	while (maxloop-- && (cmdStrobe(CC1100_SRX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_RX) // RX enable
		delay(1);
	if (maxloop == 0)		DBG_PRINTLN("CC1101: Setting RX failed");
	pinAsInput(PIN_SEND);
}

void cc1101::setTransmitMode()
{
	cmdStrobe(CC1100_SFTX);   // wird dies benoetigt? Wir verwenden kein FIFO
	setIdleMode();
	uint8_t maxloop = 0xff;
	while (maxloop-- && (cmdStrobe(CC1100_STX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_TX)  // TX enable
		delay(1);
	if (maxloop == 0) DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(F(": Setting TX failed"));
	pinAsOutput(PIN_SEND);      // gdo0Pi, sicherheitshalber bis zum CC1101 init erstmal input   

}


bool cc1101::regCheck()
{
	//char b[3];
	//uint8_t val;

	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_PKTCTRL0=")); DBG_PRINT(readReg(CC1100_PKTCTRL0, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval PKTCTRL0=")); DBG_PRINTLN(cc1101::initVal[CC1100_PKTCTRL0]);

	DBG_PRINT(FPSTR(TXT_CC1101)); 
	DBG_PRINT(F("_IOCFG2=")); DBG_PRINT(readReg(CC1100_IOCFG2, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval IOCFG2=")); DBG_PRINTLN(cc1101::initVal[CC1100_IOCFG2]);
	/*
	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_FREQ0=")); DBG_PRINT(readReg(CC1100_FREQ0, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval FREQ0=")); DBG_PRINT(cc1101::initVal[CC1100_FREQ0]);
	DBG_PRINT(F(" vs EEPROM FREQ0"));
	val = EEPROM.read(EE_CC1100_CFG + CC1100_FREQ0);
	sprintf(b, " %d", val);
	DBG_PRINTLN(b);

	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_FREQ1=")); DBG_PRINT(readReg(CC1100_FREQ1, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval FREQ1=")); DBG_PRINT(cc1101::initVal[CC1100_FREQ1]);
	DBG_PRINT(F(" vs EEPROM FREQ1="));
	val = EEPROM.read(EE_CC1100_CFG + CC1100_FREQ1);
	sprintf(b, " %d", val);
	DBG_PRINTLN(b);

	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_FREQ2=")); DBG_PRINT(readReg(CC1100_FREQ2, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval FREQ2=")); DBG_PRINT(cc1101::initVal[CC1100_FREQ2]);
	DBG_PRINT(F(" vs EEPROM FREQ2=")); 
	val = EEPROM.read(EE_CC1100_CFG+CC1100_FREQ2);
	sprintf(b, " %d", val);
	DBG_PRINTLN(b);
	*/
	return (readReg(CC1100_PKTCTRL0, CC1101_CONFIG) == cc1101::initVal[CC1100_PKTCTRL0]) && (readReg(CC1100_IOCFG2, CC1101_CONFIG) == cc1101::initVal[CC1100_IOCFG2]);
}



void cc1101::ccFactoryReset() {
	for (uint8_t i = 0; i < sizeof(cc1101::initVal); i++) {
		EEPROM.write(EE_CC1100_CFG + i, pgm_read_byte(&initVal[i]));
		DBG_PRINT(".");
	}
	for (uint8_t i = 0; i < 8; i++) {
		if (i == 1) {
			EEPROM.write(EE_CC1100_PA + i, PATABLE_DEFAULT);
		}
		else {
			EEPROM.write(EE_CC1100_PA + i, 0);
		}
	}
	#ifdef ESP8266
	EEPROM.commit();
	#endif
	MSG_PRINTLN("ccFactoryReset done");
}

void cc1101::CCinit(void) {  // initialize CC1101
	
	DBG_PRINT(FPSTR(TXT_CCINIT)); 

	cc1101_Deselect();                                  // some deselect and selects to init the cc1101
	delayMicroseconds(30);

	// Begin of power on reset
	cc1101_Select();
	delayMicroseconds(30);

	cc1101_Deselect();
	delayMicroseconds(45);

	DBG_PRINT(F("SRES Started,"));
	cmdStrobe(CC1101_SRES);                               // send reset
	DBG_PRINT(F("POR Done,"));
	delay(10);

	cc1101_Select();
	DBG_PRINT(FPSTR(TXT_EEPROM)); 	DBG_PRINT(FPSTR(TXT_BLANK));	DBG_PRINT(FPSTR(TXT_READ));

	sendSPI(CC1100_WRITE_BURST);
	for (uint8_t i = 0; i < sizeof(cc1101::initVal); i++) {              // write EEPROM value to cc11001
		sendSPI(EEPROM.read(EE_CC1100_CFG + i));
		DBG_PRINT(".");

	}
	cc1101_Deselect();
	delayMicroseconds(10);            // ### todo: welcher Wert ist als delay sinnvoll? ###

	writePatable();                                 // write PatableArray to patable reg
	DBG_PRINTLN(F("done"));

	delay(1);
	setReceiveMode();
}
