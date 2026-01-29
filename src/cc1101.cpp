#include "cc1101.h"

#ifdef ARDUINO_MAPLEMINI_F103CB
	SPIClass SPI_2(mosiPin, misoPin, sckPin);
#endif

#define ccMaxBuf 64                  // for cc1101 FIFO, variable is better to revised
uint8_t cc1101::ccmode = 3;          // MDMCFG2–Modem Configuration Bit 6:4 default ASK/OOK
uint8_t cc1101::revision = 0x01;
uint8_t ccBuf[ccMaxBuf];             // for cc1101 FIFO, if Circuit board for more cc110x -> ccBuf expand ( ccBuf[radionr][ccMaxBuf] )
int8_t cc1101::freqOffAcc = 0;
float cc1101::freqErrAvg = 0;
extern volatile bool blinkLED;
extern void MSG_PRINTtoHEX(uint8_t a);
extern bool AfcEnabled;

const uint8_t cc1101::initVal[] PROGMEM =
{
      // IDX NAME     RESET  COMMENT
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
0xB0, // 0E FREQ1     C4                 Reg Pos 0D
0x71, // 0F FREQ0     EC                 Reg Pos 0E
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
0xE9, // 23 FSCAL3    A9     E9 ??
0x2A, // 24 FSCAL2    0A
0x00, // 25 FSCAL1    20     19 ??
0x1F, // 26 FSCAL0    0D
0x41, // 27 RCCTRL1
0x00, // 28 RCCTRL0
};

#ifdef ETHERNET_PRINT
String cc1101::dec2hex(uint8_t val) { // convert one byte to hex string, return hex string "00" from dec 0
  char ret[3];
  ret[0] = (val >> 4);
  ret[1] = (val & 0x0F);
  ret[0] = (ret[0] > 9 ? ret[0] + 55 : ret[0] + '0');
  ret[1] = (ret[1] > 9 ? ret[1] + 55 : ret[1] + '0');
  ret[2] = '\0';
  return ret;
}
#endif

byte cc1101::hex2int(byte hex) {    // convert a hexdigit to int (smallest variant, sketch is bigger with printf or scanf)
	if (hex >= '0' && hex <= '9') hex = hex - '0';
	else if (hex >= 'a' && hex <= 'f') hex = hex - 'a' + 10;
	else if (hex >= 'A' && hex <= 'F') hex = hex - 'A' + 10;
	return hex;
	// printf ("%d\n",$hex) ??
}

uint8_t cc1101::sendSPI(const uint8_t val) {        // send byte via SPI
#if !defined(ESP8266) && !defined(ESP32) && !defined(ARDUINO_MAPLEMINI_F103CB)
	SPDR = val;                                     // transfer byte via SPI
	while (!(SPSR & _BV(SPIF)));                    // wait until SPI operation is terminated
	return SPDR;
#else
	#ifdef ARDUINO_MAPLEMINI_F103CB
		return SPI_2.transfer(val);                 // transfer use SPI2 on ARDUINO_MAPLEMINI_F103CB board
	#else
		return SPI.transfer(val);
	#endif
#endif
}

uint8_t cc1101::waitTo_Miso() {                     // wait with timeout until MISO goes low
	uint8_t i = 255;
	while(isHigh(misoPin)) {
		delayMicroseconds(10);
		i--;
		if (i == 0) { // timeout
			cc1101_Deselect();
			break;
		}
	}
	return i;
}

uint8_t cc1101::cmdStrobe(const uint8_t cmd) {      // send command strobe to the CC1101 IC via SPI
	cc1101_Select();                                // select CC1101
	wait_Miso_rf();                                 // wait until MISO goes low
	uint8_t ret = sendSPI(cmd);                     // send strobe command
	wait_Miso_rf();                                 // wait until MISO goes low
	cc1101_Deselect();                              // deselect CC1101
	return ret;                                     // Chip Status Byte
}

uint8_t cc1101::cmdStrobeTo(const uint8_t cmd) {    // wait MISO and send command strobe to the CC1101 IC via SPI
	cc1101_Select();                                // select CC1101
	if (waitTo_Miso() == 0) {                       // wait with timeout until MISO goes low
		return false;                               // timeout
	}
	sendSPI(cmd);                                   // send strobe command
	cc1101_Deselect();                              // deselect CC1101
	return true;
}

uint8_t cc1101::readReg(const uint8_t regAddr, const uint8_t regType) {       // read CC1101 register via SPI
	cc1101_Select();                                // select CC1101
	wait_Miso_rf();                                 // wait until MISO goes low
	sendSPI(regAddr | regType);                     // send register address
	uint8_t val = sendSPI(0x00);                    // read result
	cc1101_Deselect();                              // deselect CC1101
	return val;
}

void cc1101::writeReg(const uint8_t regAddr, const uint8_t val) {       // write single register into the CC1101 IC via SPI
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
	sendSPI(CC1101_PATABLE | CC1101_READ_BURST);    // send register address
	for (uint8_t i = 0; i < 8; i++) {
		PatableArray[i] = sendSPI(0x00);            // read result
	}
	cc1101_Deselect();

	for (uint8_t i = 0; i < 8; i++) {
    MSG_PRINT(FPSTR(TXT_BLANK));
    MSG_PRINTtoHEX(PatableArray[i]);
	}
	MSG_PRINTLN("");
}

void cc1101::writePatable(void) {
	cc1101_Select();                                // select CC1101
	wait_Miso();                                    // wait until MISO goes low
	sendSPI(CC1101_PATABLE | CC1101_WRITE_BURST);   // send register address
	for (uint8_t i = 0; i < 8; i++) {
		sendSPI(EEPROM.read(EE_CC1101_PA + i));     // send value
	}
	cc1101_Deselect();
}

void cc1101::readCCreg(const uint8_t reg) {         // read CC1101 register
	uint8_t var;
	uint8_t n;

	if (IB_1[3] == 'n' && isHexadecimalDigit(IB_1[4])) {   // C<reg>n<anz>  gibt anz+2 fortlaufende register zurueck, example: C06n2
		n = (uint8_t)strtol((const char*)IB_1 + 4, NULL, 16);
		if (reg < 0x2F) {
			n += 2;
      
      /*
       * sprintf(b, "C%02Xn%02X=", reg, n); // is 36 bytes bigger
       */
			MSG_PRINT('C'); MSG_PRINTtoHEX(reg); MSG_PRINT('n'); MSG_PRINTtoHEX(n); MSG_PRINT('='); // C06n04=

			for (uint8_t i = reg; i < reg + n; i++) {
				var = readReg(i, CC1101_CONFIG);
        MSG_PRINTtoHEX(var);
			}
			MSG_PRINTLN("");
		}
	} else {
		if (reg < 0x3E) {           // example: C06
			if (reg < 0x2F) {
				var = readReg(reg, CC1101_CONFIG);
			} else {
				var = readReg(reg, CC1101_STATUS);
			}

      /*
       * sprintf_P(b, PSTR("C%02X = %02X"), reg, var);  // is 16 bytes bigger
       */

      MSG_PRINT('C'); MSG_PRINTtoHEX(reg); MSG_PRINT(" = "); MSG_PRINTtoHEX(var); MSG_PRINTLN(""); // C06 = 3D
		}
		else if (reg == 0x3E) {     // patable, C3E
			MSG_PRINT(F("C3E ="));
			readPatable();
		}
		else if (reg == 0x99) {     // alle register, C99
			for (uint8_t i = 0; i < 0x2f; i++) {
				if (i == 0 || i == 0x10 || i == 0x20) {
					if (i > 0) {
						MSG_PRINT(FPSTR(TXT_BLANK));
					}
					MSG_PRINT(F("ccreg "));
          MSG_PRINTtoHEX(i);
          MSG_PRINT(F(": "));
				}
				var = readReg(i, CC1101_CONFIG);
        MSG_PRINTtoHEX(var);
        MSG_PRINT(' ');
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
		reg = (uint8_t)strtol(&IB_1[2], nullptr, 16);  // address strobe command | CC1101 - Table 42: Command Strobes
		if (reg < 0x3E) {
			val = cmdStrobe(reg);
			delay(1);
			val1 = cmdStrobe(0x3D);                 //  No operation. May be used to get access to the chip status byte.

      /*
       *  sprintf_P(b, PSTR("cmdStrobeReg %02X chipStatus %02X delay1 %02X"), reg, val >> 4, val1 >> 4); // is 110 bytes bigger
       */
			//

      MSG_PRINT(F("cmdStrobeReg ")); MSG_PRINTtoHEX(reg); MSG_PRINT(F(" chipStatus ")); MSG_PRINTtoHEX(val >> 4);
      MSG_PRINT(F(" delay1 ")); MSG_PRINTtoHEX(val1 >> 4); // cmdStrobeReg 36 chipStatus 00 delay1 00
      MSG_PRINTLN("");
		}
	}
}

void cc1101::writeCCreg(uint8_t reg, uint8_t var) { // write CC1101 register
	if (reg > 1 && reg < 0x40) {
		writeReg(reg - EE_CC1101_CFG, var);

	if (reg - EE_CC1101_CFG == 18) {
		ccmode = ( var & 0x70 ) >> 4;                // read modulation direct from 0x12 MDMCFG2
	}

    /*
     * sprintf(b,"W%02X%02X",reg,var); // is 48 bytes bigger
     */
		
    MSG_PRINT('W');
    MSG_PRINTtoHEX(reg);
    MSG_PRINTtoHEX(var);
    MSG_PRINTLN("");
	}
}

void cc1101::writeCCpatable(uint8_t var) {          // write 8 byte to patable (kein pa ramping)
	for (uint8_t i = 0; i < 8; i++) {
		if (i == 1) {
			EEPROM.write(EE_CC1101_PA + i, var);
		}
		else {
			EEPROM.write(EE_CC1101_PA + i, 0);
		}
	}
	#if defined(ESP8266) || defined(ESP32)
		EEPROM.commit();
	#endif
	writePatable();
}

uint8_t cc1101::chipVersionRev() {
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
#ifdef CMP_CC1101
	uint8_t version = chipVersion();                // Version
	#ifdef DEBUG
		uint8_t partnum = readReg((revision == 0x01 ? CC1101_PARTNUM_REV01 : CC1101_PARTNUM_REV00), CC1101_READ_SINGLE);  // Partnum
		DBG_PRINT(FPSTR(TXT_CCREVISION));	DBG_PRINT("0x"); DBG_PRINTLN(version, HEX);
		DBG_PRINT(FPSTR(TXT_CCPARTNUM)); DBG_PRINT("0x"); DBG_PRINTLN(partnum, HEX);
	#endif
	//checks if valid Chip ID is found. Usualy 0x03 or 0x14. if not -> abort
	if (version == 0x00 || version == 0xFF)
	{
		DBG_PRINT(F("no "));  DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(FPSTR(TXT_FOUND));
		return false;  // Todo: power down SPI etc
	}
#endif // CMP_CC1101
return true;
}


void cc1101::setup() {
#if !defined(ESP8266) && !defined(ESP32)
	pinAsOutput(sckPin);
	pinAsOutput(mosiPin);
	pinAsInput(misoPin);
#endif

pinAsOutput(csPin);                // set pins for SPI communication

#ifdef PIN_MARK433
	pinAsInputPullUp(PIN_MARK433);
#endif


#if !defined(ESP8266) && !defined(ESP32) && !defined(ARDUINO_MAPLEMINI_F103CB)
	SPCR = _BV(SPE) | _BV(MSTR);       // SPI speed = CLK/4
	digitalHigh(csPin);                // SPI init
	digitalHigh(sckPin);
	digitalLow(mosiPin);
#elif ARDUINO_MAPLEMINI_F103CB
	SPI_2.begin();                     // Initialize the SPI_2 port
	SPI_2.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
	digitalHigh(csPin);
#else
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
pinAsInput(PIN_RECEIVE);               // gdo2
pinAsInput(PIN_SEND);                  // gdo0Pi, sicherheitshalber bis zum CC1101 init erstmal input   

#ifndef ARDUINO_MAPLEMINI_F103CB
	digitalHigh(sckPin);
	digitalLow(mosiPin);
#endif
}

uint8_t cc1101::getRevision() 
{ return revision; }


uint8_t cc1101::getRSSI() {
	return readReg((revision == 0x01 ? CC1101_RSSI_REV01 : CC1101_RSSI_REV00), CC1101_STATUS);
}

uint8_t cc1101::getMARCSTATE() {                            // xFSK, Control state machine state
	return readReg(CC1101_MARCSTATE_REV00, CC1101_STATUS);  // xFSK, Pruefen ob Umwandung von uint to int den richtigen Wert zurueck gibt
}

uint8_t cc1101::getRXBYTES() {                             // xFSK
	return readReg(CC1101_SFTX,CC1101_STATUS);
}

bool readRXFIFO(uint8_t len) {                             // xFSK
	bool dup = true;
	uint8_t rx;

	cc1101_Select();                                       // select CC1101
	cc1101::sendSPI(CC1101_RXFIFO | CC1101_READ_BURST);    // send register address
	for (uint8_t i = 0; i < len; i++) {
		rx = cc1101::sendSPI(0x00);                        // read result
		if (rx != ccBuf[i]) {                              // if Circuit board for more cc110x -> ccBuf expand ( if (rx != ccBuf[radionr][i] ) )
			dup = false;
			ccBuf[i] = rx;                                 // if Circuit board for more cc110x -> ccBuf expand ( if (rx != ccBuf[radionr][i] = rx ) )
		}
	}
	cc1101_Deselect();
	return dup;
}

uint8_t cc1101::flushrx() {                                // xFSK, Flush the RX FIFO buffer
	if (cmdStrobeTo(CC1101_SIDLE) == false) {
		return false;
	}
	cmdStrobe(CC1101_SNOP);
	cmdStrobe(CC1101_SFRX);
	return true;
}

void cc1101::setIdleMode() {
	cmdStrobe(CC1101_SIDLE);                               // Idle mode
	delay(1);
}

uint8_t cc1101::currentMode() {
	return readReg((revision == 0x01 ? CC1101_MARCSTATE_REV01 : CC1101_MARCSTATE_REV00), CC1101_READ_BURST);
}

void cc1101::setReceiveMode()
{
	setIdleMode();
	uint8_t maxloop = 0xff;

	while (maxloop-- && (cmdStrobe(CC1101_SRX) & CC1101_STATUS_STATE_BM) != CC1101_STATE_RX) // RX enable
		delay(1);
#ifdef CMP_CC1101
  if (maxloop == 0) { 
	DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(F(": Setting RX failed")); 
	// ToDo only for test output faulty msg in FHEM
	MSG_PRINT(TXT_MU); MSG_PRINT(TXT_FSEP); MSG_PRINT(FPSTR(TXT_CC1101)); MSG_PRINT(F(": Setting RX failed"));  MSG_PRINTLN(TXT_FSEP);
   }
#endif
	pinAsInput(PIN_SEND);
}

void cc1101::setTransmitMode()
{
#ifdef CMP_CC1101
  if (cmdStrobeTo(CC1101_SFTX) == false) {  // flush TX with wait MISO timeout
    DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(F(": Setting TX failed"));
    return;
  }

	setIdleMode();
	uint8_t maxloop = 0xff;
	while (maxloop-- && (cmdStrobe(CC1101_STX) & CC1101_STATUS_STATE_BM) != CC1101_STATE_TX)  // TX enable
		delay(1);
	if (maxloop == 0) {
    DBG_PRINT(FPSTR(TXT_CC1101)); DBG_PRINTLN(F(": Setting TX failed"));
    return;
	}
	pinAsOutput(PIN_SEND);      // gdo0Pi, sicherheitshalber bis zum CC1101 init erstmal input
  return;
#endif
}


bool cc1101::regCheck()
{
	//char b[3];
	//uint8_t val;
#ifdef CMP_CC1101
	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("PKTCTRL0=")); DBG_PRINT(readReg(CC1101_PKTCTRL0, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval PKTCTRL0=")); DBG_PRINTLN(cc1101::initVal[CC1101_PKTCTRL0]);

	DBG_PRINT(FPSTR(TXT_CC1101)); 
	DBG_PRINT(F("IOCFG2=")); DBG_PRINT(readReg(CC1101_IOCFG2, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval IOCFG2=")); DBG_PRINTLN(cc1101::initVal[CC1101_IOCFG2]);
	/*
	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_FREQ0=")); DBG_PRINT(readReg(CC1101_FREQ0, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval FREQ0=")); DBG_PRINT(cc1101::initVal[CC1101_FREQ0]);
	DBG_PRINT(F(" vs EEPROM FREQ0"));
	val = EEPROM.read(EE_CC1101_CFG + CC1101_FREQ0);
	sprintf(b, " %d", val);
	DBG_PRINTLN(b);

	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_FREQ1=")); DBG_PRINT(readReg(CC1101_FREQ1, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval FREQ1=")); DBG_PRINT(cc1101::initVal[CC1101_FREQ1]);
	DBG_PRINT(F(" vs EEPROM FREQ1="));
	val = EEPROM.read(EE_CC1101_CFG + CC1101_FREQ1);
	sprintf(b, " %d", val);
	DBG_PRINTLN(b);

	DBG_PRINT(FPSTR(TXT_CC1101));
	DBG_PRINT(F("_FREQ2=")); DBG_PRINT(readReg(CC1101_FREQ2, CC1101_CONFIG));
	DBG_PRINT(F(" vs initval FREQ2=")); DBG_PRINT(cc1101::initVal[CC1101_FREQ2]);
	DBG_PRINT(F(" vs EEPROM FREQ2=")); 
	val = EEPROM.read(EE_CC1101_CFG+CC1101_FREQ2);
	sprintf(b, " %d", val);
	DBG_PRINTLN(b);
	*/
	return (readReg(CC1101_PKTCTRL0, CC1101_CONFIG) == cc1101::initVal[CC1101_PKTCTRL0]) && (readReg(CC1101_IOCFG2, CC1101_CONFIG) == cc1101::initVal[CC1101_IOCFG2]);
#else
	return ("");
#endif
}



void cc1101::ccFactoryReset() {                            // reset CC1101 and set default values
	for (uint8_t i = 0; i < sizeof(cc1101::initVal); i++) {
		EEPROM.write(EE_CC1101_CFG + i, pgm_read_byte(&initVal[i]));
		DBG_PRINT('.');
	}
	for (uint8_t i = 0; i < 8; i++) {
		if (i == 1) {
			EEPROM.write(EE_CC1101_PA + i, PATABLE_DEFAULT);
		}
		else {
			EEPROM.write(EE_CC1101_PA + i, 0);
		}
	}
	#if defined(ESP8266) || defined(ESP32)
		EEPROM.commit();
	#endif
  #ifdef CMP_CC1101
	  MSG_PRINTLN(FPSTR(TXT_CCFACTORYRESET));
  #endif
}

void cc1101::CCinit(void) {                                // initialize CC1101
#ifdef CMP_CC1101
	DBG_PRINT(FPSTR(TXT_CCINIT)); 

	cc1101_Deselect();                                     // some deselect and selects to init the cc1101
	delayMicroseconds(30);

	// Begin of power on reset
	cc1101_Select();
	delayMicroseconds(30);

	cc1101_Deselect();
	delayMicroseconds(45);

	DBG_PRINT(F("SRES started,"));
	cmdStrobe(CC1101_SRES);                                // send reset
	DBG_PRINT(F("POR done,"));
	delay(10);

	cc1101_Select();
	DBG_PRINT(FPSTR(TXT_EEPROM)); 	DBG_PRINT(FPSTR(TXT_BLANK));	DBG_PRINT(FPSTR(TXT_READ));
	wait_Miso();                                           // Wait until MISO goes low

	sendSPI(0x00 | CC1101_WRITE_BURST);
	for (uint8_t i = 0; i < sizeof(cc1101::initVal); i++) {         // write EEPROM value to cc1101
		sendSPI(EEPROM.read(EE_CC1101_CFG + i));
		DBG_PRINT('.');
	}

	cc1101_Deselect();
	delayMicroseconds(10);                                          // ### todo: welcher Wert ist als delay sinnvoll? ###

	ccmode = ( (EEPROM.read(EE_CC1101_CFG + 18) ) & 0x70 ) >> 4;    // first read modulation direct from 0x12 MDMCFG2

	writePatable();                                                 // write PatableArray to patable reg
	DBG_PRINTLN(F("done"));

	delay(1);
	setReceiveMode();

#endif
}


void cc1101::getRxFifo() {           // xFSK
	uint8_t fifoBytes;

	if (isHigh(PIN_RECEIVE)) {                     // wait for CC1100_FIFOTHR given bytes to arrive in FIFO
		#ifdef PIN_LED_INVERSE
			digitalWrite(PIN_LED, blinkLED);
		#else
			digitalWrite(PIN_LED, !blinkLED);
		#endif

/*
 * Ralf ( mode numbering == own selection )
 * cc1101 Mode: 0 - normal ASK/OOK, 1 - FIFO, 2 - FIFO ohne dup, 3 - FIFO LaCrosse, 4 - experimentell, 9 - FIFO mit Debug Ausgaben
 *
 * Sidey ( mode numbering == cc1101 data sheet setting numbering 0x12: MDMCFG2–Modem Configuration )
 * cc1101 Mode: 0 - FIFO LaCrosse, 3 - normal ASK/OOK
 * 
    if (ccmode == 4) {
      cc1101::ccStrobe_SIDLE(); // start over syncing
    }
*/

		fifoBytes = cc1101::getRXBYTES();          // & 0x7f; // read len, transfer RX fifo
		if (fifoBytes > 0) {
			uint8_t marcstate;
			uint8_t RSSI = cc1101::getRSSI();
			int8_t freqErr = cc1101::readReg(0x32, CC1101_READ_BURST); // 0x32 (0xF2): FREQEST – Frequency Offset Estimate from Demodulator
			if (AfcEnabled == 1) {
				cc1101::freqErrAvg = cc1101::freqErrAvg - float(cc1101::freqErrAvg / 8.0) + float(freqErr / 8.0);  // Mittelwert über Abweichung
				// freqErrAvg = freqErrAvg - float(freqErrAvg / 10.0) + float(freqErr / 10.0);  // Mittelwert über Abweichung
				// freqErrAvg = freqErrAvg - float(freqErrAvg / 12.0) + float(freqErr / 12.0);  // Mittelwert über Abweichung
				cc1101::freqOffAcc += round(cc1101::freqErrAvg);
				cc1101::writeReg(0x0C, cc1101::freqOffAcc); // 0x0C: FSCTRL0 – Frequency Synthesizer Control
			}
			if (fifoBytes < 0x80) { // 0x80 = 128, RXoverflow?
				if (fifoBytes > ccMaxBuf) {
					fifoBytes = ccMaxBuf; // 64 Byte
				}
				readRXFIFO(fifoBytes); // read FIFO in ccBuf[]
				#ifdef ETHERNET_PRINT
					String msg = "";
					msg.reserve(160); // 64 * 2 = 128 + 32 = 160 (32 Byte for "MN;D=;R=239;A=-255" etc.) 
					msg += char(MSG_START);
					msg += F("MN;D=");
					for (uint8_t i = 0; i < fifoBytes; i++) {
						msg += dec2hex(ccBuf[i]);
					}
					msg += F(";R=");
					msg += RSSI;
					msg += F(";A=");
					msg += freqErr;
					msg += TXT_FSEP;
					msg += char(MSG_END);
					msg += "\n";
					MSG_PRINT(msg);
				#else
					MSG_PRINT(char(MSG_START));
					MSG_PRINT(F("MN;D="));
					for (uint8_t i = 0; i < fifoBytes; i++) {
						MSG_PRINTtoHEX(ccBuf[i]);
					}
					MSG_PRINT(F(";R="));
					MSG_PRINT(RSSI);
					MSG_PRINT(F(";A="));
					MSG_PRINT(freqErr);
					MSG_PRINT(TXT_FSEP);
					MSG_PRINT(char(MSG_END));
					MSG_PRINT("\n");
				#endif
			}

			marcstate = cc1101::getMARCSTATE();
      		if (marcstate == 17 || cc1101::ccmode != 3) {   // RXoverflow oder nicht ASK/OOK
				if (cc1101::flushrx()) {                    // Flush the RX FIFO buffer
					cc1101::setReceiveMode();
				} else {
				    // ToDo only for test output faulty msg in FHEM
		 			MSG_PRINT(TXT_MU); MSG_PRINT(TXT_FSEP); MSG_PRINT("After failed flush fifobytes="); fifoBytes = cc1101::getRXBYTES(); MSG_PRINT(TXT_FSEP);
					MSG_PRINT(TXT_FSEP); MSG_PRINT("marcstate=") MSG_PRINT(marcstate); MSG_PRINTLN(marcstate);
				}
			}
		}
	}
}


void cc1101::sendFIFO(char *startpos, char *endpos) {
    cc1101_Select();                                // select CC1101
    sendSPI(CC1101_TXFIFO | CC1101_WRITE_BURST);    // send register address

    uint8_t val;
    for (char *i = startpos; i < endpos; i+=2) {
      val = hex2int(i[0]) * 16;
      val+= hex2int(i[1]);
      /* DBG_PRINTLN(val); // only to debug */
      sendSPI(val);    // send value
    }

    cc1101_Deselect();    //Wait for sending to finish (CC1101 will go to RX state automatically

    for(uint8_t i=0; i< 200;++i) {
      if( readReg(CC1101_MARCSTATE_REV00, CC1101_STATUS) != MarcStateTx)
        break;            //neither in RX nor TX, probably some error
      delay(1);
    }
}
