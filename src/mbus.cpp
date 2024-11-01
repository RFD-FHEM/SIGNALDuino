/*** copyright ***
  Originally initiated by D.Tostmann
  Inspired by code from TI.com AN
  License: GPL v2
  kaihs 2018: add support for WMBUS type C reception
  Ralf9 2022: rf_mbus.c (culfw) in mbus.h umbenannt und fuer den SIGNALDuino angepasst und erweitert
  elektron-bbs 2024:
*/

#include "compile_config.h"
#if defined(CMP_CC1101) && (defined(ESP8266) || defined(ESP32))
#include "mbus.h"

// Buffers
uint8_t MBpacket[291];
uint8_t MBbytes[584];

uint8_t mbus_mode = WMBUS_NONE;
RXinfoDescr RXinfo;
TXinfoDescr TXinfo;

void mbus_init(uint8_t wmBusMode)
{
  mbus_mode = wmBusMode;
  memset(&RXinfo, 0, sizeof(RXinfo));
#ifdef debug_mbus
  Serial.print(F("mbus_init "));
  Serial.println(mbus_mode);
#endif
}

/*
  void mbus_init_tx(void) {
  CC110x_CmdStrobe(CC110x_SIDLE); // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
  Chip_writeReg(CC110x_FIFOTHR, TX_FIFO_THRESHOLD);
  if (mbus_mode == WMBUS_SMODE) {
    // SYNC ist bei TX und RX (7696) verschieden
    // The TX FIFO must apply the last byte of the Synchronization word
    Chip_writeReg(CC110x_SYNC1, 0x54);
    Chip_writeReg(CC110x_SYNC0, 0x76);
  } else { // TMODE
    // SYNC ist bei TX und RX gleich
    // Set Deviation to 50 kHz (bei RX 0x44, 38 kHz)
    Chip_writeReg(CC110x_DEVIATN, 0x50);
    // Set data rate to 100 kbaud (bei RX 103 kbaud)
    Chip_writeReg(CC110x_MDMCFG4, 0x5B);
    Chip_writeReg(CC110x_MDMCFG3, 0xF8);
  }
  // Set GDO0 to be TX FIFO threshold signal
  Chip_writeReg(CC110x_IOCFG0, 0x02);
  // Set GDO2 to be high impedance
  Chip_writeReg(CC110x_IOCFG2, 0x2e);
  memset( &TXinfo, 0, sizeof( TXinfo ));
  }
*/

/*
  void mbus_init_tx_end(void) {
  if (mbus_mode == WMBUS_SMODE) {
    // SYNC ist bei TX und RX (7696) verschieden
    // SYNC RX
    Chip_writeReg(CC110x_SYNC1, 0x76);
    Chip_writeReg(CC110x_SYNC0, 0x96);
  } else { // TMODE
    // SYNC ist bei TX und RX gleich
    // Set Deviation to RX 0x44, 38 kHz)
    Chip_writeReg(CC110x_DEVIATN, 0x44);
    // Set data rate to RX 103 kbaud
    Chip_writeReg(CC110x_MDMCFG4, 0x5C);
    Chip_writeReg(CC110x_MDMCFG3, 0x04);
  }
  Chip_writeReg(CHIP_PKTLEN, 0xFF);
  // Set GDO0
  Chip_writeReg(CC110x_IOCFG0, 0x00);
  // Set GDO2
  Chip_writeReg(CC110x_IOCFG2, 0x06);
  memset( &RXinfo, 0, sizeof( RXinfo ));
  }
*/

static uint8_t mbus_on(uint8_t force)
{
  // already in RX?
  if (!force && cc1101::getMARCSTATE() == 0x0D)
    return 0;
  // init RX here, each time we're idle
  RXinfo.state = 0;
  cc1101::cmdStrobe(CC1101_SIDLE);
  cc1101::cmdStrobe(CC1101_SNOP);
  cc1101::cmdStrobe(CC1101_SFRX);

  // Initialize RX info variable
  RXinfo.lengthField = 0;        // Length Field in the wireless MBUS packet
  RXinfo.length = 0;             // Total length of bytes to receive packet
  RXinfo.bytesLeft = 0;          // Bytes left to to be read from the RX FIFO
  RXinfo.pByteIndex = MBbytes;   // Pointer to current position in the byte array
  RXinfo.format = INFINITE;      // Infinite or fixed packet mode
  RXinfo.mode = mbus_mode;       // Wireless MBUS radio mode
  RXinfo.framemode = WMBUS_NONE; // Received frame mode (Distinguish between C- and T-mode)
  RXinfo.frametype = 0;          // Frame A or B in C-mode

  cc1101::writeReg(0x03, RX_FIFO_START_THRESHOLD); // Set RX FIFO threshold to 4 bytes 0x03
  cc1101::writeReg(0x08, INFINITE_PACKET_LENGTH);  // Set infinite length
  cc1101::setReceiveMode();
  RXinfo.state = 1;
#ifdef debug_mbus
  Serial.print(F("mbus_on "));
  Serial.println(RXinfo.mode);
#endif
  return 1; // this will indicate we just have re-started RX
}

void mbus_task()
{
  uint8_t bytesDecoded[2];
  uint8_t fixedLength;
  if (mbus_mode == WMBUS_NONE)
  {
#ifdef debug_mbus
    Serial.println(mbus_mode);
#endif
    return;
  }
  switch (RXinfo.state)
  {
  case 0:
    mbus_on(TRUE);
    return;
  case 1:                    // RX active, awaiting SYNC
    if (isHigh(PIN_RECEIVE)) // PIN_RECEIVE, Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
    {
#ifdef debug_mbus
      Serial.println(F("mbt1 sync"));
#endif
      RXinfo.state = 2;
    }
    break;

  case 2:                 // awaiting pkt len to read
    if (isHigh(PIN_SEND)) // PIN_SEND, Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold.
    {
      mbus_readRXFIFO(RXinfo.pByteIndex, 3, NULL, NULL); // Read the 3 first bytes from FIFO
#ifdef debug_mbus
      Serial.print(F("mbt2 RX "));
      for (uint8_t x = 0; x < 3; x++) {
        if (RXinfo.pByteIndex[x] < 16) {
          Serial.print('0');
        }
        Serial.print(RXinfo.pByteIndex[x],HEX);
        Serial.print(' ');
      }
      Serial.println("");
#endif
      // - Calculate the total number of bytes to receive -
      if (RXinfo.mode == WMBUS_SMODE)
      {
        // S-Mode
        // Possible improvment: Check the return value from the deocding function, and abort RX if coding error.
        if (manchDecode(RXinfo.pByteIndex, bytesDecoded) != MAN_DECODING_OK)
        {
#ifdef debug_mbus
          Serial.println(F("mbt2 SMODE ERROR"));
#endif
          RXinfo.state = 0;
          return;
        }
        RXinfo.lengthField = bytesDecoded[0];
        RXinfo.length = byteSize(1, 0, (packetSize(RXinfo.lengthField)));
      }
      else
      { // End WMBUS_SMODE
        // In C-mode we allow receiving T-mode because they are similar. To not break any applications using T-mode,
        // we do not include results from C-mode in T-mode.
        // If T-mode preamble and sync is used, then the first data byte is either a valid 3outof6 byte or C-mode
        // signaling byte. (http://www.ti.com/lit/an/swra522d/swra522d.pdf#page=6)
        if (RXinfo.pByteIndex[0] == 0x54)
        {
#ifdef debug_mbus
          Serial.println(F("mbt2 WMBus_C"));
#endif
          RXinfo.framemode = WMBUS_CMODE;
          // If we have determined that it is a C-mode frame, we have to determine if it is Type A or B.
          // 54CD3144934470551225350867497A080000200B6E1711084B6E070000427C87
          if (RXinfo.pByteIndex[1] == 0xCD)
          {
#ifdef debug_mbus
            Serial.println(F("mbt2 WMBus_C Frame A"));
#endif
            RXinfo.frametype = WMBUS_FRAMEA;
            // Frame format A
            RXinfo.lengthField = RXinfo.pByteIndex[2];
            if (RXinfo.lengthField < 9)
            {
              RXinfo.state = 0;
              return;
            }
            // Number of CRC bytes = 2 * ceil((L-9)/16) + 2
            // Preamble + L-field + payload + CRC bytes
            RXinfo.length = 2 + 1 + RXinfo.lengthField + 2 * (2 + (RXinfo.lengthField - 10) / 16);
          }
          else if (RXinfo.pByteIndex[1] == 0x3D)
          {
#ifdef debug_mbus
            Serial.println(F("mbt2 WMBus_C Frame B"));
#endif
            RXinfo.frametype = WMBUS_FRAMEB;
            // Frame format B
            RXinfo.lengthField = RXinfo.pByteIndex[2];
            if (RXinfo.lengthField < 12 || RXinfo.lengthField == 128)
            {
              RXinfo.state = 0;
              return;
            }
            // preamble + L-field + payload
            RXinfo.length = 2 + 1 + RXinfo.lengthField;
          }
          else
          {
            // Unknown type, reset.
#ifdef debug_mbus
            Serial.println(F("mbt2 WMBus_C ERROR unknown type"));
#endif
            RXinfo.state = 0;
            return;
          }
          // T-Mode
          // Possible improvment: Check the return value from the deocding function, and abort RX if coding error.
        }
        else if (decode3outof6(RXinfo.pByteIndex, bytesDecoded, 0) != DECODING_3OUTOF6_OK)
        {
          RXinfo.state = 0;
#ifdef debug_mbus
          Serial.println(F("mbt2 T-MODE ERROR"));
#endif
          return;
        }
        else
        {
#ifdef debug_mbus
          Serial.println(F("mbt2 T-Mode OK"));
#endif
          RXinfo.framemode = WMBUS_TMODE;
          RXinfo.frametype = WMBUS_FRAMEA;
          RXinfo.lengthField = bytesDecoded[0];
          RXinfo.length = byteSize(0, 0, (packetSize(RXinfo.lengthField)));
        }
      }
      // check if incoming data will fit into buffer
      if (RXinfo.length > sizeof(MBbytes))
      {
        RXinfo.state = 0;
#ifdef debug_mbus
        Serial.print(F("mbt2 ERROR lenght too big "));
        Serial.println(RXinfo.length);
#endif
        return;
      }
#ifdef debug_mbus
      Serial.print(F("mbt2 len "));
      Serial.println(RXinfo.length);
#endif
      // we got the length: now start setup chip to receive this much data
      // - Length mode -
      // Set fixed packet length mode is less than 256 bytes
      if (RXinfo.length < (MAX_FIXED_LENGTH)) // < 256
      {
        cc1101::writeReg(0x06, (uint8_t)(RXinfo.length));
        cc1101::writeReg(0x08, FIXED_PACKET_LENGTH); // 0x08: PKTCTRL0 – Packet Automation Control, 0x00
        RXinfo.format = FIXED;
        // Infinite packet length mode is more than 255 bytes
        // Calculate the PKTLEN value
      }
      else
      {
        fixedLength = RXinfo.length % (MAX_FIXED_LENGTH);
        cc1101::writeReg(0x06, (uint8_t)(fixedLength));
      }
      RXinfo.pByteIndex += 3;
      RXinfo.bytesLeft = RXinfo.length - 3;
      // Set RX FIFO threshold to 32 bytes
      RXinfo.state = 3;
      cc1101::writeReg(0x03, RX_FIFO_THRESHOLD); // 0x03: FIFOTHR – RX FIFO and TX FIFO Thresholds, 0x07 = 32 Byte
    }
    break;

  // awaiting more data to be read
  case 3:
    if (isHigh(PIN_SEND)) // PIN_SEND, Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold.
    {
#ifdef debug_mbus
      Serial.println(F("mbt3 read FIFO"));
#endif
      // - Length mode -
      // Set fixed packet length mode is less than MAX_FIXED_LENGTH bytes
      if (((RXinfo.bytesLeft) < (MAX_FIXED_LENGTH)) && (RXinfo.format == INFINITE))
      {                                              // MAX_FIXED_LENGTH = 256
        cc1101::writeReg(0x08, FIXED_PACKET_LENGTH); // 0x08: PKTCTRL0 – Packet Automation Control, 0x00
        RXinfo.format = FIXED;
      }
      // Read out the RX FIFO
      // Do not empty the FIFO (See the CC110x or 2500 Errata Note)
      mbus_readRXFIFO(RXinfo.pByteIndex, RX_AVAILABLE_FIFO - 1, NULL, NULL); // read 31 Byte from FIFO
      if (RXinfo.bytesLeft < (RX_AVAILABLE_FIFO - 1))
      {
#ifdef debug_mbus
        Serial.print(F("mbt3 ERROR read RX FIFO bytesLeft < 31 "));
        Serial.println(RXinfo.bytesLeft);
#endif
        RXinfo.state = 0;
        return;
      }
      RXinfo.bytesLeft -= (RX_AVAILABLE_FIFO - 1);
      RXinfo.pByteIndex += (RX_AVAILABLE_FIFO - 1);
#ifdef debug_mbus
      Serial.print(F("mbt3 bLeft "));
      Serial.println(RXinfo.bytesLeft);
#endif
    }
    break;
  }

  // END OF PAKET
  if (isLow(PIN_RECEIVE) && RXinfo.state > 1) // PIN_RECEIVE, Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
  {
    uint8_t rssi = 0;
    uint8_t lqi = 0;
    mbus_readRXFIFO(RXinfo.pByteIndex, (uint8_t)RXinfo.bytesLeft, &rssi, &lqi);
    // CC110x_readFreqErr();
		int8_t freqErr = cc1101::readReg(0x32, CC1101_READ_BURST); // 0x32 (0xF2): FREQEST – Frequency Offset Estimate from Demodulator
		if (AfcEnabled == 1) {
			cc1101::freqErrAvg = cc1101::freqErrAvg - float(cc1101::freqErrAvg / 8.0) + float(freqErr / 8.0);  // Mittelwert über Abweichung
			// freqErrAvg = freqErrAvg - float(freqErrAvg / 10.0) + float(freqErr / 10.0);  // Mittelwert über Abweichung
			// freqErrAvg = freqErrAvg - float(freqErrAvg / 12.0) + float(freqErr / 12.0);  // Mittelwert über Abweichung
			cc1101::freqOffAcc += round(cc1101::freqErrAvg);
			cc1101::writeReg(0x0C, cc1101::freqOffAcc); // 0x0C: FSCTRL0 – Frequency Synthesizer Control
		}
    // RXinfo.state = 4; // decode!
#ifdef debug_mbus
    Serial.println(F("mbt3 END OF PAKET"));
#endif

    // decode!
    uint16_t rxStatus = PACKET_CODING_ERROR; // rxStatus = 1
    uint16_t rxLength;
#ifdef debug_mbus
    Serial.print(F("mbt3 RX "));
    for (uint16_t x = 0; x < RXinfo.length; x++)
    {
      if (MBbytes[x] < 16) {
        Serial.print('0');
      }
      Serial.print(MBbytes[x],HEX);
      Serial.print(' ');
    }
    Serial.println("");
#endif
    if (RXinfo.mode == WMBUS_SMODE)
    {
      rxStatus = decodeRXBytesSmode(MBbytes, MBpacket, packetSize(RXinfo.lengthField));
      rxLength = packetSize(MBpacket[0]);
#ifdef debug_mbus
      Serial.print(F("mbt S-MODE "));
      Serial.print(rxStatus);
      Serial.print(' ');
      Serial.println(rxLength);
#endif
    }
    else if (RXinfo.framemode == WMBUS_TMODE)
    {
      rxStatus = decodeRXBytesTmode(MBbytes, MBpacket, packetSize(RXinfo.lengthField));
      rxLength = packetSize(MBpacket[0]);
#ifdef debug_mbus
      Serial.print(F("mbt T_MODE "));
      Serial.print(rxStatus);
      Serial.print(' ');
      Serial.println(rxLength);
#endif
    }
    else if (RXinfo.framemode == WMBUS_CMODE)
    {
      if (RXinfo.frametype == WMBUS_FRAMEA)
      {
        rxLength = RXinfo.lengthField + 2 * (2 + (RXinfo.lengthField - 10) / 16) + 1;
        rxStatus = verifyCrcBytesCmodeA(MBbytes + 2, MBpacket, rxLength);
#ifdef debug_mbus
        Serial.print(F("mbt C-MODE FRAME A "));
        Serial.print(rxStatus);
        Serial.print(' ');
        Serial.println(rxLength);
#endif
      }
      else if (RXinfo.frametype == WMBUS_FRAMEB)
      {
        rxLength = RXinfo.lengthField + 1;
        rxStatus = verifyCrcBytesCmodeB(MBbytes + 2, MBpacket, rxLength);
#ifdef debug_mbus
        Serial.print(F("mbt C-MODE FRAME B "));
        Serial.print(rxStatus);
        Serial.print(' ');
        Serial.println(rxLength);
#endif
      }
    }
    if (rxStatus == PACKET_OK) // rxStatus == 0
    {
  		#ifdef PIN_LED_INVERSE
	  		digitalWrite(PIN_LED, blinkLED);
		  #else
			  digitalWrite(PIN_LED, !blinkLED);
		  #endif
#ifdef debug_mbus
      Serial.println(F("mbt PACKET_OK"));
#endif
      freqErr = cc1101::readReg(0x32, CC1101_STATUS); // 0x32 (0xF2): FREQEST – Frequency Offset Estimate from Demodulator
      uint8_t wmbusFrameTypeB = 0;
      if (RXinfo.framemode == WMBUS_CMODE && RXinfo.frametype == WMBUS_FRAMEB)
      {
        wmbusFrameTypeB = 1;
      }
      // msgOutput_MN(uint8_t * data, uint16_t lenData, uint8_t wmbusFrameTypeA, uint8_t lqi, uint8_t rssi, int8_t freqErr);
      // msgOutput_MN(MBpacket, rxLength, wmbusFrameTypeB, lqi, rssi, freqErr); // MN - Nachricht erstellen und ausgeben
			MSG_PRINT(char(MSG_START));
			MSG_PRINT(F("MN;D="));
      if (wmbusFrameTypeB) {
        MSG_PRINT(F("Y")); // special marker for frame type B
      }
      for (uint16_t i = 0; i < rxLength; i++) {
        MSG_PRINTtoHEX(MBpacket[i]);
      }
      MSG_PRINTtoHEX(lqi);
      MSG_PRINTtoHEX(rssi);
			MSG_PRINT(F(";R="));
			MSG_PRINT(rssi);
			MSG_PRINT(F(";A="));
			MSG_PRINT(freqErr);
			MSG_PRINT(';');
			MSG_PRINT(char(MSG_END));
			MSG_PRINT("\n");
    }
    RXinfo.state = 0;
    return;
  }
  mbus_on(FALSE);
}

/* TODO
  uint8_t txSendPacket(uint8_t* pPacket, uint8_t* pBytes, uint16_t rawlen, uint8_t cmode, bool debug) {
  uint16_t  bytesToWrite;
  uint16_t  fixedLength;
  uint8_t   txStatus;
  int8_t    retstate = 0;
  // uint8_t   lastMode = WMBUS_NONE; // TODO unused variable
  uint16_t  packetLength;
  uint16_t  TXn;
  #ifdef CODE_ESP
  String tmp = "";        // for temp outputs print
  //  tmp.reserve(256);
  #endif

  // Calculate total number of bytes in the wireless MBUS packet
  packetLength = packetSize(pPacket[0]);

  if (debug == true) {
    MSG_BUILD(F("packetlen="));
    MSG_BUILD(packetLength);
    MSG_BUILD(F(" rawlen="));
    MSG_BUILD_LF(rawlen);
  }
  // Check for valid length
  if ((packetLength == 0) || (packetLength > rawlen) || (packetLength > 290))
    return TX_LENGTH_ERROR;

  mbus_init_tx();

  // Data encode packet and calculate number of bytes to transmit
  // S-mode
  if (mbus_mode == WMBUS_SMODE) {
    encodeTXBytesSmode(pBytes, pPacket, packetLength);
    TXinfo.bytesLeft = byteSize(1, 1, packetLength);
    // T-mode
  } else {
    encodeTXBytesTmode(pBytes, pPacket, packetLength);
    TXinfo.bytesLeft = byteSize(0, 1, packetLength);
  }

  if (debug == true) {
    MSG_BUILD(F("TXn="));
    MSG_BUILD_LF(TXinfo.bytesLeft);
    for (uint16_t ii = 0; ii < TXinfo.bytesLeft; ii++) {
      MSG_BUILD(onlyDecToHex2Digit(pBytes[ii]));
    }
    MSG_BUILD_LF("");
  }

  // Check TX Status
  txStatus = CC110x_CmdStrobe(CC110x_SNOP);
  if ( (txStatus & CC110x_STATUS_STATE_BM) != CC110x_STATE_IDLE ) {
    CC110x_CmdStrobe(CC110x_SIDLE); // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
    return TX_TO_IDLE1_ERROR;
  }

  // Flush TX FIFO
  // Ensure that FIFO is empty before transmit is started
  if (CC110x_cmdStrobeTo(CC110x_SFTX) == false) {  // flush TX with wait MISO timeout
    return TX_FLUSH_ERROR;
  }

  TXn = TXinfo.bytesLeft;
  // Initialize the TXinfo struct.
  TXinfo.pByteIndex   = pBytes;
  TXinfo.complete     = FALSE;

  // Set fixed packet length mode if less than 256 bytes to transmit
  if (TXinfo.bytesLeft < (MAX_FIXED_LENGTH) ) {
    fixedLength = TXinfo.bytesLeft;
    Chip_writeReg(CHIP_PKTLEN, (uint8_t)(TXinfo.bytesLeft));
    Chip_writeReg(CC110x_PKTCTRL0, FIXED_PACKET_LENGTH);
    TXinfo.format = FIXED;
  }
  // Else set infinite length mode
  else {
    fixedLength = TXinfo.bytesLeft % (MAX_FIXED_LENGTH);
    Chip_writeReg(CHIP_PKTLEN, (uint8_t)fixedLength);
    Chip_writeReg(CC110x_PKTCTRL0, INFINITE_PACKET_LENGTH);
    TXinfo.format = INFINITE;
  }

  // Fill TX FIFO
  bytesToWrite = MIN(TX_FIFO_SIZE, TXinfo.bytesLeft);
  CC110x_writeBurstReg(TXinfo.pByteIndex, CC110x_TXFIFO, bytesToWrite);
  // cc1101::WriteFifo(TXinfo.pByteIndex, bytesToWrite);
  TXinfo.pByteIndex += bytesToWrite;
  TXinfo.bytesLeft  -= bytesToWrite;

  //MSG_BUILD(F("bytesLeft2="));
  //MSG_BUILD_LF(TXinfo.bytesLeft);
  // Check for completion
  if (!TXinfo.bytesLeft)
    TXinfo.complete = TRUE;

  // Strobe TX
  CC110x_CmdStrobe(CC110x_STX);

  // Wait for available space in FIFO
  //uint8_t iw = 0;
  while (!TXinfo.complete) {
    if (digitalReadFast(GDO0) == 0) {
      // if (isLow(pinSend[radionr])) {
      //txStatus = cc1101::readReg(CC110x_TXBYTES,CC110x_STATUS);
      // Write data fragment to TX FIFO
      bytesToWrite = MIN(TX_AVAILABLE_FIFO, TXinfo.bytesLeft);
      txStatus = bytesToWrite;
      CC110x_writeBurstReg(TXinfo.pByteIndex, CC110x_TXFIFO, bytesToWrite);
      // cc1101::WriteFifo(TXinfo.pByteIndex, bytesToWrite);
      //iw++;
      TXinfo.pByteIndex   += bytesToWrite;
      TXinfo.bytesLeft    -= bytesToWrite;

      if (debug == true) {
        MSG_BUILD_LF(txStatus);
      }
      // Indicate complete when all bytes are written to TX FIFO
      if (!TXinfo.bytesLeft)
        TXinfo.complete = TRUE;

      // Set Fixed length mode if less than 256 left to transmit
      if ((TXinfo.bytesLeft < (MAX_FIXED_LENGTH - TX_FIFO_SIZE)) && (TXinfo.format == INFINITE)) {
        Chip_writeReg(CC110x_PKTCTRL0, FIXED_PACKET_LENGTH);
        TXinfo.format = FIXED;
      }
    } else {
      // Check TX Status
      txStatus = CC110x_CmdStrobe(CC110x_SNOP);
      if ( (txStatus & CC110x_STATUS_STATE_BM) == CC110x_STATE_TX_UNDERFLOW ) {
        CC110x_CmdStrobe(CC110x_SFTX);
        retstate &= TX_UNDERFLOW_ERROR;
        break;
      }
    }
  }

  uint8_t maxloop = 0xff;
  while (maxloop-- && (Chip_readReg(CC110x_MARCSTATE, READ_BURST) != MARCSTATE_IDLE))
    delay(1);
  if (maxloop == 0)
    retstate &= TX_TO_IDLE2_ERROR;

  if (retstate == 0) {
    if (cmdstring.length() <= maxSendEcho) {
      MSG_BUILD(cmdstring); // echo
    } else {
      MSG_BUILD(cmdstring.substring(0, maxSendEcho)); // echo
      MSG_BUILD(F(".. "));
    }
    MSG_BUILD(F("TXn="));
    MSG_BUILD_LF(TXn);
  }
  return (retstate);
  }
*/

/* TODO
  void mbus_send(int8_t startdata) {
    int16_t enddata;
    uint8_t val;
    uint16_t pos;
    uint8_t retstate = 0;
    uint16_t i = 0;
    bool debug = false;
  #ifdef CODE_ESP
  String tmp = "";        // for temp outputs print
  //  tmp.reserve(256);
  #endif

    enddata = cmdstring.indexOf(";",startdata);   // search next   ";"
    if (enddata - startdata > 4) {
        if (cmdstring.charAt(startdata+1) == 'd') {
            debug = true;
        }
        for (pos = startdata+3; pos<enddata; pos+=2) {
            if (!isHexadecimalDigit(cmdstring.charAt(pos)) || !isHexadecimalDigit(cmdstring.charAt(pos+1))) {
                retstate = TX_NO_HEX_ERROR;
                break;
            }
            val = tools::cmdstringPos2int(pos);
            #ifdef debug_mbus
            MSG_BUILD(onlyDecToHex2Digit(val);
            #endif
            MBpacket[i] = val;
            i++;
        }
        //MSG_BUILD(" radionr=");
        //MSG_BUILD(radionr);
        if (debug == true) {
            MSG_BUILD_LF("");
        }
        if (retstate == 0) {
            char mode = cmdstring.charAt(startdata+2);
            if (mode == 's' || mode == 't') {
                retstate = txSendPacket(MBpacket, MBbytes, i, 0, debug);
                // re-enable RX
                mbus_init_tx_end();
            //} else if (mode == 'c') {               to do: send cmode
            //    retstate = txSendPacket(MBpacket, MBbytes, i, 1, debug);
            //    MSG_BUILD_LF("cmode");
            } else {
                retstate = TX_UNKNOWN_MODE_ERROR;
            }
        }
    } else {
        retstate = TX_TOO_SHORT_OR_NOEND_ERROR;
    }
    if (retstate > 0) {
        MSG_BUILD(F("wmbus send failed! ret="));
        MSG_BUILD_LF(retstate);
    }
  }
*/

void mbus_readRXFIFO(uint8_t *data, uint8_t length, uint8_t *rssi, uint8_t *lqi) // WMBus
{
  cc1101_Select();                                    // select CC1101
  cc1101::sendSPI(CC1101_RXFIFO | CC1101_READ_BURST); // send register address
  for (uint8_t i = 0; i < length; i++)
    data[i] = SPI.transfer(0); // read result
  // Optionally, two status bytes (see Table 27 and Table 28) with RSSI value, Link Quality Indication, and CRC status can be appended in the RX FIFO.
  if (rssi)
  {
    *rssi = SPI.transfer(0);
    if (lqi)
    {
      *lqi = SPI.transfer(0) & 0x7F;
    }
  }
  cc1101_Deselect();
}

/******************************************************************************
    Filename: mbus_packet.c
******************************************************************************/

//----------------------------------------------------------------------------------
//  Functions
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//  uint16 packetSize (uint8 lField)
//
//  DESCRIPTION:
//    Returns the number of bytes in a Wireless MBUS packet from
//    the L-field. Note that the L-field excludes the L-field and the
//    CRC fields
//
//  ARGUMENTS:
//    uint8 lField  - The L-field value in a Wireless MBUS packet
//
//  RETURNS
//    uint16        - The number of bytes in a wireless MBUS packet
//----------------------------------------------------------------------------------

uint16_t packetSize(uint8_t lField)
{
  uint16_t nrBytes;
  uint8_t nrBlocks;

  // The 2 first blocks contains 25 bytes when excluding CRC and the L-field
  // The other blocks contains 16 bytes when excluding the CRC-fields
  // Less than 26 (15 + 10)
  if (lField < 26)
    nrBlocks = 2;
  else
    nrBlocks = (((lField - 26) / 16) + 3);

  // Add all extra fields, excluding the CRC fields
  nrBytes = lField + 1;

  // Add the CRC fields, each block is contains 2 CRC bytes
  nrBytes += (2 * nrBlocks);

  return (nrBytes);
}

//----------------------------------------------------------------------------------
//  uint16 byteSize (uint8 Smode, uint8 transmit, uint16 packetSize)
//
//  DESCRIPTION:
//    Returns the total number of encoded bytes to receive or transmit, given the
//    total number of bytes in a Wireless MBUS packet.
//    In receive mode the postamble sequence and synchronization word is excluded
//    from the calculation.
//
//  ARGUMENTS:
//    uint8   Smode       - S-mode or T-mode
//    uint8   transmit    - Transmit or receive
//    uint16  packetSize  - Total number of bytes in the wireless MBUS packet
//
//  RETURNS
//    uint16  - The number of bytes of the encoded WMBUS packet
//----------------------------------------------------------------------------------
uint16_t byteSize(uint8_t Smode, uint8_t transmit, uint16_t packetSize)
{
  uint16_t tmodeVar;

  // S-mode, data is Manchester coded
  if (Smode)
  {
    // Transmit mode
    // 1 byte for postamble and 1 byte synchronization word
    if (transmit)
      return (2 * packetSize + 2);

    // Receive mode
    else
      return (2 * packetSize);
  }

  // T-mode
  // Data is 3 out of 6 coded
  else
  {
    tmodeVar = (3 * packetSize) / 2;

    // Transmit mode
    // + 1 byte for the postamble sequence
    if (transmit)
      return (tmodeVar + 1);

    // Receive mode
    // If packetsize is a odd number 1 extra byte
    // that includes the 4-postamble sequence must be
    // read.
    else
    {
      if (packetSize % 2)
        return (tmodeVar + 1);
      else
        return (tmodeVar);
    }
  }
}

//----------------------------------------------------------------------------------
//  void encodeTXPacket(uint8_t* pPacket, uint8_t* pData, uint8 dataSize)
//
//  DESCRIPTION:
//    Encode n data bytes into a Wireless MBUS packet format.
//    The function will add all the control field, calculates and inserts the
//    the CRC fields
//
//   ARGUMENTS:
//    uint8 *pPacket    - Pointer to the WMBUS packet byte table
//    uint8 *pData      - Pointer to user data byte table
//    uint8  dataSize   - Number of user data bytes. Max size is 245
//----------------------------------------------------------------------------------
void encodeTXPacket(uint8_t *pPacket, uint8_t *pData, uint8_t dataSize)
{

  uint8_t loopCnt;
  uint8_t dataRemaining;
  uint8_t dataEncoded;
  uint16_t crc;

  dataRemaining = dataSize;
  dataEncoded = 0;
  crc = 0;

  // **** Block 1 *****

  // - L-Field -
  // The length field excludes all CRC-fields and the L-field,
  // e.g. L = dataSize + 10
  *pPacket = dataSize + 10;
  crc = crcCalc(crc, *pPacket);
  pPacket++;

  // - C-Field -
  *(pPacket) = PACKET_C_FIELD;
  crc = crcCalc(crc, *pPacket);
  pPacket++;

  // - M-Field -
  *(pPacket) = (uint8_t)MAN_CODE;
  crc = crcCalc(crc, *pPacket);
  pPacket++;
  *(pPacket) = (uint8_t)(MAN_CODE >> 8);
  crc = crcCalc(crc, *pPacket);
  pPacket++;

  // - A-Field -
  *(pPacket) = (uint8_t)MAN_NUMBER;
  crc = crcCalc(crc, *pPacket);
  pPacket++;
  *(pPacket) = (uint8_t)(MAN_NUMBER >> 8);
  crc = crcCalc(crc, *pPacket);
  pPacket++;
  *(pPacket) = (uint8_t)(MAN_NUMBER >> 16);
  crc = crcCalc(crc, *pPacket);
  pPacket++;
  *(pPacket) = (uint8_t)(MAN_NUMBER >> 24);
  crc = crcCalc(crc, *pPacket);
  pPacket++;
  *(pPacket) = (uint8_t)(MAN_ID);
  crc = crcCalc(crc, *pPacket);
  pPacket++;
  *(pPacket) = (uint8_t)(MAN_VER);
  crc = crcCalc(crc, *pPacket);
  pPacket++;

  // - CRC -
  *(pPacket) = HI_UINT16(~crc);
  pPacket++;
  *(pPacket) = LO_UINT16(~crc);
  pPacket++;
  crc = 0;

  // **** Block 2 *****

  // - CI-Field -
  *(pPacket) = PACKET_CI_FIELD;
  crc = crcCalc(crc, *pPacket);
  pPacket++;

  // Check if last Block
  if (dataRemaining < 16)
  {
    // Data Fields
    for (loopCnt = 0; loopCnt < dataRemaining; loopCnt = loopCnt + 1)
    {
      *(pPacket) = pData[dataEncoded];
      crc = crcCalc(crc, *pPacket);
      pPacket++;
      dataEncoded++;
    }

    // CRC
    *(pPacket) = HI_UINT16(~crc);
    pPacket++;
    *(pPacket) = LO_UINT16(~crc);
    pPacket++;
    crc = 0;
    dataRemaining = 0;
  }

  else
  {
    // Data Fields
    for (loopCnt = 0; loopCnt < 15; loopCnt = loopCnt + 1)
    {
      *(pPacket) = pData[dataEncoded];
      crc = crcCalc(crc, *pPacket);
      pPacket++;
      dataEncoded++;
    }

    *(pPacket) = HI_UINT16(~crc);
    pPacket++;
    *(pPacket) = LO_UINT16(~crc);
    pPacket++;
    crc = 0;
    dataRemaining -= 15;
  }

  // **** Block n *****
  while (dataRemaining)
  {
    // Check if last Block
    if (dataRemaining < 17)
    {
      // Data Fields
      for (loopCnt = 0; loopCnt < dataRemaining; loopCnt = loopCnt + 1)
      {
        *(pPacket) = pData[dataEncoded];
        crc = crcCalc(crc, *pPacket);
        pPacket++;
        dataEncoded++;
      }

      // CRC
      *(pPacket) = HI_UINT16(~crc);
      pPacket++;
      *(pPacket) = LO_UINT16(~crc);
      pPacket++;
      crc = 0;
      dataRemaining = 0;
    }

    else
    {
      // Data Fields
      for (loopCnt = 0; loopCnt < 16; loopCnt = loopCnt + 1)
      {
        *(pPacket) = pData[dataEncoded];
        crc = crcCalc(crc, *pPacket);
        pPacket++;
        dataEncoded++;
      }

      // CRC
      *(pPacket) = HI_UINT16(~crc);
      pPacket++;
      *(pPacket) = LO_UINT16(~crc);
      pPacket++;
      crc = 0;
      dataRemaining -= 16;
    }
  }
}

//----------------------------------------------------------------------------------
//  void encodeTXBytesSmode(uint8_t* pByte, uint8_t* pPacket, uint16 packetSize)
//
//  DESCRIPTION:
//    Encodes a wireless MBUS packet into a SMODE packet. This includes
//    - Add Least Significant Byte of synchronization word to the TX array
//    - Manchester encode the Wireless MBUS packet.
//    - Add postamble sequence to the TX array
//
//   ARGUMENTS:
//    uint8_t* pByte        - Pointer to SMODE packet to transmit
//    uint8_t* pPacket      - Pointer to Wireless MBUS packet
//    uint16 packetSize   - Total Size of the uncoded Wireless MBUS packet
//
//   RETURNS
//    uint16              - Total size of bytes to transmit
//----------------------------------------------------------------------------------

void encodeTXBytesSmode(uint8_t *pByte, uint8_t *pPacket, uint16_t packetSize)
{
  uint16_t bytesEncoded;

  bytesEncoded = 0;

  // Last byte of synchronization word
  (*(pByte)) = 0x96;
  pByte++;

  // Manchester encode packet
  while (bytesEncoded < packetSize)
  {
    manchEncode((pPacket + bytesEncoded), (pByte + 2 * bytesEncoded));
    bytesEncoded++;
  }

  // Append the postamble sequence
  (*(pByte + 2 * bytesEncoded)) = 0x55;
}

//----------------------------------------------------------------------------------
//  void encodeTXBytesTmode(uint8_t* pByte, uint8_t* pPacket, uint16 packetSize)
//
//  DESCRIPTION:
//    Encodes a wireless MBUS packet into a TMODE packet. This includes
//    - 3 out of 6 encode the Wireless MBUS packet.
//    - Append postamble sequence to the TX array
//
//   ARGUMENTS:
//    uint8_t* pByte        - Pointer to TMODE packet
//    uint8_t* pPacket      - Pointer to Wireless MBUS packet
//    uint16 packetSize   - Total size of the Wireless MBUS packet
//----------------------------------------------------------------------------------
void encodeTXBytesTmode(uint8_t *pByte, uint8_t *pPacket, uint16_t packetSize)
{
  uint16_t bytesRemaining;

  bytesRemaining = packetSize;

  // 3 our of 6 encode packet
  while (bytesRemaining)
  {
    // If 1 byte left to encode, include
    // Postamble in "3 out of 6" encoding routine
    if (bytesRemaining == 1)
    {
      encode3outof6(pPacket, pByte, 1);
      bytesRemaining -= 1;
    }

    // Else if 2 byte left to encode, append Postamble
    else if (bytesRemaining == 2)
    {
      encode3outof6(pPacket, pByte, 0);

      // Append postamble
      pByte += 3;
      *pByte = 0x55;
      bytesRemaining -= 2;
    }
    else
    {
      encode3outof6(pPacket, pByte, 0);
      pByte += 3;
      pPacket += 2;
      bytesRemaining -= 2;
    }
  }
}

//----------------------------------------------------------------------------------
//  uint16 decodeRXBytesSmode(uint8_t* pByte, uint8_t* pPacket, uint16 packetSize)
//
//  DESCRIPTION:
//    Decode a SMODE packet into a Wireless MBUS packet. Checks for 3 out of 6
//    decoding errors and CRC errors.
//
//   ARGUMENTS:
//    uint8_t *pByte        - Pointer to SMBUS packet
//    uint8_t *pPacket      - Pointer to Wireless MBUS packet
//    uint16 packetSize   - Total Size of the Wireless MBUS packet
//
//    RETURNS:
//    PACKET_OK              0
//    PACKET_CODING_ERROR    1
//    PACKET_CRC_ERROR       2
//----------------------------------------------------------------------------------
uint16_t decodeRXBytesSmode(uint8_t *pByte, uint8_t *pPacket, uint16_t packetSize)
{
  uint16_t bytesRemaining;
  uint16_t bytesEncoded;
  uint16_t decodingStatus;
  uint16_t crc;       // Current CRC
  uint16_t crcField1; // Current byte is CRC high byte
  uint16_t crcField0; // Current byte is CRC low byte

  bytesRemaining = packetSize;
  bytesEncoded = 0;
  crcField1 = 0;
  crcField0 = 0;
  crc = 0;

  // Decode packet
  while (bytesRemaining)
  {
    decodingStatus = manchDecode(pByte, pPacket);

    // Check for valid Manchester decoding
    if (decodingStatus != MAN_DECODING_OK)
      return (PACKET_CODING_ERROR);

    // Check if current field is CRC field number 1, e.g.
    // - Field 10 + 18*n
    // - Less than 2 bytes
    if (bytesRemaining == 2)
      crcField1 = 1;

    else if (bytesEncoded > 9)
      crcField1 = !((bytesEncoded - 10) % 18);

    // If CRC field number 0, check the low byte of the CRC
    if (crcField0)
    {
      if (LO_UINT16(~crc) != *pPacket)
        return (PACKET_CRC_ERROR);

      crcField0 = 0;
      crc = 0;
    }

    // If CRC field number 1, check the high byte of the CRC
    else if (crcField1)
    {
      if (HI_UINT16(~crc) != *pPacket)
        return (PACKET_CRC_ERROR);

      // Next field is CRC field 1
      crcField0 = 1;
      crcField1 = 0;
    }

    // If not a CRC field, increment CRC calculation
    else
      crc = crcCalc(crc, *pPacket);

    bytesRemaining--;
    bytesEncoded++;
    pByte += 2;
    pPacket++;
  }
  return (PACKET_OK);
}

//----------------------------------------------------------------------------------
//  uint16 decodeRXBytesTmode(uint8_t* pByte, uint8_t* pPacket, uint16 packetSize)
//
//  DESCRIPTION:
//    Decode a TMODE packet into a Wireless MBUS packet. Checks for 3 out of 6
//    decoding errors and CRC errors.
//
//   ARGUMENTS:
//    uint8_t *pByte        - Pointer to TMBUS packet
//    uint8_t *pPacket      - Pointer to Wireless MBUS packet
//    uint16 packetSize   - Total Size of the Wireless MBUS packet
//
//   RETURNS:
//    PACKET_OK              0
//    PACKET_CODING_ERROR    1
//    PACKET_CRC_ERROR       2
//----------------------------------------------------------------------------------
uint16_t decodeRXBytesTmode(uint8_t *pByte, uint8_t *pPacket, uint16_t packetSize)
{

  uint16_t bytesRemaining;
  uint16_t bytesEncoded;
  uint16_t decodingStatus;
  uint16_t crc;      // Current CRC value
  uint16_t crcField; // Current fields are a CRC field

  bytesRemaining = packetSize;
  bytesEncoded = 0;
  crcField = 0;
  crc = 0;

  // Decode packet
  while (bytesRemaining)
  {
    // If last byte
    if (bytesRemaining == 1)
    {
      decodingStatus = decode3outof6(pByte, pPacket, 1);

      // Check for valid 3 out of 6 decoding
      if (decodingStatus != DECODING_3OUTOF6_OK)
        return (PACKET_CODING_ERROR);

      bytesRemaining -= 1;
      bytesEncoded += 1;

      // The last byte the low byte of the CRC field
      if (LO_UINT16(~crc) != *(pPacket))
        return (PACKET_CRC_ERROR);
    }

    else
    {

      decodingStatus = decode3outof6(pByte, pPacket, 0);

      // Check for valid 3 out of 6 decoding
      if (decodingStatus != DECODING_3OUTOF6_OK)
        return (PACKET_CODING_ERROR);

      bytesRemaining -= 2;
      bytesEncoded += 2;

      // Check if current field is CRC fields
      // - Field 10 + 18*n
      // - Less than 2 bytes
      if (bytesRemaining == 0)
        crcField = 1;
      else if (bytesEncoded > 10)
        crcField = !((bytesEncoded - 12) % 18);

      // Check CRC field
      if (crcField)
      {
        if (LO_UINT16(~crc) != *(pPacket + 1))
          return (PACKET_CRC_ERROR);
        if (HI_UINT16(~crc) != *pPacket)
          return (PACKET_CRC_ERROR);

        crcField = 0;
        crc = 0;
      }

      // If 1 bytes left, the field is the high byte of the CRC
      else if (bytesRemaining == 1)
      {
        crc = crcCalc(crc, *(pPacket));
        // The packet byte is a CRC-field
        if (HI_UINT16(~crc) != *(pPacket + 1))
          return (PACKET_CRC_ERROR);
      }

      // Perform CRC calculation
      else
      {
        crc = crcCalc(crc, *(pPacket));
        crc = crcCalc(crc, *(pPacket + 1));
      }

      pByte += 3;
      pPacket += 2;
    }
  }

  return (PACKET_OK);
}

uint16_t verifyCrcBytesCmodeA(uint8_t *pByte, uint8_t *pPacket, uint16_t packetSize)
{
  uint16_t crc = 0;
  uint16_t i = 0;

  while (i < 10)
  {
    crc = crcCalc(crc, pByte[i]);
    pPacket[i] = pByte[i];
    ++i;
  }

  if ((uint16_t)(~crc) != (pByte[i] << 8 | pByte[i + 1]))
  {
    return (PACKET_CRC_ERROR);
  }

  pPacket[i] = pByte[i];
  ++i;
  pPacket[i] = pByte[i];
  ++i;
  crc = 0;

  int cycles = (packetSize - 12) / 18;
  while (cycles > 0)
  {
    for (int j = 0; j < 16; ++j)
    {
      crc = crcCalc(crc, pByte[i]);
      pPacket[i] = pByte[i];
      ++i;
    }

    if ((uint16_t)(~crc) != (pByte[i] << 8 | pByte[i + 1]))
    {
      return (PACKET_CRC_ERROR);
    }

    pPacket[i] = pByte[i];
    ++i;
    pPacket[i] = pByte[i];
    ++i;
    crc = 0;

    --cycles;
  }

  if (i == packetSize)
  {
    return (PACKET_OK);
  }

  while (i < packetSize - 2)
  {
    crc = crcCalc(crc, pByte[i]);
    pPacket[i] = pByte[i];
    ++i;
  }

  if ((uint16_t)(~crc) != (pByte[i] << 8 | pByte[i + 1]))
  {
    return (PACKET_CRC_ERROR);
  }

  pPacket[i] = pByte[i];
  ++i;
  pPacket[i] = pByte[i];
  ++i;

  return (PACKET_OK);
}

uint16_t verifyCrcBytesCmodeB(uint8_t *pByte, uint8_t *pPacket, uint16_t packetSize)
{
  uint16_t crc = 0;
  uint16_t i = 0;
  if (packetSize > 128)
  {
    while (i < 126)
    {
      crc = crcCalc(crc, pByte[i]);
      pPacket[i] = pByte[i];
      ++i;
    }

    if ((uint16_t)(~crc) != (pByte[i] << 8 | pByte[i + 1]))
    {
      return (PACKET_CRC_ERROR);
    }

    pPacket[i] = pByte[i];
    ++i;
    pPacket[i] = pByte[i];
    ++i;
    crc = 0;
  }

  while (i < packetSize - 2)
  {
    crc = crcCalc(crc, pByte[i]);
    pPacket[i] = pByte[i];
    ++i;
  }

  if ((uint16_t)(~crc) != (pByte[i] << 8 | pByte[i + 1]))
  {
    return (PACKET_CRC_ERROR);
  }

  pPacket[packetSize - 2] = pByte[packetSize - 2];
  pPacket[packetSize - 1] = pByte[packetSize - 1];

  return (PACKET_OK);
}

/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

/******************************************************************************
    File name: mbus_manchester.c
******************************************************************************/

//----------------------------------------------------------------------------
// Variables
//----------------------------------------------------------------------------

// Table for encoding 4-bit data into a 8-bit Manchester encoding.
static uint8_t manchEncodeTab[16] = {0xAA,  // 0x0 Manchester encoded
                                     0xA9,  // 0x1 Manchester encoded
                                     0xA6,  // 0x2 Manchester encoded
                                     0xA5,  // 0x3 Manchester encoded
                                     0x9A,  // 0x4 Manchester encoded
                                     0x99,  // 0x5 Manchester encoded
                                     0x96,  // 0x6 Manchester encoded
                                     0x95,  // 0x7 Manchester encoded
                                     0x6A,  // 0x8 Manchester encoded
                                     0x69,  // 0x9 Manchester encoded
                                     0x66,  // 0xA Manchester encoded
                                     0x65,  // 0xB Manchester encoded
                                     0x5A,  // 0xC Manchester encoded
                                     0x59,  // 0xD Manchester encoded
                                     0x56,  // 0xE Manchester encoded
                                     0x55}; // 0xF Manchester encoded

// Table for decoding 4-bit Manchester encoded data into 2-bit
// data. 0xFF indicates invalid Manchester encoding
static uint8_t manchDecodeTab[16] = {0xFF,  //  Manchester encoded 0x0 decoded
                                     0xFF,  //  Manchester encoded 0x1 decoded
                                     0xFF,  //  Manchester encoded 0x2 decoded
                                     0xFF,  //  Manchester encoded 0x3 decoded
                                     0xFF,  //  Manchester encoded 0x4 decoded
                                     0x03,  //  Manchester encoded 0x5 decoded
                                     0x02,  //  Manchester encoded 0x6 decoded
                                     0xFF,  //  Manchester encoded 0x7 decoded
                                     0xFF,  //  Manchester encoded 0x8 decoded
                                     0x01,  //  Manchester encoded 0x9 decoded
                                     0x00,  //  Manchester encoded 0xA decoded
                                     0xFF,  //  Manchester encoded 0xB decoded
                                     0xFF,  //  Manchester encoded 0xC decoded
                                     0xFF,  //  Manchester encoded 0xD decoded
                                     0xFF,  //  Manchester encoded 0xE decoded
                                     0xFF}; //  Manchester encoded 0xF decoded

//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  void manchEncode(uint8_t *uncodedData, uint8_t *encodedData)
//
//  DESCRIPTION:
//    Perfoms Manchester coding on 8-bit data
//
//  ARGUMENTS:
//        uint8_t *uncodedData    - Pointer to data
//        uint8_t *encodedData    - Pointer to store the encoded data
//----------------------------------------------------------------------------

void manchEncode(uint8_t *uncodedData, uint8_t *encodedData)
{
  uint8_t data0, data1;

  // - Shift to get 4-bit data values
  data1 = (((*uncodedData) >> 4) & 0x0F);
  data0 = ((*uncodedData) & 0x0F);

  // - Perform Manchester encoding -
  *encodedData = (manchEncodeTab[data1]);
  *(encodedData + 1) = manchEncodeTab[data0];
}

//----------------------------------------------------------------------------
//  uint8_t manchDecode(uint8_t *encodedData, uint8_t *decodedData)
//
//  DESCRIPTION:
//    Perfoms Manchester decoding on 16-bit data
//
//  ARGUMENTS:
//        uint8_t *encodedData    - Pointer to encoded data
//        uint8_t *decodedData    - Pointer to store the decoded data
//
//  RETURNS
//        MAN_DECODING_OK      0
//        MAN_DECODING_ERROR   1
//----------------------------------------------------------------------------

uint8_t manchDecode(uint8_t *encodedData, uint8_t *decodedData)
{
  uint8_t data0, data1, data2, data3;

  // - Shift to get 4 bit data and decode
  data3 = ((*encodedData >> 4) & 0x0F);
  data2 = (*encodedData & 0x0F);
  data1 = ((*(encodedData + 1) >> 4) & 0x0F);
  data0 = ((*(encodedData + 1)) & 0x0F);

  // Check for invalid Manchester encoding
  if ((manchDecodeTab[data3] == 0xFF) | (manchDecodeTab[data2] == 0xFF) |
      (manchDecodeTab[data1] == 0xFF) | (manchDecodeTab[data0] == 0xFF))
  {
    return (MAN_DECODING_ERROR);
  }

  // Shift result into a byte
  *decodedData = (manchDecodeTab[data3] << 6) | (manchDecodeTab[data2] << 4) |
                 (manchDecodeTab[data1] << 2) | manchDecodeTab[data0];

  return (MAN_DECODING_OK);
}

/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

/******************************************************************************
    File name: mbus_3outof6.c
******************************************************************************/

//----------------------------------------------------------------------------
// Variables
//----------------------------------------------------------------------------

// Table for encoding for a 4-bit data into 6-bit
// "3 out of 6" coded data.
static uint8_t encodeTab[16] = {0x16,  // 0x0 "3 out of 6" encoded
                                0x0D,  // 0x1 "3 out of 6" encoded
                                0x0E,  // 0x2 "3 out of 6" encoded
                                0x0B,  // 0x3 "3 out of 6" encoded
                                0x1C,  // 0x4 "3 out of 6" encoded
                                0x19,  // 0x5 "3 out of 6" encoded
                                0x1A,  // 0x6 "3 out of 6" encoded
                                0x13,  // 0x7 "3 out of 6" encoded
                                0x2C,  // 0x8 "3 out of 6" encoded
                                0x25,  // 0x9 "3 out of 6" encoded
                                0x26,  // 0xA "3 out of 6" encoded
                                0x23,  // 0xB "3 out of 6" encoded
                                0x34,  // 0xC "3 out of 6" encoded
                                0x31,  // 0xD "3 out of 6" encoded
                                0x32,  // 0xE "3 out of 6" encoded
                                0x29}; // 0xF "3 out of 6" encoded

// Table for decoding a 6-bit "3 out of 6" encoded data into 4-bit
// data. The value 0xFF indicates invalid "3 out of 6" coding
static uint8_t decodeTab[64] = {0xFF,  //  "3 out of 6" encoded 0x00 decoded
                                0xFF,  //  "3 out of 6" encoded 0x01 decoded
                                0xFF,  //  "3 out of 6" encoded 0x02 decoded
                                0xFF,  //  "3 out of 6" encoded 0x03 decoded
                                0xFF,  //  "3 out of 6" encoded 0x04 decoded
                                0xFF,  //  "3 out of 6" encoded 0x05 decoded
                                0xFF,  //  "3 out of 6" encoded 0x06 decoded
                                0xFF,  //  "3 out of 6" encoded 0x07 decoded
                                0xFF,  //  "3 out of 6" encoded 0x08 decoded
                                0xFF,  //  "3 out of 6" encoded 0x09 decoded
                                0xFF,  //  "3 out of 6" encoded 0x0A decoded
                                0x03,  //  "3 out of 6" encoded 0x0B decoded
                                0xFF,  //  "3 out of 6" encoded 0x0C decoded
                                0x01,  //  "3 out of 6" encoded 0x0D decoded
                                0x02,  //  "3 out of 6" encoded 0x0E decoded
                                0xFF,  //  "3 out of 6" encoded 0x0F decoded
                                0xFF,  //  "3 out of 6" encoded 0x10 decoded
                                0xFF,  //  "3 out of 6" encoded 0x11 decoded
                                0xFF,  //  "3 out of 6" encoded 0x12 decoded
                                0x07,  //  "3 out of 6" encoded 0x13 decoded
                                0xFF,  //  "3 out of 6" encoded 0x14 decoded
                                0xFF,  //  "3 out of 6" encoded 0x15 decoded
                                0x00,  //  "3 out of 6" encoded 0x16 decoded
                                0xFF,  //  "3 out of 6" encoded 0x17 decoded
                                0xFF,  //  "3 out of 6" encoded 0x18 decoded
                                0x05,  //  "3 out of 6" encoded 0x19 decoded
                                0x06,  //  "3 out of 6" encoded 0x1A decoded
                                0xFF,  //  "3 out of 6" encoded 0x1B decoded
                                0x04,  //  "3 out of 6" encoded 0x1C decoded
                                0xFF,  //  "3 out of 6" encoded 0x1D decoded
                                0xFF,  //  "3 out of 6" encoded 0x1E decoded
                                0xFF,  //  "3 out of 6" encoded 0x1F decoded
                                0xFF,  //  "3 out of 6" encoded 0x20 decoded
                                0xFF,  //  "3 out of 6" encoded 0x21 decoded
                                0xFF,  //  "3 out of 6" encoded 0x22 decoded
                                0x0B,  //  "3 out of 6" encoded 0x23 decoded
                                0xFF,  //  "3 out of 6" encoded 0x24 decoded
                                0x09,  //  "3 out of 6" encoded 0x25 decoded
                                0x0A,  //  "3 out of 6" encoded 0x26 decoded
                                0xFF,  //  "3 out of 6" encoded 0x27 decoded
                                0xFF,  //  "3 out of 6" encoded 0x28 decoded
                                0x0F,  //  "3 out of 6" encoded 0x29 decoded
                                0xFF,  //  "3 out of 6" encoded 0x2A decoded
                                0xFF,  //  "3 out of 6" encoded 0x2B decoded
                                0x08,  //  "3 out of 6" encoded 0x2C decoded
                                0xFF,  //  "3 out of 6" encoded 0x2D decoded
                                0xFF,  //  "3 out of 6" encoded 0x2E decoded
                                0xFF,  //  "3 out of 6" encoded 0x2F decoded
                                0xFF,  //  "3 out of 6" encoded 0x30 decoded
                                0x0D,  //  "3 out of 6" encoded 0x31 decoded
                                0x0E,  //  "3 out of 6" encoded 0x32 decoded
                                0xFF,  //  "3 out of 6" encoded 0x33 decoded
                                0x0C,  //  "3 out of 6" encoded 0x34 decoded
                                0xFF,  //  "3 out of 6" encoded 0x35 decoded
                                0xFF,  //  "3 out of 6" encoded 0x36 decoded
                                0xFF,  //  "3 out of 6" encoded 0x37 decoded
                                0xFF,  //  "3 out of 6" encoded 0x38 decoded
                                0xFF,  //  "3 out of 6" encoded 0x39 decoded
                                0xFF,  //  "3 out of 6" encoded 0x3A decoded
                                0xFF,  //  "3 out of 6" encoded 0x3B decoded
                                0xFF,  //  "3 out of 6" encoded 0x3C decoded
                                0xFF,  //  "3 out of 6" encoded 0x3D decoded
                                0xFF,  //  "3 out of 6" encoded 0x3E decoded
                                0xFF}; // "3 out of 6" encoded 0x3F decoded

//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// void encode3outof6 (uint8_t *uncodedData, uint8_t *encodedData, uint8_t lastByte)
//
//  DESCRIPTION:
//    Performs the "3 out 6" encoding on a 16-bit data value into a
//    24-bit data value. When encoding on a 8 bit variable, a postamle
//    sequence is added.
//
//  ARGUMENTS:
//        uint8_t *uncodedData      - Pointer to data
//        uint8_t *encodedData      - Pointer to store the encoded data
//        uint8_t lastByte          - Only one byte left in data buffer
//----------------------------------------------------------------------------

void encode3outof6(uint8_t *uncodedData, uint8_t *encodedData, uint8_t lastByte)
{

  uint8_t data[4];

  // - Perform encoding -

  // If last byte insert postamble sequence
  if (lastByte)
  {
    data[1] = 0x14;
  }
  else
  {
    data[0] = encodeTab[*(uncodedData + 1) & 0x0F];
    data[1] = encodeTab[(*(uncodedData + 1) >> 4) & 0x0F];
  }

  data[2] = encodeTab[(*uncodedData) & 0x0F];
  data[3] = encodeTab[((*uncodedData) >> 4) & 0x0F];

  // - Shift the encoded 6-bit values into a byte buffer -
  *(encodedData + 0) = (data[3] << 2) | (data[2] >> 4);
  *(encodedData + 1) = (data[2] << 4) | (data[1] >> 2);

  if (!lastByte)
  {
    *(encodedData + 2) = (data[1] << 6) | data[0];
  }
}

//----------------------------------------------------------------------------
// uint8_t decode3outof6 (uint8_t *encodedData, uint8_t *decodedData, uint8_t lastByte)
//
//  DESCRIPTION:
//    Performs the "3 out 6" decoding of a 24-bit data value into 16-bit
//    data value. If only 2 byte left to decoded,
//    the postamble sequence is ignored
//
//  ARGUMENTS:
//        uint8_t *encodedData      - Pointer to encoded data
//        uint8_t *decodedData      - Pointer to store the decoded data
//        uint8_t lastByte          - Only one byte left in data buffer
//
//  RETURNS
//        DECODING_3OUTOF6_OK      0
//        DECODING_3OUTOF6_ERROR   1
//----------------------------------------------------------------------------

uint8_t decode3outof6(uint8_t *encodedData, uint8_t *decodedData, uint8_t lastByte)
{

  uint8_t data[4];

  // - Perform decoding on the input data -
  if (!lastByte)
  {
    data[0] = decodeTab[(*(encodedData + 2) & 0x3F)];
    data[1] = decodeTab[((*(encodedData + 2) & 0xC0) >> 6) | ((*(encodedData + 1) & 0x0F) << 2)];
  }
  // If last byte, ignore postamble sequence
  else
  {
    data[0] = 0x00;
    data[1] = 0x00;
  }

  data[2] = decodeTab[((*(encodedData + 1) & 0xF0) >> 4) | ((*encodedData & 0x03) << 4)];
  data[3] = decodeTab[((*encodedData & 0xFC) >> 2)];

  // - Check for invalid data coding -
  if ((data[0] == 0xFF) | (data[1] == 0xFF) |
      (data[2] == 0xFF) | (data[3] == 0xFF))

    return (DECODING_3OUTOF6_ERROR);

  // - Shift the encoded values into a byte buffer -
  *decodedData = (data[3] << 4) | (data[2]);
  if (!lastByte)
    *(decodedData + 1) = (data[1] << 4) | (data[0]);

  return (DECODING_3OUTOF6_OK);
}

/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

/***********************************************************************************
    Filename: mbus_crc.c

    Copyright 2008 Texas Instruments, Inc.
***********************************************************************************/

//-------------------------------------------------------------------------------------------------------
//  uint16 crcCalc(uint16 crcReg, uint8_t crcData)
//
//  DESCRIPTION:
//      Calculates the 16-bit CRC. The function requires that the CRC_POLYNOM is defined,
//      which gives the wanted CRC polynom.
//
//  ARGUMENTS:
//      uint8_t  crcData  - Data to perform the CRC-16 operation on.
//      uint16 crcReg   - Current or initial value of the CRC calculation
//
//  RETURN:
//      The value returned is the 16-bit CRC (of the data supplied so far).
//-------------------------------------------------------------------------------------------------------

uint16_t crcCalc(uint16_t crcReg, uint8_t crcData)
{
  uint8_t i;

  for (i = 0; i < 8; i++)
  {
    // If upper most bit is 1
    if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
      crcReg = (crcReg << 1) ^ CRC_POLYNOM;
    else
      crcReg = (crcReg << 1);

    crcData <<= 1;
  }

  return crcReg;
}

/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/
#endif
