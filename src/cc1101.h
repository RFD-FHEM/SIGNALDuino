// cc1101.h esp 
#pragma once
#ifndef _CC1101_h
#define _CC1101_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
//	#include "WProgram.h"
#endif
#include "compile_config.h"

#include <EEPROM.h>
#include "output.h"

extern char IB_1[14];

// only MAPLE_Mini
#ifdef MAPLE_Mini
  extern uint8_t radionr;
#endif


#if defined(ESP8266) || defined(ESP32) || defined(MAPLE_Mini)
#include <SPI.h>
#endif
namespace cc1101 {
	/*
	#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
	#define SS					  8  
	#define PIN_MARK433			  4  // LOW -> 433Mhz | HIGH -> 868Mhz
	
	#elif ARDUINO_ATMEGA328P_MINICUL  // 8Mhz 
	#define PIN_MARK433			  0
	#endif
				
	*/
						
#ifdef MAPLE_SDUINO
	const uint8_t radioCsPin[] = {31, 12, 15, 3};
	#else
	#define csPin	SS	   // CSN  out
	#define mosiPin MOSI   // MOSI out
	#define misoPin MISO   // MISO in
	#define sckPin  SCK    // SCLK out	
	#endif
			
	#define CC1101_WRITE_BURST    0x40
	#define CC1101_WRITE_SINGLE   0x00
	#define CC1101_READ_BURST     0xC0
	#define CC1101_READ_SINGLE    0x80
	#define CC1101_CONFIG         CC1101_READ_SINGLE
	#define CC1101_STATUS         CC1101_READ_BURST
	
	#define CC1101_FREQ2       0x0D  // Frequency control word, high byte
	#define CC1101_FREQ1       0x0E  // Frequency control word, middle byte
	#define CC1101_FREQ0       0x0F  // Frequency control word, low byte
	#define CC1101_PATABLE     0x3E  // 8 byte memory
	#define CC1101_IOCFG2      0x00  // GDO2 output configuration
	#define CC1101_PKTCTRL0    0x08  // Packet config register

    extern uint8_t revision;
	extern const uint8_t initVal[];
	// Status registers - newer version base on 0xF0
	#define CC1101_PARTNUM_REV01      0xF0 // Chip ID
	#define CC1101_VERSION_REV01      0xF1 // Chip ID
	#define CC1101_RSSI_REV01         0xF4 // Received signal strength indication
	#define CC1101_MARCSTATE_REV01    0xF5 // Control state machine state

  // Status registers - older version base on 0x30
	#define CC1101_PARTNUM_REV00      0x30 // Chip ID
	#define CC1101_VERSION_REV00      0x31 // Chip ID
	#define CC1101_RSSI_REV00         0x34 // Received signal strength indication
	#define CC1101_MARCSTATE_REV00    0x35 // Control state machine state
	 
	// Strobe commands
	#define CC1101_SRES     0x30  // reset
	#define CC1101_SFSTXON  0x31  // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
	#define CC1101_SCAL     0x33  // Calibrate frequency synthesizer and turn it off
	#define CC1101_SRX      0x34  // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
	#define CC1101_STX      0x35  // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1
	#define CC1101_SIDLE    0x36  // Exit RX / TX, turn off frequency synthesizer
	#define CC1101_SAFC     0x37  // Perform AFC adjustment of the frequency synthesizer
	#define CC1101_SFTX     0x3B  // Flush the TX FIFO buffer.
	#define CC1101_SNOP 	  0x3D	// 

  enum CC1101_MarcState {
    MarcStateSleep          = 0x00u
  , MarcStateIdle           = 0x01u
  , MarcStateXOff           = 0x02u
  , MarcStateVConnManCal    = 0x03u
  , MarcStateRegOnManCal    = 0x04u
  , MarcStateManCal         = 0x05u
  , MarcStateVConnFSWakeUp  = 0x06u
  , MarcStateRegOnFSWakeUp  = 0x07u
  , MarcStateStartCalibrate = 0x08u
  , MarcStateBWBoost        = 0x09u
  , MarcStateFSLock         = 0x0Au
  , MarcStateIfadCon        = 0x0Bu
  , MarcStateEndCalibrate   = 0x0Cu
  , MarcStateRx             = 0x0Du
  , MarcStateRxEnd          = 0x0Eu
  , MarcStateRxRst          = 0x0Fu
  , MarcStateTxRxSwitch     = 0x10u
  , MarcStateRxFifoOverflow = 0x11u
  , MarcStateFsTxOn         = 0x12u
  , MarcStateTx             = 0x13u
  , MarcStateTxEnd          = 0x14u
  , MarcStateRxTxSwitch     = 0x15u
  , MarcStateTxFifoUnerflow = 0x16u
  };
  
#if defined(ESP8266) || defined(ESP32)
	#define pinAsInput(pin) pinMode(pin, INPUT)
	#define pinAsOutput(pin) pinMode(pin, OUTPUT)
	#define pinAsInputPullUp(pin) pinMode(pin, INPUT_PULLUP)
	
	#ifndef digitalLow
		#define digitalLow(pin) digitalWrite(pin, LOW)
	#endif
	#ifndef digitalHigh
		#define digitalHigh(pin) digitalWrite(pin, HIGH)
	#endif
	#ifndef isHigh
		#define isHigh(pin) (digitalRead(pin) == HIGH)
	#endif
#endif

	#define wait_Miso()       while(isHigh(misoPin) ) { static uint8_t miso_count=255;delay(1); if(miso_count==0) return; miso_count--; }      // wait until SPI MISO line goes low 
    #define wait_Miso_rf()    while(isHigh(misoPin) ) { static uint8_t miso_count=255;delay(1); if(miso_count==0) return false; miso_count--; }      // wait until SPI MISO line goes low 

#ifdef MAPLE_Mini
	#define cc1101_Select()   digitalLow(radioCsPin[radionr])          // select (SPI) CC1101
	#define cc1101_Deselect() digitalHigh(radioCsPin[radionr])
#else
#define cc1101_Select()   digitalLow(csPin)          // select (SPI) CC1101
#define cc1101_Deselect() digitalHigh(csPin)
#endif

	#define EE_CC1101_CFG        2
	#define EE_CC1101_CFG_SIZE   0x29
	#define EE_CC1101_PA         0x30  //  (EE_CC1101_CFG+EE_CC1101_CFG_SIZE)  // 2C
	#define EE_CC1101_PA_SIZE    8
	
	#define PATABLE_DEFAULT      0x84   // 5 dB default value for factory reset

	//------------------------------------------------------------------------------
	// Chip Status Byte
	//------------------------------------------------------------------------------

	// Bit fields in the chip status byte
	#define CC1101_STATUS_CHIP_RDYn_BM             0x80
	#define CC1101_STATUS_STATE_BM                 0x70
	#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

	// Chip states
	#define CC1101_STATE_IDLE                      0x00
	#define CC1101_STATE_RX                        0x10
	#define CC1101_STATE_TX                        0x20
	#define CC1101_STATE_FSTXON                    0x30
	#define CC1101_STATE_CALIBRATE                 0x40
	#define CC1101_STATE_SETTLING                  0x50
	#define CC1101_STATE_RX_OVERFLOW               0x60
	#define CC1101_STATE_TX_UNDERFLOW              0x70
	

	#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
	//uint8_t RADINOVARIANT = 0;            // Standardwert welcher je radinoVarinat ge寤ert wird
	#endif


  
	byte hex2int(byte hex);							     // convert a hexdigit to int    // Todo: printf oder scanf nutzen
	uint8_t sendSPI(const uint8_t val);					 // send byte via SPI
	uint8_t cmdStrobe(const uint8_t cmd);
	uint8_t readReg(const uint8_t regAddr, const uint8_t regType);	// read CC1101 register via SPI
	void writeReg(const uint8_t regAddr, const uint8_t val);		// write single register into the CC1101 IC via SPI
	void readPatable(void);
	void writePatable(void);
	void readCCreg(const uint8_t reg);								// read CC1101 register
	void commandStrobes(void);
	void writeCCreg(uint8_t reg, uint8_t var);						// write CC1101 register
	void writeCCpatable(uint8_t var);								// write 8 byte to patable (kein pa ramping)
	void ccFactoryReset();

	uint8_t chipVersionRev();
	uint8_t chipVersion();	
	bool checkCC1101();	
	void setup();
	uint8_t getRevision();
	uint8_t getRSSI();
	void setIdleMode();
	uint8_t currentMode();
	void setReceiveMode();
	void setTransmitMode();

	void CCinit(void);             // initialize CC1101

	bool regCheck();

}

#endif
