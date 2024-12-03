/*** copyright ***
  Originally initiated by D.Tostmann
  Inspired by code from TI.com AN
  License: GPL v2
  kaihs 2018: add support for WMBUS type C reception
  Ralf9 2022: rf_mbus.c (culfw) in mbus.h umbenannt und fuer den SIGNALDuino angepasst und erweitert
  elektron-bbs 2024: 
*/

#ifndef MBUS_H
#define MBUS_H

// #define debug_mbus

#if defined (CMP_CC1101) && (defined (ESP8266) || defined (ESP32))
#include <Arduino.h>
#include "cc1101.h"

#define maxSendEcho 100

// extern String cmdstring; // for send
// extern uint8_t radionr; // for send nur bei mehreren CC110x
extern volatile bool blinkLED;
// extern bool LEDenabled;
extern String msg;                                  // RAW (nur für serial/telnet, keine HTML Ausgabe) & Serial Input
extern uint32_t msgCount;                           // Nachrichtenzähler über alle empfangenen Nachrichten
extern int8_t freqErr;                              // CC110x automatic Frequency Synthesizer Control
// extern uint8_t rssi;
// extern uint8_t revision;
extern bool AfcEnabled;
extern void CC110x_readFreqErr();
extern void msgOutput_MN(uint8_t * data, uint16_t lenData, uint8_t wmbusFrameTypeA, uint8_t lqi, uint8_t rssi, int8_t freqErr);
extern void MSG_PRINTtoHEX(uint8_t a);
extern void DBG_PRINTtoHEX(uint8_t b);

void mbus_init(uint8_t wmBusMode);
void mbus_task();
void mbus_send(int8_t startdata);
void mbus_readRXFIFO(uint8_t* data, uint8_t length, uint8_t *rssi, uint8_t *lqi);

#define WMBUS_NONE 0

#define RX_FIFO_THRESHOLD         0x47
#define RX_FIFO_START_THRESHOLD   0x40
#define RX_FIFO_SIZE              64
#define RX_OCCUPIED_FIFO          32    // Occupied space
#define RX_AVAILABLE_FIFO         32    // Free space

#define FIXED_PACKET_LENGTH       0x00
#define INFINITE_PACKET_LENGTH    0x02
#define INFINITE                  0
#define FIXED                     1
#define MAX_FIXED_LENGTH          256

typedef struct RXinfoDescr {
  uint8_t  lengthField;         // The L-field in the WMBUS packet
  uint16_t length;              // Total number of bytes to to be read from the RX FIFO
  uint16_t bytesLeft;           // Bytes left to to be read from the RX FIFO
  uint8_t *pByteIndex;          // Pointer to current position in the byte array
  uint8_t format;               // Infinite or fixed packet mode
  uint8_t mode;                 // S-mode or T-mode
  uint8_t framemode;            // C-mode or T-mode frame
  uint8_t frametype;            // Frame type A or B when in C-mode
  uint8_t state;
} RXinfoDescr;

#define TX_FIFO_THRESHOLD       0x07
#define TX_OCCUPIED_FIF0        32    // Occupied space
#define TX_AVAILABLE_FIFO       32    // Free space
#define TX_FIFO_SIZE            64

#define FIXED_PACKET_LENGTH     0x00
#define INFINITE_PACKET_LENGTH  0x02
#define INFINITE                0
#define FIXED                   1
#define MAX_FIXED_LENGTH        256

#define TX_OK                   0
#define TX_LENGTH_ERROR         1
#define TX_TO_IDLE1_ERROR       2
#define TX_FLUSH_ERROR          4
#define TX_UNDERFLOW_ERROR      8
#define TX_TO_IDLE2_ERROR      16

#define TX_TOO_SHORT_OR_NOEND_ERROR 250
#define TX_NO_HEX_ERROR             251
#define TX_UNKNOWN_MODE_ERROR       252

// Struct. used to hold information used for TX
typedef struct TXinfoDescr {
  uint16_t bytesLeft;           // Bytes left that are to be written to the TX FIFO
  uint8_t *pByteIndex;          // Pointer to current position in the byte array
  uint8_t  format;              // Infinite or fixed length packet mode
  uint8_t  complete;            // Packet Send
} TXinfoDescr;

/***********************************************************************************
    Filename: mbus_defs.h
    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include <inttypes.h>

//----------------------------------------------------------------------------------
//  Standard Defines
//----------------------------------------------------------------------------------
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL (void *)0
#endif


//----------------------------------------------------------------------------------
//  Macros
//----------------------------------------------------------------------------------

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif


/* takes a byte out of a uint32 : var - uint32,  ByteNum - byte to take out (0 - 3) */
#define BREAK_UINT32( var, ByteNum ) \
  (byte)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
  ((uint32)((uint32)((Byte0) & 0x00FF) \
            + ((uint32)((Byte1) & 0x00FF) << 8) \
            + ((uint32)((Byte2) & 0x00FF) << 16) \
            + ((uint32)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
  ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
  ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

/*
    This macro is for use by other macros to form a fully valid C statement.
    Without this, the if/else conditionals could show unexpected behavior.

    For example, use...
      #define SET_REGS()  st( ioreg1 = 0; ioreg2 = 0; )
    instead of ...
      #define SET_REGS()  { ioreg1 = 0; ioreg2 = 0; }
    or
      #define  SET_REGS()    ioreg1 = 0; ioreg2 = 0;
    The last macro would not behave as expected in the if/else construct.
    The second to last macro will cause a compiler error in certain uses
    of if/else construct

    It is not necessary, or recommended, to use this macro where there is
    already a valid C statement.  For example, the following is redundant...
      #define CALL_FUNC()   st(  func();  )
    This should simply be...
      #define CALL_FUNC()   func()

   (The while condition below evaluates false without generating a
    constant-controlling-loop type of warning on most compilers.)
*/
#define st(x)      do { x } while (__LINE__ == -1)

/***********************************************************************************
    Filename: mbus_paket.h
***********************************************************************************/

//----------------------------------------------------------------------------------
//  Constants
//----------------------------------------------------------------------------------

#define PACKET_C_FIELD  0x44
#define MAN_CODE        0x0CAE
#define MAN_NUMBER      0x12345678
#define MAN_ID          0x01
#define MAN_VER         0x07
#define PACKET_CI_FIELD 0x78

#define PACKET_OK              0
#define PACKET_CODING_ERROR    1
#define PACKET_CRC_ERROR       2

#define WMBUS_SMODE            1
#define WMBUS_TMODE            2
#define WMBUS_CMODE            3

#define WMBUS_FRAMEA           1
#define WMBUS_FRAMEB           2

//----------------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------------

void encodeTXPacket(uint8_t* pPacket, uint8_t* pData, uint8_t dataSize);

uint16_t packetSize(uint8_t lField);
uint16_t byteSize(uint8_t Smode, uint8_t transmit, uint16_t packetSize);

void   encodeTXBytesTmode(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize);
uint16_t decodeRXBytesTmode(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize);

void   encodeTXBytesSmode(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize);
uint16_t decodeRXBytesSmode(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize);

uint16_t verifyCrcBytesCmodeA(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize);
uint16_t verifyCrcBytesCmodeB(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize);

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
    Filename: mbus_manchester.h
***********************************************************************************/

//----------------------------------------------------------------------------------
// Constants
//----------------------------------------------------------------------------------

#define MAN_DECODING_OK      0
#define MAN_DECODING_ERROR   1

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------
void manchEncode(uint8_t *uncodedData, uint8_t *encodedData);
uint8_t manchDecode(uint8_t *encodedData, uint8_t *decodedData);

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
    Filename: mbus_3outof6.h
***********************************************************************************/

//----------------------------------------------------------------------------------
// Constants
//----------------------------------------------------------------------------------

#define DECODING_3OUTOF6_OK      0
#define DECODING_3OUTOF6_ERROR   1

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------
void encode3outof6 (uint8_t *uncodedData, uint8_t *encodedData, uint8_t lastByte);
uint8_t decode3outof6(uint8_t *encodedData, uint8_t *decodedData, uint8_t lastByte);

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
    Filename: mbus_crc.h
***********************************************************************************/

//----------------------------------------------------------------------------------
//  Constants
//----------------------------------------------------------------------------------

#define CRC_POLYNOM         0x3D65

//----------------------------------------------------------------------------------
//  Function Declareration
//----------------------------------------------------------------------------------

uint16_t crcCalc(uint16_t crcReg, uint8_t crcData);

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
#endif
