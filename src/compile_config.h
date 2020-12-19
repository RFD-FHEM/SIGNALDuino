#pragma once

#ifndef _COMPILE_CONFIG_h
#define _COMPILE_CONFIG_h

/*
 * Config flags for compiling correct options / boards Define only one!
 * ********************************************************************
 * nothing to define                                             // boards without CC1101 (example, ESP8266, ESP32, nano, ...)
*/
//#define CMP_CC1101
//#define ARDUINO_ATMEGA328P_MINICUL 1                           // minicul with CC1101
//#define ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101 1;  // radino with CC1101
//#define OTHER_BOARD_WITH_CC1101  1                             // boards with CC1101 (example: ESP8266, ESP32, Maple Mini ...)


/*
 * compiling notes
 * ***************

  Platform IO
  radino_CC1101@debug  - size (29942 bytes) is greater than maximum allowed (28672 bytes)
*/


/*
 * enable debug option here
 * ************************
 */
//#define DEBUG


/*
 * ONLY for STM32F103CBT6 - MAPLE MINI
 * enabled ARDUINO_MAPLEMINI_F103CB Watchdog option
 */
#ifdef ARDUINO_MAPLEMINI_F103CB
  //#define WATCHDOG_STM32 1
#endif


/*
 * do NOT change anything below this line !
 * ****************************************
*/

#define PROGVERS               "3.5.0-dev+20201219"

#ifdef OTHER_BOARD_WITH_CC1101
  #define CMP_CC1101
#endif

#ifdef ARDUINO_ATMEGA328P_MINICUL
  #define CMP_CC1101
#endif

#ifdef ARDUINO_RADINOCC1101
  #define CMP_CC1101
#endif

#ifdef CMP_CC1101
  #ifdef ARDUINO_RADINOCC1101
    #define PIN_LED               13
    #define PIN_SEND              9   // GDO0 Pin TX out
    #define PIN_RECEIVE           7
    #define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
    #define PIN_MARK433           4
    #define SS                    8  
  #elif ARDUINO_ATMEGA328P_MINICUL    // 8Mhz
    #define PIN_LED               4
    #define PIN_SEND              2   // GDO0 Pin TX out
    #define PIN_RECEIVE           3
    #define PIN_MARK433           A0
  #elif ARDUINO_MAPLEMINI_F103CB
    const uint8_t pinReceive[] = {11, 18, 16, 14};  // compatible to variant -> Circuit board for four connected cc110x devices
    #define PIN_LED              33
    #define PIN_SEND             17                 // GDO Pin TX out
    #define PIN_RECEIVE          pinReceive[1]      // GDO2
    #define PIN_WIZ_RST          27                 // for LAN
  #elif defined(ESP8266)
    #define PIN_RECEIVE          5    // D1
    #define PIN_LED              16   // some boards have no LED or this LED has a different PIN defined
    #define PIN_SEND             4    // D2  // gdo0Pin TX out
    #define ETHERNET_PRINT
    //#define PIN_LED_INVERSE           // use this setting for the LED_BUILTIN on WEMOS boards
  #elif defined(ESP32)
    #define PIN_RECEIVE          13   // D13 | G13 (depending on type / clone / seller) --> old 16, not good (serial) and not all boards n.c.
    #define PIN_LED              2    // D2  | G2 (depending on type / clone / seller)
    #define PIN_SEND             4    // D4  | G4 (depending on type / clone / seller) // GDO0 Pin TX out
    #define ETHERNET_PRINT
  #else
    #define PIN_LED              9
    #define PIN_SEND             3    // gdo0Pin TX out
    #define PIN_RECEIVE          2
  #endif
#else
  #ifdef ESP8266
    #define PIN_RECEIVE          5    // D1
    #define PIN_LED              16   // some boards have no LED or this LED has a different PIN defined
    #define PIN_SEND             4    // D2  // gdo0Pin TX out
    #define ETHERNET_PRINT
    //#define PIN_LED_INVERSE           // use this setting for the LED_BUILTIN on WEMOS boards
  #elif defined(ESP32)
    #define PIN_RECEIVE          16   // D16 | G16 (depending on type / clone / seller)
    #define PIN_LED              2    // D2  | G2 (depending on type / clone / seller)
    #define PIN_SEND             4    // D4  | G4 (depending on type / clone / seller) // GDO0 Pin TX out
    #define ETHERNET_PRINT
  #elif ARDUINO_MAPLEMINI_F103CB
    #define PIN_LED              33
    #define PIN_SEND             17   // gdo0 Pin TX out
    #define PIN_RECEIVE          18   // gdo2
    #define PIN_WIZ_RST          27   // for LAN
  #else
    #define PIN_RECEIVE          2
    #define PIN_LED              13   // Message-LED
    #define PIN_SEND             11
  #endif
#endif


#endif  /* END _COMPILE_CONFIG_h */
