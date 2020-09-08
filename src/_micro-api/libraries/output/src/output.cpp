#include "Arduino.h"

#if defined(ESP32) || defined(ESP8266)
  #define ETHERNET_PRINT 1                // need for right options in output.h
#endif

#include "output.h"

/* 
 * =================================================
 *  helper functions for smaller sketch size for uC
 * =================================================
 *
 * Arduino IDE statistic´s:
 * ------------------------
 * cnt opt. - size with hardware
 *    0     - 28806 size nano with debug
 *   10     - 28382 size nano with debug
 *   15     - 28268 size nano with debug
 *   19     - 28066 size nano with debug (- 740 Bytes)
 *
 *   15     - 29018 size radino without debug
 *   19     - 28810 size radino without debug (- 208 Bytes) | (open 138 Bytes)
 */


void MSG_PRINTtoHEX(uint8_t a) {
/* 
 * this function is the alternative to 
 * sprintf(b, "%02x", xxx(i));
 * output via MSG_PRINT
 */
  if(a < 16) {
    MSG_PRINT(0);
  }
  MSG_PRINT(a , HEX);
}

void DBG_PRINTtoHEX(uint8_t b) {
/* 
 * this function is the alternative to 
 * sprintf(b, "%02x", yyy(i));
 * output via DBG_PRINT
 */
#ifdef DEBUG
  if(b < 16) {
    DBG_PRINT(0);
  }
  DBG_PRINT(b , HEX);
#endif
}


/* 
 * =====================
 *  END helper functions
 * =====================
 */
