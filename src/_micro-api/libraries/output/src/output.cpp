#include "Arduino.h"

#ifdef _COMPILE_CONFIG_h                      /* to break Dependency, _COMPILE_CONFIG_h is only available in the SIGNALduino project */
  #ifdef PLATFORMIO                           // intern variable only in Software Platform IO (example 40304), in Arduino IDE undef
    #include "../../../../compile_config.h"   // Platform IO  - need for right options in output.h
  #else
    #include "compile_config.h"               // Arduino IDE  - need for right options in output.h
  #endif
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
 *    0     - 28212 size nano with debug (branch dev-r3.4, without xFSK support, only ASK/OOK support)
 *    0     - 28806 size nano with debug (branch dev-r3.5_xFSK_pre, radino without sending xFSK, ASK/OOK support)
 *   29     - 27302 size nano with debug (branch dev-r3.5_xFSK_pre_size_opt_v6 -1504 Bytes opt | -910 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *
 *    0     - 25638 size nano without debug (branch dev-r3.4, without xFSK support, only ASK/OOK support)
 *    0     - 27226 size nano without debug (branch dev-r3.5_xFSK_pre, radino without sending xFSK, ASK/OOK support)
 *   29     - 24086 size nano without debug (branch dev-r3.5_xFSK_pre_size_opt_v6 -3140 Bytes opt | -1552 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *
 *    0     - 27700 size radino without debug (branch dev-r3.4, without xFSK support, only ASK/OOK support)
 *    0     - 29298 size radino without debug (branch dev-r3.5_xFSK_pre, radino without sending xFSK, ASK/OOK support)
 *   29     - 26190 size radino without debug (branch dev-r3.5_xFSK_pre_size_opt_v6 -3108 Bytes opt | -1510 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
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
