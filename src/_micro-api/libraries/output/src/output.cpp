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
 *    x     - 28212 size nano CC1101 with debug (branch dev-r3.4, without xFSK support, only ASK/OOK support)
 *    0     - 28806 size nano CC1101 with debug (branch dev-r3.5_xFSK_pre, radino without sending xFSK, ASK/OOK support)
 *   29     - 27302 size nano CC1101 with debug (branch dev-r3.5_xFSK_pre_size_opt_v6 -1504 Bytes opt | -910 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *   30 MSG - 23268 size nano CC1101 with debug (branch dev-r3.5_xFSK_revised_v2 -5538 Bytes opt | -4944 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *   31 SDC - 27168 size nano CC1101 with debug (branch dev-r3.5_xFSK_revised_v2_TEST_without_SD_CPP -1638 Bytes opt | -1044 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *
 *    x     - 25638 size nano CC1101 without debug (branch dev-r3.4, without xFSK support, only ASK/OOK support)
 *    0     - 27226 size nano CC1101 without debug (branch dev-r3.5_xFSK_pre, radino without sending xFSK, ASK/OOK support)
 *   29     - 24086 size nano CC1101 without debug (branch dev-r3.5_xFSK_pre_size_opt_v6 -3140 Bytes opt | -1552 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *   30 MSG - 23576 size nano CC1101 without debug (branch dev-r3.5_xFSK_revised_v2 -3650 Bytes opt | -2062 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *   31 SDC - 25820 size nano CC1101 without debug (branch dev-r3.5_xFSK_revised_v2_TEST_without_SD_CPP -1406 Bytes opt | +182 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *
 *    x     - 27700 size radino CC1101 without debug (branch dev-r3.4, without xFSK support, only ASK/OOK support)
 *    0     - 29298 size radino CC1101 without debug (branch dev-r3.5_xFSK_pre, radino without sending xFSK, ASK/OOK support)
 *   29     - 26190 size radino CC1101 without debug (branch dev-r3.5_xFSK_pre_size_opt_v6 -3108 Bytes opt | -1510 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *   30 MSG - 25372 size radino CC1101 without debug (branch dev-r3.5_xFSK_revised_v2 -3926 Bytes opt | -2328 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
 *   31 SDC - 27922 size radino CC1101 without debug (branch dev-r3.5_xFSK_revised_v2_TEST_without_SD_CPP -1376 Bytes opt | +222 Bytes to dev-r3.4 , full xFSK support, ASK/OOK support)
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
