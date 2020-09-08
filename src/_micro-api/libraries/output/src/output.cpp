#include "Arduino.h"

#ifdef PLATFORMIO                           // intern variable only in Software Platform IO (example 40304), in Arduino IDE undef
  #include "../../../../compile_config.h"   // Platform IO  - need for right options in output.h
#else
  #include "compile_config.h"               // Arduino IDE  - need for right options in output.h
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
 *    0     - 28806 size nano with debug (standing branch dev-r3.5_xFSK_pre, radino without sending xFSK)
 *   24     - 29356 size nano with debug (+ 550 Bytes, full xFSK support)
 *
 *    0     - 29298 size radino without debug  (standing branch dev-r3.5_xFSK_pre, radino without sending xFSK)
 *   24     - 28576 size radino without debug (- 722 Bytes, , full xFSK support)
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
