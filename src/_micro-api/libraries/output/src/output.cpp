#include "Arduino.h"
#include "output.h"

/* These functions are only for the overall project. Libraries work without these functions. */

void MSG_PRINTtoHEX(uint8_t a) { // this function is the alternative to sprintf(b, "%02x", xxx(i))
  if(a < 16) {
    MSG_PRINT(0);
  }
  MSG_PRINT(a , HEX);
}

void DBG_PRINTtoHEX(uint8_t b) {  // this function is the alternative to sprintf(b, "%02x", yyy(i));
#ifdef DEBUG
  if(b < 16) {
    DBG_PRINT(0);
  }
  DBG_PRINT(b , HEX);
#endif
}
