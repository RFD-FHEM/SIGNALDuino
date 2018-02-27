#!/bin/bash
set -ev
if [ "${RECEIVER}" = "cc1101" ]; then
  echo "compiler.cpp.extra_flags=-DOTHER_BOARD_WITH_CC1101=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
fi

if [ "${BOARD}" = "nano" ]; then
  arduino --board arduino:avr:nano --save-prefs
fi

echo "Compile now for ${BOARD} with ${RECEIVER}"
arduino -v --verbose-build --verify $PWD/RF_Receiver/RF_Receiver.ino 2>&1
