#!/bin/bash
set -ev
if [ "${RECEIVER}" = "cc1101" ]; then
  echo "compiler.cpp.extra_flags=-DOTHER_BOARD_WITH_CC1101=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
fi

if [ "${BOARD}" = "nano" ]; then
  arduino --board arduino:avr:nano --save-prefs 2>&1
elif [ "${BOARD}" = "minicul" ]; then
  echo "compiler.cpp.extra_flags=-DARDUINO_ATMEGA328P_MINICUL=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
  arduino --board arduino:avr:pro:pu=atmega368,f_cpu=8000000L--save-prefs 2>&1
elif [ "${BOARD}" = "radino" ]; then
  echo "compiler.cpp.extra_flags=-DARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
 arduino --install-boards "In-Circuit:avr"  2>&1
 arduino --board In-Circuit:avr:radinoCC1101 --save-prefs 2>&1
fi


echo "Compile now for ${BOARD} with ${RECEIVER}"
arduino -v --verbose-build --verify $PWD/RF_Receiver/RF_Receiver.ino 2>&1
