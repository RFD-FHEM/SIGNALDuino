#!/bin/bash
#set -e
while test $# -gt 0
do
    case "$1" in
        BOARD*)   
               export BOARD=`echo $1 | cut -d= -f2`
        ;;
        RECEIVER*) 
               export RECEIVER=`echo $1 | cut -d= -f2`
		;;
        *) echo "got argument $1"
        ;;
    esac
    shift
done


if [ "${RECEIVER}" = "cc1101" ]; then
  echo "compiler.cpp.extra_flags=-DOTHER_BOARD_WITH_CC1101=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
  find ~/.arduino15/packages/ -type f  -name platform.txt | xargs dirname | xargs -I{} bash -c 'echo "compiler.cpp.extra_flags=-DOTHER_BOARD_WITH_CC1101=1" > $1/platform.local.txt' -- {}
fi


if [ "${BOARD}" = "nano328" ]; then
  arduino --board arduino:avr:nano:cpu=atmega328 --save-prefs 2>&1

elif [ "${BOARD}" = "minicul" ]; then
  echo "compiler.cpp.extra_flags=-DARDUINO_ATMEGA328P_MINICUL=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
  arduino --board arduino:avr:pro:cpu=8MHzatmega328 --save-prefs 2>&1
elif [ "${BOARD}" = "promini" ]; then
  arduino --board arduino:avr:pro:cpu=8MHzatmega328 --save-prefs 2>&1
elif [ "${BOARD}" = "radino" ]; then
  echo "compiler.cpp.extra_flags=-DARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101=1" > /usr/local/share/arduino/hardware/arduino/avr/platform.local.txt
 arduino --install-boards "In-Circuit:avr"  2>&1
 arduino --board In-Circuit:avr:radinoCC1101 --save-prefs 2>&1
fi


echo "Compile now for ${BOARD} with ${RECEIVER}"
arduino -v --verbose-build --verify $PWD/src/arduino-ide/SIGNALDuino/SIGNALDuino.ino 2>&1
ARDUINO_RETURN_CODE=$?
echo "Compile ${BOARD} ${RECEIVER} finshed"
exit $ARDUINO_RETURN_CODE