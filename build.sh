#!/bin/bash
set -ev
if [ "${RECEIVER}" = "cc1101" ]; then
  echo "setting compiler flags for cc1101"
fi

if [ "${BOARD}" = "nano" ]; then  
  echo "setting board to NANO"
fi

echo "Compile now for ${BOARD} with ${RECEIVER}"
