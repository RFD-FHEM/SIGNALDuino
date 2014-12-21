RFDuino
=======

System to receive radio signals and use them in FHEM. Currently tested with 433 MHZ, but not limited to that frequency.

Getting started
===

Just clone the repo and open the project file with code:blocks.
Compile it and have fun.

Tested microcontrollers
===
Arduino mega 2560
Aduino Nano

My sensor ist not detected
===
Uuncomment #define debugdetect in libs/remotesensor/patterdecoder.h
look at the output, paste it  and open an issue.

How to develop new things
===
1. Open one ore more issue for your development.
2. Creae a new branch. based on the master. Name it dev-<some words to identify your work>
3. Extend the code in your branch. 
4. When you are finished, test your code
5. Finally merge with master.
