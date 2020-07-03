# SIGNALDuino uC v3.4.0-development  with cc1101 support 
[![Build Status](https://travis-ci.org/RFD-FHEM/SIGNALDuino.svg?branch=master)](https://travis-ci.org/RFD-FHEM/SIGNALDuino)

### Getting started


System to receive digital signals and provide them to other systems pro demodulatiob. Currently tested with 433 MHZ, but not limited to that frequency or media.


Just clone the repo and open the project file with visualstudio. (only available for windows)
You can also open it with the Arduino IDE. 
Compile it and have fun.
If you are using the Arduino IDE, you have to copy all the libs into your sketch folder and alter some includes.

### Using SIGNALDuino in FHEM

If you want to use the SIGNALDuino with FHEM, you can use it directly from FHEM. No neet to compile any sourcode.
You find more Information here:
http://www.fhemwiki.de/wiki/SIGNALDuino



### Tested microcontrollers

* Arduino Nano
* Arduino Pro Mini
* ESP32 (ESP32-WROOM-32 / ESP32-WROOM-32D)
* ESP8266
* RadinoCC1101
* STM32 F103CBT6 (Maple Mini, bootloader v1.0)

### ESP32 Notes

If you encounter problems compiling for ESP32, sorry the code for this microcontroller is currently not finished tested with all variants from ESP32. Contributors are welcome. If you have mane errors from fastDelegate.h try adding this compiler flag:
 -Wno-unused-local-typedef

### Signal from my device ist not detected

We have a pattern detection engine, that detect serval signal types. May not all, but most of them.

Uncomment #define debugdetect in libs/remotesensor/patterdecoder.h
Search for some output which describes a pattern with serval bits received.
If you find something, open an issue and provide as much as possible informations with it.


### You found a bug

First, sorry. This software is not perfect.
1. Open a issue
-With helpful title - use descriptive keywords in the title and body so others can find your bug (avoiding duplicates).
- Which branch, what microcontroller, what setup
- Steps to reproduce the problem, with actual vs. expected results
- If you find a bug in our code, post the files and the lines. 

### Contributing

1. Open one ore more issue for your development.
2. Ask to be added to our repository or just fork it.
3. Make your modifications and test them.
4. Create a branch (git checkout -b my_branch)
5. Commit your changes (git commit -am "<some description>")
6 .Push to a developer branch (git push dev-<xyz >my_branch)
7. Open a Pull Request, put some useful informations there, what your extension does and why we should add it, reference to the open issues which are fixed whith this pull requet.