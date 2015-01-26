# RFDuino

System to receive radio signals and use them in FHEM. Currently tested with 433 MHZ, but not limited to that frequency.

### Getting started


Just clone the repo and open the project file with code:blocks. (currently this works only for windows)
Compile it and have fun.

### Tested microcontrollers

* Arduino mega 2560

* Aduino Nano

### My sensor ist not detected

We have a pattern detection engine, that detect serval Signal types. May not all, but most of them.

Uncomment #define debugdetect in libs/remotesensor/patterdecoder.h
Search for some output which describes a pattern with serval bits received.
If you find something,open an issue and paste as much as possible informations with it.


### You found a bug

First, sorry. The software is not perfect.
1. Open a Issue
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


