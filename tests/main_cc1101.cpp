// main_cc1101.cpp : test runner for the serial commands that need CC1101 support.
//
// Built as a separate executable (TestProjectCC1101) with CMP_CC1101 defined,
// so the CC1101 branches in commands.h, send.h and functions.h are compiled.
// The unmodified src/cc1101.cpp runs against the fake chip in fakeCC1101.h.

#include <stdio.h>

#include <arduino-mock/Arduino.h>
#include <arduino-mock/Serial.h>
#include <arduino-mock/EEPROM.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#ifndef CMP_CC1101
#error "TestProjectCC1101 must be compiled with CMP_CC1101"
#endif

#define PROGNAME               " SIGNALduino "
#define VERSION_1              0x33
#define VERSION_2              0x1d
#define BAUDRATE               115200
#define FIFO_LENGTH            200
#define EE_MAGIC_OFFSET        0
#define addr_features          0xff

char IB_1[14]; // Input Buffer one - capture commands
void disableReceive();
void enableReceive();
int freeRam() { return 100; };
void cli() {};
void sei() {};

bool hasCC1101 = true;
bool AfcEnabled = true; // AFC on or off

volatile unsigned long lastTime = 0;
volatile bool blinkLED = false;

bool isDigit(int8_t value)
{
  return value >= '0' && value <= '9';
}

bool isHexadecimalDigit(int8_t value)
{
  return  isdigit(value) ||
          (value >= 'a' && value <= 'f') ||
          (value >= 'A' && value <= 'F') ;
}

#include "SimpleFIFO.h"
#include <signalDecoder.h>
SimpleFIFO<int, FIFO_LENGTH> FiFo;
SignalDetectorClass musterDec;

#include "fakeCC1101.h"
#include "serialCommandHelper.h"

#include "../src/cc1101.cpp"

// The arduino-mock micros() always returns 0; map it to the fake clock for send.h
#define micros() serialtest::fakeMicros()
#include <send.h>
#undef micros
#include <commands.h>

#include "test_serialCommand_cc1101.cpp"

int main(int argc, char **argv)
{
  ::testing::GTEST_FLAG(output) = "xml:testArduinoCC1101.testResults.xml";
  ::testing::GTEST_FLAG(print_time) = true;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
