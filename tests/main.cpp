// main.cpp : Defines the entry point for the console application.
//

#include <stdio.h>

#include <arduino-mock/Arduino.h>
#include "arduino-mock/Serial.h"


#include <gtest/gtest.h>
#include <gmock/gmock.h>
// #include <ctype.h>

char IB_1[14]; // Input Buffer one - capture commands
void disableReceive();
void enableReceive();
int freeRam() { return 100; };
void cli() {};
void sei() {};

// bool hasCC1101=true;
bool hasCC1101=false;
bool AfcEnabled=true; // AFC on or off

volatile unsigned long lastTime=0;
volatile bool blinkLED=false;

bool isDigit(int8_t value)
{
  bool digit = value >= '0' && value <= '9';
  return digit;
}

bool isHexadecimalDigit(int8_t value)
{
  return  isdigit(value) ||
          (value >= 'a' && value <= 'f') ||
          (value >= 'A' && value <= 'F') ;
}

#include "tests.cpp"
#include "test_serialCommand.cpp"

int main(int argc, char **argv)
{


  ::testing::GTEST_FLAG(output) = "xml:testArduino.testResults.xml";
  ::testing::GTEST_FLAG(print_time) = true;

  ::testing::InitGoogleTest(&argc, argv);
 // ::testing::GTEST_FLAG(filter) = "*mcHideki2";

  int wResult = RUN_ALL_TESTS(); //Find and run all tests

   //system("pause");

  return wResult; // returns 0 if all the tests are successful, or 1 otherwise
}
