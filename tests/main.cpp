// main.cpp : Defines the entry point for the console application.
//

#include <stdio.h>

#include <arduino-mock/Arduino.h>
#include "arduino-mock/Serial.h"


#include <gtest/gtest.h>
#include <gmock/gmock.h>

char IB_1[14]; // Input Buffer one - capture commands
void disableReceive() {} ;
void enableReceive() {} ;


#include "tests.cpp"
//#include "test_send.cpp"

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
