// main.cpp : Defines the entry point for the console application.
//

#include "targetver.h"

#include <stdio.h>
//#include <tchar.h>
//#include <arduino.h>

#include "Arduino.h"
#include "Serial.h"


#include <gtest/gtest.h>
#include <gmock/gmock.h>


#include <signalDecoder.h>
//#include <output/src/output.h>

int main(int argc, char **argv)
{


  ::testing::GTEST_FLAG(output) = "xml:testWin32Arduino.testResults.xml";
  ::testing::GTEST_FLAG(print_time) = true;

  ::testing::InitGoogleTest(&argc, argv);
 // ::testing::GTEST_FLAG(filter) = "*mcHideki2";

  int wResult = RUN_ALL_TESTS(); //Find and run all tests

   //system("pause");

  return wResult; // returns 0 if all the tests are successful, or 1 otherwise


}
