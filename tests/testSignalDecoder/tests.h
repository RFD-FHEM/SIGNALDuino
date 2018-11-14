#pragma once

#include <gtest/gtest.h>
#include "signalDecoder.h"
//#include "send.h"




namespace arduino {
	namespace test
	{
	  int duration;

	  class Tests : public ::testing::Test
	  {
	  public:
		Tests() : mcdecoder(&ooDecode) { } ;

		SignalDetectorClass ooDecode;
		ManchesterpatternDecoder mcdecoder;

		#define MSG_START char(0x2)		// this is a non printable Char
		#define MSG_END   char(0x3)			// this is a non printable Char

		bool state;

		bool DigitalSimulate(const int pulse);
		bool import_sigdata(std::string *cmdstring, const bool raw_mode = false);
		bool import_mcdata(std::string *cmdstring, const uint8_t startpos, const uint8_t endpos, const int16_t clock);
		std::string geFullMCString();


		virtual void SetUp();
		virtual void TearDown();

		static void SetUpTestCase() {
	
		};
	  };

	}; // End namespace test
}; // End namespace arduino
