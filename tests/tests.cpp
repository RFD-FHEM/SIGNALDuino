//
//  win32Arduino Library - v1.0 - 05/13/2016
//  Copyright (C) 2016 Antoine Beauchamp
//  The code & updates for the library can be found on http://end2endzone.com
//
// AUTHOR/LICENSE:
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 3.0 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License (LGPL-3.0) for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
// DISCLAIMER:
//  This software is furnished "as is", without technical support, and with no 
//  warranty, express or implied, as to its usefulness for any purpose.
//
// PURPOSE:
//  The win32Arduino is a win32 library that implementation of most used arduino
//  functions which allows a library developer to unit test his code outside of
//  the arduino platform.
//
//  This library allows a windows user to easily test an arduino library using
//  your testing framework of choice. For instance, the unit tests of win32Arduino
//  library are executed using the Google Test framework.
//
// USAGE:
//  The following instructions show how to easily test an arduino library using
//  the Google Test framework. It assumes that you are already familiar with the
//  test API.
//
//  1. Create an executable project and configure the main() function to launch
//     Google Test's RUN_ALL_TESTS() macro.
//  2. Create a second static library project and add all the arduino files of
//     the desired library you need to test.
//  3. The library files are expected to include arduino.h. Modify the project's
//     Additionnal Include Directories to point to the win32Arduino library.
//
//  The project should compile properly without errors or unresolved extensions
//  allowing you to properly unit test each functions.
//
// HISTORY:
// 05/13/2016 v1.0 - Initial release.
//

#undef max
#undef min

#include "tests.h"
#include <string>
#include "arduino-mock/Arduino.h"
#if defined(GTEST_OS_WINDOWS)
#define ARDUINO 101
#define NOSTRING
#endif

// #include <signalDecoder.h>


namespace arduino { 
	namespace test
		{
		std::string outputStr = "";

		//============================== Write callback =========================================
		size_t writeCallback(const uint8_t *buf, uint8_t len = 1)
		{
			const char *c = (const char*)buf;
			outputStr.append(c, len);
			return len;
		}

		bool Tests::DigitalSimulate(const int pulse)
		{
			bool state = false;
			if (duration != 0 && (pulse ^ duration) < 0) // true if a and b have opposite signs
			{
				state = ooDecode.decode(&duration);
				duration = 0;
			}
			duration += pulse;
			return state;
		}

		bool Tests::import_sigdata(std::string *cmdstring, const bool raw_mode)
		{

			std::string msg_part;
			int16_t startpos = 0;
			int16_t endpos = 0;
			int buckets[8] = {};
			bool state = false;
			while (true)
			{
				startpos = endpos + 1;    // Set startpos to endpos to extract next part
				endpos = cmdstring->find(";", startpos);     			 // search next   ";"
				if (endpos == -1 || startpos == -1) break;

				if (cmdstring->at(startpos) == 'P' && cmdstring->at(startpos + 2) == '=') // Do some basic detection if data matches what we expect
				{
					uint8_t counter = atoi(cmdstring->substr(startpos + 1, startpos + 2).c_str()); // extract the pattern number
					buckets[counter] = atoi(cmdstring->substr(startpos + 3, endpos).c_str());
					if (raw_mode)
					{
						ooDecode.pattern[counter] = buckets[counter];
					}

				}
				else if (cmdstring->at(startpos) == 'D') {
					char cpulse[2];
					for (int i = startpos + 2; i < endpos; i++)
					{

						cpulse[0] = cmdstring->at(i);
						cpulse[1] = '\0';
						const int ipulse = atoi(cpulse);

						//state = DigitalSimulate(buckets[cmdstring->substring(i, i + 1).toInt()]);
						if (!raw_mode) {
							state = ooDecode.decode(&buckets[ipulse]);

							//Serial.println(buckets[ipulse]);
						}
						else {
							ooDecode.addData(ipulse);
						}
					}

				}

			}
			return state;


		}

		bool Tests::import_mcdata(std::string *cmdstring, const uint8_t startpos, const uint8_t endpos, const int16_t clock)
		{
			int8_t b;
			char c;
			uint8_t bit;
			bool state = false;
			unsigned long stoptime = micros();
			int pwidth = 0;

			for (uint8_t i = startpos; i < endpos; i++) {
				c = cmdstring->at(i);
				b = ((byte)c) - (c <= '9' ? 0x30 : 0x37);

				for (bit = 0x8; bit>0; bit >>= 1) {
					for (byte i = 0; i <= 1; i++) {
						if ((i == 0 ? (b & bit) : !(b & bit))) {
							pwidth = -1 * clock;
						}
						else {
							pwidth = clock;
						}
						state = DigitalSimulate(pwidth);
					}

				}

			}

			return state;
			// MSG_PRINTLN("");
		}


		std::string Tests::geFullMCString()
		{
			std::string mcStr;
			std::string lenStr;
			std::string pulseStr;
			std::string clockStr;

			mcStr = mcdecoder.getMessageHexStr();
			lenStr = mcdecoder.getMessageLenStr();
			pulseStr = mcdecoder.getMessagePulseStr();
			clockStr = mcdecoder.getMessageClockStr();

		
			return std::string("MC" + pulseStr + "D=" + mcStr + lenStr + clockStr+ "\n");

		}


		//--------------------------------------------------------------------------------------------------
		void Tests::SetUp()
		{
			outputStr.clear();
			mcdecoder.reset();
			ooDecode.reset();
			ooDecode.MSenabled = true;
			ooDecode.MCenabled = true;
			ooDecode.MUenabled = true;
			// ooDecode.setStreamCallback(&writeCallback);
			ooDecode.MredEnabled = false;
			duration = 0;
			state = false;
			mcdecoder.setMinBitLen(17);
	
		}

		//--------------------------------------------------------------------------------------------------
		void Tests::TearDown()
		{
		}

		  TEST_F(Tests,testInTolerance)
		  {
			  ASSERT_FALSE(ooDecode.inTol(-1061, 908, 212));
			  ASSERT_TRUE(ooDecode.inTol(1061, 908, 212));
		  }

		  TEST_F(Tests, testFindPattern)
		  {
			  ooDecode.pattern[0] = 908;
			  ooDecode.patternLen = 1;
			  int8_t idx = ooDecode.findpatt(-1061);

			  ASSERT_LT(idx, 0);

			  idx = ooDecode.findpatt(900);
			  ASSERT_GE(idx, 0);

		  }

		  TEST_F(Tests,testSamesign)
		  {
			  bool state;
			  int pulse;

			  pulse = -500;
			  state = ooDecode.decode(&pulse);
			  ASSERT_EQ(ooDecode.patternLen, 1);
		  
			  pulse = -32001;
			  state = ooDecode.decode(&pulse);
			  ASSERT_EQ(ooDecode.patternLen, 1);

			  pulse = 1000;
			  state = ooDecode.decode(&pulse);
			  ASSERT_EQ(ooDecode.patternLen, 2);
			  ASSERT_EQ(ooDecode.messageLen, 2);
			  state = ooDecode.decode(&pulse);
			  ASSERT_EQ(ooDecode.patternLen, 1);
			  ASSERT_EQ(ooDecode.messageLen, 1);

			  ASSERT_FALSE(state);
		  }

		  TEST_F(Tests,testCompressPattern)
		  {

			  int pulse = 500;

			  DigitalSimulate(-(pulse * 10));
			  DigitalSimulate(pulse*2.1);

			  for (uint8_t i = 0; i < 122; i++)
			  {
				  DigitalSimulate(-(pulse));
				  if (i % 2)
					  DigitalSimulate(pulse * 2);
				  else
					  DigitalSimulate(pulse * 4);
			  }
			  ASSERT_EQ(ooDecode.pattern[0], -5000);

			  ooDecode.pattern[0] = -599; // Modify the first entry in


			  ooDecode.compress_pattern();

			  ooDecode.printOut();

			  ASSERT_EQ(ooDecode.patternLen, 4);
			  ASSERT_EQ(ooDecode.pattern[0], -500);
			  ASSERT_EQ(ooDecode.pattern[1], 1000);
			  ASSERT_EQ(ooDecode.pattern[2], 0);
			  ASSERT_EQ(ooDecode.pattern[3], 2000);

			  ASSERT_EQ(ooDecode.histo[0], 123);
			  ASSERT_EQ(ooDecode.histo[1], 61);
			  ASSERT_EQ(ooDecode.histo[2], 0);
			  ASSERT_EQ(ooDecode.histo[3], 61);

		  }

		  TEST_F(Tests, testOperatorPredecrement)
		  {

			  // mock mcdcoder
			  mcdecoder.longlow = 0;
			  mcdecoder.longhigh = 1;
			  mcdecoder.shortlow = 2;
			  mcdecoder.shorthigh = 3;

			  uint8_t message[5] = { 0,1,2,3,0 };
			  uint8_t i = 3;

			  if (mcdecoder.isShort(message[i - 1]) && --i>2)
			  {
				  ASSERT_EQ(i, 3);
			  }
			  i--;
			  ASSERT_EQ(i, 1);

		  }


		  TEST_F(Tests, testMCbasic1)
		  {

			  ooDecode.reset();
			  mcdecoder.reset();
			  mcdecoder.minbitlen = 7;               
			  std::string dstr("MU;P0=-300;P1=300;P2=-600;P3=600;D=10101010323010;"); //00001011       1 1 1 1 0 1 00
			  bool state = import_sigdata(&dstr);                                                    

	 	
			  //ASSERT_FALSE(mcdecoder.isManchester());
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  //ASSERT_TRUE(mcdecoder.isManchester());
			  ASSERT_FALSE(state);
			  mcdecoder.longlow = 3;
			  mcdecoder.longhigh = 2;
			  mcdecoder.shortlow = 1;
			  mcdecoder.shorthigh = 0;
			  mcdecoder.clock = 299;
		  
			  bool result = mcdecoder.doDecode();
			 // ASSERT_TRUE(result);
			 // ASSERT_TRUE(mcdecoder.mc_start_found);
			 // ASSERT_TRUE(mcdecoder.mc_sync);
			  ASSERT_FALSE(mcdecoder.pdec->mcDetected);
			  //assertTrue(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);  // 7 Bits
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 8);

			  const char *mcStr;
			  std::string base="F4";

			  outputStr.clear();
			  mcStr=mcdecoder.getMessageHexStr();
	  
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
				  //pass();
		  }


		  TEST_F(Tests, testMCbasicA1)
		  {

			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-4200;P1=5700;P2=-6250;P3=1673;P4=-1260;P5=3145;P6=-2718;D=012345436;"); //1100
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 6;
			  mcdecoder.longhigh = 5;
			  mcdecoder.shortlow = 4;
			  mcdecoder.shorthigh = 3;
			  mcdecoder.clock = 1482;
			  bool result = mcdecoder.doDecode(); // For preamble the decoding will fail
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);  // 7 Bits
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 4);
			  const char *mcStr;
			  std::string base = "C";
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }

		  TEST_F(Tests, testMCbasicNA1)
		  {

			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-4200;P1=1673;P2=-1260;P3=3145;P4=-2718;D=01234;"); //110
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 4;
			  mcdecoder.longhigh = 3;
			  mcdecoder.shortlow = 2;
			  mcdecoder.shorthigh = 1;
			  mcdecoder.clock = 1482;
			  bool result = mcdecoder.doDecode(); // For preamble the decoding will fail
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);  
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount,3 );
			  const char *mcStr;
			  std::string base = "C";
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }

		  TEST_F(Tests, testMCbasicNA2)
		  {
			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-4200;P1=3145;P2=-1673;P3=1260;P4=-2718;D=01234;"); //100
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 4;
			  mcdecoder.longhigh = 1;
			  mcdecoder.shortlow = 2;
			  mcdecoder.shorthigh = 3;
			  mcdecoder.clock = 1482;
			  bool result = mcdecoder.doDecode(); // For preamble the decoding will fail
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 3);
			  const char *mcStr;
			  std::string base = "8";
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }


		  TEST_F(Tests, testMCbasicA2)
		  {
			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-4200;P1=5700;P2=-6250;P3=3145;P4=-1260;P5=1673;P6=-2718;D=012345456;"); //1000
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 6;
			  mcdecoder.longhigh = 3;
			  mcdecoder.shortlow = 4;
			  mcdecoder.shorthigh = 5;
			  mcdecoder.clock = 1482;
			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);  // 4 Bits
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 4);
			  const char *mcStr;
			  std::string base = "8";
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }


		  TEST_F(Tests, testMCbasicB)
		  {
			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-4200;P1=5700;P2=-6000;P3=1673;P4=-1260;P5=-2718;P6=3145;D=01234356;"); //001
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 5;
			  mcdecoder.longhigh = 6;
			  mcdecoder.shortlow = 4;
			  mcdecoder.shorthigh = 3;
			  mcdecoder.clock = 1482;
			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);  
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 3);
			  const char *mcStr;
			  std::string base = "2";
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }

		  TEST_F(Tests, testMCbasicC)
		  {
			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-2400;P1=2400;P2=4550;P3=-630;P4=660;P5=-1280;P6=1280;D=01023456;"); //001
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 5;
			  mcdecoder.longhigh = 6;
			  mcdecoder.shortlow = 3;
			  mcdecoder.shorthigh = 4;
			  mcdecoder.clock = 630;
			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 3);
			  const char *mcStr;
			  std::string base = "2";
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }

		  TEST_F(Tests, testMCbasicD)
		  {
			  mcdecoder.minbitlen = 3;
			  std::string dstr("MU;P0=-2400;P1=2400;P2=4550;P3=-1280;P4=1280;P5=-630;P6=640;D=01023456;"); //010
			  bool state = import_sigdata(&dstr);
			  ooDecode.calcHisto();
			  ooDecode.printOut();
			  mcdecoder.longlow = 3;
			  mcdecoder.longhigh = 4;
			  mcdecoder.shortlow = 5;
			  mcdecoder.shorthigh = 6;
			  mcdecoder.clock = 630;
			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 0);
			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 3);
			  const char *mcStr;
			  std::string base = "4"; //101
			  outputStr.clear();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr, base.c_str()); // may not compile or give warning
		  }


		  TEST_F(Tests, testMCosv2)
		  {
	
			// RTHN318_13_5   T: 19.3 BAT: ok   Full Message = MC;LL=-1063;LH=881;SL=-598;SH=385;D=55555555334B2D4D52CCD2CAAAD2CB4AAAD352ACCD0;L=169;C=487;

			ooDecode.reset();
			mcdecoder.reset();
			// other osv2 detected as MU;P0=882;P1=-1069;P2=404;P3=-578;D=010101010101010101010103212303212303210101010101230101010101010103210123032123032123010321012303210101012301032123032101012301032101230321012301010321010101010101010101012303210123032101010123032101230321010101010123010321010101230321010123010103210123;CP=0;R=248;O;; // Todo impl. test
			std::string fullpreamble("MU;P1=-1063;P2=881;D=212121212121212121212121212121212;");
			std::string shortpreamble("MU;P0=385;P1=-1063;P2=881;D=01212121212121212121212;");
			std::string data("MU;P0=385;P1=-1063;P2=881;P3=-598;P4=-3536;D=301032301032123012103230121032121230103212121230121032301032301032123012103230121212121212103212301210323012103212301212121212121032123010321212301212121032301032301032124;");

			//String dstr(F("MU;P0=385;P1=-1063;P2=881;P3=-598;P4=-3536;D=01212121212121212121212301032301032123012103230121032121230103212121230121032301032301032123012103230121212121212103212301210323012103212301212121212121032123010321212301212121032301032301032124212121212121212121212121212121212301032301032123012103230121;"));
			//import_sigdata(&dstr);
			import_sigdata(&shortpreamble);
			import_sigdata(&data);
			//import_sigdata(&fullpreamble);
			//import_sigdata(&data);

			//assertFalse(mcdecoder.isManchester());
			ooDecode.calcHisto();
			ooDecode.printOut();

			ASSERT_TRUE(mcdecoder.isManchester());

			bool result = mcdecoder.doDecode();
			ASSERT_TRUE(result);
			std::cout << outputStr;

			ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 19);

			std::string mcStr;
			std::string base;


			mcStr = mcdecoder.getMessageHexStr();  
			// alternativ 55555665A69595699969AAA969A5AAA9656A9994 ohne --i
			base = "555554CD2CB5354B334B2AAB4B2D2AAB4D4AB334"; // was 0000055B3EFC37FFFF9E but unclear why
			ASSERT_STREQ(mcStr.c_str(), base.c_str());
			mcStr.clear();

			char lstr[10];

			sprintf(lstr, "%d", mcdecoder.ManchesterBits.valcount);
			mcStr = mcdecoder.getMessageLenStr();
			base = "L="+ std::string(lstr) +std::string(";");
			//ASSERT_STREQ(mcStr.c_str(), base.c_str());
			//Todo getMessagePulsestr

			ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 159); // Oder 158?

			ooDecode.reset();
			mcdecoder.reset();



			import_sigdata(&shortpreamble);
			import_sigdata(&data);

			import_sigdata(&fullpreamble);
			import_sigdata(&data);

			//int max_val = -32001;
			//ooDecode.decode(&max_val);
			ooDecode.printOut();
			ooDecode.calcHisto();
			ooDecode.printOut();

			ASSERT_TRUE(mcdecoder.isManchester());

			result = mcdecoder.doDecode();
			ASSERT_TRUE(result);
			// std::cout << outputStr;

			ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 21);

		
			mcStr = mcdecoder.getMessageHexStr();
			base = "55555555334B2D4D52CCD2CAAAD2CB4AAAD352ACCD0";
			ASSERT_STREQ(mcStr.c_str(), base.c_str());
			mcStr.clear();

			//char lstr[10];
			// std::cout << geFullMCString();


			sprintf(lstr, "%d", mcdecoder.ManchesterBits.valcount);
			mcStr = mcdecoder.getMessageLenStr();
			base = std::string(";")  + "L="+ std::string(lstr) ;
			ASSERT_STREQ(mcStr.c_str(), base.c_str());
			//Todo getMessagePulsestr

			ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 169); // Oder 170?

		  }

		  //MU;P0=-1580;P1=873;P2=-1071;P3=-591;P4=388;P5=-3076;D=01212121213424313424313424313424313421243121213424312121213421243134243134243121342124313421212121243134243121342124313421243121342121212121212121212121212431342431342124313424313421245121212121212121212121212121212121342431342431342431342431342124312121;CP=4;O;
		  TEST_F(Tests, testMCosv2_b)
		  {
			  // RTHN318_13_5 T : 19.3°C  Full Message= MC;LL=-1070;LH=873;SL=-591;SH=387;D=5555555533332D4D52CCD2CAACD2CB4AAAAAACCB328;L=169;C=486;

			  // OSV2 which has a short pulse before gap instead of a long pulse

			  // other osv2 detected as MU;P0=882;P1=-1069;P2=404;P3=-578;D=010101010101010101010103212303212303210101010101230101010101010103210123032123032123010321012303210101012301032123032101012301032101230321012301010321010101010101010101012303210123032101010123032101230321010101010123010321010101230321010123010103210123;CP=0;R=248;O;; // Todo impl. test
			  std::string fullpreamble("MU;P1=-1063;P2=881;D=212121212121212121212121212121212;");
			  std::string shortpreamble("MU;P0=385;P1=-1063;P2=881;D=01212121212121212121212;");
			  std::string data("MU;P0=-1580;P1=873;P2=-1071;P3=-591;P4=388;P5=-3076;D=3424313424313424313424313421243121213424312121213421243134243134243121342124313421212121243134243121342124313421243121342121212121212121212121212431342431342124313424313421245;");
			  std::string spDataHex = "555554CCCCB5354B334B2AB34B2D2AAAAAB32CCA";  // Short Preamble + data in HEX Format
			  std::string fpDataHex = "5555555533332D4D52CCD2CAACD2CB4AAAAAACCB328";  // Short Preamble + data in HEX Format

			  //String dstr(F("MU;P0=385;P1=-1063;P2=881;P3=-598;P4=-3536;D=01212121212121212121212301032301032123012103230121032121230103212121230121032301032301032123012103230121212121212103212301210323012103212301212121212121032123010321212301212121032301032301032124212121212121212121212121212121212301032301032123012103230121;"));
			  import_sigdata(&shortpreamble);
			  import_sigdata(&data);

			  //assertFalse(mcdecoder.isManchester());
			  ooDecode.calcHisto();
			  ooDecode.printOut();

			  ASSERT_TRUE(mcdecoder.isManchester());
			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  // std::cout << outputStr;

			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 19);

			  std::string mcStr;


			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr.c_str(), spDataHex.c_str());
			  mcStr.clear();

			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 159); // Oder 158?

			  ooDecode.reset();
			  mcdecoder.reset();



			  import_sigdata(&shortpreamble);
			  import_sigdata(&data);

			  import_sigdata(&fullpreamble);
			  import_sigdata(&data);

			  ooDecode.calcHisto();
			  ooDecode.printOut();

			  ASSERT_TRUE(mcdecoder.isManchester());

			  result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  // std::cout << outputStr;

			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 21);

			  //std::cout << geFullMCString();
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr.c_str(), fpDataHex.c_str());
			  mcStr.clear();

			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 169); 
		  }


		  TEST_F(Tests, testMCosv2wopause)
		  {
			  // RTHN318_13_5 T: 19.3 BAT: ok      FULL Message = MC;LL=-1063;LH=881;SL=-598;SH=385;D=55555555334B2D4D52CCD2CAAAD2CB4AAAD352ACCD0;L=169;C=487;
			  ooDecode.reset();
			  mcdecoder.reset();
			  // other osv2 detected as MU;P0=882;P1=-1069;P2=404;P3=-578;D=010101010101010101010103212303212303210101010101230101010101010103210123032123032123010321012303210101012301032123032101012301032101230321012301010321010101010101010101012303210123032101010123032101230321010101010123010321010101230321010123010103210123;CP=0;R=248;O;; // Todo impl. test
			  std::string fullpreamble("MU;P1=-1063;P2=881;D=212121212121212121212121212121212;");
			  std::string shortpreamble("MU;P0=385;P1=-1063;P2=881;D=01212121212121212121212;");
			  std::string data("MU;P0=385;P1=-1063;P2=881;P3=-598;P4=-3536;D=301032301032123012103230121032121230103212121230121032301032301032123012103230121212121212103212301210323012103212301212121212121032123010321212301212121032301032301032124;");

			  //String dstr(F("MU;P0=385;P1=-1063;P2=881;P3=-598;P4=-3536;D=01212121212121212121212301032301032123012103230121032121230103212121230121032301032301032123012103230121212121212103212301210323012103212301212121212121032123010321212301212121032301032301032124212121212121212121212121212121212301032301032123012103230121;"));
			  //import_sigdata(&dstr);
			  import_sigdata(&shortpreamble);
			  import_sigdata(&data);
			  //import_sigdata(&fullpreamble);
			  //import_sigdata(&data);

			  //assertFalse(mcdecoder.isManchester());
			  ooDecode.calcHisto();
			  ooDecode.printOut();

			  ASSERT_TRUE(mcdecoder.isManchester());

			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  //std::cout << outputStr;

			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 19);

			  std::string mcStr;
			  std::string base;


			  mcStr = mcdecoder.getMessageHexStr();
			  // alternativ 55555665A69595699969AAA969A5AAA9656A9994 ohne --i
			  base = "555554CD2CB5354B334B2AAB4B2D2AAB4D4AB334"; // was 0000055B3EFC37FFFF9E but unclear why
			  ASSERT_STREQ(mcStr.c_str(), base.c_str());
			  mcStr.clear();

			  char lstr[10];
			  sprintf(lstr, "%d", mcdecoder.ManchesterBits.valcount);
			  mcStr = mcdecoder.getMessageLenStr();
			  base = std::string(";") +"L=" + std::string(lstr) ;
			  ASSERT_STREQ(mcStr.c_str(), base.c_str());
			  //Todo getMessagePulsestr

			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 159); // Oder 158?

			  ooDecode.reset();
			  mcdecoder.reset();



			  import_sigdata(&shortpreamble);
			  import_sigdata(&data);

			  import_sigdata(&fullpreamble);
			  import_sigdata(&data);

			  //int max_val = -32001;
			  //ooDecode.decode(&max_val);
			  ooDecode.printOut();
			  ooDecode.calcHisto();
			  ooDecode.printOut();

			  ASSERT_TRUE(mcdecoder.isManchester());

			  result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  //std::cout << outputStr;

			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 21);


			  mcStr = mcdecoder.getMessageHexStr();
			  base = "55555555334B2D4D52CCD2CAAAD2CB4AAAD352ACCD0";
			  ASSERT_STREQ(mcStr.c_str(), base.c_str());
			  mcStr.clear();
			  //std::cout << geFullMCString();

			  //char lstr[10];

			  sprintf(lstr, "%d", mcdecoder.ManchesterBits.valcount);
			  mcStr = mcdecoder.getMessageLenStr();
			  base = ";L=" + std::string(lstr);
			  ASSERT_STREQ(mcStr.c_str(), base.c_str());
			  //Todo getMessagePulsestr

			  ASSERT_EQ(mcdecoder.ManchesterBits.valcount, 169); // Oder 158?

		  }
		  TEST_F(Tests, mcOSV2THGR228N)
		  {
			  // THGR228N_41_2  T: 15.6 H : 38 BAT : ok     MC;LL=-994;LH=956;SL=-517;SH=463;;D=AAAAAAAA66959A6555659559556999955556A55669A5A696;L=191;;C=488;
			  std::string data = "MU;P0=-7452;P1=956;P2=-994;P3=-517;P4=463;D=01212121212121212121212121212121342431342431213421212431342431213424313421212121212124313421243134212121212431342121212121243121342431342431342431342121212121212121212431212134212121212431342431213424312134212431213424312134212431;CP=1;";
			  std::string refMCstr = "AAAAAAAA66959A6555659559556999955556A55669A5A696";



			  import_sigdata(&data);

			  ooDecode.calcHisto();
			  ooDecode.printOut();

			  ASSERT_TRUE(mcdecoder.isManchester());

			  bool result = mcdecoder.doDecode();
			  ASSERT_TRUE(result);
			  // std::cout << outputStr;

			  ASSERT_EQ(mcdecoder.ManchesterBits.bytecount, 23);
			  std::string mcStr;
			  mcStr = mcdecoder.getMessageHexStr();
			  ASSERT_STREQ(mcStr.c_str(), refMCstr.c_str());
		

			  // std::cout << geFullMCString();
		  }


		  TEST_F(Tests, msNCWS)
		  {
			bool state;

			//s5FA80C43C000
			//MS; P0 = -3886; P1 = 481; P2 = -1938; P3 = -9200; D = w; CP = 1; SP = 3; O; 

			//MS;P0=-9166;P1=463;P2=-3918;P3=-1954;D=10131213121213121312121313121313131313131312121312131212131313121213131313;CP=1;SP=0;R=3;m0;

			int pData[] = {
				-9200,481,200,-400,-1938,-3886
			};

			uint8_t s_Stream[] = {
				1,0,1,4,1,5,1,4,1,5,1,5,1,5,1,5,1,5,1,5,1,4,1,5,1,4,1,5,1,4,1,4,1,4,1,4,1,4,1,4,1,4,1,5,1,5,1,4,1,4,1,4,1,5,1,4,1,4,1,4,1,4,1,5,1,5,1,5,1,5,1,4,1,4,
			};

			uint16_t len = sizeof(s_Stream) / sizeof(s_Stream[0]);

			uint16_t i = 0;
			bool decoded;
			for (uint8_t j = 1, i = 5; j < 5; j++)
			{
				for (; i < len; i++)
				{
					state = ooDecode.decode(&pData[s_Stream[i]]);
					if (state)
						decoded = true;
				}
				if (j < 2) {
					ASSERT_EQ(ooDecode.patternLen, 3);
					ASSERT_FALSE(state);
				}
				i = 0;
			}
			//std::cout << outputStr.c_str();
			std::string base = "MS;P0=-3886;P1=481;P2=-1938;P3=-9200;D=13121012101010101010121012101212121212121210101212121012121212101010101212;CP=1;SP=3;O;m2;";
			ASSERT_STREQ(outputStr.substr(outputStr.find_first_of(MSG_START)+1, outputStr.find_first_of(MSG_END)- outputStr.find_first_of(MSG_START)-1).c_str() , base.c_str());
			ASSERT_FALSE(state);
		  }


		  TEST_F(Tests, msNCWS_redu)
		  {
			  ooDecode.MredEnabled = true;
			  int pData[] = {
	  -9200,481,200,-400,-1938,-3886
			  };

			  uint8_t s_Stream[] = {
				  1,0,1,4,1,5,1,4,1,5,1,5,1,5,1,5,1,5,1,5,1,4,1,5,1,4,1,5,1,4,1,4,1,4,1,4,1,4,1,4,1,4,1,5,1,5,1,4,1,4,1,4,1,5,1,4,1,4,1,4,1,4,1,5,1,5,1,5,1,5,1,4,1,4,
			  };

			  uint16_t len = sizeof(s_Stream) / sizeof(s_Stream[0]);

			  uint16_t i = 0;
			  bool decoded;
			  for (uint8_t j = 1, i = 5; j < 5; j++)
			  {
				  for (; i < len; i++)
				  {
					  state = ooDecode.decode(&pData[s_Stream[i]]);
					  if (state)
						  decoded = true;
				  }
				  if (j < 2) {
					  ASSERT_EQ(ooDecode.patternLen, 3);
					  ASSERT_FALSE(state);
				  }
				  i = 0;
			  }
			  std::string base = "d"; // mend is equal
			  base += 129; //  Appends char 129 which is 1 + mstart unequal 128

			  // Only first two chars are checked
			  ASSERT_STREQ(outputStr.substr(outputStr.find_first_of('d') , 2).c_str(), base.c_str());
			  ASSERT_FALSE(state);
			  ooDecode.MredEnabled = false;


			  // Todo convert the received chars to readable signaldata and compare results
		  }

		  TEST_F(Tests, msITV1)
		  {
			  bool state;
			  int pData[] = {
				  142,-446,-1056,972,-10304,250,-340
			  };

			  uint8_t s_Stream[] = {
				  5,4,5,2,3,6,5,2,3,6,5,2,5,2,5,2,3,6,5,2,3,6,5,2,5,2,5,2,3,6,5,2,3,6,5,2,3,6,5,2,3,6,5,2,5,2,5,2,3,6
			  };

			  uint16_t len = sizeof(s_Stream) / sizeof(s_Stream[0]);

			  uint16_t i = 0;
			  bool decoded;
			  for (uint8_t j = 1, i = 5; j < 7; j++)
			  {
				  for (; i < len; i++)
				  {
					  state = ooDecode.decode(&pData[s_Stream[i]]);
					  if (state)
						  decoded = true;
				  }
				  if (j < 2) {
					  ASSERT_EQ(ooDecode.patternLen, 4);
					  ASSERT_FALSE(state);
				  }
				  i = 0;
			  }
			  //std::cout << outputStr.c_str();
			  std::string base = "MS;P0=-340;P1=250;P2=-1056;P3=972;P4=-10304;D=14123012301212123012301212123012301230123012121230;CP=1;SP=4;O;m2;";
			  ASSERT_STREQ(outputStr.substr(outputStr.find_first_of(MSG_START) + 1, outputStr.find_first_of(MSG_END) - outputStr.find_first_of(MSG_START) - 1).c_str(), base.c_str());
			  ASSERT_FALSE(state);
		  }


		  TEST_F(Tests, muTX3)
		  {
			bool state;

			//	TXAE07540540
			//	MU; P0 = -27698; P1 = -180; P2 = 1274; P3 = -1037; P4 = 505; D = 12323232343234323434343232323232323434343234323432343232323232323234323432343232323232340232323234323432343434323232323232343434323432343234323232323232323432343234323232323234023232323432343234343432323232323234343432343234323432323232323232343234323432; CP = 4; O;
			//  MU;P0=1274;P1=-1037;P2=505;P3=-27698;                          D=0101010121012101212121010101010101212121012101210121010101010101012101210121010101010123010101012101210121212101010101010121212101210121012101010101010101210121012101010101012301010101210121012121210101010101012121210121012101210101010101010121012101210;CP=2;



			int pData[] = {
				-27698,180,1274,-1037,505
			};

			uint8_t s_Stream[] = {
				1,2,3,2,3,2,3,2,3,4,3,2,3,4,3,2,3,4,3,4,3,4,3,2,3,2,3,2,3,2,3,2,3,2,3,4,3,4,3,4,3,2,3,4,3,2,3,4,3,2,3,4,3,2,3,2,3,2,3,2,3,2,3,2,3,2,3,4,3,2,3,4,3,2,3,4,3,2,3,2,3,2,3,2,3,2,3,4,0,2,3,2,3,2,3,2,3,4,3,2,3,4,3,2,3,4,3,4,3,4,3,2,3,2,3,2,3,2,3,2,3,2,3,4,3,4,3,4,3,2,3,4,3,2,3,4,3,2,3,4,3,2,3,2,3,2,3,2,3,2,3,2,3,2,3,4,3,2,3,4,3,2,3,4,3,2,3,2,3,2,3,2,3,2,3,4,0,2,3,2,3,2,3,2,3,4,3,2,3,4,3,2,3,4,3,4,3,4,3,2,3,2,3,2,3,2,3,2,3,2,3,4,3,4,3,4,3,2,3,4,3,2,3,4,3,2,3,4,3,2,3,2,3,2,3,2,3,2,3,2,3,2,3,4,3,2,3,4,3,2,3,4,3,2,
			};

			uint16_t len = sizeof(s_Stream) / sizeof(s_Stream[0]);

			uint16_t i = 0;

			for (uint8_t j = 1, i = 5; j < 5; j++)
			{

				for (; i < len; i++)
				{
					state = ooDecode.decode(&pData[s_Stream[i]]);
				}
				if (j < 2) {
					ASSERT_EQ(ooDecode.patternLen, 4);
					ASSERT_FALSE(state);
				}
				i = 0;
			}
			std::string base = "MU;P0=1274;P1=-1037;P2=505;P3=-27698;D=010121012101212121010101010101212121012101210121010101010101012101210121010101010123010101012101210121212101010101010121212101210121012101010101010101210121012101010101012301010101210121012121210101010101012121210121012101210101010101010121012101210;CP=2;";
			ASSERT_STREQ(outputStr.substr(outputStr.find_first_of(MSG_START) + 1, outputStr.find_first_of(MSG_END) - outputStr.find_first_of(MSG_START) - 1).c_str(), base.c_str());

		  
		  }
		TEST_F(Tests,mcLong1)
		{
			bool state;
			std::string dstr2 = "0B0F9FFA555AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAA";
			state = import_mcdata(&dstr2, 0, dstr2.length(), 450);
			ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_EQ(253, ooDecode.messageLen);

			ooDecode.calcHisto();
			ooDecode.printOut();

			ASSERT_TRUE(mcdecoder.isManchester());
			ASSERT_FALSE(state);

			bool result = mcdecoder.doDecode();
			ASSERT_EQ(226, mcdecoder.ManchesterBits.valcount - 1);
			ASSERT_EQ(0, ooDecode.messageLen); // in doDecode wird bufferMove ausgeführt, und das löst einen reset aus, da wir am Ende sind.
			ASSERT_FALSE(mcdecoder.pdec->mcDetected);
			ASSERT_TRUE(result);

			std::string mcStr;
			std::string base;
			mcStr=mcdecoder.getMessageHexStr();
			//      161F3FF4AAB5555555555555555555555555555555555555555575554									  ->  000101100001111100111111111101001010101010110101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101110101010101010100
			//      09E0C00B554AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA8AAAA									  -> 000010011110000011000000000010110101010101001010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010001010101010101010
			//      09E0C00B554AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA8AAAA
			//base = "0B0F9FFA555AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAA";  //57 Hex digits (57*4=228)       -> 000010110000111110011111111110100101010101011010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010111010101010101010
			ASSERT_STREQ(mcStr.c_str(), dstr2.c_str()); // may not compile or give warning
									//mcdecoder.reset();
			ASSERT_EQ(1, mcdecoder.ManchesterBits.getValue(220));
			ASSERT_EQ(0, mcdecoder.ManchesterBits.getValue(221));
			ASSERT_EQ(1, mcdecoder.ManchesterBits.getValue(222));


		}

		
		TEST_F(Tests,mcLoop1) // causes endless loop
		{
			std::string dstr2 = "MU;P0=-623;P1=231;P2=599;P3=-243;P4=-839;P5=857;D=01023102310102310145454541023102323232310102310101010102323101010232323231023232310102310101023102310102310101010102323101010232323231023232310102310101023102310102310545454541023102323232310102310101010102323101010232323231023232310102310101023102310102;";
			state =  import_sigdata(&dstr2, false);
			// std::cout << outputStr << "\n";

			ASSERT_EQ(254,ooDecode.messageLen);
			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_TRUE(mcdecoder.isManchester());
			bool result = mcdecoder.doDecode();
			ASSERT_EQ(false, result);
			ASSERT_EQ(0, mcdecoder.ManchesterBits.valcount);
		}

		TEST_F(Tests,mcLoop2) // causes endless loop
		{
			std::string dstr2 = "MU;P0=-251;P1=231;P2=-613;P3=840;P4=-858;P5=607;D=01234343434125012505052501212121212505012105012121250505050125050125050125012125050125050501234343434125012505052501212121212505012121250505050125050125050125012125050125050501234343434125012505050501212501212121212505012121250505050125050125050125012125;";
			state =  import_sigdata(&dstr2, false);
			// std::cout << outputStr << "\n";

			ASSERT_EQ(254,ooDecode.messageLen);
			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_TRUE(mcdecoder.isManchester());
			bool result = mcdecoder.doDecode();
			ASSERT_EQ(false, result);
			ASSERT_EQ(0, mcdecoder.ManchesterBits.valcount);
		}

		TEST_F(Tests,mcLong2) //Maverick et733
		{
				const int pause = 5000;
				const int pulse = -100;
				//ooDecode.mcdecoder = &mcdecoder;
				std::string dstr2 = "AA9995595555595999A9A9A669";
				for (uint8_t r = 0; r < 4; r++)
				{
					for (uint8_t i = 0; i < 8; i++)
					{
						DigitalSimulate(pause);
						DigitalSimulate(pulse);
					}
					DigitalSimulate(pause);
					state = import_mcdata(&dstr2, 0, dstr2.length(), 250);
				}
				DigitalSimulate(-32001);


				// std::cout << outputStr << "\n";

				ASSERT_EQ(147,ooDecode.messageLen);
				ooDecode.calcHisto();
				ooDecode.printOut();
				ASSERT_TRUE(mcdecoder.isManchester());
				bool result = mcdecoder.doDecode();
				ASSERT_EQ(104, mcdecoder.ManchesterBits.valcount);
				std::string mcStr;
				mcStr=mcdecoder.getMessageHexStr();
				ASSERT_STREQ(mcStr.c_str(), dstr2.c_str()); // may not compile or give warning
		}


		TEST_F(Tests, mcMaverick1)
		{
			// Fehlerhafte Taktdate beim Sender. Erkennung als MC nicht möglich.
			// protocolid: temp1=24, temp2=-532;
			//std::string dstr = "MU;P0=-4913;P1=228;P2=361;P3=-632;P4=-382;P5=153;P6=106;D=0101010101010101023232323245354245354245323232323542463232323232323232323236424;";
			std::string dstr = "MU;P0=-288;P1=211;P2=467;P3=-4872;P4=-527;D=3131313131313131324242424201410201410201424242424102014102014242410201410242014242424242424242424242424241024201410242014102420142424102014102;";
			std::string baseStr = "AA999559959A5555555A69A566"; // Invertiert FFAA999559959A5555555A69A566
			import_sigdata(&dstr);     
			ooDecode.printOut();

			ASSERT_EQ(142, ooDecode.messageLen);
			ooDecode.calcHisto();
			ASSERT_TRUE(mcdecoder.isManchester());
			ASSERT_TRUE(mcdecoder.doDecode());
			ASSERT_EQ(103, mcdecoder.ManchesterBits.valcount);
			//std::cout << geFullMCString();

			std::string mcStr;
			mcStr = mcdecoder.getMessageHexStr();
			ASSERT_STREQ(mcStr.c_str(), baseStr.c_str()); // may not compile or give warning

		}

		TEST_F(Tests, mcbufferfullresume)
		{
			bool state;
			ooDecode.mcdecoder = &mcdecoder; //Inject our mcdecoder object into ooDecode object
			//std::string dstr2 = "0B0F9FFA555AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAAA";
			std::string dstr2 = "0B0F9FFA555AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB63AAA";
			state = import_mcdata(&dstr2, 0, dstr2.length(), 450);
			DigitalSimulate(1);
			ooDecode.printOut();
			ASSERT_FALSE(state);
			ASSERT_EQ(4, ooDecode.messageLen);  // 254+3 is the number of pulses.
			ASSERT_TRUE(ooDecode.mcDetected);  
			ASSERT_EQ(223, mcdecoder.ManchesterBits.valcount-1);  // 254+3 is the number of pulses. At this time, we have processed only the buffer after bufferoverflow which is equal to 224 bits

			ooDecode.calcHisto();
			ooDecode.printOut();

			/*mcdecoder.shorthigh = 0;
			mcdecoder.shortlow = 1;
			mcdecoder.longlow = 2;
			mcdecoder.longhigh = 3;
			mcdecoder.clock = 450;
			*/

			bool result = mcdecoder.doDecode();
			ASSERT_EQ(226, mcdecoder.ManchesterBits.valcount - 1);
			ASSERT_TRUE(result);

			std::string mcStr;
			std::string base;
			mcStr = mcdecoder.getMessageHexStr();
			ASSERT_STREQ(mcStr.c_str(), dstr2.c_str()); // may not compile or give warning
														//mcdecoder.reset();
			ASSERT_EQ(1, mcdecoder.ManchesterBits.getValue(220));
			ASSERT_EQ(0, mcdecoder.ManchesterBits.getValue(221));
			ASSERT_EQ(1, mcdecoder.ManchesterBits.getValue(222));
		}

		TEST_F(Tests, mcOSV11)
		{
				bool state;

				std::string dstr = "MU;P0=-32001;P1=1673;P2=-1260;P3=-4304;P4=5712;P5=-6752;P6=3145;P7=-2718;D=12121212121212121212121345126712671212121267621712121212121212121212621712126;CP=1;";
				state = import_sigdata(&dstr);
				dstr =  "MU;P0=-27224;P1=1673;P2=-1260;P3=-4304;P4=5712;P5=-6752;P6=3145;P7=-2718;D=012121212121212121212121345126712671212121267621712121212121212121212621712126;CP=1;"; // Lange pause zwischen den Wiederholungen einbauen
			
				std::string mcHex = "DBE9FFCE";
				std::string lenStr = ";L=32";

				state = import_sigdata(&dstr);

				ASSERT_FALSE(mcdecoder.isManchester());
				ooDecode.calcHisto();
				ooDecode.getClock();
				ooDecode.getSync();

				//ooDecode.printOut();

				ASSERT_TRUE(mcdecoder.isManchester());
				ASSERT_TRUE(mcdecoder.doDecode());

				std::string mcStr;


				mcStr=mcdecoder.getMessageHexStr();
				ASSERT_STREQ(mcStr.c_str(), mcHex.c_str()); // may not compile or give warning
				mcStr.clear();
				mcStr =mcdecoder.getMessageLenStr();
				ASSERT_STREQ(mcStr.c_str(), lenStr.c_str()); // may not compile or give warning
										  // Second repeat
				ooDecode.printOut();
				mcdecoder.reset();
				ooDecode.calcHisto();
				ooDecode.getClock();
				ooDecode.getSync();
				ASSERT_TRUE(mcdecoder.isManchester());
				ASSERT_TRUE(mcdecoder.doDecode());
				mcStr=mcdecoder.getMessageHexStr();
				ASSERT_STREQ(mcStr.c_str(), mcHex.c_str()); // may not compile or give warning

		}

		TEST_F(Tests, mcHideki2)
		{
			// protocol not invert
			// model=Hideki_30, sensor id=a1, channel=4, cnt=1, bat=ok, temp=23.1, humidity=28,
			int data[] = { 996, -972, 980, -980, 968, -492, 488, -492, 488, -972, 488, -492, 976, -492, 488, -484, 488, -492, 488, -488, 492, -972, 488, -488, 980, -972, 488, -492, 980, -968, 980, -492, 488, -488, 484, -980, 976, -976, 488, -488, 980, -980, 972, -976, 492, -488, 972, -488, 492, -980, 972, -488, 488, -492, 488, -488, 484, -980, 976, -492, 480, -492, 488, -980, 488, -492, 972, -488, 488, -492, 488, -484, 488, -488, 492, -980, 968, -980, 492, -488, 488, -484, 488, -492, 976, -492, 488, -972, 488, -492, 488, -492, 480, -492, 488, -488, 492, -488, 488, -484, 980, -980, 968, -980, 980, -980, 480, -492, 976, -492, 488, -484, 488, -980, 488, -492, 488, -480, 492, -488, 492, -488, 972, -488, 492, -976, 980, -972, 980, -488, 488, -972, 492, -488, 116, -32001 };
			uint16_t len = sizeof(data) / sizeof(data[0]);
			uint16_t i = 0;
			bool decoded;
			ooDecode.mcdecoder = &mcdecoder;
			for (int j = 0, i=10; j <3; j++)
			{
				for ( ;i < len; i++)
				{
					state = ooDecode.decode(&data[i]);
					if (state) {
						decoded = true;
						// std::cout << outputStr;
						outputStr = "";
						// std::cout << geFullMCString();

					}
				}
				i = 0;
			}
			ASSERT_TRUE(state);

		}
		TEST_F(Tests, mcHideki3)
		{
			// model=Hideki_30, sensor id=bb, channel=4, cnt=1, bat=ok, temp=22.7, humidity=28, 
			// hideki protocol not inverted
			std::string dstr = "MU;P0=-100;P1=943;P2=-1011;P3=-539;P4=437;D=01212134342431243121342434312134342124312124313421213434243434343134343434212434343134243434312431212121342124343434343134342434343430;";

			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//ASSERT_TRUE(state);
			// std::cout << geFullMCString();
		                      
			std::string hexRef = "51B4E8B5947C179ED52FC78";
			std::string lenRef = ";L=89";

			ASSERT_STREQ(mcdecoder.getMessageHexStr(), hexRef.c_str());
			ASSERT_STREQ(mcdecoder.getMessageLenStr(), lenRef.c_str());
		}

		TEST_F(Tests, mcHideki4)
		{
			// model=Hideki_30, sensor id=bb, channel=4, cnt=3, bat=ok, temp=22.7, humidity=28, 
			//hideki protocol not inverted  
			std::string dstr = "MU;P0=-12568;P1=264;P2=-4668;P3=948;P4=-1008;P5=-514;P6=456;D=012343435656465346534356465653435656434653434653465343565646565656535656565643465656535646565653465343434356434656534653465653465650;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//ASSERT_TRUE(state);
			//std::cout << geFullMCString();

			std::string hexRef = "A8DA745ADA3E0BCF6A976EC";
			std::string lenRef = ";L=90";
			ASSERT_STREQ(mcdecoder.getMessageLenStr(), lenRef.c_str());
			ASSERT_STREQ(mcdecoder.getMessageHexStr(), hexRef.c_str());
		}

		TEST_F(Tests, mcHideki5)
		{
			// model=Hideki_30, sensor id=bb, channel=4, cnt=2, bat=ok, temp=22.7, humidity=28,
			//hideki protocol not inverted
			std::string dstr = "MU;P0=-32001;P1=922;P2=-1033;P3=-546;P4=425;D=0121213434243124312134243431213434212431212434312121343424343434313434343421243434313424343431243121212134212431243434312134243430;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//ASSERT_TRUE(state);
			// std::cout << geFullMCString();

			std::string hexRef = "A8DA745AEA3E0BCF6A96F4C";
			std::string lenRef = ";L=90";
			ASSERT_STREQ(mcdecoder.getMessageLenStr(), lenRef.c_str());
			ASSERT_STREQ(mcdecoder.getMessageHexStr(), hexRef.c_str());
		}

		TEST_F(Tests, mcHideki6)
		{
			// model=Hideki_30, sensor id=bb, channel=4, cnt=3, bat=ok, temp=22.7, humidity=28,
			//hideki protocol not inverted  
			std::string dstr = "MU;P0=-32001;P1=272;P2=-1007;P3=944;P5=-536;P6=444;D=12323235656265326532356265653235656232653232653265323565626565656535656565623265656535626565653265323232356232656532653265653265650;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//ASSERT_TRUE(state);
			// std::cout << geFullMCString();

			std::string hexRef = "A8DA745ADA3E0BCF6A976EC";
			std::string lenRef = ";L=90";
			ASSERT_STREQ(mcdecoder.getMessageLenStr(), lenRef.c_str());
			ASSERT_STREQ(mcdecoder.getMessageHexStr(), hexRef.c_str());
		}
		TEST_F(Tests, mcHideki7)
		{
			//model=Hideki_30, sensor id=bb, channel=4, cnt=1, bat=ok, temp=22.7, humidity=28,
			//hideki protocol not inverted   
			std::string dstr = "MU;P0=-544;P1=956;P2=-1016;P3=436;D=01212103032301230121032303012103032123012123010321210303230303030103030303212303030103230303012301212121032123030303030103032303030300;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//ASSERT_TRUE(state);
			// std::cout << geFullMCString();

			std::string hexRef = "A8DA745ACA3E0BCF6A97E3C";
			std::string lenRef = ";L=90";
			ASSERT_STREQ(mcdecoder.getMessageLenStr(), lenRef.c_str());
			ASSERT_STREQ(mcdecoder.getMessageHexStr(), hexRef.c_str());
		}

		TEST_F(Tests, mcInvalidMC)
		{
			// FS20
			std::string dstr = "DMc;P0=-2448;P1=403;P2=-388;P3=308;P4=620;P5=-603;P6=-8960;D=01212321212121212121212124512121245124545451212451212121212124512121212121212121212124512124545124512451212121245454545451212454512121216121212121212121212121212451212124512454545121245121212121212451212121212121212121212451212454512451245121212124545454;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_FALSE(mcdecoder.isManchester());

			ASSERT_TRUE(mcdecoder.doDecode());
			//ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_FALSE(state);
		}



		TEST_F(Tests, mc_calcHistodebug)
		{
			// OSV2
			std::string dstr = "DMc;P0=9156;P1=-11380;P2=879;P3=-1069;P4=-598;P5=383;P6=-3072;D=01232323232323232323232453542453542324532354245323542323245354232323245323542453542453542324532354245323232323542453235424532354245323542324532323232323232354232323232453232323235424535424532356232323232323232323232323232323232453542453542324532354245323;";
			ooDecode.mcdecoder = &mcdecoder;
			state = import_sigdata(&dstr);
			ooDecode.processMessage();
			ASSERT_TRUE(ooDecode.success);
			// std::cout << outputStr;
		}

		TEST_F(Tests, mcInvalidMC2)
		{
			std::string dstr = "DMC;P0=32001;P1=-24044;P2=404;P3=-384;P4=596;P5=-600;P6=-9660;D=01232323232323232323232323454523232345454523234545452323452323234545454545232345232323232323234523452345454545232345462323232323232323232323234545232323454545232345454523234523232345454545452323452323232323232345234523454545452323454623232323232323232323;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//std::cout << outputStr;

			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_FALSE(mcdecoder.isManchester());

			ASSERT_TRUE(mcdecoder.doDecode());
			//ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_FALSE(state);

		}

		TEST_F(Tests, mcInvalidMC3)
		{
			std::string dstr = "DMC;P0=-32001;P1=-24044;P2=404;P3=-384;P4=596;P5=-600;P6=-9660;D=01232323232323232323232323454523232345454523234545452323452323234545454545232345232323232323234523452345454545232345462323232323232323232323234545232323454545232345454523234523232345454545452323452323232323232345234523454545452323454623232323232323232323;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//std::cout << outputStr;

			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_FALSE(mcdecoder.isManchester());

			ASSERT_TRUE(mcdecoder.doDecode());
			//ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_FALSE(state);

		}

		TEST_F(Tests, mcInvalidMC4)
		{
			std::string dstr = "DMC;P0=-402;P1=-9280;P2=208;P3=-603;P4=382;P7=585;D=2343232373737340404040737340407340407373737373404040404040404040404040404073737373407373404041;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//std::cout << outputStr;

			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_FALSE(mcdecoder.isManchester());

			ASSERT_TRUE(mcdecoder.doDecode());
			//ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_FALSE(state);

		}

		TEST_F(Tests, mcInvalidMC5)
		{
			std::string dstr = "DMC;P0=-402;P1=228;P3=394;P4=575;P5=-615;P6=-9296;D=010103040404040304545303045454530303030454530304530304530303030303045304530303030303030303030303030453030304536;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//std::cout << outputStr;

			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_FALSE(mcdecoder.isManchester());

			ASSERT_TRUE(mcdecoder.doDecode());
			//ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_FALSE(state);

		}
		TEST_F(Tests, mcInvalidMC6)
		{
			std::string dstr = "DMC;P0=581;P1=-398;P2=-9512;P3=248;P4=-600;P5=385;P6=188;D=23454646404040451515151040451510451510451515151515104045151515151515151515151515151045151045152;";
			ooDecode.mcdecoder = &mcdecoder;

			state = import_sigdata(&dstr);
			//std::cout << outputStr;

			ooDecode.calcHisto();
			ooDecode.printOut();
			ASSERT_FALSE(mcdecoder.isManchester());


			ASSERT_TRUE(mcdecoder.doDecode());
			//ASSERT_FALSE(mcdecoder.isManchester());
			ASSERT_FALSE(state);
		}
		
		TEST_F(Tests, msHeidemann)
		{	
			unsigned int DMSG = 0x610;
			unsigned int lastDMSG = DMSG + 2;
			std::string bstr;

			// Nachrichten senden
			for (DMSG; DMSG <= lastDMSG; DMSG++)
			{
				//Wiederholungen
				for (byte Repeat = 0; Repeat < 5; Repeat++)
				{
					// Start sequenz
					DigitalSimulate(-5000);
					DigitalSimulate(330);

					// 12 Bits senden
					unsigned int msg = DMSG;
					for (unsigned int i = 0; i < 12; ++i)
					{
						unsigned long long bit = msg & 1;
						if (bit == 1)
						{
							DigitalSimulate(-330);
							DigitalSimulate(660);
						}
						else //zero
						{
							DigitalSimulate(-660);
							DigitalSimulate(330);
						}
						msg >>= 1;
					}
					if (Repeat < 2) {
						ASSERT_FALSE(state);
					}
				}
				DigitalSimulate(-32001); // Pause zwischen Wiederholungen
				DigitalSimulate(0); // Letzten Pulse (Pause) hier enden lassen

				switch (DMSG)
				{
				case 0x610:                                            // 0 0 0 0 1 0 0 0 0 1 1 0
					bstr = "MS;P0=-5000;P1=330;P2=-660;P3=-330;P4=660;D=10121212121342121212134342;CP=1;SP=0;m2;";
					break;
				case 0x611:                                            // 1 0 0 0 1 0 0 0 0 1 1 0
					bstr = "MS;P0=-5000;P1=330;P2=-660;P3=-330;P4=660;D=10121212121342121212134342;CP=1;SP=0;m1;";
					break;                                          
				case 0x612:                                            // 0 1 0 0 1 0 0 0 0 1 1 0
					bstr = "MS;P0=-5000;P1=330;P2=-330;P3=660;P4=-660;D=10123414141234141414123234;CP=1;SP=0;m1;";
					break;
				}
				int msgStartPos = outputStr.find_first_of(MSG_START) + 1;
				int msgEndPos = outputStr.find_first_of(MSG_END, msgStartPos);
				std::string Message = outputStr.substr(msgStartPos, msgEndPos - msgStartPos);

				//printf("%i - %s\n", DMSG, Message.c_str());
				//ASSERT_STREQ(Message.c_str(), bstr.c_str());  // Disabled this test

				outputStr = "";
			}
		}
		
		TEST_F(Tests, msS522)
		{
			//S522  temperature : 24.0  batteryState : RSSI : -58.5

			std::string dstr = "MS;P1=-8055;P2=488;P3=-2049;P4=-3956;D=2121232324232323232323242323242323242323232424242324242323232323232323232323232324242324;";

			for (int i = 0; i < 3; i++)
			{
				state = import_sigdata(&dstr);
				//if (i==2) 			ASSERT_TRUE(state);

			}
			int msgStartPos = outputStr.find_first_of(MSG_START) + 1;
			int msgEndPos = outputStr.find_first_of(MSG_END, msgStartPos);
			std::string Message = outputStr.substr(msgStartPos, msgEndPos - msgStartPos);
			std::string bstr = "MS;P0=488;P1=-8055;P2=-2049;P3=-3956;D=0101020203020202020202030202030202030202020303030203030202020202020202020202020203030203;CP=0;SP=1;O;m2;";
			ASSERT_STREQ(Message.c_str(), bstr.c_str()) << "Message = " << Message.c_str() << "bstr = " << bstr.c_str();
		}



		TEST_F(Tests, muHeidemann)
		{
			unsigned int DMSG = 0x610;
			unsigned int lastDMSG = DMSG + 2;
			std::string bstr;

			//ooDecode.MSenabled = false;
			// Nachrichten senden
			for (DMSG; DMSG <= lastDMSG; DMSG++)
			{
				//Wiederholungen
				for (byte Repeat = 0; Repeat < 5; Repeat++)
				{
					// Start sequenz
					DigitalSimulate(-5000);
					DigitalSimulate(330);
					// 12 Bits senden
					unsigned int msg = DMSG;
					for (unsigned int i = 0; i < 12; ++i)
					{
						unsigned long long bit = msg & 1;
						//printf("%i ", bit);
						if (bit == 1) {
							DigitalSimulate(-330);
							DigitalSimulate(660);
						} else //zero
						{
							DigitalSimulate(-660);
							DigitalSimulate(330);
						}
						msg >>= 1;
					}
					//printf("\n");
					if (Repeat < 2) {
						ASSERT_FALSE(state);
					}
					
				}
				DigitalSimulate(-32001); // Pause zwischen Wiederholungen
				DigitalSimulate(0); // Letzten Pulse (Pause) hier enden lassen

				switch (DMSG)
				{
					case 0x610 :                                            // 0 0 0 0 1 0 0 0 0 1 1 0
						bstr = "MU;P0=-5000;P1=330;P2=-660;P3=-330;P4=660;D=0121212121342121212134342101212121213421212121343421012121212134212121213434210121212121342121212134342101212121213421212121343421;CP=1;";
						break;
					case 0x611:                                            // 1 0 0 0 1 0 0 0 0 1 1 0
						bstr = "MU;P0=-5000;P1=330;P2=-330;P3=660;P4=-660;D=0123414141234141414123234101234141412341414141232341012341414123414141412323410123414141234141414123234101234141412341414141232341;CP=1;";
						break;                                          //  0123414141234141414123234101234141412341414141232341012341414123414141412323410123414141234141414123234101234141412341414141232341
					case 0x612:                                            // 0 1 0 0 1 0 0 0 0 1 1 0
						bstr = "MU;P0=-5000;P1=330;P2=-660;P3=-330;P4=660;D=0121342121342121212134342101213421213421212121343421012134212134212121213434210121342121342121212134342101213421213421212121343421;CP=1;";
						break;
				}
				int msgStartPos = outputStr.find_first_of(MSG_START) + 1;
				int msgEndPos =  outputStr.find_first_of(MSG_END, msgStartPos);
				std::string Message = outputStr.substr(msgStartPos, msgEndPos - msgStartPos);

				//printf("%i - %s\n", DMSG, Message.c_str());
				ASSERT_STREQ(Message.c_str(), bstr.c_str());

				outputStr = "";
			}
		}

		TEST_F(Tests, muReedTripRadio1)
		{
			std::string dstr = "MU;P0=-1532;P1=337;P2=-5136;P3=998;P4=-333;P5=-982;P6=-10211;D=012343434343434343434343434151515153415151516343434343434343434343434343434341515151534151515163434343434343434343434343434343415151515341515151634343434343434343434343434343434151515153415151;CP=3;R=58;";

			state = import_sigdata(&dstr);

			DigitalSimulate(-32001); // Pause zwischen Wiederholungen
			DigitalSimulate(1); // Letzten Pulse (Pause) hier enden lassen

			// ooDecode.printOut();

			int msgStartPos = outputStr.find_first_of(MSG_START) + 1;
			int msgEndPos = outputStr.find_first_of(MSG_END, msgStartPos);
			std::string Message = outputStr.substr(msgStartPos, msgEndPos - msgStartPos);
			std::string bstr = "MS;P1=337;P3=998;P4=-333;P5=-982;P6=-10211;D=16343434343434343434343434343434341515151534151515;CP=1;SP=6;m2;";
			ASSERT_STREQ(Message.c_str(), bstr.c_str());  
		}

		TEST_F(Tests, muReedTripRadio2)
		{
			std::string dstr = "MU;P0=-20560;P1=988;P2=-342;P3=335;P4=-1007;P5=-10281;D=012121212121212121212121212121212343434341234123435121212121212121212121212121212123434343412341234351212121212121212121212121212121234343434123412343512121212121212121212121212121212343434341234123;CP=1;R=47;";

			state = import_sigdata(&dstr);

			DigitalSimulate(1); // Letzten Pulse (Pause) hier enden lassen
			DigitalSimulate(-32001); // Pause zwischen Wiederholungen
			

			// ooDecode.printOut();

			int msgStartPos = outputStr.find_first_of(MSG_START) + 1;
			int msgEndPos = outputStr.find_first_of(MSG_END, msgStartPos);
			std::string Message = outputStr.substr(msgStartPos, msgEndPos - msgStartPos);
			std::string bstr = "MS;P1=988;P2=-342;P3=335;P4=-1007;P5=-10281;D=35121212121212121212121212121212123434343412341234;CP=3;SP=5;m2;";
			ASSERT_STREQ(Message.c_str(), bstr.c_str());  
		}
		

	  //--------------------------------------------------------------------------------------------------
	  /*
	  TEST_F(Tests, testDigitalPinString)
	  {
		std::string sHIGH = toDigitalPinString(HIGH);
		std::string sLOW = toDigitalPinString(LOW);
		std::string sOTHER = toDigitalPinString(234);

		ASSERT_EQ("HIGH", sHIGH);
		ASSERT_EQ("LOW", sLOW);
		ASSERT_TRUE(sHIGH != sOTHER && sLOW != sOTHER);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testBitOrderString)
	  {
		std::string sMSBFIRST = toBitOrderString(MSBFIRST);
		std::string sLSBFIRST = toBitOrderString(LSBFIRST);
		std::string sOTHER = toBitOrderString(234);

		ASSERT_EQ("MSBFIRST", sMSBFIRST);
		ASSERT_EQ("LSBFIRST", sLSBFIRST);
		ASSERT_TRUE(sMSBFIRST != sOTHER && sLSBFIRST != sOTHER);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testPinModeString)
	  {
		std::string sOUTPUT = toPinModeString(OUTPUT);
		std::string sINPUT = toPinModeString(INPUT);
		std::string sINPUT_PULLUP = toPinModeString(INPUT_PULLUP);
		std::string sOTHER = toPinModeString(234);

		ASSERT_EQ("OUTPUT", sOUTPUT);
		ASSERT_EQ("INPUT", sINPUT);
		ASSERT_EQ("INPUT_PULLUP", sINPUT_PULLUP);
		ASSERT_TRUE(sOUTPUT != sOTHER && sINPUT != sOTHER && sINPUT_PULLUP != sOTHER);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testPinMode)
	  {
		pinMode(13, OUTPUT);      // sets the digital pin 13 as output
		std::string pinOutput = arduino_stub::getLastCommand();
		ASSERT_EQ("pinMode(13, OUTPUT);\n", pinOutput);

		pinMode(13, INPUT);      // sets the digital pin 7 as input
		std::string pinInput = arduino_stub::getLastCommand();
		ASSERT_EQ("pinMode(13, INPUT);\n", pinInput);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testDigitalWrite)
	  {
		std::string lastCall;

		digitalWrite(13, HIGH);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("digitalWrite(13, HIGH);\n", lastCall);

		digitalWrite(13, LOW);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("digitalWrite(13, LOW);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testAnalogWrite)
	  {
		std::string lastCall;

		analogWrite(13, 56);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("analogWrite(13, 56);\n", lastCall);

		analogWrite(13, 255);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("analogWrite(13, 255);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testDigitalRead)
	  {
		std::string lastCall;
		uint8_t value;

		value = digitalRead(13);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("digitalRead(13);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testAnalogRead)
	  {
		std::string lastCall;
		uint16_t value;

		value = analogRead(13);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("analogRead(13);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testAnalogReadResolution)
	  {
		std::string lastCall;

		analogReadResolution(8);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("analogReadResolution(8);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testAnalogWriteResolution)
	  {
		std::string lastCall;

		analogWriteResolution(7);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("analogWriteResolution(7);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testShiftOut)
	  {
		std::string lastCall;

		shiftOut(3, 4, MSBFIRST, 78);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("shiftOut(3, 4, MSBFIRST, 78);\n", lastCall);

		shiftOut(5, 6, LSBFIRST, 99);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("shiftOut(5, 6, LSBFIRST, 99);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testShiftIn)
	  {
		std::string lastCall;
		uint8_t value;

		value = shiftIn(3, 4, MSBFIRST);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("shiftIn(3, 4, MSBFIRST);\n", lastCall);

		value = shiftIn(5, 6, LSBFIRST);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("shiftIn(5, 6, LSBFIRST);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testPulseIn)
	  {
		std::string lastCall;
		uint32_t value;

		value = pulseIn(3, HIGH);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("pulseIn(3, HIGH);\n", lastCall);

		value = pulseIn(4, LOW);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("pulseIn(4, LOW);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testMicrosRealtime)
	  {
		arduino_stub::setClockStrategy(arduino_stub::CLOCK_REALTIME);
		std::string lastCall;

		uint32_t value1 = micros();
		delay(10);
		uint32_t value2 = micros();

		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("micros();\n", lastCall);

		ASSERT_GT(value2, value1);

		uint32_t elapsedMicros = value2-value1;
		ASSERT_NEAR(10000, elapsedMicros, 5000); //no usec precision, only milliseconds. Allows 5ms diff
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testMillisRealtime)
	  {
		arduino_stub::setClockStrategy(arduino_stub::CLOCK_REALTIME);
		std::string lastCall;

		uint32_t value1 = millis();
		delay(30);
		uint32_t value2 = millis();

		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("millis();\n", lastCall);

		ASSERT_GT(value2, value1);

		uint32_t elapsedMillis = value2-value1;
		ASSERT_NEAR(30, elapsedMillis, 4); //4ms epsilon
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testMicrosSimulation)
	  {
		arduino_stub::setClockStrategy(arduino_stub::CLOCK_SIMULATION);
		std::string lastCall;

		uint32_t value1 = micros();
		delay(10);
		uint32_t value2 = micros();

		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("micros();\n", lastCall);

		ASSERT_GT(value2, value1);

		uint32_t elapsedMicros = value2-value1;
		ASSERT_NEAR(10000, elapsedMicros, 10); //10 usec max epsilon (emulated)
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testMillisSimulation)
	  {
		arduino_stub::setClockStrategy(arduino_stub::CLOCK_SIMULATION);
		std::string lastCall;

		uint32_t value1 = millis();
		delay(30);
		uint32_t value2 = millis();

		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("millis();\n", lastCall);

		ASSERT_GT(value2, value1);

		uint32_t elapsedMillis = value2-value1;
		ASSERT_EQ(30, elapsedMillis); //0ms epsilon
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testMath)
	  {
		std::string lastCall;

		ASSERT_EQ(5, abs(5));
		ASSERT_EQ(5, abs(-5));

		ASSERT_EQ(8, max(5, 8));
		ASSERT_EQ(-5, max(-5, -7));-

		ASSERT_EQ(5, min(5, 8));
		ASSERT_EQ(-7, min(-5, -7));

		ASSERT_EQ(5, constrain(3, 5, 8));
		ASSERT_EQ(6, constrain(6, 5, 8));
		ASSERT_EQ(8, constrain(9, 5, 8));

		ASSERT_EQ(10, map(1, 0, 10, 0, 100));
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testInterruptModeString)
	  {
		std::string sCHANGE = toInterruptModeString(CHANGE);
		std::string sRISING = toInterruptModeString(RISING);
		std::string sFALLING = toInterruptModeString(FALLING);
		std::string sOTHER = toInterruptModeString(234);

		ASSERT_EQ("CHANGE", sCHANGE);
		ASSERT_EQ("RISING", sRISING);
		ASSERT_EQ("FALLING", sFALLING);

		ASSERT_TRUE(sCHANGE   != sOTHER &&
					sRISING   != sOTHER &&
					sFALLING  != sOTHER );
	  }
	  //--------------------------------------------------------------------------------------------------
	  void myInterruptFunction()
	  {
	  }
	  TEST_F(Tests, testAttachInterrupt)
	  {
		std::string lastCall;

		attachInterrupt(2, &myInterruptFunction, RISING);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_NE(std::string::npos, lastCall.find("attachInterrupt(2, "));
		ASSERT_NE(std::string::npos, lastCall.find(", RISING);"));
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testDetachInterrupt)
	  {
		std::string lastCall;

		detachInterrupt(3);
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("detachInterrupt(3);\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testNoInterrupts)
	  {
		std::string lastCall;

		noInterrupts();
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("noInterrupts();\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testInterrupts)
	  {
		std::string lastCall;

		interrupts();
		lastCall = arduino_stub::getLastCommand();
		ASSERT_EQ("interrupts();\n", lastCall);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testRandom)
	  {
		int16_t seed = 0x5656;
		randomSeed(seed);

		bool found[15] = {0};
		for(int i=0; i<100; i++)
		{
		  int32_t value = random(5, 10);
		  found[value] = true;
		}

		ASSERT_EQ(false, found[0]);
		ASSERT_EQ(false, found[1]);
		ASSERT_EQ(false, found[2]);
		ASSERT_EQ(false, found[3]);
		ASSERT_EQ(false, found[4]);
		ASSERT_EQ( true, found[5]);
		ASSERT_EQ( true, found[6]);
		ASSERT_EQ( true, found[7]);
		ASSERT_EQ( true, found[8]);
		ASSERT_EQ( true, found[9]);
		ASSERT_EQ(false, found[10]);
		ASSERT_EQ(false, found[11]);
		ASSERT_EQ(false, found[12]);
		ASSERT_EQ(false, found[13]);
		ASSERT_EQ(false, found[14]);

		memset(&found, 0, sizeof(found));
		for(int i=0; i<100; i++)
		{
		  int32_t value = random(5);
		  found[value] = true;
		}

		ASSERT_EQ(true, found[0]);
		ASSERT_EQ(true, found[1]);
		ASSERT_EQ(true, found[2]);
		ASSERT_EQ(true, found[3]);
		ASSERT_EQ(true, found[4]);
		ASSERT_EQ(false, found[5]);
		ASSERT_EQ(false, found[6]);
		ASSERT_EQ(false, found[7]);
		ASSERT_EQ(false, found[8]);
		ASSERT_EQ(false, found[9]);
		ASSERT_EQ(false, found[10]);
		ASSERT_EQ(false, found[11]);
		ASSERT_EQ(false, found[12]);
		ASSERT_EQ(false, found[13]);
		ASSERT_EQ(false, found[14]);

	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testBitsAndBytes)
	  {
		uint32_t value = 0xC2000044;

		ASSERT_EQ(0x44, lowByte(value));
		ASSERT_EQ(0xC2, highByte(value));

		//assert bitRead(), 0x44 low byte
		ASSERT_EQ(0, bitRead(value, (uint32_t)0));
		ASSERT_EQ(0, bitRead(value, (uint32_t)1));
		ASSERT_EQ(1, bitRead(value, (uint32_t)2));
		ASSERT_EQ(0, bitRead(value, (uint32_t)3));
		ASSERT_EQ(0, bitRead(value, (uint32_t)4));
		ASSERT_EQ(0, bitRead(value, (uint32_t)5));
		ASSERT_EQ(1, bitRead(value, (uint32_t)6));
		ASSERT_EQ(0, bitRead(value, (uint32_t)7));

		//assert bitRead(), 0xC2 high byte
		ASSERT_EQ(0, bitRead(value, (uint32_t)24));
		ASSERT_EQ(1, bitRead(value, (uint32_t)25));
		ASSERT_EQ(0, bitRead(value, (uint32_t)26));
		ASSERT_EQ(0, bitRead(value, (uint32_t)27));
		ASSERT_EQ(0, bitRead(value, (uint32_t)28));
		ASSERT_EQ(0, bitRead(value, (uint32_t)29));
		ASSERT_EQ(1, bitRead(value, (uint32_t)30));
		ASSERT_EQ(1, bitRead(value, (uint32_t)31));

		//assert bitWrite(), high byte
		ASSERT_EQ(0xC2000044, value);
		bitWrite(value, (uint32_t)31, (uint32_t)0);
		ASSERT_EQ(0x42000044, value);
		bitWrite(value, (uint32_t)30, (uint32_t)0);
		ASSERT_EQ(0x02000044, value);
		bitWrite(value, (uint32_t)31, (uint32_t)1);
		ASSERT_EQ(0x82000044, value);
		bitWrite(value, (uint32_t)30, (uint32_t)1);
		ASSERT_EQ(0xC2000044, value);

		//assert bitSet() is the same as bitWrite(x, n, 1)
		{
		  uint32_t valueSet = 0;
		  uint32_t valueWrite = 0;
		  for(uint32_t i=0; i<=31; i++)
		  {
			bitWrite(valueWrite, i, (uint32_t)1);
			bitSet(valueSet, i);
			ASSERT_EQ(valueWrite, valueSet);
		  }
		}

		//assert bitClear() is the same as bitWrite(x, n, 0)
		{
		  uint32_t valueSet = 0xFFFFFFFF;
		  uint32_t valueWrite = 0xFFFFFFFF;
		  for(uint32_t i=0; i<=31; i++)
		  {
			bitWrite(valueWrite, i, (uint32_t)0);
			bitClear(valueSet, i);
			ASSERT_EQ(valueWrite, valueSet);
		  }
		}

		//assert bit()
		ASSERT_EQ(    1, bit(0));
		ASSERT_EQ(    2, bit(1));
		ASSERT_EQ(    4, bit(2));
		ASSERT_EQ(    8, bit(3));
		ASSERT_EQ(   16, bit(4));
		ASSERT_EQ(   32, bit(5));
		ASSERT_EQ(   64, bit(6));
		ASSERT_EQ(  128, bit(7));
		ASSERT_EQ(  256, bit(8));
		ASSERT_EQ(  512, bit(9));
		ASSERT_EQ( 1024, bit(10));
		ASSERT_EQ( 2048, bit(11));
		ASSERT_EQ( 4096, bit(12));
		ASSERT_EQ( 8192, bit(13));
		ASSERT_EQ(16384, bit(14));
		ASSERT_EQ(32768, bit(15));
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testSetMicrosResolution)
	  {
		arduino_stub::setClockStrategy(arduino_stub::CLOCK_SIMULATION);
		arduino_stub::setMicrosResolution(1);

		uint32_t a = micros();
		uint32_t b = micros();

		EXPECT_NEAR(a, b, 1);

		//back to default
		arduino_stub::setMicrosResolution(8);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testSetMicrosCounter)
	  {
		arduino_stub::setClockStrategy(arduino_stub::CLOCK_SIMULATION);
		arduino_stub::setMicrosResolution(1);
		uint32_t before = micros();
		arduino_stub::setMicrosCounter(256);

		uint32_t a = micros();

		EXPECT_NEAR(a, 256, 1);

		//back to previous value
		arduino_stub::setMicrosCounter(before);
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testStatusRegister)
	  {
		uint8_t before = SREG;
    
		noInterrupts();
		uint8_t a = SREG;
		interrupts();
		uint8_t b = SREG;
    
		EXPECT_NE(a,b);

		//back to default
		SREG = before;
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testSerial)
	  {
		Serial.print("Hello World from ");
		Serial.print(__FUNCTION__);
		Serial.println("() function!");
	  }
	  //--------------------------------------------------------------------------------------------------
	  TEST_F(Tests, testDemos)
	  {
		demoSerial();
		demoMillis();
		demoLog();
	  }
	  //--------------------------------------------------------------------------------------------------
	  */


	} // End namespace t/est
} // End namespace arduino
