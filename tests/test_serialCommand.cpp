#include <arduino-mock/Arduino.h>
#include <arduino-mock/Serial.h>
#include "SimpleFIFO.h"

#define PROGNAME               " SIGNALESP "
#define VERSION_1              0x33
#define VERSION_2              0x1d
#define BAUDRATE               115200
#define FIFO_LENGTH            200
#define isHigh(pin) (digitalRead(pin) == HIGH)
#define isLow(pin) (digitalRead(pin) == LOW)
#define EE_MAGIC_OFFSET        0
#define addr_features          0xff
#define MAX_SRV_CLIENTS        2



#if defined(GTEST_OS_WINDOWS)
    #define ARDUINO 101
    #define NOSTRING
#endif

using ::testing::_;
using ::testing::Return;
using ::testing::Matcher;
using ::testing::AtLeast;
using ::testing::Invoke;



//SimpleFIFO<int,90> FiFo; //store FIFO_LENGTH # ints
SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints

SignalDetectorClass musterDec;

#include "serialCommandHelper.h"

// The arduino-mock micros() always returns 0; the busy-wait loops in send.h
// would never end. Map micros() to the fake clock for send.h only.
// disableReceive()/enableReceive() are recorded for send.h only, so the tests
// can check whether send_cmd() switches the receiver on again.
namespace serialtest {
inline void recordDisableReceive() { receiveState().disables++; receiveState().enabled = false; disableReceive(); }
inline void recordEnableReceive() { receiveState().enables++; receiveState().enabled = true; enableReceive(); }
} // namespace serialtest
#define micros() serialtest::fakeMicros()
#define disableReceive() serialtest::recordDisableReceive()
#define enableReceive() serialtest::recordEnableReceive()
#include <send.h>
#undef enableReceive
#undef disableReceive
#undef micros
#include <commands.h>

namespace serialtest {
// Runs a short command through the firmware command dispatcher.
// Like serialEvent() in the firmware, the line terminator stays in IB_1,
// so "\n" is appended unless terminate is false.
inline void runShortCommand(const char* cmd, bool terminate = true) {
    std::string line(cmd);
    if (terminate) line += '\n';
    ASSERT_TRUE(loadCommand(line.c_str())) << "command does not fit into IB_1: " << cmd;
    commands::HandleShortCommand();
}
} // namespace serialtest

// ---------------------------------------------------------------------------
// Self tests for the serial test helpers (no firmware involved)
// ---------------------------------------------------------------------------
TEST(serialHelper, captureStringsAndChars) {
    serialtest::SerialCapture cap;
    Serial.print("ab");
    Serial.print('c');
    Serial.println("d");
    Serial.println();
    EXPECT_EQ("abcd\n\n", cap.output());
}

TEST(serialHelper, captureNumbersLikeArduinoPrint) {
    serialtest::SerialCapture cap;
    Serial.print(0, DEC);
    Serial.print(' ');
    Serial.print(-42, DEC);
    Serial.print(' ');
    Serial.print((unsigned char)0x0a, HEX);
    Serial.print(' ');
    Serial.print(255u, HEX);
    Serial.print(' ');
    Serial.print(5L, BIN);
    Serial.print(' ');
    Serial.println(4294967295UL, DEC);
    EXPECT_EQ("0 -42 A FF 101 4294967295\n", cap.output());
}

TEST(serialHelper, captureFlashStrings) {
    serialtest::SerialCapture cap;
    Serial.print(FPSTR(TXT_EEPROM));
    Serial.println(FPSTR(TXT_BLANK));
    EXPECT_EQ("EEPROM \n", cap.output());
}

TEST(serialHelper, loadCommandRespectsBufferSize) {
    EXPECT_TRUE(serialtest::loadCommand("1234567890123"));   // 13 chars + '\0' fit
    EXPECT_STREQ("1234567890123", IB_1);
    EXPECT_FALSE(serialtest::loadCommand("12345678901234")); // 14 chars do not fit
    EXPECT_TRUE(serialtest::loadCommand("P"));
    EXPECT_EQ('\0', IB_1[1]) << "buffer must be zero padded";
}

// Wiring check: helper + real command dispatcher
TEST(serialHelper, runShortCommandReachesDispatcher) {
    serialtest::SerialCapture cap;
    serialtest::runShortCommand("P");
    EXPECT_EQ("OK\n", cap.output());
}

// static size_t println(int, int = DEC);
TEST(serial, println1) {
    SerialMock* serialMock = serialMockInstance();
    int num = 1;
    EXPECT_CALL(*serialMock, print(Matcher<int>(num), Matcher<int>(DEC)))
        .WillRepeatedly(Return(1));
    EXPECT_EQ(1, Serial.print((int)num, DEC));
    releaseSerialMock();
}

TEST(serial, read) {
    SerialMock* serialMock = serialMockInstance();
   
    std::string serialString = "SR;";
    const int length = serialString.length()+1;
    
    // declaring character array (+1 for null terminator)
    char* char_array = new char[length];
 
    // copying the contents of the string to char array
    strcpy(char_array, serialString.c_str());
    
    serialMock->DelegateToSerialFake();
    serialMock->mock_buffer_load(char_array,length,true);

    EXPECT_EQ(length,Serial.available());
    EXPECT_EQ('S',Serial.read());
    EXPECT_EQ('R',Serial.read());
    EXPECT_EQ(';',Serial.read());
    EXPECT_EQ('\0',Serial.read());
    EXPECT_EQ(0,Serial.available());
 
    releaseSerialMock();
}

TEST(serial, command_sendCorrupt) {
    SerialMock* serialMock = serialMockInstance();
   
    std::string serialString = "SR;";
    const int length = serialString.length()+1;
    
    // declaring character array (+1 for null terminator)
    char* char_array = new char[length];
 
    // copying the contents of the string to char array
    // strcpy(char_array, serialString.c_str());
    strcpy(IB_1, serialString.c_str());
    
    serialMock->DelegateToSerialFake();
    serialMock->mock_buffer_load(char_array,length,false);

    // call send_cmd processing
    send_cmd();
    releaseSerialMock();
}

TEST(serial, command_sendCorrupt2) {
    using ::testing::SetArrayArgument;
    using ::testing::StrEq;
    using ::testing::TypedEq;
    using ::testing::Matcher;

    SerialMock* serialMock = serialMockInstance();
   
    std::string serialString = "R=2;";

    const int length = serialString.length()+1;
    
    // declaring character array (+1 for null terminator)
    char* char_array = new char[length];
 
    // copying the contents of the string to char array
    // strcpy(char_array, serialString.c_str());
    strcpy(IB_1, "SR;");
    strcpy(char_array, serialString.c_str());
    
    serialMock->DelegateToSerialFake();
    serialMock->mock_buffer_load(char_array,length,false);

    // We have more in our buffer
    EXPECT_CALL(*serialMock,available())
      .WillRepeatedly(Return(1));

    // Check for abswer
    EXPECT_CALL(*serialMock,write(IB_1,3))
        .WillOnce(Return(1));
    EXPECT_CALL(*serialMock,readBytes)
        .Times(6);
    EXPECT_CALL(*serialMock, print(Matcher<const char*>(StrEq("R=2;"))));
    //EXPECT_CALL(*serialMock,readBytes)
    //    .Times(1).WillOnce(Return(0));

    //EXPECT_CALL(*serialMock, print(Matcher<__FlashStringHelper*>)));
    //EXPECT_CALL(*serialMock, println(Matcher<__FlashStringHelper*>)));

    // call send_cmd processing
    send_cmd();

    releaseSerialMock();
}



TEST(serial, registerWrite1) {
    using ::testing::SetArrayArgument;
    using ::testing::StrEq;
    using ::testing::TypedEq;
    using ::testing::Matcher;

    EEPROMMock* eepromMock = EEPROMMockInstance();
    strcpy(IB_1, "W1400");
    
    EXPECT_CALL(*eepromMock,write)
        .Times(1);

    // call processing
    commands::HandleShortCommand();

    //releaseSerialMock();
    releaseEEPROMMock();
}

TEST(serial, registerWriteVolatile) {
    EEPROMMock* eepromMock = EEPROMMockInstance();
    strcpy(IB_1, "w1400");
    blinkLED = false;

    EXPECT_CALL(*eepromMock,write)
        .Times(0);

    // call processing
    commands::HandleShortCommand();
    EXPECT_TRUE(blinkLED); // unsupported commands return before blinkLED is set

    releaseEEPROMMock();
}

TEST(serial, configVolatile) {
    EEPROMMock* eepromMock = EEPROMMockInstance();
    musterDec.MUenabled = false;
    strcpy(IB_1, "CVEU");

    EXPECT_CALL(*eepromMock,write)
        .Times(0);

    // call processing
    commands::HandleShortCommand();
    EXPECT_TRUE(musterDec.MUenabled);

    strcpy(IB_1, "CVDU");
    commands::HandleShortCommand();
    EXPECT_FALSE(musterDec.MUenabled);

    releaseEEPROMMock();
}

TEST(serial, configPersistent) {
    EEPROMMock* eepromMock = EEPROMMockInstance();
    musterDec.MUenabled = false;
    strcpy(IB_1, "CEU");

    EXPECT_CALL(*eepromMock,write)
        .Times(1);

    // call processing
    commands::HandleShortCommand();
    EXPECT_TRUE(musterDec.MUenabled);

    releaseEEPROMMock();
}
