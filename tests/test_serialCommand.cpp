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

#include <send.h>
#include <commands.h>

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
