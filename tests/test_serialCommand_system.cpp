// Unit tests for the serial system and configuration commands documented in
// AI_AGENT_COMMANDS.md (native build, without CMP_CC1101).
//
// Each command is tested with a valid input and with faulty / boundary inputs.
// The expectations describe the current firmware behaviour; no firmware code
// is changed. See tests/COMMAND_COVERAGE.md for the coverage matrix.

#include <ctime>
#include <regex>

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Return;
using ::testing::StrictMock;

namespace serialtest {

// Saves and restores the global state touched by the command handlers.
class ShortCommandTest : public ::testing::Test {
protected:
    void SetUp() override {
        savedMS_ = musterDec.MSenabled;
        savedMU_ = musterDec.MUenabled;
        savedMC_ = musterDec.MCenabled;
        savedMred_ = musterDec.MredEnabled;
        savedMcMinBitLen_ = musterDec.mcMinBitLen;
        savedHasCC1101_ = hasCC1101;
        savedAfc_ = AfcEnabled;
        blinkLED = false;
    }

    void TearDown() override {
        musterDec.MSenabled = savedMS_;
        musterDec.MUenabled = savedMU_;
        musterDec.MCenabled = savedMC_;
        musterDec.MredEnabled = savedMred_;
        musterDec.mcMinBitLen = savedMcMinBitLen_;
        hasCC1101 = savedHasCC1101_;
        AfcEnabled = savedAfc_;
        blinkLED = false;
        releaseEEPROMMock();
    }

    // EEPROM mock that fails the test on any unexpected access
    EEPROMMock* strictEeprom() {
        EEPROMMock* m = EEPROMMockInstance();
        EXPECT_CALL(*m, read(_)).Times(0);
        EXPECT_CALL(*m, write(_, _)).Times(0);
        return m;
    }

    SerialCapture cap;

private:
    bool savedMS_, savedMU_, savedMC_, savedMred_, savedHasCC1101_, savedAfc_;
    uint8_t savedMcMinBitLen_;
};

// Uptime as the arduino-mock millis() computes it (see arduino-mock Arduino.cc)
inline unsigned long mockUptimeSeconds() {
    const time_t sysTime = time(0) - (time_t)946684800UL;
    return static_cast<unsigned long>((sysTime * 1000) & UINT32_MAX) / 1000;
}

} // namespace serialtest

using serialtest::ShortCommandTest;
using serialtest::runShortCommand;

// ---------------------------------------------------------------------------
// Unknown / empty commands
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, unknownCommandIsRejected) {
    strictEeprom();
    runShortCommand("Z");
    EXPECT_EQ("Unsupported short command\n", cap.output());
    EXPECT_FALSE(blinkLED) << "unsupported commands must not trigger the LED";
}

TEST_F(ShortCommandTest, emptyLineIsRejected) {
    strictEeprom();
    runShortCommand("");
    EXPECT_EQ("Unsupported short command\n", cap.output());
    EXPECT_FALSE(blinkLED);
}

TEST_F(ShortCommandTest, controlCharacterIsRejected) {
    strictEeprom();
    runShortCommand("\x01");
    EXPECT_EQ("Unsupported short command\n", cap.output());
}

TEST_F(ShortCommandTest, commandsAreCaseSensitive) {
    strictEeprom();
    runShortCommand("v");   // 'v' is not 'V'
    EXPECT_EQ("Unsupported short command\n", cap.output());
    cap.clear();
    runShortCommand("p");   // 'p' is not 'P'
    EXPECT_EQ("Unsupported short command\n", cap.output());
}

// ---------------------------------------------------------------------------
// P - ping
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, pingAnswersOK) {
    strictEeprom();
    runShortCommand("P");
    EXPECT_EQ("OK\n", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, pingIgnoresTrailingCharacters) {
    strictEeprom();
    runShortCommand("Pxyz");
    EXPECT_EQ("OK\n", cap.output());
}

TEST_F(ShortCommandTest, pingWithoutLineTerminator) {
    strictEeprom();
    runShortCommand("P", false);
    EXPECT_EQ("OK\n", cap.output());
}

// ---------------------------------------------------------------------------
// V - version
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, versionString) {
    strictEeprom();
    runShortCommand("V");
    const std::string expected =
        std::string("V ") + PROGVERS + PROGNAME + " - compiled at " + __DATE__ + " " + __TIME__ + "\n";
    // __TIME__ of this translation unit and of commands.h are identical (single TU build)
    EXPECT_EQ(expected, cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, versionStartsWithV) {
    strictEeprom();
    runShortCommand("V");
    EXPECT_TRUE(std::regex_match(cap.output(),
        std::regex("^V [0-9]+\\.[0-9]+\\.[0-9]+.* - compiled at .*\n$")))
        << cap.output();
}

TEST_F(ShortCommandTest, versionIgnoresTrailingCharacters) {
    strictEeprom();
    runShortCommand("V");
    const std::string plain = cap.output();
    cap.clear();
    runShortCommand("Vabc");
    EXPECT_EQ(plain, cap.output());
}

// ---------------------------------------------------------------------------
// ? - help
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, helpListsCommands) {
    strictEeprom();
    runShortCommand("?");
    // without CMP_CC1101 the CC1101 commands x and e are not listed
    EXPECT_EQ("? Use one of V R t X S P C r W s \n", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, helpWithoutCC1101HardwareIsIdentical) {
    strictEeprom();
    hasCC1101 = false;
    runShortCommand("?");
    EXPECT_EQ("? Use one of V R t X S P C r W s \n", cap.output());
}

// ---------------------------------------------------------------------------
// R - free RAM
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, freeRamPrintsNumber) {
    strictEeprom();
    runShortCommand("R");
    EXPECT_EQ(std::to_string(freeRam()) + "\n", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, freeRamIgnoresTrailingCharacters) {
    strictEeprom();
    runShortCommand("R123");
    EXPECT_EQ(std::to_string(freeRam()) + "\n", cap.output());
}

// ---------------------------------------------------------------------------
// t - uptime
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, uptimePrintsSeconds) {
    strictEeprom();
    ArduinoMock* arduinoMock = arduinoMockInstance();
    EXPECT_CALL(*arduinoMock, millis()).Times(AnyNumber());

    const unsigned long before = serialtest::mockUptimeSeconds();
    runShortCommand("t");
    const unsigned long after = serialtest::mockUptimeSeconds();
    releaseArduinoMock();

    std::smatch m;
    const std::string out = cap.output();
    ASSERT_TRUE(std::regex_match(out, m, std::regex("^([0-9]+)\n$"))) << out;
    const unsigned long value = std::stoul(m[1]);
    if (before <= after) { // no 32 bit millis overflow between the two samples
        EXPECT_GE(value, before);
        EXPECT_LE(value, after);
    }
    EXPECT_TRUE(blinkLED);
}

// ---------------------------------------------------------------------------
// s - status (CC1101 check)
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, statusWithoutCC1101SupportPrintsNothing) {
    strictEeprom();
    runShortCommand("s");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED) << "s is a known command";
}

// ---------------------------------------------------------------------------
// XQ / XE - receiver off / on
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, receiverDisable) {
    strictEeprom();
    runShortCommand("XQ");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, receiverEnable) {
    strictEeprom();
    runShortCommand("XE");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, receiverWithoutFlag) {
    strictEeprom();
    runShortCommand("X");
    EXPECT_EQ("", cap.output());
}

TEST_F(ShortCommandTest, receiverWithUnknownFlag) {
    strictEeprom();
    runShortCommand("Xz");
    EXPECT_EQ("", cap.output());
    runShortCommand("Xq");  // lower case is not accepted
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// CG - get decoder configuration
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, getConfigAllEnabled) {
    strictEeprom();
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = true;
    runShortCommand("CG");
    EXPECT_EQ("MS=1;MU=1;MC=1;Mred=1\n", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, getConfigMixed) {
    strictEeprom();
    musterDec.MSenabled = true;
    musterDec.MUenabled = false;
    musterDec.MCenabled = true;
    musterDec.MredEnabled = false;
    runShortCommand("CG");
    EXPECT_EQ("MS=1;MU=0;MC=1;Mred=0\n", cap.output());
}

TEST_F(ShortCommandTest, configWithoutSubcommandPrintsNothing) {
    strictEeprom();
    runShortCommand("C");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED) << "C is a known command";
}

TEST_F(ShortCommandTest, configRegisterReadNotAvailableWithoutCC1101) {
    strictEeprom();
    runShortCommand("C0D");   // C<reg> needs CMP_CC1101
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// CE<x> / CD<x> - enable / disable decoder
// ---------------------------------------------------------------------------
struct DecoderFlagCase {
    char flag;
    bool SignalDetectorClass::*member;
    uint8_t bit;
};

class DecoderFlagTest : public ShortCommandTest,
                        public ::testing::WithParamInterface<DecoderFlagCase> {};

TEST_P(DecoderFlagTest, enableSetsFlagAndStoresInEeprom) {
    const DecoderFlagCase c = GetParam();
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    AfcEnabled = false;

    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(addr_features, (uint8_t)(1 << c.bit))).Times(1);
    EXPECT_CALL(*eeprom, read(_)).Times(0);

    const char cmd[] = { 'C', 'E', c.flag, '\0' };
    runShortCommand(cmd);
    EXPECT_TRUE(musterDec.*c.member);
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_P(DecoderFlagTest, disableClearsFlagAndStoresInEeprom) {
    const DecoderFlagCase c = GetParam();
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = true;
    AfcEnabled = false;

    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(addr_features, (uint8_t)(0x0F & ~(1 << c.bit)))).Times(1);
    EXPECT_CALL(*eeprom, read(_)).Times(0);

    const char cmd[] = { 'C', 'D', c.flag, '\0' };
    runShortCommand(cmd);
    EXPECT_FALSE(musterDec.*c.member);
    EXPECT_EQ("", cap.output());
}

INSTANTIATE_TEST_SUITE_P(serialCommand, DecoderFlagTest, ::testing::Values(
    DecoderFlagCase{ 'S', &SignalDetectorClass::MSenabled, 0 },
    DecoderFlagCase{ 'U', &SignalDetectorClass::MUenabled, 1 },
    DecoderFlagCase{ 'C', &SignalDetectorClass::MCenabled, 2 },
    DecoderFlagCase{ 'R', &SignalDetectorClass::MredEnabled, 3 }
), [](const ::testing::TestParamInfo<DecoderFlagCase>& info) {
    return std::string(1, info.param.flag);
});

TEST_F(ShortCommandTest, configStoresAfcBitFromGlobalState) {
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    AfcEnabled = true;
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(addr_features, (uint8_t)((1 << 0) | (1 << 4)))).Times(1);
    runShortCommand("CES");
}

TEST_F(ShortCommandTest, configEnableUnknownDecoderIsIgnored) {
    strictEeprom();
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    runShortCommand("CEZ");
    runShortCommand("CEA");   // AFC needs CMP_CC1101
    runShortCommand("CEW");   // WMBus needs CMP_CC1101 + ESP/STM
    runShortCommand("CET");   // WMBus_T needs CMP_CC1101 + ESP/STM
    EXPECT_FALSE(musterDec.MSenabled || musterDec.MUenabled || musterDec.MCenabled || musterDec.MredEnabled);
    EXPECT_EQ("", cap.output());
}

TEST_F(ShortCommandTest, configEnableWithoutDecoderIsIgnored) {
    strictEeprom();
    runShortCommand("CE");
    runShortCommand("CD");
    EXPECT_EQ("", cap.output());
}

TEST_F(ShortCommandTest, configFlagsAreCaseSensitive) {
    strictEeprom();
    musterDec.MSenabled = false;
    runShortCommand("CEs");
    runShortCommand("Ces");
    EXPECT_FALSE(musterDec.MSenabled);
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// CSmcmbl=<val> - MC minimum bit length
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, setMcMinBitLen) {
    strictEeprom();
    runShortCommand("CSmcmbl=20");
    EXPECT_EQ(20, musterDec.mcMinBitLen);
    EXPECT_EQ("20 bits set", cap.output()) << "answer has no line terminator";
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, setMcMinBitLenBoundaries) {
    strictEeprom();
    runShortCommand("CSmcmbl=0");
    EXPECT_EQ(0, musterDec.mcMinBitLen);
    EXPECT_EQ("0 bits set", cap.output());
    cap.clear();
    runShortCommand("CSmcmbl=255");
    EXPECT_EQ(255, musterDec.mcMinBitLen);
    EXPECT_EQ("255 bits set", cap.output());
}

TEST_F(ShortCommandTest, setMcMinBitLenOverflowWrapsToUint8) {
    strictEeprom();
    runShortCommand("CSmcmbl=256");   // mcMinBitLen is uint8_t
    EXPECT_EQ(0, musterDec.mcMinBitLen);
    EXPECT_EQ("0 bits set", cap.output());
    cap.clear();
    runShortCommand("CSmcmbl=-1");
    EXPECT_EQ(255, musterDec.mcMinBitLen);
    EXPECT_EQ("255 bits set", cap.output());
}

TEST_F(ShortCommandTest, setMcMinBitLenWithoutValueSetsZero) {
    strictEeprom();
    musterDec.mcMinBitLen = 17;
    runShortCommand("CSmcmbl=");
    EXPECT_EQ(0, musterDec.mcMinBitLen);
    EXPECT_EQ("0 bits set", cap.output());
}

TEST_F(ShortCommandTest, setMcMinBitLenNonNumericSetsZero) {
    strictEeprom();
    musterDec.mcMinBitLen = 17;
    runShortCommand("CSmcmbl=abc");
    EXPECT_EQ(0, musterDec.mcMinBitLen);
    EXPECT_EQ("0 bits set", cap.output());
}

TEST_F(ShortCommandTest, setMcMinBitLenLongestValueThatFits) {
    strictEeprom();
    runShortCommand("CSmcmbl=0020");   // 12 chars + '\n' = full IB_1
    EXPECT_EQ(20, musterDec.mcMinBitLen);
    EXPECT_EQ("20 bits set", cap.output());
}

TEST_F(ShortCommandTest, setUnknownKeyIsIgnored) {
    strictEeprom();
    musterDec.mcMinBitLen = 17;
    runShortCommand("CSfoo=5");
    runShortCommand("CS");
    EXPECT_EQ(17, musterDec.mcMinBitLen);
    EXPECT_EQ("", cap.output());
}

TEST_F(ShortCommandTest, setMcMinBitLenKeyIsCaseSensitive) {
    strictEeprom();
    musterDec.mcMinBitLen = 17;
    runShortCommand("CSMCMBL=5");
    EXPECT_EQ(17, musterDec.mcMinBitLen);
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// W<reg><val> - write EEPROM
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, writeEeprom) {
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(0x00, 0x01)).Times(1);
    runShortCommand("W0001");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, writeEepromBoundaries) {
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(0x00, 0x00)).Times(1);
    EXPECT_CALL(*eeprom, write(0xFF, 0xFF)).Times(1);
    runShortCommand("W0000");
    runShortCommand("WFFFF");
}

TEST_F(ShortCommandTest, writeEepromAcceptsLowerCaseHex) {
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(0x1a, 0xbc)).Times(1);
    runShortCommand("W1abc");
}

TEST_F(ShortCommandTest, writeEepromIgnoresTrailingCharacters) {
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, write(0x12, 0x34)).Times(1);
    runShortCommand("W123456");
}

TEST_F(ShortCommandTest, writeEepromTooShortIsIgnored) {
    strictEeprom();
    runShortCommand("W");
    runShortCommand("W1");
    runShortCommand("W12");
    runShortCommand("W123");
    EXPECT_EQ("", cap.output());
}

TEST_F(ShortCommandTest, writeEepromInvalidHexIsIgnored) {
    strictEeprom();
    runShortCommand("WG001");
    runShortCommand("W0G01");
    runShortCommand("W00G1");
    runShortCommand("W000G");
    runShortCommand("W 0 1");
    EXPECT_EQ("", cap.output());
}

TEST_F(ShortCommandTest, writeStrobeDoesNotWriteEeprom) {
    strictEeprom();
    runShortCommand("WS34");   // strobe needs CMP_CC1101, must not be parsed as register write
    runShortCommand("WS36");
    runShortCommand("WS3A");
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// r<addr> / r<addr>n - read EEPROM
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, readEepromByte) {
    hasCC1101 = true;
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, read(0x00)).WillOnce(Return(0x33));
    EXPECT_CALL(*eeprom, write(_, _)).Times(0);
    runShortCommand("r00");
    EXPECT_EQ("EEPROM 00 = 33\n", cap.output());
    EXPECT_TRUE(blinkLED);
}

TEST_F(ShortCommandTest, readEepromPadsHexWithZero) {
    hasCC1101 = true;
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, read(0x0a)).WillOnce(Return(0x05));
    runShortCommand("r0a");
    EXPECT_EQ("EEPROM 0A = 05\n", cap.output());
}

TEST_F(ShortCommandTest, readEepromLastAddress) {
    hasCC1101 = true;
    EEPROMMock* eeprom = EEPROMMockInstance();
    EXPECT_CALL(*eeprom, read(0xff)).WillOnce(Return(0xff));
    runShortCommand("rFF");
    EXPECT_EQ("EEPROM FF = FF\n", cap.output());
}

TEST_F(ShortCommandTest, readEepromBlock) {
    hasCC1101 = true;
    EEPROMMock* eeprom = EEPROMMockInstance();
    for (int i = 0; i < 16; i++) {
        EXPECT_CALL(*eeprom, read(0x10 + i)).WillOnce(Return((uint8_t)(i * 0x11)));
    }
    runShortCommand("r10n");
    EXPECT_EQ("EEPROM 10 : 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF\n", cap.output());
}

TEST_F(ShortCommandTest, readEepromBlockAtEndReadsBeyond0xFF) {
    // r<addr>n always reads 16 bytes: starting at 0xF8 the addresses 0x100..0x107 are read
    hasCC1101 = true;
    EEPROMMock* eeprom = EEPROMMockInstance();
    for (int i = 0; i < 16; i++) {
        EXPECT_CALL(*eeprom, read(0xF8 + i)).WillOnce(Return(0));
    }
    runShortCommand("rF8n");
    EXPECT_EQ("EEPROM F8 : 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\n", cap.output());
}

TEST_F(ShortCommandTest, readEepromInvalidAddressIsIgnored) {
    hasCC1101 = true;
    strictEeprom();
    runShortCommand("r");
    runShortCommand("r0");
    runShortCommand("rG0");
    runShortCommand("r0G");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(blinkLED) << "r is a known command";
}

TEST_F(ShortCommandTest, readEepromRequiresCC1101Flag) {
    // current behaviour: r<addr> is only answered when hasCC1101 is true (commands.h, cmd_read)
    hasCC1101 = false;
    strictEeprom();
    runShortCommand("r00");
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// CC1101-only commands are not compiled in the native build
// ---------------------------------------------------------------------------
TEST_F(ShortCommandTest, cc1101OnlyCommandsAreUnsupportedWithoutCC1101Support) {
    strictEeprom();
    runShortCommand("e");
    EXPECT_EQ("Unsupported short command\n", cap.output());
    cap.clear();
    runShortCommand("x12");
    EXPECT_EQ("Unsupported short command\n", cap.output());
}
