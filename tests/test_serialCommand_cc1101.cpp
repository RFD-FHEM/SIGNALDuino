// Unit tests for the serial commands that need CC1101 support (CMP_CC1101):
// e, s, ?, V, C<reg>, CEA/CDA, CG (FSK), W<reg><val>, WS<strobe>, x<val>, SN and F=.
// The unmodified firmware (commands.h, send.h, cc1101.cpp) runs against the
// fake chip in fakeCC1101.h. See tests/COMMAND_COVERAGE.md.

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Return;

namespace cc1101test {

// EEPROM backed by an array
class FakeEeprom {
public:
    FakeEeprom() : mock_(EEPROMMockInstance()) {
        memset(data, 0, sizeof(data));
        EXPECT_CALL(*mock_, read(_)).Times(AnyNumber())
            .WillRepeatedly([this](int a) { return (a >= 0 && a < (int)sizeof(data)) ? data[a] : (uint8_t)0xFF; });
        EXPECT_CALL(*mock_, write(_, _)).Times(AnyNumber())
            .WillRepeatedly([this](int a, uint8_t v) {
                writes.push_back(std::make_pair(a, v));
                if (a >= 0 && a < (int)sizeof(data)) data[a] = v;
            });
    }
    ~FakeEeprom() { releaseEEPROMMock(); }
    uint8_t data[512];
    std::vector<std::pair<int, uint8_t>> writes;
private:
    EEPROMMock* mock_;
};

// Arduino environment: routes chip select to the fake chip, records the send pin
class Cc1101Test : public ::testing::Test {
protected:
    Cc1101Test() : arduino_(arduinoMockInstance()) {
        fakecc::chip().reset();
        fakecc::pinDirections().calls.clear();
        serialtest::fakeMicrosCounter() = 0;
        EXPECT_CALL(*arduino_, pinMode(_, _)).Times(AnyNumber())
            .WillRepeatedly([this](uint8_t p, uint8_t level) {
                if (p == csPin) {
                    if (level == LOW) fakecc::chip().select(); else fakecc::chip().deselect();
                } else if (p == PIN_SEND) {
                    sendEvents.push_back(serialtest::PinEvent{ level, serialtest::fakeMicrosCounter() - 1 });
                }
            });
        EXPECT_CALL(*arduino_, digitalRead(_)).Times(AnyNumber()).WillRepeatedly(Return(LOW)); // MISO low = chip ready
        EXPECT_CALL(*arduino_, digitalWrite(_, _)).Times(AnyNumber());
        EXPECT_CALL(*arduino_, delay(_)).Times(AnyNumber()).WillRepeatedly([this](int) { delayCalls++; });
        EXPECT_CALL(*arduino_, millis()).Times(AnyNumber());
    }
    ~Cc1101Test() override { releaseArduinoMock(); }

    void SetUp() override {
        savedHasCC1101_ = hasCC1101;
        savedAfc_ = AfcEnabled;
        savedCcmode_ = cc1101::ccmode;
        savedRevision_ = cc1101::revision;
        savedMS_ = musterDec.MSenabled; savedMU_ = musterDec.MUenabled;
        savedMC_ = musterDec.MCenabled; savedMred_ = musterDec.MredEnabled;
        hasCC1101 = true;
        cc1101::ccmode = 3;
        cc1101::revision = 0x01;
        blinkLED = false;
    }
    void TearDown() override {
        EXPECT_EQ(0u, fakecc::chip().unselectedBytes) << "SPI byte without chip select";
        hasCC1101 = savedHasCC1101_;
        AfcEnabled = savedAfc_;
        cc1101::ccmode = savedCcmode_;
        cc1101::revision = savedRevision_;
        musterDec.MSenabled = savedMS_; musterDec.MUenabled = savedMU_;
        musterDec.MCenabled = savedMC_; musterDec.MredEnabled = savedMred_;
    }

    void runShort(const char* cmd) {
        std::string line(cmd);
        line += '\n';
        ASSERT_TRUE(serialtest::loadCommand(line.c_str()));
        commands::HandleShortCommand();
    }
    void runSend(const std::string& full) {
        const size_t semi = full.find(';');
        ASSERT_NE(std::string::npos, semi);
        ASSERT_TRUE(serialtest::loadCommand(full.substr(0, semi + 1).c_str()));
        cap.setInput(full.substr(semi + 1));
        send_cmd();
    }
    fakecc::FakeChip& chip() { return fakecc::chip(); }

    serialtest::SerialCapture cap;
    FakeEeprom eeprom;
    std::vector<serialtest::PinEvent> sendEvents;
    unsigned delayCalls = 0;

private:
    ArduinoMock* arduino_;
    bool savedHasCC1101_, savedAfc_, savedMS_, savedMU_, savedMC_, savedMred_;
    uint8_t savedCcmode_, savedRevision_;
};

inline std::string hexList(const std::vector<uint8_t>& v) {
    std::string r;
    char b[4];
    for (uint8_t x : v) { snprintf(b, sizeof(b), "%02X ", x); r += b; }
    return r;
}

} // namespace cc1101test

using cc1101test::Cc1101Test;
using cc1101test::hexList;


static const uint8_t kInitValSize = 0x29;

// ---------------------------------------------------------------------------
// ? - help lists the CC1101 commands
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, helpListsCC1101Commands) {
    runShort("?");
    EXPECT_EQ("? Use one of V R t X S P C r W s x e \n", cap.output());
}

TEST_F(Cc1101Test, helpWithoutCC1101HardwareOmitsCC1101Commands) {
    hasCC1101 = false;
    runShort("?");
    EXPECT_EQ("? Use one of V R t X S P C r W s \n", cap.output());
}

// ---------------------------------------------------------------------------
// V - version with chip type
// ---------------------------------------------------------------------------
struct ChipVersionCase { uint8_t version; const char* name; };
// stable test names in ctest/gtest output (default: raw bytes incl. the pointer)
inline void PrintTo(const ChipVersionCase& c, std::ostream* os) {
    char b[8];
    snprintf(b, sizeof(b), "0x%02X", c.version);
    *os << b << " CC110" << c.name;
}

class Cc1101VersionTest : public Cc1101Test,
                          public ::testing::WithParamInterface<ChipVersionCase> {};

TEST_P(Cc1101VersionTest, versionShowsChipType) {
    chip().status[0x31 - 0x30] = GetParam().version;
    runShort("V");
    const std::string expected = std::string("V ") + PROGVERS + PROGNAME + "cc1101 (chip CC110" +
        GetParam().name + ") - compiled at " + __DATE__ + " " + __TIME__ + "\n";
    EXPECT_EQ(expected, cap.output());
}

INSTANTIATE_TEST_SUITE_P(serialCommand, Cc1101VersionTest, ::testing::Values(
    ChipVersionCase{ 0x03, "0" },
    ChipVersionCase{ 0x04, "1" },
    ChipVersionCase{ 0x14, "1" },
    ChipVersionCase{ 0x05, "0E" },
    ChipVersionCase{ 0x07, "L" },
    ChipVersionCase{ 0x17, "L" },
    ChipVersionCase{ 0x42, " unknown" }
), [](const ::testing::TestParamInfo<ChipVersionCase>& info) {
    char b[16];
    snprintf(b, sizeof(b), "v%02X", info.param.version);
    return std::string(b);
});

TEST_F(Cc1101Test, versionReadsOldAddressWhenChipReturns00OrFF) {
    // current behaviour: with VERSION 0x00/0xFF chipVersion() switches to revision 0
    // and reads 0x31 with a single read header (0xB1). On the CC1101 this is the
    // command strobe SFSTXON (data sheet, table 42), not the VERSION register.
    for (uint8_t v : { (uint8_t)0x00, (uint8_t)0xFF }) {
        chip().reset();
        cc1101::revision = 0x01;
        chip().status[0x31 - 0x30] = v;
        runShort("V");
        EXPECT_EQ(0x00, cc1101::revision) << (int)v;
        EXPECT_NE(chip().strobes.end(), std::find(chip().strobes.begin(), chip().strobes.end(), 0x31))
            << "SFSTXON strobe, version " << (int)v;
        cap.clear();
    }
}

TEST_F(Cc1101Test, versionWithoutCC1101Hardware) {
    hasCC1101 = false;
    runShort("V");
    const std::string expected = std::string("V ") + PROGVERS + PROGNAME + " - compiled at " + __DATE__ + " " + __TIME__ + "\n";
    EXPECT_EQ(expected, cap.output());
    EXPECT_EQ(0u, chip().selectCount) << "chip is not accessed";
}

// ---------------------------------------------------------------------------
// s - CC1101 register check
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, statusOkWhenAskOokRegistersMatch) {
    chip().cfg[0x08] = 0x32;   // PKTCTRL0 initVal
    chip().cfg[0x00] = 0x0D;   // IOCFG2 initVal
    runShort("s");
    EXPECT_EQ("OK\n", cap.output());
}

TEST_F(Cc1101Test, statusAsksForFactoryResetOnWrongPktctrl0) {
    chip().cfg[0x08] = 0x05;
    chip().cfg[0x00] = 0x0D;
    runShort("s");
    EXPECT_EQ("cc1101 is not correctly set for ASK/OOK. Please do a factory reset via command e\n", cap.output());
}

TEST_F(Cc1101Test, statusAsksForFactoryResetOnWrongIocfg2) {
    chip().cfg[0x08] = 0x32;
    chip().cfg[0x00] = 0x2E;
    runShort("s");
    EXPECT_EQ("cc1101 is not correctly set for ASK/OOK. Please do a factory reset via command e\n", cap.output());
}

TEST_F(Cc1101Test, statusWithoutCC1101HardwareIsOk) {
    hasCC1101 = false;
    runShort("s");
    EXPECT_EQ("OK\n", cap.output());
    EXPECT_EQ(0u, chip().selectCount);
}

// ---------------------------------------------------------------------------
// e - factory reset
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, factoryResetWritesDefaultsToEepromAndChip) {
    runShort("e");
    EXPECT_EQ("ccFactoryReset done\n", cap.output());
    for (uint8_t i = 0; i < kInitValSize; i++) {
        EXPECT_EQ(cc1101::initVal[i], eeprom.data[EE_CC1101_CFG + i]) << "EEPROM cfg " << (int)i;
        EXPECT_EQ(cc1101::initVal[i], chip().cfg[i]) << "chip register " << (int)i;
    }
    const uint8_t pa[8] = { 0x00, PATABLE_DEFAULT, 0, 0, 0, 0, 0, 0 };
    for (uint8_t i = 0; i < 8; i++) {
        EXPECT_EQ(pa[i], eeprom.data[EE_CC1101_PA + i]) << "EEPROM patable " << (int)i;
        EXPECT_EQ(pa[i], chip().patable[i]) << "chip patable " << (int)i;
    }
    ASSERT_FALSE(chip().strobes.empty());
    EXPECT_EQ(0x30, chip().strobes.front()) << "starts with SRES";
    EXPECT_EQ((int)fakecc::FakeChip::STATE_RX, (int)chip().state) << "receiver is on afterwards";
    EXPECT_EQ(3, cc1101::ccmode) << "ASK/OOK from MDMCFG2 default";
    EXPECT_TRUE(blinkLED);
}

TEST_F(Cc1101Test, factoryResetWithoutCC1101HardwareDoesNothing) {
    hasCC1101 = false;
    runShort("e");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(eeprom.writes.empty());
    EXPECT_EQ(0u, chip().selectCount);
}

TEST_F(Cc1101Test, factoryResetThenStatusIsOk) {
    runShort("e");
    cap.clear();
    runShort("s");
    EXPECT_EQ("OK\n", cap.output());
}

// ---------------------------------------------------------------------------
// C<reg> / C<reg>n<x> / C3E / C99 - read CC1101 registers
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, readConfigRegister) {
    chip().cfg[0x0D] = 0x21;
    runShort("C0D");
    EXPECT_EQ("C0D = 21\n", cap.output());
}

TEST_F(Cc1101Test, readConfigRegisterLowerCaseHex) {
    chip().cfg[0x0D] = 0x05;
    runShort("C0d");
    EXPECT_EQ("C0D = 05\n", cap.output());
}

TEST_F(Cc1101Test, readFirstAndLastConfigRegister) {
    chip().cfg[0x00] = 0x0D;
    chip().cfg[0x2E] = 0xAB;
    runShort("C00");
    runShort("C2E");
    EXPECT_EQ("C00 = 0D\nC2E = AB\n", cap.output());
}

TEST_F(Cc1101Test, readStatusRegister) {
    chip().status[0x35 - 0x30] = 0x0D;   // MARCSTATE RX
    runShort("C35");
    EXPECT_EQ("C35 = 0D\n", cap.output());
    EXPECT_TRUE(chip().strobes.empty()) << "status registers are read with burst bit, not as strobe";
}

TEST_F(Cc1101Test, readLastStatusRegister) {
    chip().status[0x3D - 0x30] = 0x42;
    runShort("C3D");
    EXPECT_EQ("C3D = 42\n", cap.output());
}

TEST_F(Cc1101Test, readPatable) {
    const uint8_t pa[8] = { 0x00, 0x84, 0x01, 0x02, 0x03, 0x04, 0x05, 0xFF };
    memcpy(chip().patable, pa, 8);
    runShort("C3E");
    EXPECT_EQ("C3E = 00 84 01 02 03 04 05 FF\n", cap.output());
}

TEST_F(Cc1101Test, readRegisterRange) {
    chip().cfg[0x06] = 0x3D; chip().cfg[0x07] = 0x04; chip().cfg[0x08] = 0x32; chip().cfg[0x09] = 0x00;
    runShort("C06n2");   // n + 2 registers
    EXPECT_EQ("C06n04=3D043200\n", cap.output());
}

TEST_F(Cc1101Test, readRegisterRangeLargestOneDigitCount) {
    for (uint8_t i = 0; i < 0x2F; i++) chip().cfg[i] = i;
    runShort("C00nF");   // 0x0F + 2 = 17 registers
    EXPECT_EQ("C00n11=000102030405060708090A0B0C0D0E0F10\n", cap.output());
}

TEST_F(Cc1101Test, readRegisterRangeCountHasMoreDigits) {
    // current behaviour: strtol reads all hex digits of the count, not only one
    for (uint8_t i = 0; i < 0x2F; i++) chip().cfg[i] = i;
    runShort("C00n1F");   // 0x1F + 2 = 33 registers
    std::string expected = "C00n21=";
    for (uint8_t i = 0; i < 0x21; i++) { char b[4]; snprintf(b, sizeof(b), "%02X", i); expected += b; }
    EXPECT_EQ(expected + "\n", cap.output());
}

TEST_F(Cc1101Test, readRegisterRangeAcrossStatusAreaSendsAllStrobes) {
    // current behaviour: C2Dn10 reads 0x2D..0x3E with single read headers;
    // 0x30..0x3D are the command strobes, among them SRES, SPWD and SFTX
    runShort("C2Dn10");
    std::vector<uint8_t> expected;
    for (uint8_t i = 0x30; i <= 0x3D; i++) expected.push_back(i);
    EXPECT_EQ(hexList(expected), hexList(chip().strobes));
}

TEST_F(Cc1101Test, readRegisterRangeCountOverflowDoesNotEnd) {
    // current behaviour: n (uint8_t) = 0xEE + 2 = 0xF0, the loop
    // "for (uint8_t i = reg; i < reg + n; i++)" runs until i reaches 0x100, which
    // an uint8_t never does: the firmware hangs. The fake chip stops it after
    // transferLimit SPI bytes.
    EXPECT_THROW(runShort("C10nEE"), fakecc::TooManyTransfers);
}

TEST_F(Cc1101Test, readRegisterRangeEndingAtLastConfigRegister) {
    chip().cfg[0x2D] = 0x01; chip().cfg[0x2E] = 0x02; chip().cfg[0x2F] = 0x03;
    runShort("C2Dn1");
    EXPECT_EQ("C2Dn03=010203\n", cap.output());
    EXPECT_TRUE(chip().strobes.empty());
}

TEST_F(Cc1101Test, readRegisterRangeBeyond0x2FSendsStrobe) {
    // current behaviour: the range is not limited to the config registers. Reading
    // 0x30 with a single read header (0xB0) is the SRES command strobe for the
    // CC1101 (data sheet, table 42), so C2En1 resets the chip.
    runShort("C2En1");
    EXPECT_EQ(0, cap.output().find("C2En03=")) << cap.output();
    ASSERT_EQ(1u, chip().strobes.size());
    EXPECT_EQ(0x30, chip().strobes[0]);
}

TEST_F(Cc1101Test, readRegisterRangeForStatusRegisterIsIgnored) {
    runShort("C2Fn1");   // range only for reg < 0x2F
    EXPECT_EQ("", cap.output());
}

TEST_F(Cc1101Test, readRegisterRangeWithInvalidCountReadsSingleRegister) {
    chip().cfg[0x06] = 0x3D;
    runShort("C06nG");
    EXPECT_EQ("C06 = 3D\n", cap.output());
}

TEST_F(Cc1101Test, readAllConfigRegisters) {
    for (uint8_t i = 0; i < 0x2F; i++) chip().cfg[i] = i;
    runShort("C99");
    std::string expected = "ccreg 00: ";
    for (uint8_t i = 0; i < 0x2F; i++) {
        if (i == 0x10) expected += " ccreg 10: ";
        if (i == 0x20) expected += " ccreg 20: ";
        char b[4];
        snprintf(b, sizeof(b), "%02X ", i);
        expected += b;
    }
    expected += "\n";
    EXPECT_EQ(expected, cap.output());
}

TEST_F(Cc1101Test, readUnknownRegisterIsIgnored) {
    runShort("C3F");
    runShort("C98");
    runShort("CFF");
    EXPECT_EQ("", cap.output());
    EXPECT_EQ(0u, chip().selectCount);
}

TEST_F(Cc1101Test, readRegisterInvalidHexIsIgnored) {
    runShort("C0");    // one digit
    runShort("C0Z");
    runShort("CZ0");
    EXPECT_EQ("", cap.output());
    EXPECT_EQ(0u, chip().selectCount);
}

TEST_F(Cc1101Test, readRegisterWithoutCC1101HardwareIsIgnored) {
    hasCC1101 = false;
    runShort("C0D");
    EXPECT_EQ("", cap.output());
    EXPECT_EQ(0u, chip().selectCount);
}

// ---------------------------------------------------------------------------
// W<reg><val> - write EEPROM and CC1101 register (register = EEPROM address - 2)
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, writeRegisterToEepromAndChip) {
    runShort("W0F10");
    EXPECT_EQ("W0F10\n", cap.output());
    EXPECT_EQ(0x10, eeprom.data[0x0F]);
    EXPECT_EQ(0x10, chip().cfg[0x0D]);
}

TEST_F(Cc1101Test, writeFirstChipRegister) {
    runShort("W0201");
    EXPECT_EQ("W0201\n", cap.output());
    EXPECT_EQ(0x01, chip().cfg[0x00]);
}

TEST_F(Cc1101Test, writeEepromHeaderBytesDoesNotTouchChip) {
    runShort("W0005");
    runShort("W0106");
    EXPECT_EQ("", cap.output());
    EXPECT_EQ(0x05, eeprom.data[0x00]);
    EXPECT_EQ(0x06, eeprom.data[0x01]);
    EXPECT_EQ(0u, chip().selectCount);
}

TEST_F(Cc1101Test, writeAbove0x3FOnlyWritesEeprom) {
    runShort("W4001");
    runShort("WFF02");
    EXPECT_EQ("", cap.output());
    EXPECT_EQ(0x01, eeprom.data[0x40]);
    EXPECT_EQ(0x02, eeprom.data[0xFF]);
    EXPECT_EQ(0u, chip().selectCount);
}

TEST_F(Cc1101Test, writeEepromPatableAreaSendsStrobe) {
    // current behaviour: EEPROM 0x30..0x37 holds the PATABLE, but W32xx..W3Fxx are
    // also written to CC1101 address 0x30..0x3D, which are command strobes
    // (here 0x30 = SRES, data sheet table 42)
    runShort("W3201");
    EXPECT_EQ("W3201\n", cap.output());
    EXPECT_EQ(0x01, eeprom.data[0x32]);
    ASSERT_EQ(1u, chip().strobes.size());
    EXPECT_EQ(0x30, chip().strobes[0]);
}

TEST_F(Cc1101Test, writeMdmcfg2SetsModulationMode) {
    runShort("W1400");    // MDMCFG2 = 0x00 -> 2-FSK
    EXPECT_EQ(0, cc1101::ccmode);
    runShort("W1430");    // MDMCFG2 = 0x30 -> ASK/OOK
    EXPECT_EQ(3, cc1101::ccmode);
    EXPECT_EQ("W1400\nW1430\n", cap.output());
}

TEST_F(Cc1101Test, writeBandwidthAdjustsFrend1AndFifothrForAskOok) {
    eeprom.data[2] = 0x0D;   // IOCFG2 = 0x0D (ASK/OOK settings)
    runShort("W1257");       // MDMCFG4 = 0x57 (325 kHz)
    EXPECT_EQ("W1257\nW23B6\nW0547\n", cap.output());
    EXPECT_EQ(0x57, chip().cfg[0x10]);
    EXPECT_EQ(0xB6, chip().cfg[0x21]);   // FREND1
    EXPECT_EQ(0x47, chip().cfg[0x03]);   // FIFOTHR
}

TEST_F(Cc1101Test, writeBandwidthBoundaries) {
    eeprom.data[2] = 0x0D;
    runShort("W12C7");   // >= 0xC7 -> FREND1 0x56
    EXPECT_EQ("W12C7\nW2356\nW0547\n", cap.output());
    cap.clear();
    runShort("W12C6");   // < 0xC7 -> FREND1 0xB6
    EXPECT_EQ("W12C6\nW23B6\nW0547\n", cap.output());
    cap.clear();
    runShort("W1256");   // < 0x57 -> FIFOTHR 0x07
    EXPECT_EQ("W1256\nW23B6\nW0507\n", cap.output());
}

TEST_F(Cc1101Test, writeBandwidthWithoutAskOokSettingsIsNotAdjusted) {
    eeprom.data[2] = 0x2E;
    runShort("W1257");
    EXPECT_EQ("W1257\n", cap.output());
}

TEST_F(Cc1101Test, writeWithoutCC1101HardwareOnlyWritesEeprom) {
    hasCC1101 = false;
    runShort("W0F10");
    EXPECT_EQ("", cap.output());
    EXPECT_EQ(0x10, eeprom.data[0x0F]);
    EXPECT_EQ(0u, chip().selectCount);
}

// ---------------------------------------------------------------------------
// WS<strobe> - command strobe
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, strobeRxReportsChipStatus) {
    runShort("WS34");
    EXPECT_EQ("cmdStrobeReg 34 chipStatus 00 delay1 01\n", cap.output());
    const std::vector<uint8_t> expected = { 0x34, 0x3D };
    EXPECT_EQ(expected, chip().strobes);
    EXPECT_EQ((int)fakecc::FakeChip::STATE_RX, (int)chip().state);
}

TEST_F(Cc1101Test, strobeIdleFromRx) {
    chip().state = fakecc::FakeChip::STATE_RX;
    runShort("WS36");
    EXPECT_EQ("cmdStrobeReg 36 chipStatus 01 delay1 00\n", cap.output());
    EXPECT_EQ((int)fakecc::FakeChip::STATE_IDLE, (int)chip().state);
}

TEST_F(Cc1101Test, strobeFlushRx) {
    runShort("WS3A");
    EXPECT_EQ("cmdStrobeReg 3A chipStatus 00 delay1 00\n", cap.output());
    EXPECT_EQ(0x3A, chip().strobes.at(0));
}

TEST_F(Cc1101Test, strobeLowerCaseHex) {
    runShort("WS3a");
    EXPECT_EQ("cmdStrobeReg 3A chipStatus 00 delay1 00\n", cap.output());
}

TEST_F(Cc1101Test, strobeLastAndOutOfRange) {
    runShort("WS3D");
    EXPECT_EQ("cmdStrobeReg 3D chipStatus 00 delay1 00\n", cap.output());
    cap.clear();
    runShort("WS3E");   // >= 0x3E is not a strobe
    runShort("WS3F");
    EXPECT_EQ("", cap.output());
}

TEST_F(Cc1101Test, strobeInvalidIsIgnored) {
    runShort("WS3");
    runShort("WS3G");
    runShort("WS2F");   // only WS3x is a strobe command
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(chip().strobes.empty());
}

TEST_F(Cc1101Test, strobeWithMoreDigitsUsesLowByte) {
    // current behaviour: strtol reads all digits, only the low byte is used:
    // WS3412 sends 0x12 (a config register address, no strobe) to the chip
    runShort("WS3412");
    EXPECT_EQ("cmdStrobeReg 12 chipStatus 00 delay1 00\n", cap.output());
    EXPECT_EQ("3D ", hexList(chip().strobes)) << "only the SNOP for delay1, 0x12 is no strobe";
}

TEST_F(Cc1101Test, strobeIgnoresMissingCC1101Hardware) {
    // current behaviour: unlike C, x and e, WS does not check hasCC1101
    hasCC1101 = false;
    runShort("WS34");
    EXPECT_EQ("cmdStrobeReg 34 chipStatus 00 delay1 01\n", cap.output());
}

// ---------------------------------------------------------------------------
// x<val> - PA table
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, patableWrite) {
    runShort("x12");
    EXPECT_EQ("write 12 to PATABLE done\n", cap.output());
    const uint8_t pa[8] = { 0x00, 0x12, 0, 0, 0, 0, 0, 0 };
    for (uint8_t i = 0; i < 8; i++) {
        EXPECT_EQ(pa[i], eeprom.data[EE_CC1101_PA + i]) << (int)i;
        EXPECT_EQ(pa[i], chip().patable[i]) << (int)i;
    }
}

TEST_F(Cc1101Test, patableBoundaries) {
    runShort("x00");
    EXPECT_EQ("write 00 to PATABLE done\n", cap.output());
    EXPECT_EQ(0x00, chip().patable[1]);
    cap.clear();
    runShort("xff");
    EXPECT_EQ("write FF to PATABLE done\n", cap.output());
    EXPECT_EQ(0xFF, chip().patable[1]);
}

TEST_F(Cc1101Test, patableInvalidIsIgnored) {
    runShort("x");
    runShort("x1");
    runShort("xG1");
    runShort("x1G");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(eeprom.writes.empty());
}

TEST_F(Cc1101Test, patableWithoutCC1101HardwareIsIgnored) {
    hasCC1101 = false;
    runShort("x12");
    EXPECT_EQ("", cap.output());
    EXPECT_TRUE(eeprom.writes.empty());
}

// ---------------------------------------------------------------------------
// CEA / CDA - AFC, CG in FSK mode
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, enableAfc) {
    AfcEnabled = false;
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    runShort("CEA");
    EXPECT_TRUE(AfcEnabled);
    EXPECT_EQ(1 << 4, eeprom.data[addr_features]);
    EXPECT_TRUE(chip().writes.empty()) << "FSCTRL0 is only reset when AFC is disabled";
    EXPECT_EQ("", cap.output());
}

TEST_F(Cc1101Test, disableAfcResetsFrequencyOffset) {
    AfcEnabled = true;
    cc1101::freqOffAcc = 5;
    cc1101::freqErrAvg = 1.5;
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    runShort("CDA");
    EXPECT_FALSE(AfcEnabled);
    EXPECT_EQ(0, cc1101::freqOffAcc);
    EXPECT_EQ(0, cc1101::freqErrAvg);
    ASSERT_EQ(1u, chip().writes.size());
    EXPECT_EQ(0x0C, chip().writes[0].addr);   // FSCTRL0
    EXPECT_EQ(0x00, chip().writes[0].value);
    EXPECT_EQ(0, eeprom.data[addr_features]);
}

TEST_F(Cc1101Test, afcFlagIsCaseSensitive) {
    AfcEnabled = false;
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    runShort("CEa");
    runShort("Cea");
    EXPECT_FALSE(AfcEnabled);
    EXPECT_TRUE(eeprom.writes.empty());
    AfcEnabled = true;
    runShort("CDa");
    EXPECT_TRUE(AfcEnabled);
    EXPECT_TRUE(eeprom.writes.empty());
    EXPECT_EQ("", cap.output());
}

TEST_F(Cc1101Test, enableAfcWithoutCC1101Hardware) {
    // the flag is stored, the chip is not accessed (AFC on does not touch FSCTRL0)
    hasCC1101 = false;
    AfcEnabled = false;
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    runShort("CEA");
    EXPECT_TRUE(AfcEnabled);
    EXPECT_EQ(0u, chip().selectCount);
}

TEST_F(Cc1101Test, decoderFlagWithAfcOffWritesFsctrl0) {
    // current behaviour: every CE<x>/CD<x> calls storeFunctions(); with AFC off it
    // resets the frequency offset and writes FSCTRL0 = 0, also without CC1101 hardware
    for (bool hw : { true, false }) {
        chip().reset();
        hasCC1101 = hw;
        AfcEnabled = false;
        cc1101::freqOffAcc = 3;
        runShort("CES");
        EXPECT_EQ(0, cc1101::freqOffAcc) << hw;
        ASSERT_EQ(1u, chip().writes.size()) << hw;
        EXPECT_EQ(0x0C, chip().writes[0].addr) << hw;
    }
}

TEST_F(Cc1101Test, getConfigInFskModeIgnoresTrailingCharacters) {
    cc1101::ccmode = 0;
    AfcEnabled = true;
    runShort("CGxyz");
    EXPECT_EQ("MN=1;AFC=1\n", cap.output());
}

TEST_F(Cc1101Test, readEepromNeedsCC1101Hardware) {
    // current behaviour: r<addr> reads only the EEPROM, but is only answered with hasCC1101
    eeprom.data[0x10] = 0x42;
    runShort("r10");
    EXPECT_EQ("EEPROM 10 = 42\n", cap.output());
    cap.clear();
    hasCC1101 = false;
    runShort("r10");
    EXPECT_EQ("", cap.output());
}

TEST_F(Cc1101Test, getConfigInFskMode) {
    cc1101::ccmode = 0;
    AfcEnabled = true;
    runShort("CG");
    EXPECT_EQ("MN=1;AFC=1\n", cap.output());
    cap.clear();
    AfcEnabled = false;
    runShort("CG");
    EXPECT_EQ("MN=1;AFC=0\n", cap.output());
}

TEST_F(Cc1101Test, getConfigInAskOokMode) {
    cc1101::ccmode = 3;
    musterDec.MSenabled = true; musterDec.MUenabled = true; musterDec.MCenabled = false; musterDec.MredEnabled = true;
    runShort("CG");
    EXPECT_EQ("MS=1;MU=1;MC=0;Mred=1\n", cap.output());
}

TEST_F(Cc1101Test, wmbusFlagsAreNotAvailableInNativeBuild) {
    // CEW/CDW/CET/CDT need ESP8266, ESP32 or ARDUINO_MAPLEMINI_F103CB in addition to CMP_CC1101
    musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = musterDec.MredEnabled = false;
    runShort("CEW");
    runShort("CET");
    EXPECT_TRUE(eeprom.writes.empty());
    EXPECT_EQ("", cap.output());
}

// ---------------------------------------------------------------------------
// SN - send xFSK via TX FIFO, F= - temporary frequency registers
// ---------------------------------------------------------------------------
TEST_F(Cc1101Test, xfskSendsDataThroughTxFifo) {
    const std::string cmd = "SN;R=2;D=91C6;\n";
    runSend(cmd);
    EXPECT_EQ(cmd, cap.output());
    const std::vector<uint8_t> expected = { 0x91, 0xC6, 0x91, 0xC6 };
    EXPECT_EQ(expected, chip().txFifo);
    EXPECT_NE(chip().strobes.end(), std::find(chip().strobes.begin(), chip().strobes.end(), 0x35)) << "STX";
    EXPECT_EQ((int)fakecc::FakeChip::STATE_RX, (int)chip().state) << "receiver is on afterwards";
    EXPECT_TRUE(sendEvents.empty()) << "xFSK does not toggle the send pin";
}

TEST_F(Cc1101Test, xfskLowerCaseHex) {
    runSend("SN;D=91c6;\n");
    const std::vector<uint8_t> expected = { 0x91, 0xC6 };
    EXPECT_EQ(expected, chip().txFifo);
}

TEST_F(Cc1101Test, xfskIgnoresUnknownParameterN) {
    // N= appears in the documented example but is not evaluated by send_cmd()
    runSend("SN;R=1;N=2;D=91C6;\n");
    EXPECT_EQ("SN;R=1;N=2;D=91C6;\n", cap.output());
    const std::vector<uint8_t> expected = { 0x91, 0xC6 };
    EXPECT_EQ(expected, chip().txFifo);
}

TEST_F(Cc1101Test, xfskOddNumberOfHexDigits) {
    // current behaviour: the last nibble is combined with the ';' (hex2int returns
    // non hex characters unchanged): 'C' * 16 + ';' = 0xC0 + 0x3B = 0xFB
    runSend("SN;D=91C;\n");
    const std::vector<uint8_t> expected = { 0x91, 0xFB };
    EXPECT_EQ(expected, chip().txFifo);
}

TEST_F(Cc1101Test, xfskWithoutCC1101HardwareStillWritesFifo) {
    // current behaviour: setTransmitMode() is skipped, sendFIFO() is not
    hasCC1101 = false;
    runSend("SN;D=91C6;\n");
    const std::vector<uint8_t> expected = { 0x91, 0xC6 };
    EXPECT_EQ(expected, chip().txFifo);
    EXPECT_EQ(chip().strobes.end(), std::find(chip().strobes.begin(), chip().strobes.end(), 0x35)) << "no STX";
}

TEST_F(Cc1101Test, xfskCombinedPartsUseDataOfLastPart) {
    // current behaviour: send_cmd() calls sendFIFO(command[cmdNo].datastart,
    // command[c].dataend) with cmdNo instead of c, so the first SN part is not sent
    // (expected: 91 C6)
    runSend("SC;R=1;SN;D=91;SN;D=C6;\n");
    EXPECT_EQ("C6 ", hexList(chip().txFifo));
}

TEST_F(Cc1101Test, transmitModeFailureIsNotDetected) {
    // current behaviour: after "while (maxloop-- && ...)" maxloop is 255, not 0, when
    // the chip never reaches TX, so setTransmitMode() does not report the failure and
    // still switches PIN_SEND to output; the message is sent anyway
    chip().stuckStrobe = 0x35;   // STX does not switch to TX
    runSend("SR;P0=-400;P1=800;D=01;\n");
    EXPECT_EQ(255, std::count(chip().strobes.begin(), chip().strobes.end(), 0x35)) << "255 STX tries";
    const auto& dirs = fakecc::pinDirections().calls;
    EXPECT_NE(dirs.end(), std::find(dirs.begin(), dirs.end(), std::make_pair((uint8_t)PIN_SEND, (uint8_t)1)))
        << "PIN_SEND set to output";
    EXPECT_EQ(3u, sendEvents.size()) << "the message is sent";
}

TEST_F(Cc1101Test, receiveModeFailureIsNotDetected) {
    // same in setReceiveMode(): 255 SRX tries, no error is reported
    chip().stuckStrobe = 0x34;   // SRX does not switch to RX
    runSend("SR;P0=-400;P1=800;D=01;\n");
    EXPECT_EQ(255, std::count(chip().strobes.begin(), chip().strobes.end(), 0x34)) << "255 SRX tries";
    EXPECT_EQ((int)fakecc::FakeChip::STATE_IDLE, (int)chip().state);
}

TEST_F(Cc1101Test, rawSendSwitchesChipToTxAndBack) {
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    runSend(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_NE(chip().strobes.end(), std::find(chip().strobes.begin(), chip().strobes.end(), 0x35)) << "STX";
    EXPECT_EQ((int)fakecc::FakeChip::STATE_RX, (int)chip().state);
    EXPECT_EQ(5u, sendEvents.size()) << "4 pulses + final low";
}

TEST_F(Cc1101Test, frequencyRegistersAreSetForSendAndRestored) {
    chip().cfg[0x0D] = 0x10; chip().cfg[0x0E] = 0xB0; chip().cfg[0x0F] = 0x71;
    runSend("SR;P0=-400;P1=800;D=01;F=10AB85;\n");
    ASSERT_EQ(6u, chip().writes.size());
    const uint8_t expectedAddr[6] = { 0x0D, 0x0E, 0x0F, 0x0D, 0x0E, 0x0F };
    const uint8_t expectedVal[6]  = { 0x10, 0xAB, 0x85, 0x10, 0xB0, 0x71 };
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(expectedAddr[i], chip().writes[i].addr) << i;
        EXPECT_EQ(expectedVal[i], chip().writes[i].value) << i;
    }
    EXPECT_EQ(0x71, chip().cfg[0x0F]) << "original value restored";
}

TEST_F(Cc1101Test, frequencyRegistersMaximumFive) {
    for (uint8_t i = 0; i < 5; i++) chip().cfg[0x0D + i] = 0xA0 + i;
    runSend("SR;P0=-400;P1=800;D=01;F=1122334455;\n");
    ASSERT_EQ(10u, chip().writes.size()) << "5 written + 5 restored";
    for (uint8_t i = 0; i < 5; i++) {
        EXPECT_EQ(0x0D + i, chip().writes[i].addr) << (int)i;
        EXPECT_EQ(0x11 * (i + 1), chip().writes[i].value) << (int)i;
        EXPECT_EQ(0x0D + i, chip().writes[5 + i].addr) << (int)i;
        EXPECT_EQ(0xA0 + i, chip().writes[5 + i].value) << "restored " << (int)i;
    }
}

// Current behaviour: when F= is not applied (more than 5 registers or no CC1101
// hardware), ccParamAnz stays > 0 and send_cmd() still "restores" the registers
// from ccReg[], which was never filled. The written values are indeterminate,
// so only the addresses are checked.
inline std::vector<uint8_t> writeAddresses(const fakecc::FakeChip& c) {
    std::vector<uint8_t> a;
    for (const auto& w : c.writes) a.push_back(w.addr);
    return a;
}

TEST_F(Cc1101Test, frequencyRegistersMoreThanFiveAreNotSetButWrittenBack) {
    runSend("SR;P0=-400;P1=800;D=01;F=112233445566;\n");
    const std::vector<uint8_t> expected = { 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12 };
    EXPECT_EQ(expected, writeAddresses(chip())) << "only the write back, with uninitialised values";
}

TEST_F(Cc1101Test, frequencyRegistersWithoutCC1101HardwareAreNotSetButWrittenBack) {
    hasCC1101 = false;
    runSend("SR;P0=-400;P1=800;D=01;F=10AB85;\n");
    const std::vector<uint8_t> expected = { 0x0D, 0x0E, 0x0F };
    EXPECT_EQ(expected, writeAddresses(chip())) << "only the write back, with uninitialised values";
}

// The following F= cases read ccReg[] beyond its 6 entries (stack, undefined
// behaviour, ASan: stack-buffer-overflow in send_cmd()). They describe the
// proposed behaviour and are disabled until send_cmd() is fixed; run them with
// --gtest_also_run_disabled_tests.
TEST_F(Cc1101Test, DISABLED_frequencyRegistersSevenAreNotWrittenBack) {
    runSend("SR;P0=-400;P1=800;D=01;F=11223344556677;\n");
    EXPECT_TRUE(chip().writes.empty()) << "F= is not applied, nothing to restore";
}

TEST_F(Cc1101Test, DISABLED_frequencyRegistersLongListDoesNotResetChip) {
    // 36 registers: the write back reaches 0x30 = SRES (writeReg(0x30, ...))
    runSend("SR;P0=-400;P1=800;D=01;F=" + std::string(72, '1') + ";\n");
    EXPECT_EQ(chip().strobes.end(), std::find(chip().strobes.begin(), chip().strobes.end(), 0x30)) << "no SRES";
    EXPECT_TRUE(chip().writes.empty());
}
