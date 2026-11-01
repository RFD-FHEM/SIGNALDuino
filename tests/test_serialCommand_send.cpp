// Unit tests for the serial send commands SR, SM, SC and SN documented in
// AI_AGENT_COMMANDS.md (native build, without CMP_CC1101).
//
// A send command reaches send_cmd() like in serialEvent(): the first part up to
// ';' is in IB_1, the rest is read from Serial byte by byte. The tests check the
// echo on Serial and the signal on the send pin (recorded with a fake
// microsecond clock, see serialCommandHelper.h). Expectations describe the
// current firmware behaviour; no firmware code is changed.
namespace serialtest {

class SendCommandTest : public ::testing::Test {
protected:
    SendCommandTest() : rec(PIN_SEND) { receiveState() = ReceiveState(); }
    void run(const std::string& full, size_t gapAt = std::string::npos);
    SerialCapture cap;
    SendRecorder rec;
};

// Splits a complete send command like serialEvent() does: everything up to and
// including the first ';' is in IB_1, the rest is still in the serial buffer.
// gapAt (position in full) simulates a serial timeout before that byte.
inline void runSendCommand(SerialCapture& cap, const std::string& full,
                           size_t gapAt = std::string::npos) {
    const size_t semi = full.find(';');
    ASSERT_NE(std::string::npos, semi);
    ASSERT_TRUE(loadCommand(full.substr(0, semi + 1).c_str()));
    ASSERT_TRUE(gapAt == std::string::npos || gapAt > semi);
    cap.setInput(full.substr(semi + 1), gapAt == std::string::npos ? gapAt : gapAt - semi - 1);
    send_cmd();
}
// Compares a recorded signal with the expected durations. The fake clock
// advances once per micros() call, so a few us of jitter are tolerated.
inline ::testing::AssertionResult signalNear(const std::vector<long>& actual,
                                             const std::vector<long>& expected,
                                             long tolerance = 3) {
    bool ok = actual.size() == expected.size();
    for (size_t i = 0; ok && i < actual.size(); i++) {
        ok = ((actual[i] > 0) == (expected[i] > 0)) && std::labs(actual[i] - expected[i]) <= tolerance;
    }
    if (ok) return ::testing::AssertionSuccess();
    std::string a, e;
    for (long v : actual) { if (!a.empty()) a += ','; a += std::to_string(v); }
    for (long v : expected) { if (!e.empty()) e += ','; e += std::to_string(v); }
    return ::testing::AssertionFailure() << "signal\n  actual:   " << a << "\n  expected: " << e;
}

// Repeats a signal n times and merges same-level neighbours at the seams
inline std::vector<long> repeatSignal(const std::vector<long>& s, int n) {
    std::vector<long> out;
    for (int r = 0; r < n; r++) {
        for (long v : s) {
            if (!out.empty() && ((out.back() > 0) == (v > 0))) out.back() += v;
            else out.push_back(v);
        }
    }
    return out;
}

inline void SendCommandTest::run(const std::string& full, size_t gapAt) { runSendCommand(cap, full, gapAt); }

} // namespace serialtest

using serialtest::SendCommandTest;
using serialtest::signalNear;
using serialtest::repeatSignal;
using serialtest::receiveState;
using serialtest::fakeMicrosCounter;
using serialtest::fakeMicrosCalls;


// ---------------------------------------------------------------------------
// SR - send raw
// ---------------------------------------------------------------------------
TEST_F(SendCommandTest, rawSendsPatternSequence) {
    const std::string cmd = "SR;R=3;P0=-400;P1=800;D=0101;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output()) << "the command is echoed unchanged";
    EXPECT_TRUE(signalNear(rec.signal(), repeatSignal({ -400, 800, -400, 800 }, 3)));
    EXPECT_EQ(LOW, rec.lastLevel()) << "send pin ends low";
    EXPECT_EQ(0u, rec.delayCalls()) << "no extra delay for raw messages";
    EXPECT_EQ(0u, cap.inputLeft());
}

TEST_F(SendCommandTest, rawDefaultIsOneRepetition) {
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, rawAllEightPatterns) {
    const std::string cmd = "SR;P0=-100;P1=200;P2=-300;P3=400;P4=-500;P5=600;P6=-700;P7=800;D=01234567;\n";
    ASSERT_LT(cmd.size(), 255u);
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -100, 200, -300, 400, -500, 600, -700, 800 }));
}

TEST_F(SendCommandTest, rawPatternIndexAboveSevenIsIgnored) {
    run("SR;P8=100;P0=-400;P1=800;D=0101;\n");
    // P8 is neither stored nor echoed
    EXPECT_EQ("SR;P0=-400;P1=800;D=0101;\n", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, rawEchoHasNoStaleBytesWhenDataPartIsShorter) {
    // the echo buffer is null terminated after the line end
    run("SR;P0=-400;P1=800;D=01;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;D=01;\n", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800 }));
}

TEST_F(SendCommandTest, rawRepeatZeroSendsNothing) {
    run("SR;R=0;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ("SR;R=0;P0=-400;P1=800;D=0101;\n", cap.output());
    EXPECT_TRUE(rec.signal().empty());
}

TEST_F(SendCommandTest, rawRepeatTwo) {
    run("SR;R=2;P0=-400;P1=800;D=0101;\n");
    EXPECT_TRUE(signalNear(rec.signal(), repeatSignal({ -400, 800, -400, 800 }, 2)));
}

TEST_F(SendCommandTest, rawRepeatOverflowWrapsToZero) {
    // repeats is uint8_t: R=256 becomes 0
    run("SR;R=256;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ("SR;R=256;P0=-400;P1=800;D=0101;\n", cap.output());
    EXPECT_TRUE(rec.signal().empty());
}

TEST_F(SendCommandTest, rawRepeatOverflowWrapsTo1) {
    run("SR;R=257;P0=-400;P1=800;D=0101;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, rawExtremePulseLengths) {
    // P values are int16_t, the sign selects the level
    run("SR;P0=-32767;P1=32767;D=01;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -32767, 32767 }));
}

TEST_F(SendCommandTest, rawCorruptWhenNothingFollows) {
    run("SR;");
    EXPECT_EQ("SR;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty()) << "nothing is sent";
}

TEST_F(SendCommandTest, rawCorruptWhenCutInsidePart) {
    run("SR;P0=-4");
    EXPECT_EQ("SR;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, rawCorruptWithoutLineEnd) {
    run("SR;P0=-400;P1=800;D=0101;");
    EXPECT_EQ("SR;P0=-400;P1=800;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, rawLongestAcceptedCommand) {
    // 261 characters in total (incl. "\n") are still accepted
    const std::string cmd = "SR;P0=-4;P1=8;D=" + std::string(243, '0') + ";\n";
    ASSERT_EQ(261u, cmd.size());
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -4 * 243 }, 250)) << "fake clock jitter adds up over 243 pulses";
}

TEST_F(SendCommandTest, rawTooLongCommandIsRejected) {
    const std::string cmd = "SR;P0=-4;P1=8;D=" + std::string(244, '0') + ";\n";
    ASSERT_EQ(262u, cmd.size());
    run(cmd);
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    EXPECT_EQ(0u, cap.inputLeft()) << "rest of the line is discarded";
}

// ---------------------------------------------------------------------------
// SM - send manchester
// ---------------------------------------------------------------------------
// bit 1 = low/high, bit 0 = high/low, MSB first, each half bit = C us
TEST_F(SendCommandTest, manchesterEncodesBits) {
    const std::string cmd = "SM;C=400;D=A5;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    // A = 1010, 5 = 0101
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -800, 800, -400, 400, -800, 800, -800, 400 }));
    EXPECT_EQ(LOW, rec.lastLevel());
    EXPECT_EQ(1u, rec.delayCalls()) << "manchester messages add delay(1) per repetition";
}

TEST_F(SendCommandTest, manchesterRepeats) {
    run("SM;R=2;C=400;D=A;\n");
    EXPECT_EQ("SM;R=2;C=400;D=A;\n", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), repeatSignal({ -400, 800, -800, 800, -400 }, 2)));
}

TEST_F(SendCommandTest, manchesterAllZeroAndAllOne) {
    // 0000 = H L H L H L H L, 1111 = L H L H L H L H
    run("SM;C=250;D=0F;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { 250, -250, 250, -250, 250, -250, 250, -500, 250, -250, 250, -250, 250, -250, 250 }));
}

TEST_F(SendCommandTest, manchesterLowerCaseHexEqualsUpperCase) {
    run("SM;C=400;D=a;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -800, 800, -400 }));
}

TEST_F(SendCommandTest, manchesterInvalidHexIsSentAsZeroNibble) {
    // 'G' - 0x37 = 16, only the low nibble (0) is sent
    run("SM;C=400;D=G;\n");
    EXPECT_EQ("SM;C=400;D=G;\n", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { 400, -400, 400, -400, 400, -400, 400, -400 }));
}

TEST_F(SendCommandTest, manchesterWithoutClockSendsZeroLengthBits) {
    run("SM;D=A;\n");
    EXPECT_EQ("SM;D=A;\n", cap.output());
    std::vector<uint8_t> levels;
    for (const auto& e : rec.events()) levels.push_back(e.level);
    const std::vector<uint8_t> expected = { LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW };
    EXPECT_EQ(expected, levels);
    for (long d : rec.signal()) EXPECT_LE(std::labs(d), 3) << "C missing -> clock 0";
}

// ---------------------------------------------------------------------------
// SC - send combined
// ---------------------------------------------------------------------------
TEST_F(SendCommandTest, combinedSendsAllPartsWithRepeats) {
    const std::string cmd = "SC;R=2;SM;C=400;D=A;SR;P0=-2500;P1=400;D=010;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    const std::vector<long> once = { -400, 800, -800, 800, -400, -2500, 400, -2500 };
    EXPECT_TRUE(signalNear(rec.signal(), repeatSignal(once, 2)));
    EXPECT_EQ(0u, rec.delayCalls()) << "no extra delay for combined messages";
}

TEST_F(SendCommandTest, combinedRepeatZeroSendsOnce) {
    // current behaviour: R=0 on SC is ignored (repeats > 0 check), default 1 is used
    run("SC;R=0;SM;C=400;D=A;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -800, 800, -400 }));
}

TEST_F(SendCommandTest, combinedWithoutRepeatSendsOnce) {
    run("SC;SM;C=400;D=A;SR;P0=-2500;P1=400;D=010;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -800, 800, -2900, 400, -2500 }));
}

TEST_F(SendCommandTest, combinedCorruptWhenCut) {
    run("SC;R=2;SM;C=400;D=A;SR;P0=-25");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}

// ---------------------------------------------------------------------------
// SN - send xFSK (needs CC1101)
// ---------------------------------------------------------------------------
TEST_F(SendCommandTest, xfskWithoutCC1101SupportIsNotSent) {
    const std::string cmd = "SN;R=13;N=2;D=91C6;\n";
    run(cmd);
    EXPECT_EQ(cmd + "no supported device to send\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
}

// ---------------------------------------------------------------------------
// Send command that does not start with 'S' (issue #267, fix in PR #268)
// ---------------------------------------------------------------------------
// FHEM/telnet sometimes prepends a stray character: "<x>SR;R=3;...". The
// dispatcher still treats it as send command (it is recognised by the ';'), but
// IB_1 does not start with 'S'. Before the fix cmdNo stayed 255, the following
// parts were written to command[255] (array holds 5 entries) and the firmware
// hung in an endless loop. Now characters in front of the first 'S' are skipped;
// without any 'S' the message is rejected.
TEST_F(SendCommandTest, garbagePrefixIsSkippedAndCommandSentWithItsRepeats) {
    run("\x01SR;R=3;P0=500;P1=-9000;P2=-4000;P3=-2000;D=0302030;\n");
    // R=3 must be honoured (not 255): three repetitions of the regular signal
    EXPECT_TRUE(signalNear(rec.signal(),
        repeatSignal({ 500, -2000, 500, -4000, 500, -2000, 500 }, 3), 40)) << cap.output();
    EXPECT_EQ(0u, cap.inputLeft());
}

TEST_F(SendCommandTest, garbagePrefixFillingIB_1IsSkipped) {
    // longest first part serialEvent() stores: 14 characters, ';' at IB_1[13]
    run("XXXXXXXXXXXSR;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;D=0101;\n", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, garbagePrefixIsNotEchoed) {
    run("\x01SR;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;D=0101;\n", cap.output());
}

TEST_F(SendCommandTest, unknownSendTypeIsRejected) {
    run("SX;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ("send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    EXPECT_EQ("", cap.remainingInput());
}

TEST_F(SendCommandTest, garbagePrefixWithoutValidPartIsRejectedAsCorrupt) {
    run("X;P0=500;D=0;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, onlySemicolonIsRejectedAsCorrupt) {
    run(";\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, validCommandStillWorksAfterRejectedOne) {
    run("X;P0=500;D=0;\n");
    cap.clear();
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_FALSE(rec.signal().empty());
}

// ===========================================================================
// Faulty transmissions, timing and unusual send commands
// ===========================================================================
// Before the fix in send.h several of these cases crashed, hung, left the
// receiver switched off or left the rest of the line in the serial buffer
// (see tests/COMMAND_COVERAGE.md).

// --- receiver state ---------------------------------------------------------
TEST_F(SendCommandTest, receiverIsEnabledAgainAfterSend) {
    run("SR;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ(1u, receiveState().disables);
    EXPECT_EQ(1u, receiveState().enables);
    EXPECT_TRUE(receiveState().enabled);
}

TEST_F(SendCommandTest, corruptCommandEnablesReceiverAgain) {
    run("SR;");
    EXPECT_EQ("SR;send cmd corrupt\n", cap.output());
    EXPECT_EQ(1u, receiveState().disables);
    EXPECT_EQ(1u, receiveState().enables);
    EXPECT_TRUE(receiveState().enabled);
}

TEST_F(SendCommandTest, tooLongCommandEnablesReceiverAgain) {
    run("SR;P0=-4;P1=8;D=" + std::string(244, '0') + ";\n");
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_TRUE(receiveState().enabled);
}

// --- serial timeouts (race between host and firmware) -------------------------
TEST_F(SendCommandTest, serialTimeoutInsideCommandDiscardsRestOfLine) {
    // the host sends the command in two TCP packets / with a pause (> setTimeout):
    // the rest of the line is discarded instead of being parsed as a new command
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    run(cmd, cmd.find("P1="));
    EXPECT_EQ("SR;P0=-400;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    EXPECT_EQ("", cap.remainingInput());
    EXPECT_TRUE(receiveState().enabled);
}

TEST_F(SendCommandTest, serialTimeoutBeforeLineEndDropsCompleteCommand) {
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    run(cmd, cmd.size() - 1);
    EXPECT_EQ("SR;P0=-400;P1=800;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty()) << "complete command is not sent";
    EXPECT_EQ("", cap.remainingInput());
}

TEST_F(SendCommandTest, serialTimeoutRestIsNotSentAsNewCommand) {
    // if the rest arrives later than the timeout, serialEvent() passes "P1=800;"
    // to send_cmd(): rejected, no endless loop (issue #267)
    run("P1=800;D=0101;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

// --- buf[256] limit with parts that stay in buf ---------------------------------
// buffer_left does not count the ';' of parts that stay in buf (C=, D=, F=,
// unknown parts, all parts after the first S part). Before the fix the write
// position ran behind buf (stack overflow).
TEST_F(SendCommandTest, manchesterLongestDataThatFitsIntoBuf) {
    // buf: "C=400;D=" + 245 x 'A' + ";\n" = 255 bytes + '\0'
    const std::string cmd = "SM;C=400;D=" + std::string(245, 'A') + ";\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    // 'A' = 1010, see manchesterEncodesBits
    EXPECT_TRUE(signalNear(rec.signal(), repeatSignal({ -400, 800, -800, 800, -400 }, 245)));
}

TEST_F(SendCommandTest, manchesterDataBeyondBufIsTooLong) {
    // one character more: '\n' would be at buf[255], '\0' at buf[256]
    run("SM;C=400;D=" + std::string(246, 'A') + ";\n");
    EXPECT_EQ("SM;send cmd to long \n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    EXPECT_EQ("", cap.remainingInput());
    EXPECT_TRUE(receiveState().enabled);
}

TEST_F(SendCommandTest, manyShortPartsBeyondBufAreTooLong) {
    std::string parts;
    for (int i = 0; i < 140; i++) parts += "X;";
    run("SR;P0=-4;P1=8;" + parts + "D=01;\n");
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    EXPECT_EQ("", cap.remainingInput());
}

// The old code also answered "to long" here, but only after writing behind buf;
// only a build with -fsanitize=address detects that regression.
TEST_F(SendCommandTest, combinedPartsBeyondBufAreTooLong) {
    run("SC;SR;P0=1;P1=1;D=01;SM;C=1;D=" + std::string(240, 'A') + ";\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd to long \n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
    EXPECT_EQ("", cap.remainingInput());
}

TEST_F(SendCommandTest, tooLongAtLineEndKeepsNextCommand) {
    // the line end is read exactly when the length limit is reached: the next
    // command must not be discarded
    const std::string next = "SR;P0=-400;P1=800;D=0101;\n";
    run("SR;P0=-4;P1=8;D=" + std::string(244, '0') + "\n" + next);
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_EQ(next, cap.remainingInput());
}

TEST_F(SendCommandTest, veryLongCommandIsDiscardedCompletely) {
    // the whole rest of the line is discarded, not only 255 further bytes
    run("SR;P0=-4;P1=8;D=" + std::string(600, '0') + ";\n");
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_EQ("", cap.remainingInput());
}

// --- line endings and empty parts -------------------------------------------
TEST_F(SendCommandTest, rawCrLfLineEndIsAccepted) {
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\r\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, rawCarriageReturnOnlyIsCorrupt) {
    // '\r' does not end a send command; the next read times out
    run("SR;P0=-400;P1=800;D=0101;\r");
    EXPECT_EQ("SR;P0=-400;P1=800;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    EXPECT_TRUE(receiveState().enabled);
}

TEST_F(SendCommandTest, rawEmptyPartAfterDataEndsCommand) {
    // ";;" ends the parsing: sent, the rest of the line is discarded and the echo
    // gets a line end
    run("SR;P0=-400;P1=800;D=0101;;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;D=0101;;\n", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
    EXPECT_EQ("", cap.remainingInput());
}

TEST_F(SendCommandTest, rawEmptyPartBeforeDataIsRejected) {
    // before the fix: segmentation fault (D= was never parsed, datastart/dataend
    // were not initialised)
    run("SR;P0=-400;P1=800;;D=0101;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, rawLastPartWithoutSemicolonIsRejected) {
    // common typing error "D=0101" + line end; before the fix: segmentation fault
    // (a part ended by '\n' is not parsed)
    run("SR;P0=-400;P1=800;D=0101\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, rawEmptyDataPartSendsNothing) {
    // before the fix: memchr() for the end of "D=" started behind the ';', the
    // bytes after it were sent as pattern indices (read outside of buckets[])
    run("SR;P0=-400;P1=800;D=;\n");
    EXPECT_TRUE(rec.signal().empty());
}

TEST_F(SendCommandTest, rawUnsetPatternIsRejected) {
    // before the fix: buckets[2] was not initialised (random pulse length)
    run("SR;P0=-400;P1=800;D=012;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, rawInvalidPatternIndexIsRejected) {
    // before the fix: ' ' - '0' and '8' read outside of buckets[]
    run("SR;P0=-400;P1=800;D= 01;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    cap.clear();
    run("SR;P0=-400;P1=800;D=018;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, rawWithoutDataPartIsRejected) {
    // before the fix: datastart/dataend were not initialised
    run("SR;P0=-400;P1=800;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, combinedPartWithoutPatternsIsRejected) {
    // example from the comment in send.h: the last SR part has no Pn= of its own
    run("SC;R=4;SM;C=400;D=AFFFFFFFFE;SR;P0=-2500;P1=400;D=010;SM;D=AB6180;SR;D=101;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

// --- value ranges (unchanged behaviour) ----------------------------------------
TEST_F(SendCommandTest, rawRepeatNegativeBecomes255) {
    // strtoul("-1") wraps, repeats is uint8_t
    run("SR;R=-1;P0=-4;P1=8;D=01;\n");
    EXPECT_EQ(510u, rec.signal().size());
    EXPECT_TRUE(signalNear(rec.signal(), repeatSignal({ -4, 8 }, 255)));
}

TEST_F(SendCommandTest, rawRepeatWithoutValueSendsNothing) {
    run("SR;R=;P0=-4;P1=8;D=01;\n");
    EXPECT_EQ("SR;R=;P0=-4;P1=8;D=01;\n", cap.output());
    EXPECT_TRUE(rec.signal().empty());
}

TEST_F(SendCommandTest, rawPatternAboveInt16FlipsLevel) {
    // P0=40000 is stored as int16_t -25536: sent as LOW instead of HIGH
    run("SR;P0=40000;P1=400;D=01;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -25536, 400 }));
}

TEST_F(SendCommandTest, manchesterClockAboveInt16Wraps) {
    // C=70000 is stored as int16_t 4464
    run("SM;C=70000;D=8;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -4464, 8928, -4464, 4464, -4464, 4464, -4464 }));
}

TEST_F(SendCommandTest, manchesterNegativeClockSendsWithoutDelay) {
    // stoptime += -400 moves the stop time into the past: no wait at all
    fakeMicrosCounter() = 1000000;
    run("SM;C=-400;D=A;\n");
    EXPECT_EQ(9u, rec.events().size());
    for (long d : rec.signal()) EXPECT_LE(std::labs(d), 3);
}

TEST_F(SendCommandTest, manchesterNegativeClockDoesNotBlock) {
    // directly after start or after a micros() overflow (micros() smaller than
    // |C| * number of half bits) the stop time wrapped to the maximum value: before
    // the fix the busy wait did not end (on the device up to ~71 minutes). The fake
    // clock starts at 0; a busy wait that does not end throws BusyWaitDoesNotEnd.
    ASSERT_EQ(0u, fakeMicrosCounter());
    try {
        run("SM;C=-400;D=A;\n");
    } catch (const serialtest::BusyWaitDoesNotEnd& e) {
        FAIL() << e.what();
    }
    EXPECT_EQ("SM;C=-400;D=A;\n", cap.output());
    // A = 1010: low/high, high/low, low/high, high/low without any wait, then low
    std::vector<uint8_t> levels;
    for (const auto& e : rec.events()) levels.push_back(e.level);
    const std::vector<uint8_t> expected = { LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW };
    EXPECT_EQ(expected, levels);
    for (long d : rec.signal()) EXPECT_LE(std::labs(d), 3);
    EXPECT_LT(fakeMicrosCalls(), 100u) << "no busy wait";
    EXPECT_TRUE(receiveState().enabled);
}

// --- micros() overflow (every 2^32 us = ~71.6 min on the device) --------------
// The fake clock is unsigned long like on the device; on the 64 bit host it
// wraps at 2^64, the comparison "stoptime > micros()" behaves the same.
// Before the fix the pulses after the overflow shrank to ~0 us.
TEST_F(SendCommandTest, rawMicrosOverflowKeepsPulseLengths) {
    fakeMicrosCounter() = ULONG_MAX - 1000;
    run("SR;P0=-400;P1=800;D=0101;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, manchesterMicrosOverflowKeepsBitLengths) {
    fakeMicrosCounter() = ULONG_MAX - 1000;
    run("SM;C=400;D=A5;\n");
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -800, 800, -400, 400, -800, 800, -800, 400 }));
}

// --- several parts -------------------------------------------------------------
TEST_F(SendCommandTest, rawSecondSendPartWithoutCombinedIsSentToo) {
    // a second S part without SC is accepted and sent like a combined message
    const std::string cmd = "SR;P0=-400;P1=800;D=01;SR;P0=-200;P1=200;D=10;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -2, 200, -200 }));
}

TEST_F(SendCommandTest, combinedFivePartsIsTheMaximum) {
    // command[5]: SC plus four send parts
    const std::string cmd = "SC;SR;P0=-10;P1=10;D=01;SR;P0=-20;P1=20;D=01;"
                            "SR;P0=-30;P1=30;D=01;SR;P0=-40;P1=40;D=01;\n";
    run(cmd);
    EXPECT_EQ(cmd, cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -10, 10, -20, 20, -30, 30, -40, 40 }));
}

TEST_F(SendCommandTest, combinedMoreThanFivePartsIsRejected) {
    // before the fix: the sixth part was written to command[5] (outside of the array)
    run("SC;SR;P0=-10;P1=10;D=01;SR;P0=-20;P1=20;D=01;SR;P0=-30;P1=30;D=01;"
        "SR;P0=-40;P1=40;D=01;SR;P0=-50;P1=50;D=01;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}
