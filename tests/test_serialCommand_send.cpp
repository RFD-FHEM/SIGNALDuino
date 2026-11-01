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
    // P8 is neither stored nor echoed; current behaviour: the skipped part shifts
    // the echo buffer, so a stale rest of "P0=-400;" is echoed after the line end
    EXPECT_EQ("SR;P0=-400;P1=800;D=0101;\n0=-400;", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
}

TEST_F(SendCommandTest, rawEchoContainsStaleBytesWhenDataPartIsShorter) {
    // current behaviour: the data part is written over the previous part in the
    // echo buffer and not null terminated, so a rest of "P1=800;" is echoed
    run("SR;P0=-400;P1=800;D=01;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;D=01;\n;", cap.output());
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
// IB_1 does not start with 'S', so cmdNo stays 255 and the following parts are
// written to command[255] (array holds 5 entries); the result is an endless loop.
// Without the fix of PR #268 these tests hang (endless loop), therefore they are
// DISABLED_ until it is merged: remove the prefix then (run with
// --gtest_also_run_disabled_tests to try them).
// Without the fix of PR #268 these tests hang. With the fix, characters are
// skipped until an 'S' is found; without any 'S' the message is rejected.
TEST_F(SendCommandTest, DISABLED_garbagePrefixIsSkippedAndCommandSentWithItsRepeats) {
    run("\x01SR;R=3;P0=500;P1=-9000;P2=-4000;P3=-2000;D=0302030;\n");
    // R=3 must be honoured (not 255): three repetitions of the regular signal
    EXPECT_TRUE(signalNear(rec.signal(),
        repeatSignal({ 500, -2000, 500, -4000, 500, -2000, 500 }, 3), 40)) << cap.output();
    EXPECT_EQ(0u, cap.inputLeft());
}

TEST_F(SendCommandTest, DISABLED_garbagePrefixWithoutValidPartIsRejectedAsCorrupt) {
    run("X;P0=500;D=0;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, DISABLED_onlySemicolonIsRejectedAsCorrupt) {
    run(";\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, DISABLED_validCommandStillWorksAfterRejectedOne) {
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
// Tests without DISABLED_ describe the current firmware behaviour (also where it
// is questionable, see tests/COMMAND_COVERAGE.md). DISABLED_ tests crash or hang
// with the current firmware; they describe the proposed behaviour and can be run
// with --gtest_also_run_disabled_tests.

// --- receiver state ---------------------------------------------------------
TEST_F(SendCommandTest, receiverIsEnabledAgainAfterSend) {
    run("SR;P0=-400;P1=800;D=0101;\n");
    EXPECT_EQ(1u, receiveState().disables);
    EXPECT_EQ(1u, receiveState().enables);
    EXPECT_TRUE(receiveState().enabled);
}

TEST_F(SendCommandTest, corruptCommandLeavesReceiverDisabled) {
    // current behaviour: send_cmd() returns before enableReceive()
    run("SR;");
    EXPECT_EQ("SR;send cmd corrupt\n", cap.output());
    EXPECT_EQ(1u, receiveState().disables);
    EXPECT_EQ(0u, receiveState().enables);
    EXPECT_FALSE(receiveState().enabled) << "receiver stays off until XE or restart";
}

TEST_F(SendCommandTest, tooLongCommandLeavesReceiverDisabled) {
    run("SR;P0=-4;P1=8;D=" + std::string(244, '0') + ";\n");
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_FALSE(receiveState().enabled) << "receiver stays off until XE or restart";
}

// --- serial timeouts (race between host and firmware) -------------------------
TEST_F(SendCommandTest, serialTimeoutInsideCommandLeavesRestInBuffer) {
    // the host sends the command in two TCP packets / with a pause (> setTimeout)
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    run(cmd, cmd.find("P1="));
    EXPECT_EQ("SR;P0=-400;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty());
    // the rest is processed by serialEvent() as a new command; it does not start
    // with 'S' (see issue #267 and the DISABLED_ test below)
    EXPECT_EQ("P1=800;D=0101;\n", cap.remainingInput());
    EXPECT_FALSE(receiveState().enabled);
}

TEST_F(SendCommandTest, serialTimeoutBeforeLineEndDropsCompleteCommand) {
    const std::string cmd = "SR;P0=-400;P1=800;D=0101;\n";
    run(cmd, cmd.size() - 1);
    EXPECT_EQ("SR;P0=-400;P1=800;send cmd corrupt\n", cap.output());
    EXPECT_TRUE(rec.events().empty()) << "complete command is not sent";
    EXPECT_EQ("\n", cap.remainingInput());
}

TEST_F(SendCommandTest, DISABLED_serialTimeoutRestIsNotSentAsNewCommand) {
    // second half of serialTimeoutInsideCommandLeavesRestInBuffer: serialEvent()
    // passes "P1=800;" to send_cmd(). Hangs without the fix of PR #268.
    run("P1=800;D=0101;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n")) << cap.output();
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, veryLongCommandLeavesRestInBuffer) {
    // "too long" discards at most 255 further bytes; the rest stays in the buffer
    // and is parsed by serialEvent() (its last part "000...0;" has no leading 'S')
    run("SR;P0=-4;P1=8;D=" + std::string(600, '0') + ";\n");
    EXPECT_EQ("SR;P0=-4;P1=8;send cmd to long \n", cap.output());
    EXPECT_EQ(std::string(100, '0') + ";\n", cap.remainingInput());
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
    EXPECT_FALSE(receiveState().enabled);
}

TEST_F(SendCommandTest, rawEmptyPartAfterDataEndsCommandEarly) {
    // ";;" ends the parsing; the echo has no line end and "\n" stays in the buffer
    run("SR;P0=-400;P1=800;D=0101;;\n");
    EXPECT_EQ("SR;P0=-400;P1=800;D=0101;;", cap.output());
    EXPECT_TRUE(signalNear(rec.signal(), { -400, 800, -400, 800 }));
    EXPECT_EQ("\n", cap.remainingInput());
}

TEST_F(SendCommandTest, DISABLED_rawEmptyPartBeforeDataIsRejected) {
    // currently: segmentation fault (D= is never parsed, datastart/dataend are
    // not initialised)
    run("SR;P0=-400;P1=800;;D=0101;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, DISABLED_rawLastPartWithoutSemicolonIsRejected) {
    // common typing error "D=0101" + line end; currently: segmentation fault
    // (a part ended by '\n' is never parsed)
    run("SR;P0=-400;P1=800;D=0101\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}

TEST_F(SendCommandTest, DISABLED_rawEmptyDataPartSendsNothing) {
    // currently: memchr() for the end of "D=" starts behind the ';', the bytes
    // after it are sent as pattern indices (read outside of buckets[])
    run("SR;P0=-400;P1=800;D=;\n");
    EXPECT_TRUE(rec.signal().empty());
}

// --- value ranges -------------------------------------------------------------
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

TEST_F(SendCommandTest, DISABLED_manchesterNegativeClockDoesNotBlock) {
    // directly after start or after a micros() overflow (micros() < 400) the stop
    // time wraps to the maximum value: the busy wait does not end (on the device
    // it lasts up to ~71 minutes). Currently hangs.
    run("SM;C=-400;D=A;\n");
    SUCCEED();
}

// --- micros() overflow (every 2^32 us = ~71.6 min on the device) --------------
// The fake clock is unsigned long like on the device; on the 64 bit host it
// wraps at 2^64, the comparison "stoptime > micros()" behaves the same.
TEST_F(SendCommandTest, rawMicrosOverflowShortensPulses) {
    fakeMicrosCounter() = ULONG_MAX - 1000;
    run("SR;P0=-400;P1=800;D=0101;\n");
    const std::vector<long> sig = rec.signal();
    ASSERT_EQ(4u, sig.size());
    EXPECT_NEAR(-400, sig[0], 3);
    // current behaviour: after the stop time wrapped, the wait ends at once
    for (size_t i = 1; i < sig.size(); i++) EXPECT_LE(std::labs(sig[i]), 3) << i;
}

TEST_F(SendCommandTest, manchesterMicrosOverflowShortensBits) {
    fakeMicrosCounter() = ULONG_MAX - 1000;
    run("SM;C=400;D=A5;\n");
    const std::vector<long> sig = rec.signal();
    ASSERT_EQ(10u, sig.size());
    EXPECT_NEAR(-400, sig[0], 3);
    EXPECT_NEAR(400, sig[1], 3);
    for (size_t i = 2; i < sig.size(); i++) EXPECT_LE(std::labs(sig[i]), 3) << i;
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

TEST_F(SendCommandTest, DISABLED_combinedMoreThanFivePartsIsRejected) {
    // currently: the sixth part is written to command[5] (outside of the array)
    run("SC;SR;P0=-10;P1=10;D=01;SR;P0=-20;P1=20;D=01;SR;P0=-30;P1=30;D=01;"
        "SR;P0=-40;P1=40;D=01;SR;P0=-50;P1=50;D=01;\n");
    EXPECT_NE(std::string::npos, cap.output().find("send cmd corrupt\n"));
    EXPECT_TRUE(rec.events().empty());
}
