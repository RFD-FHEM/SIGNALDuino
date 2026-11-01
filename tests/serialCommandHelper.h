// serialCommandHelper.h
//
// Test helpers for the serial command unit tests.
//
// SerialCapture  - installs catch-all expectations on the arduino-mock SerialMock
//                  and records everything the firmware prints via MSG_PRINT /
//                  MSG_PRINTLN / MSG_WRITE into a std::string.
// runShortCommand - copies a command into the input buffer IB_1 (bounds checked)
//                  and calls commands::HandleShortCommand().
//
// Output conventions of the capture:
//   - println(...) appends "\n" (Arduino sends "\r\n"; the line ending is a property
//     of the Serial library, not of the firmware under test).
//   - Numbers are formatted like the Arduino Print class: DEC, HEX (upper case),
//     OCT, BIN without prefix or leading zeros.
//
// No firmware code is changed for these tests.

#ifndef SERIAL_COMMAND_HELPER_H
#define SERIAL_COMMAND_HELPER_H

#include <string>
#include <cstring>
#include <algorithm>
#include <vector>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <arduino-mock/Arduino.h>
#include <arduino-mock/Serial.h>

extern char IB_1[14];

namespace serialtest {

// Formats a number like Arduino's Print::printNumber / Print::print(double, digits)
template <typename T>
std::string formatNumber(T value, int base) {
    if (base != DEC && base != HEX && base != OCT && base != BIN) base = DEC;
    bool negative = false;
    unsigned long long n;
    if (value < 0) {
        // Arduino prints negative numbers only with '-' for base DEC;
        // for other bases the two's complement of a long is printed.
        if (base == DEC) {
            negative = true;
            n = static_cast<unsigned long long>(-(static_cast<long long>(value)));
        } else {
            n = static_cast<unsigned long>(static_cast<long>(value));
        }
    } else {
        n = static_cast<unsigned long long>(value);
    }
    std::string digits;
    do {
        const unsigned d = static_cast<unsigned>(n % base);
        digits.insert(digits.begin(), static_cast<char>(d < 10 ? '0' + d : 'A' + d - 10));
        n /= base;
    } while (n);
    return negative ? "-" + digits : digits;
}

inline std::string formatDouble(double value, int digits) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", digits < 0 ? 0 : digits, value);
    return buf;
}

class SerialCapture {
public:
    SerialCapture() : mock_(serialMockInstance()) {
        using ::testing::_;
        using ::testing::AnyNumber;
        using ::testing::Matcher;

        // print(...)
        EXPECT_CALL(*mock_, print(Matcher<const char*>(_))).Times(AnyNumber())
            .WillRepeatedly([this](const char* s) { return append(s ? s : ""); });
        EXPECT_CALL(*mock_, print(Matcher<char>(_))).Times(AnyNumber())
            .WillRepeatedly([this](char c) { return append(std::string(1, c)); });
        EXPECT_CALL(*mock_, print(Matcher<unsigned char>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](unsigned char v, int b) { return append(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, print(Matcher<int>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](int v, int b) { return append(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, print(Matcher<unsigned int>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](unsigned int v, int b) { return append(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, print(Matcher<long>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](long v, int b) { return append(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, print(Matcher<unsigned long>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](unsigned long v, int b) { return append(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, print(Matcher<double>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](double v, int d) { return append(formatDouble(v, d)); });
        EXPECT_CALL(*mock_, print(Matcher<__FlashStringHelper*>(_))).Times(AnyNumber())
            .WillRepeatedly([this](__FlashStringHelper* s) {
                return append(s ? reinterpret_cast<const char*>(s) : ""); });

        // println(...)
        EXPECT_CALL(*mock_, println(Matcher<const char*>(_))).Times(AnyNumber())
            .WillRepeatedly([this](const char* s) { return appendLine(s ? s : ""); });
        EXPECT_CALL(*mock_, println(Matcher<char>(_))).Times(AnyNumber())
            .WillRepeatedly([this](char c) { return appendLine(std::string(1, c)); });
        EXPECT_CALL(*mock_, println(Matcher<unsigned char>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](unsigned char v, int b) { return appendLine(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, println(Matcher<int>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](int v, int b) { return appendLine(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, println(Matcher<unsigned int>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](unsigned int v, int b) { return appendLine(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, println(Matcher<long>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](long v, int b) { return appendLine(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, println(Matcher<unsigned long>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](unsigned long v, int b) { return appendLine(formatNumber(v, b)); });
        EXPECT_CALL(*mock_, println(Matcher<double>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](double v, int d) { return appendLine(formatDouble(v, d)); });
        EXPECT_CALL(*mock_, println(Matcher<__FlashStringHelper*>(_))).Times(AnyNumber())
            .WillRepeatedly([this](__FlashStringHelper* s) {
                return appendLine(s ? reinterpret_cast<const char*>(s) : ""); });
        EXPECT_CALL(*mock_, println()).Times(AnyNumber())
            .WillRepeatedly([this]() { return appendLine(""); });

        // write(...)
        EXPECT_CALL(*mock_, write(Matcher<uint8_t>(_))).Times(AnyNumber())
            .WillRepeatedly([this](uint8_t c) { return append(std::string(1, static_cast<char>(c))); });
        EXPECT_CALL(*mock_, write(Matcher<const char*>(_))).Times(AnyNumber())
            .WillRepeatedly([this](const char* s) { return append(s ? s : ""); });
        EXPECT_CALL(*mock_, write(Matcher<const uint8_t*>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](const uint8_t* b, size_t n) {
                return append(std::string(reinterpret_cast<const char*>(b), n)); });
        EXPECT_CALL(*mock_, write(Matcher<const char*>(_), _)).Times(AnyNumber())
            .WillRepeatedly([this](const char* b, size_t n) { return append(std::string(b, n)); });
    }

    ~SerialCapture() { releaseSerialMock(); }

    // Provides bytes for Serial.available / read / readBytes / readBytesUntil,
    // i.e. the rest of a send command that is still in the serial buffer.
    // gapAt: simulates a serial timeout (e.g. a delayed TCP packet). The first
    // read that would deliver the byte at this position returns nothing; the
    // remaining bytes stay in the buffer and are delivered by later reads.
    void setInput(const std::string& in, size_t gapAt = std::string::npos) {
        using ::testing::_;
        using ::testing::AnyNumber;
        in_ = in;
        inPos_ = 0;
        readCalls_ = 0;
        gapAt_ = gapAt;
        EXPECT_CALL(*mock_, available()).Times(AnyNumber())
            .WillRepeatedly([this]() { return static_cast<uint8_t>(std::min<size_t>(255, in_.size() - inPos_)); });
        EXPECT_CALL(*mock_, read()).Times(AnyNumber())
            .WillRepeatedly([this]() { return inPos_ < in_.size() ? static_cast<uint8_t>(in_[inPos_++]) : static_cast<uint8_t>(0xFF); });
        EXPECT_CALL(*mock_, readBytes(_, _)).Times(AnyNumber())
            .WillRepeatedly([this](char buf[], uint8_t len) {
                readCalls_++;
                if (inPos_ == gapAt_) { gapAt_ = std::string::npos; return static_cast<uint8_t>(0); }
                uint8_t n = 0;
                while (n < len && inPos_ < in_.size()) buf[n++] = in_[inPos_++];
                return n;
            });
        EXPECT_CALL(*mock_, readBytesUntil(_, _, _)).Times(AnyNumber())
            .WillRepeatedly([this](uint8_t term, const char[], uint8_t len) {
                // consumes the bytes like Stream::readBytesUntil (terminator is discarded)
                uint8_t n = 0;
                while (n < len && inPos_ < in_.size()) {
                    const char c = in_[inPos_++];
                    if (static_cast<uint8_t>(c) == term) break;
                    n++;
                }
                return n;
            });
    }
    size_t inputLeft() const { return in_.size() - inPos_; }
    std::string remainingInput() const { return in_.substr(inPos_); }
    unsigned readCalls() const { return readCalls_; }

    SerialCapture(const SerialCapture&) = delete;
    SerialCapture& operator=(const SerialCapture&) = delete;

    SerialMock* mock() { return mock_; }
    const std::string& output() const { return out_; }
    void clear() { out_.clear(); }

private:
    size_t append(const std::string& s) { out_ += s; return s.size(); }
    size_t appendLine(const std::string& s) { out_ += s; out_ += "\n"; return s.size() + 1; }

    SerialMock* mock_;
    std::string out_;
    std::string in_;
    size_t inPos_ = 0;
    size_t gapAt_ = std::string::npos;
    unsigned readCalls_ = 0;
};

// ---------------------------------------------------------------------------
// Fake microsecond clock for the send functions.
// The arduino-mock micros() always returns 0, which would make the busy-wait
// loops in send_raw()/send_mc() endless. test_serialCommand.cpp maps micros()
// to fakeMicros() for send.h only (preprocessor, no firmware change).
// Every call advances the clock by 1 us, so a busy wait for d us ends exactly
// d us after it started.
// ---------------------------------------------------------------------------
inline unsigned long& fakeMicrosCounter() {
    static unsigned long counter = 0;
    return counter;
}
inline unsigned long fakeMicros() { return fakeMicrosCounter()++; }

// ---------------------------------------------------------------------------
// Receiver state as switched by send_cmd(). test_serialCommand.cpp maps
// disableReceive()/enableReceive() to these wrappers for send.h only
// (preprocessor, no firmware change); they call the real functions.
// ---------------------------------------------------------------------------
struct ReceiveState {
    unsigned disables = 0;
    unsigned enables = 0;
    bool enabled = true;
};
inline ReceiveState& receiveState() {
    static ReceiveState state;
    return state;
}

// One level change on the send pin
struct PinEvent {
    uint8_t level;      // HIGH / LOW
    unsigned long time; // fake micros at the time of the change
};

// Records level changes of the send pin (digitalLow/digitalHigh map to
// pinMode(pin, LOW/HIGH) in the native build, see output.h) and delay() calls.
class SendRecorder {
public:
    explicit SendRecorder(uint8_t pin) : mock_(arduinoMockInstance()) {
        using ::testing::_;
        using ::testing::AnyNumber;
        fakeMicrosCounter() = 0;
        EXPECT_CALL(*mock_, pinMode(_, _)).Times(AnyNumber())
            .WillRepeatedly([this, pin](uint8_t p, uint8_t level) {
                if (p == pin) events_.push_back(PinEvent{ level, fakeMicrosCounter() - 1 });
                else otherPinCalls_++;
            });
        EXPECT_CALL(*mock_, delay(_)).Times(AnyNumber())
            .WillRepeatedly([this](int) { delayCalls_++; });
        EXPECT_CALL(*mock_, digitalWrite(_, _)).Times(AnyNumber());
    }
    ~SendRecorder() { releaseArduinoMock(); }

    SendRecorder(const SendRecorder&) = delete;
    SendRecorder& operator=(const SendRecorder&) = delete;

    const std::vector<PinEvent>& events() const { return events_; }

    // Signal as "+high_us" / "-low_us" durations between level changes.
    // Consecutive changes to the same level are merged.
    std::vector<long> signal() const {
        std::vector<long> out;
        for (size_t i = 0; i + 1 < events_.size(); i++) {
            const long d = static_cast<long>(events_[i + 1].time - events_[i].time);
            const long v = events_[i].level == HIGH ? d : -d;
            if (!out.empty() && ((out.back() > 0) == (v > 0)) && v != 0) out.back() += v;
            else if (d != 0) out.push_back(v);
        }
        return out;
    }
    uint8_t lastLevel() const { return events_.empty() ? 0xFF : events_.back().level; }
    unsigned delayCalls() const { return delayCalls_; }
    unsigned otherPinCalls() const { return otherPinCalls_; }

private:
    ArduinoMock* mock_;
    std::vector<PinEvent> events_;
    unsigned delayCalls_ = 0;
    unsigned otherPinCalls_ = 0;
};

// Copies cmd into IB_1 (zero padded) and returns false if it does not fit
// (IB_1 holds 13 characters plus terminator).
inline bool loadCommand(const char* cmd) {
    const size_t len = strlen(cmd);
    if (len >= sizeof(IB_1)) return false;
    memset(IB_1, 0, sizeof(IB_1));
    memcpy(IB_1, cmd, len);
    return true;
}

} // namespace serialtest

#endif // SERIAL_COMMAND_HELPER_H
