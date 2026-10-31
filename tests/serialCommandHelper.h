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
#include <gtest/gtest.h>
#include <gmock/gmock.h>
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
