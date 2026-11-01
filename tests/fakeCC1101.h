// fakeCC1101.h
//
// Test-side model of a CC1101 transceiver behind the AVR hardware SPI, used by
// the CC1101 test target (TestProjectCC1101, built with CMP_CC1101).
//
// src/cc1101.cpp talks to the chip through the AVR SPI registers
// (SPDR = val; while (!(SPSR & _BV(SPIF))); return SPDR;). This header maps
// SPDR/SPSR/SPCR to a fake register that forwards every byte to FakeChip, so
// the unmodified firmware code runs against the model. Chip select is
// cc1101_Select()/cc1101_Deselect() -> pinMode(csPin, LOW/HIGH) in the native
// build and is routed to the model by the test environment.
//
// The model follows the SPI access scheme of the CC1101 (data sheet, section 10):
//   header byte: bit7 = read, bit6 = burst, bit5..0 = address
//   0x30..0x3D without burst  -> command strobe
//   0x30..0x3D with burst     -> status register
//   0x3E                      -> PATABLE (8 bytes, index advances per byte)
//   0x3F                      -> TX FIFO (write) / RX FIFO (read)
// Every header byte returns the chip status byte (state in bits 6..4) as it
// was before the strobe is executed.
//
// It is a simplified model, not a complete one. Not modelled:
//   - MISO / CHIP_RDYn: the test environment always reads MISO low (chip ready),
//     so the timeout paths of wait_Miso()/waitTo_Miso() never run
//   - MARCSTATE (0x35) does not follow the state: it keeps the value set by the
//     test (0x01 = IDLE), so the wait for the end of a transmission in
//     sendFIFO() ends at once
//   - FIFO byte counts in bits 3..0 of the chip status byte (always 0)
//   - reset values: SRES sets all config registers to 0
//   - SFTX/SFRX: txFifo is a log of all bytes written to the TX FIFO
//   - 0x2F is handled like a config register (it is not one on the chip)
// Options for error cases:
//   - stuckStrobe: this strobe does not change the state (e.g. STX that fails)
//   - every SPI byte is counted; after transferLimit bytes TooManyTransfers is
//     thrown, so an endless loop in the firmware fails the test instead of
//     hanging the test run
//
// No firmware code is changed for these tests.

#ifndef FAKE_CC1101_H
#define FAKE_CC1101_H

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

// SPI pins of an ATmega328 (Arduino nano) - not defined by arduino-mock
#ifndef SS
#define SS   10
#endif
#ifndef MOSI
#define MOSI 11
#endif
#ifndef MISO
#define MISO 12
#endif
#ifndef SCK
#define SCK  13
#endif

namespace fakecc {

struct RegWrite {
    uint8_t addr;
    uint8_t value;
};

struct TooManyTransfers : std::runtime_error {
    TooManyTransfers() : std::runtime_error("too many SPI transfers to the CC1101 (endless loop?)") {}
};

class FakeChip {
public:
    enum : uint8_t { STATE_IDLE = 0x00, STATE_RX = 0x01, STATE_TX = 0x02 };

    FakeChip() { reset(); }

    void reset() {
        memset(cfg, 0, sizeof(cfg));
        memset(status, 0, sizeof(status));
        memset(patable, 0, sizeof(patable));
        status[0x31 - 0x30] = 0x14;   // VERSION of a current CC1101
        status[0x35 - 0x30] = 0x01;   // MARCSTATE idle
        state = STATE_IDLE;
        strobes.clear();
        writes.clear();
        txFifo.clear();
        selected = false;
        selectCount = 0;
        unselectedBytes = 0;
        transferCount = 0;
        stuckStrobe = 0;
        inHeader = true;
    }

    void select() { selected = true; selectCount++; inHeader = true; }
    void deselect() { selected = false; inHeader = true; }

    uint8_t transfer(uint8_t b) {
        if (++transferCount > transferLimit) throw TooManyTransfers();
        if (!selected) { unselectedBytes++; return 0xFF; }
        return inHeader ? headerByte(b) : dataByte(b);
    }

    uint8_t cfg[0x30];
    uint8_t status[0x0E];
    uint8_t patable[8];
    uint8_t state;
    std::vector<uint8_t> strobes;
    std::vector<RegWrite> writes;
    std::vector<uint8_t> txFifo;
    unsigned selectCount = 0;
    unsigned unselectedBytes = 0;   // SPI bytes without chip select (checked after every test)
    unsigned long transferCount = 0;
    static constexpr unsigned long transferLimit = 100000UL;   // a factory reset needs < 200
    uint8_t stuckStrobe = 0;        // 0 = none

private:
    uint8_t headerByte(uint8_t b) {
        const uint8_t statusByte = static_cast<uint8_t>(state << 4);
        addr = b & 0x3F;
        read = (b & 0x80) != 0;
        burst = (b & 0x40) != 0;
        patIndex = 0;
        const bool isStrobe = addr >= 0x30 && addr <= 0x3D && !burst;
        if (isStrobe) strobe(addr);
        inHeader = isStrobe;   // a strobe has no data bytes
        return statusByte;
    }

    uint8_t dataByte(uint8_t b) {
        if (addr < 0x30) return configByte(b);
        if (addr <= 0x3D) return status[addr - 0x30];   // status registers are read only
        if (addr == 0x3E) return patableByte(b);
        if (!read) txFifo.push_back(b);                  // 0x3F FIFO
        return 0;
    }

    uint8_t configByte(uint8_t b) {
        uint8_t ret = 0;
        if (read) ret = cfg[addr];
        else { cfg[addr] = b; writes.push_back(RegWrite{ addr, b }); }
        if (burst) addr++;
        return ret;
    }

    uint8_t patableByte(uint8_t b) {
        uint8_t ret = 0;
        if (read) ret = patable[patIndex & 7];
        else patable[patIndex & 7] = b;
        if (burst) patIndex++;
        return ret;
    }

    void strobe(uint8_t s) {
        strobes.push_back(s);
        if (s == stuckStrobe) return;
        switch (s) {
            case 0x30: // SRES
                memset(cfg, 0, sizeof(cfg));
                state = STATE_IDLE;
                break;
            case 0x34: state = STATE_RX; break;   // SRX
            case 0x35: state = STATE_TX; break;   // STX
            case 0x36: state = STATE_IDLE; break; // SIDLE
            default: break;
        }
    }

    bool selected = false;
    bool inHeader = true;
    uint8_t addr = 0;
    bool read = false;
    bool burst = false;
    uint8_t patIndex = 0;
};

inline FakeChip& chip() {
    static FakeChip c;
    return c;
}

// Stand-in for the AVR SPI data register
class FakeSpdr {
public:
    FakeSpdr& operator=(uint8_t v) { last_ = chip().transfer(v); return *this; }
    operator uint8_t() const { return last_; }
private:
    uint8_t last_ = 0;
};

inline FakeSpdr& spdr() {
    static FakeSpdr r;
    return r;
}

inline uint8_t& spcr() {
    static uint8_t r = 0;
    return r;
}

} // namespace fakecc

// Pin helpers that output.h leaves undefined for native (__linux__/WIN32) builds.
// The main test build defines isHigh/isLow the same way in test_serialCommand.cpp.
// Pin direction changes are only counted, they must not look like the
// pinMode(pin, LOW/HIGH) calls that digitalLow/digitalHigh use natively.
namespace fakecc {
struct PinDirections {
    std::vector<std::pair<uint8_t, uint8_t>> calls;  // pin, 0 = input, 1 = output, 2 = input pullup
};
inline PinDirections& pinDirections() {
    static PinDirections d;
    return d;
}
inline void pinDir(uint8_t pin, uint8_t dir) { pinDirections().calls.push_back(std::make_pair(pin, dir)); }
} // namespace fakecc
#ifndef isHigh
#define isHigh(pin) (digitalRead(pin) == HIGH)
#endif
#ifndef isLow
#define isLow(pin) (digitalRead(pin) == LOW)
#endif
#ifndef pinAsInput
#define pinAsInput(pin) fakecc::pinDir(pin, 0)
#endif
#ifndef pinAsOutput
#define pinAsOutput(pin) fakecc::pinDir(pin, 1)
#endif
#ifndef pinAsInputPullUp
#define pinAsInputPullUp(pin) fakecc::pinDir(pin, 2)
#endif

#define SPDR (fakecc::spdr())
#define SPSR 0xFF            // transfer is always complete
#define SPCR (fakecc::spcr())
#ifndef SPIF
#define SPIF 7
#endif
#ifndef SPE
#define SPE 6
#endif
#ifndef MSTR
#define MSTR 4
#endif
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#endif // FAKE_CC1101_H
