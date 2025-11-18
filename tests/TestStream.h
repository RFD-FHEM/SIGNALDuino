#pragma once
 
 
#include "Stream.h" // Versuch, das Include zu vereinfachen, da es in der Testumgebung oft anders gehandhabt wird
#include <vector>
#include <string>
 
// Eine einfache Implementierung von Stream für Testzwecke, die geschriebene Daten speichert.
class TestStream : public Stream {
public:
    std::vector<uint8_t> buffer;
    std::string last_string;

    TestStream() : last_string("") {}

    // Implementierung der virtuellen Methode write(uint8_t) aus Stream
    size_t write(uint8_t byte) {
        return write(&byte, 1);
    }

    // Implementierung der virtuellen Methode write(const uint8_t*, size_t) aus Stream
    size_t write(const uint8_t *buffer_ptr, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            buffer.push_back(buffer_ptr[i]);
        }
        return size;
    }

    // Hinzugefügte Überladung für char, wie vom Benutzer vorgeschlagen
    size_t write(const char *str) {
    	return write((const uint8_t*)str, strlen(str));
    }

    // Hilfsfunktion, um den Inhalt als String zu erhalten (nützlich für Tests)
    std::string getContent() const {
        return std::string(buffer.begin(), buffer.end());
    }

    // Implementierung der read-Funktionen, die für einen vollständigen Stream oft benötigt werden.
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }
    void flush() {}

    // Implementierung der fehlenden reinen virtuellen Methoden von Stream
    bool find(char *target) { return false; }
    bool find(uint8_t *target) { return false; }
    bool find(char *target, size_t length) { return false; }
    bool find(uint8_t *target, size_t length) { return false; }
    size_t readBytes( char *buffer, size_t length) { return 0; }
    size_t readBytes( uint8_t *buffer, size_t length) { return 0; }
};
