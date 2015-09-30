/*
*   Library for storing and retrieving multibple bits in one byte
*   Copyright (C) 2014  S.Butzek
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BITSTORE_H
#define BITSTORE_H

#include "Arduino.h"

class BitStore
{
    public:
        /** Default constructor */
        BitStore(uint8_t bitlength=2, uint8_t bufsize=10);
        ~BitStore();
        void addValue(char value);
        unsigned char getValue(uint8_t pos);
        const uint8_t getSize();
        unsigned char *datastore;  // Reserve 40 Bytes for our store. Should be edited to aa dynamic way
        void reset();
        unsigned char getByte(uint8_t idx);
        uint8_t bytecount;  // Number of stored bytes
        uint8_t valcount;  // Number of total values stored
    protected:

    private:
        uint8_t valuelen;   // Number of bits for every value
        uint8_t bmask;
        uint8_t bcnt;
        uint8_t buffsize;

};




#endif // BITSTORE_H
