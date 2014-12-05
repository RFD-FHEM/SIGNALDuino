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

#include "bitstore.h"

BitStore::BitStore(uint8_t bitlength,uint8_t bufsize)
{
    valuelen = bitlength; // How many bits shoudl be reserved for one value added ?
    bmask=0;
    BitStore::buffsize = bufsize;
    datastore= (unsigned char*) calloc(bufsize,sizeof(char)); // Speicher allokieren und 0 zuweisen
    reset();
    for (uint8_t x=7;x>(7-valuelen);x--)
    {
        bmask = bmask | (1<<x);
    }
}

void BitStore::addValue(char value)
{
    if (bytecount >=buffsize ) return; // Out of Buffer
    //store[bytecount]=datastore[bytecount] | (value<<bcnt)
    datastore[bytecount]=datastore[bytecount] | (value<<bcnt);  // (valcount*valuelen%8)
/*
    Serial.print("Adding value:");   Serial.print(value,DEC);
    Serial.print(" at byte: ");   Serial.print(bytecount,DEC);
    Serial.print(" at pos: ");   Serial.print(bcnt,DEC);
    Serial.print("  datastore is (bin)");   Serial.print(datastore[bytecount],BIN);
    Serial.print("  (dec)");   Serial.print(datastore[bytecount],DEC);
    Serial.println("");
*/
    valcount++;
    if ((bcnt-valuelen) >= 0)  // Soalnge nicht 8 Bit geppeichert wurden, erhöhen wir den counter zum verschieben
    {
        bcnt=bcnt-valuelen; //+valuelen
    } else {
        bcnt=7;
        bytecount++;
        datastore[bytecount]=0;
    }

}
const uint8_t BitStore::getSize()
{
    return valcount;
}

unsigned char BitStore::getValue(uint8_t pos)
{
   if ((pos*valuelen/8) >=buffsize ) return -1; // Out of Buffer

   uint8_t mask; // Local modified bitmask
   unsigned char ret;
   //Serial.print("Bitmask:");   Serial.println(bmask,DEC);
   //ret= (datastore[pos*valuelen/8]>>(pos*valuelen%8))&bmask;
   mask = bmask >> (pos*valuelen%8);            //Mask the position we want to retrieve
   ret= datastore[pos*valuelen/8]&mask;         // Combine the mask with our store to extract the bit
   ret=ret>>(7-(pos*valuelen%8));               // Align the the bits to the right edge
   return ret;
}

unsigned char BitStore::getByte(uint8_t idx)
{
  return (datastore[idx]);
}


void BitStore::reset()
{
  for (uint8_t i=0;i<buffsize;i++)
  {
      datastore[i]=0;
  }
  bytecount=0;
  valcount=0;
  bcnt=7;
}
