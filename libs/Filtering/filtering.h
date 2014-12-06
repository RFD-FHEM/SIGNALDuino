/*
*   Library for digital signal filtering v1.1
*   2014  N.Butzk, S.Butzek
*
*   The library offers different functionality to perform digital signal
*   filtering. The core element is a fifo ring buffer to store the data, which
*   is used to store the data for filter calculation. As the buffer is equipped
*   with additional functionality, is can also be used separately, which we do
*   alot. While the buffer and the fir classes are tested, the IIR class is still
*   under development.Although the basic functionality is implemented, it is not properly
*   tested yet. Please keep this in mind, if you intend to use it.
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
#ifndef filtering_h
#define filtering_h

#include "Arduino.h"

/*
struct Puffer {
  int Value;   // Kann auch byte etc sein. Bin mir aber nicht sicher ob wir so einen Array anlegen können
} ;
*/


class RingBuffer {
	public:
	RingBuffer();
    RingBuffer(int Lsize, int StartValue=0);

    void addValue(int *M);
	void resetBuffer();
	void resetFReadPointer();       // Funktion liefert das Nächste Element des feien Lesezeigers und verschiebt diesen
	void setFReadPointerToRead(int offset=0);
	void moveFReadPointer(int offset=0);  /* Verschiebt den freien Lesezeiger um das benannte Offset */
    int* getNextValue();
    int* getPrevValue();
	bool getNewValue(int* retValue);
	int* getBuffer();
	void clearBuffer(int StartValue);
	bool checkMemory();
	int getBuffSize();
	void dump();

	volatile bool _newAvailable;
	int *head, *end;
	int * _write;
	int *_read, *_readFree;

	int* getMemory(int Lsize);
	int _Lsize;
	//private:

	protected:
//	int **_ptrwrite;

	bool _memoryOk;
	void skipFwd( int **pointer);
	void skipBwd( int **pointer);
	int* movePointer(int* refpointer, int offset);
};

/*
  **Experimentell**
  Multiring ist eine Erweiterung des Ringbuffers, mit der Idee dass mehrere Instanzen von Multiring,
  den gleichen Puffer nutzen (*head und *end sind statisch über alle Instanzen). Dadurch können mehrere
  Klassen vom typ MultiRing den gleichen Ringbuffer nutzen, haben aber alle eigene Zeiger.
*/
class MultiRing : public RingBuffer {
	public:
	    static bool _newAvailable;
		static int *stathead, *statend, *statwrite;    // Variable statisch machen!

		static MultiRing *first_obj;   // Zeiger auf 1. Objekt
		MultiRing *next_obj; // Zeiger auf Nächstes Objekt

		MultiRing(int Lsize, int StartValue=0);
		void addValue(int *M);  // Fehlerhaft, deshalb auskommentiert
		bool getNewValue(int* retValue);

};


class MovingAverage : RingBuffer {
  public:
    MovingAverage(int Lsize, int StartValue=0);
    float filterMA(int *M);
  private:
	void addValue(int *M);
    int _sum;
};

class IIR
{
  #define IIRmaxLength 6
  public:
    IIR(int order, int fparams[], int bparams[], int StartValue);
    int filterIIR(int *newValue);
	int _order;
	int _fparams[IIRmaxLength], _bparams[IIRmaxLength];

    RingBuffer _forward;
	RingBuffer _backward;
  private:
//	void addValue(int *M);
//    int _sum;
};
#endif
