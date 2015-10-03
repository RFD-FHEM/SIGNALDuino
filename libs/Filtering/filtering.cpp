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

#include "Arduino.h"
#include "filtering.h"

//#define DEBUG

// Konstructor für RingBuffer ohne Parameter
RingBuffer::RingBuffer()
{
  // Hier passiert eigentlich nichts, damit kann eine Kindklasse alles selbst organisieren
}

// Konstructor für RingBuffer Größe Lsize
RingBuffer::RingBuffer(int Lsize, int StartValue)
{

#ifdef DEBUG
	Serial.println("Init Liste ->");
#endif
	head=getMemory(Lsize);
	if (!_memoryOk)
		return; // Fehler
	end = head+_Lsize-1;
	_write = end;   // _write Zeiger setzen
	//RingBuffer::_ptrwrite = &_write;
	RingBuffer::clearBuffer(StartValue); //führt auch ResetBuffer aus und setzt damit _read=_write
	RingBuffer::resetFReadPointer();


#ifdef DEBUG
	Serial.println("<- Init Liste");
#endif
}
void RingBuffer::dump()
{
	Serial.print("head:" ); Serial.println((int)head);
	Serial.print("end:" ); Serial.println((int)end);
	Serial.print("_read:" ); Serial.print((int)_read); Serial.print(" = head + " );Serial.println((int)_read-(int)head);
	Serial.print("_write:" ); Serial.print((int)_write); Serial.print(" = head + " );Serial.println((int)_write-(int)head);
	Serial.print("_readFree:" ); Serial.print((int)_readFree); Serial.print(" = head + " );Serial.println((int)_readFree-(int)head);
}

int* RingBuffer::getMemory(int Lsize)
{
	_Lsize = Lsize; // Versuche die geforderten(Lsize) Elemente unter zu bekommen
	int cnt = 0;
	do {
		head = (int*) calloc(_Lsize,sizeof(int)); // Speicher allokieren und 0 zuweisen
		if(head == NULL) {
			cnt++;
			_memoryOk = false;
			_Lsize = int(round(Lsize*(1-cnt/float(10)))); // um 10% der ursprüglichen Pufferlänge reduzieren; Das kann theoretisch einen Überlauf verursachen. Auf dem Arduino allerdings erstmal wohl eher nicht.
			if(_Lsize<=0)
				_Lsize=1; // _LSize soll mindestens 1 Element anfordern
		} else {
			_memoryOk = true;
		}
	} while (!_memoryOk && (_Lsize>= (0.1*Lsize)));  //mindestens 10% der geforderten Elemente sollen geliefert werden.
	return head;
}

void RingBuffer::skipFwd( int **pointer)
{
	if (*pointer == end) {
		*pointer = head;
	} else {
		(*pointer)++;
	}
}

void RingBuffer::skipBwd( int **pointer)
{
    if (*pointer == head) {
		*pointer = end;
	} else {
		(*pointer)--;
	}
}

bool RingBuffer::checkMemory()
{
	return _memoryOk;
}

int RingBuffer::getBuffSize()
{
	return _Lsize;
}

// Adds Value M to the next position in the ringbuffer
void RingBuffer::addValue(const int *M)
{
	RingBuffer::skipFwd(&_write);
	*_write= *M; //Kopie des Wertes vom übergebenen Zeiger im Speicher ablegen
	if (_newAvailable)
	{
		int *readNext = _read;
		RingBuffer::skipFwd(&readNext);
		if ((_write == readNext)) { //Wenn der Schreibzeiger den Lesezeiger überholt, während gleichzeitig neue Werte im Puffer sind, dann wird der Puffer überschrieben
		   _read = _write; // Deshalb Lesezeiger mitnehmen, sodass als nächstes der älteste Wert im Puffer gelesen wird.
		}
		return;
	}
	_newAvailable = true;
#ifdef DEBUG
	Serial.print("Wert hinzugefugt:"); Serial.println(_write);
#endif
}

bool RingBuffer::getNewValue( int* retValue)
{
	if (_newAvailable){
		RingBuffer::skipFwd(&_read);
		*retValue = *_read; // Wir müssen den Wert von *_read in den Speicher von *retValue kopieren
		//if (_read == _write) { // Ist das der letzte gültige Wert?
		if (_read == _write) { // Ist das der letzte gültige Wert?
			_newAvailable = false;
		}
		return true; //Neuer Wert ist gültig
	} else {
		return false; // Der übergebene Wert wurde gehalten, d.h. er wurde bei der letzten Abfrage schonmal gelesen.
	}
}


/*
   Funktion liefert das nächste Element und verschiebt dazu den freien Lesezeiger (_readFree)
   Die Funktion setzt das Einlesen fort, sobald sie am Ende angelangt ist.
*/
int* RingBuffer::getNextValue()
{
	RingBuffer::skipFwd(&_readFree);
	#ifdef DEBUG
		Serial.print("Lese nächsten Wert:"); Serial.println(*_readFree);
	#endif
	return _readFree;
}

/*
Liefert den Zeiger readfree zurück und verschiebt ihn anschließend eine Position nach Hinten im Puffer.
*/
int* RingBuffer::getPrevValue()
{
	int* temp = _readFree;
	RingBuffer::skipBwd(&_readFree);
	return temp;
}
/*
  Die Funktion kopiert den Puffer in einen neuen Speicherbereich und gibt einen Zeiger auf den Start zurück.
  Wenn nicht genug Speicher frei ist, gibt sie null zurück.
  Achtung, der Speicher muss mit free wieder vom Hauptprogramm freigegeben werden, wenn er nicht mehr verwendet wird.
*/
//#define DEBUG;
int* RingBuffer::getBuffer()
{
	void *Values;
	Values=malloc(_Lsize*sizeof(int)); // Speicher allokieren
	#ifdef DEBUG
	Serial.print((int) Values);
	Serial.println(" <-Values Zeiger nach initalisierung");
	#endif
	if (Values)
	{
		#ifdef DEBUG
		Serial.print("Kopiere  ");
		Serial.print((end-write)*sizeof(int));
		Serial.println(" bytes");
		#endif

		int* writeNext = _write;
		RingBuffer::skipFwd(&writeNext); 		// Copy write+1 (via skipfwd prüfen)
		memcpy(Values, writeNext, (end-_write)*sizeof(int)); // Von write+1 bis ende kopieren


		int *Values_part=(int*) Values;
		Values_part+=(end-_write); // Speicher für darauf folgende Werte finden und 2 byte weiter den Zeiger setzen, damit nichts überschrieben wird

		#ifdef DEBUG
		Serial.print((int)Values_part);
		Serial.println(" <-Values_part Zeiger vor 2. Kopieren");
		Serial.print("Kopiere  ");
		Serial.print((*_ptrwrite-head+1)*sizeof(int));
		Serial.println(" bytes");
		#endif

		memcpy(Values_part, head, (_write-head+1)*sizeof(int)); // Von head bis write
	}

	return (int*)Values; // Im Fehlerfall ist dies NULL
}

void RingBuffer::clearBuffer(int StartValue)
{
	// Todo: Optimieren mit memset...
	for(int* idx=head; idx<=end; idx++) {
		*idx=StartValue;
	}
	RingBuffer::resetBuffer();

}

void RingBuffer::resetBuffer()
{
	_read = _write; //Lesezeiger auf den Schreibzeiger setzen.
	_newAvailable = false;
}

void RingBuffer::resetFReadPointer()
{
	_readFree = _write; //Lesezeiger auf den Schreibzeiger setzen.
}

int* RingBuffer::movePointer(int* refpointer, int offset)
{
	int move = offset;
	int toEnd = end-refpointer;
	int toHead = head-refpointer;
	// Prüfen, ob die Nahtstelle (Sprung vom Anfang zum Ende des Speichers) überschritten wird
	if (abs(move)>_Lsize){
		move=0; //Einmal herum ist der Anschlag und das ist der Ausgangspunkt
	}
	if (move>toEnd){//Vorwärts, aber über das Ende hinaus
		move-=_Lsize;//=> Zeiger rückwärts bewegen
	}
	if (move<toHead){//Rückwärts vom Lesezeiger
		move+=_Lsize;//=> Zeiger rückwärts bewegen
	}
	return refpointer+move; //Lesezeiger verschieben

};

void RingBuffer::setFReadPointerToRead(int offset)
{
	_readFree = RingBuffer::movePointer(_read, offset);
}

// Todo: Funktion optimieren, so viele Variablen brauchen wir hier eigentluch nicht initalisieren
void RingBuffer::moveFReadPointer(int offset)
{
	_readFree = RingBuffer::movePointer(_readFree, offset);
}

int *MultiRing::stathead=NULL;
int *MultiRing::statend=NULL;
int *MultiRing::statwrite=NULL;
MultiRing *MultiRing::first_obj=NULL;
bool MultiRing::_newAvailable=false;
// Constrctor für MultiRing, der prüft ob schon mal ein MultiRing Objekt erzeugt wurde.
MultiRing::MultiRing(int Lsize, int StartValue) : RingBuffer()
{
#ifdef DEBUG
	Serial.println("Init Liste ->");
#endif
	if (MultiRing::stathead == NULL)
	{
		stathead=RingBuffer::getMemory(Lsize);
		if (!RingBuffer::_memoryOk)
			return; // Fehler
		statend = stathead+_Lsize-1;
		//statwrite = statend;   // _write Zeiger setzen
		//RingBuffer::_ptrwrite = &statwrite;
		RingBuffer::clearBuffer(StartValue); //führt auch ResetBuffer aus und setzt damit _read=_write
	}
    next_obj=NULL;

    if (first_obj == NULL) {
        first_obj = this;
    } else {
        _Lsize = first_obj->_Lsize;
        MultiRing *obj;
        //obj =  this;
        obj= first_obj;
        //*this._Lsize = *first._LSize;
        while (! (obj->next_obj == NULL) ) {
            obj = obj->next_obj;
        }
        obj->next_obj = this;

    }

	//RingBuffer::_ptrwrite = &statwrite; // Muss immer ausgeführt werden
    RingBuffer::_write=statend;
    //RingBuffer::_ptrwrite = &_write;  // Test für next obj Variante
	RingBuffer::head = stathead;
	RingBuffer::end = statend;
	//RingBuffer::_write = _write = statend;   // _write Zeiger setzen


	RingBuffer::resetBuffer();
	RingBuffer::resetFReadPointer();
#ifdef DEBUG
	Serial.println("<- Init Liste");
#endif
}

// Adds Value M to the next position in the ringbuffer

void MultiRing::addValue(int *M)
{
	first_obj->RingBuffer::addValue(M);
	//RingBuffer::addValue(M);
    MultiRing *obj;
    obj =  first_obj;
    while (! (obj->next_obj == NULL) ) {
          obj = obj->next_obj;
          obj->RingBuffer::_newAvailable = first_obj->RingBuffer::_newAvailable;
          obj->_write = first_obj->_write;

    }
	_newAvailable = first_obj->RingBuffer::_newAvailable;
}


bool MultiRing::getNewValue(int* retValue)
{
	RingBuffer::_newAvailable = _newAvailable;
	return RingBuffer::getNewValue(retValue);
}


// Constructor for MovingAverage
// Moving Average überläd die Klasse Ringpuffer.
MovingAverage::MovingAverage(int Lsize, int StartValue) : RingBuffer(Lsize, StartValue)
{
	_sum=_Lsize*StartValue;
}

// Neuer Wert (M) in den Puffer schreiben, ältesten Wert aus Summe entfernen und neuen hinzuaddieren, Durchschnitt bilden.
float MovingAverage::filterMA(int *M)
{
    MovingAverage::addValue(M);
#ifdef DEBUG
	Serial.print("Mittelwert:"); Serial.println(_sum/_Lsize);
#endif
	return _sum / _Lsize;
}
void MovingAverage::addValue(int *M)
{
    _sum -= *MovingAverage::getNextValue(); // Der Lesezeiger wandert immer eins vor dem Schreibzeiger her => ältester Wert im Puffer
	_sum += *M;
	_newAvailable=false; // ist das dadurch schneller?
    RingBuffer::addValue(M);
}

/* IIR Filter
   1. Zunächst müssen hier Arrays (fparams, bparams)
   übergeben werden deren Länge  nicht feststeht => Pointer
   2. Dann müssen zwei Ringpufferobjekte erstellt werden, einen der die Eingangswerte aufnimmt
   und einen der die Ausgangswerte aufnimmt.
   3. Berechnung des Ausgangswertes:
   y(i) = filter * u(i); mit filter = FWD(i)-BWD(i)
   FWD(i) = p0*u_i + p1*u_i-1 + p2*ui-2 + ... + pn*ui-n+1
   BWD(i) =          r1*y_i-1 + r2*yi-2 + ... + rn*yi-n+1
   mit
   fparams[] = {pn, pn-1, ..., p1, p0}; mit n = ordnung;
   fparams[] = {rn, rn-1, ..., r1}; mit n = ordnung-1;
   =>
   FWD(i) = SUM_k=0:n(fparams(k)*u_i-k); BWD(i) = SUM_l=0:n(bparams(l)*y_i-l);
   4. Initialisieren entfällt wegen der Rückwärtswerte
   5. FIR Filter haben keine Rückwärtsparameter (bparams = 0), in diesem Fall wird nur der
   Vorwärtspuffer erstellt und die Rückwärtsberechnung (DEN=1) übersprungen.

   Todo 27.7.2014:
   1. filterIIR mit passenden Parametern testen
   2. bei Punkt 5 weiter machen
   3. Auf float umstellen, wie genau muss das Ergebnis sein und lohnt sich das auf einem Miktokontroller?
   */


//Constructor for IIR
IIR::IIR(int order, int fparams[], int bparams[], int StartValue) :
_forward(order,StartValue), _backward(order-1,StartValue)
{
  //Filterparameter in Attributen speichern
  //order gibt die Anzahl der Elemente von fparams,
  //fparams enthält 1 element mehr, als bparams
  _order = order-1;

  // Todo: Die Schleifen lässt sich direkt mit dem Zeiger realisieren, dadurch braucht man weniger zyklen.
  for(int idx = 0; idx <= _order; idx++) {
	_fparams[idx] = *(fparams+idx);
  }
  for(int idx = 0; idx < _order; idx++) {
	_bparams[idx] = *(bparams+idx); //
  }
}

int IIR::filterIIR(int* newValue)
{
	int _fwd=0, _bwd=0, _out;
	//Neuer Eingangswert in den Vorwärtspuffer
	_forward.addValue(newValue);
	_forward.resetFReadPointer(); //Lesezeiger ausrichten;
	_backward.resetFReadPointer(); //Lesezeiger ausrichten;
	//Bruch berechnen

	// Todo: Die Schleifen lässt sich direkt mit dem Zeiger realisieren, dadurch braucht man weniger zyklen.
	for(int idx = 0; idx <= _order; idx++) {
		_fwd += *_forward.getNextValue() * _fparams[idx];
	//	Serial.print("F{"); Serial.print(_fwd); Serial.print("}");
	}
	for(int idx = 0; idx < _order; idx++) {
		_bwd +=  *_backward.getNextValue() * _bparams[idx];
	//	Serial.print("B{"); Serial.print(_bwd); Serial.print("}");
	}
	_out = _fwd-_bwd;
	#ifdef DEBUG // Ausgabe der Parameter
		Serial.println(".");
		Serial.print("Forward Params:");
		for(int idx = 0; idx <= _order; idx++) {
		Serial.print(_fparams[idx]);Serial.print(", ");
		}
		Serial.print("Values:");
		for(int idx = 0; idx <= _order; idx++) {
		Serial.print(*_forward.getNextValue());Serial.print(", ");
		}
		Serial.print("Sum: "); Serial.print(_fwd); Serial.println(".");
		Serial.print("Backward Params:");
		for(int idx = 0; idx < _order; idx++) {
		Serial.print(_bparams[idx]);Serial.print(", ");
		}
		Serial.print("Values:");
		for(int idx = 0; idx < _order; idx++) {
		Serial.print(*_backward.getNextValue());Serial.print(", ");
		}
		Serial.print("Sum: "); Serial.print(_bwd); Serial.println(".");
		Serial.print("Out: "); Serial.print(_out); Serial.println(".");
	#endif
	//Neuer Ausgangswert in den Rückwärtspuffer
	_backward.addValue(&_out);
	_backward._read = _backward._write; //Lesezeiger ausrichten;
	return _out;
}

