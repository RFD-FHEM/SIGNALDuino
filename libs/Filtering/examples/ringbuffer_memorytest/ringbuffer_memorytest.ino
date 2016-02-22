/*
  Bespiel zum Testen der Funktion Ringpuffer der Filterbibliothek
  Hier geht es besonders um die Problematik, wenn mehr Speicher angefordert wird, als zur Verfügung steht
  Norman Butzek, 2014
*/
#include <filtering.h>
#define spec_flaenge 20000
RingBuffer WertePuffer(spec_flaenge, 0); // Erzeuge ein Objekt Ringpuffer
static int Wert = 0;

void lesePuffer(RingBuffer *Puffer)
{
  // Auslesen der Werte im Ringpuffer
  Puffer->resetFReadPointer();
  for(int idx=1; idx<=Puffer->getBuffSize(); ++idx) {
    Serial.print(*Puffer->getNextValue()); Serial.print(", ");
  }
  Serial.println(". ");
}

void lesePufferRueckwaerts(RingBuffer *Puffer, int flaenge){
  for(int idx=1; idx<=flaenge; idx++) {
    Serial.print(*Puffer->getPrevValue()); Serial.print(", ");
  }
  Serial.println(". ");
}

void setup()
{

  randomSeed(analogRead(0));
  Serial.begin(115200);
  Serial.println("Start");
  int flaenge = WertePuffer.getBuffSize(); // tatsächliche Puffergröße bestimmen
  bool memoryOK = WertePuffer.checkMemory();
  if (memoryOK) {
    Serial.print("MemOK: ");
  } else {
    Serial.print("MemError, konnte den geforderten Speicher nicht bereitstellen! Ich steige aus...");
    while (true) {}
  }
  Serial.print(flaenge); Serial.print(" von "); Serial.print(spec_flaenge); Serial.println(" geforderten Werten allokiert.");
  Serial.println("---------------------");
  Serial.println(("Ausgangszustand des Puffers:   "));
  lesePuffer(&WertePuffer); // Mit der getBuffer Funktion, die getNextValue verwendet.
  Serial.println(("Puffer rueckwaerts:   "));
  WertePuffer.resetFReadPointer();
  lesePufferRueckwaerts(&WertePuffer, flaenge);
  Serial.println(("Jetzt den Puffer Aufsteigend bis zum letzten Element beschreiben:"));
  for (int cnt = 1; cnt <= flaenge; ++cnt ) {
    WertePuffer.addValue(&cnt);
  }
  Serial.println("Puffer:");
  lesePuffer(&WertePuffer); // Mit der getBuffer Funktion, die getNextValue verwendet.
  Serial.println(F("Loop beginnt, lese Pufferabschnitte rueckwaerts:"));
}
  
  void loop()
  {
    lesePufferRueckwaerts(&WertePuffer, 10+WertePuffer.getBuffSize()); //ohne neu auszurichten
    delay(500);
  }
