/*
  Bespiel zum Testen der Funktion Ringpuffer der Filterbibliothek
  Norman Butzek, 2014
*/
#include <filtering.h>
const int flaenge = 10;
int Werte[flaenge];
RingBuffer WertePuffer(flaenge, 0); // Erzeuge ein Objekt Ringpuffer

void lesePuffer(RingBuffer *Puffer)
{
  // Auslesen der Werte im Ringpuffer
//  int aktPuffer[flaenge];
//  Puffer->getBuffer(aktPuffer);
  Puffer->resetFReadPointer();
  for (int idx = 0; idx < flaenge; idx++) {   
//    Serial.print(aktPuffer[idx]); Serial.print(". ");
    Serial.print(*(Puffer->getNextValue())); Serial.print(". ");
  }
  Serial.println(".");
}

void setup()
{
  randomSeed(analogRead(0));
  Serial.begin(115200);
  /* ====================================================================================================
    Werte in den Ringpuffer schreiben und beobachten, wie sich die ZAhlen an dem "Fenster" vorbei bewegen
  */
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Werte Schreiben und den Pufferinhalt anzeigen");
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Der Ringpuffer ist mit Nullen initialisiert und aufsteigend beschrieben und bei jedem Schritt komplett angezeigt.");
  Serial.println("Der Puffer stellt damit ein Fenster mit definierter Breite auf die letzten Werte einer Folge dar, ");
  Serial.println("an dessen linkem Ende sich der aelteste Wert und am rechten Ende der juengste Wert befindet");
  Serial.println("RingBuffer.clearBuffer(0) Fuellt den Puffer mit Nullen.");
  Serial.println("RingBuffer.addValue(&Wert) fuegt einen Wert zum Puffer hinzu.");
  Serial.println("RingBuffer.getBuffer(Array) kopiert den Pufferinhalt ab dem Schreibzeiger in das uebergebene Array.");
  Serial.println("----------------------------------------------------------------------------");
  WertePuffer.clearBuffer(0);
  Serial.print("Ausgangszustand des Puffers:   ");
  lesePuffer(&WertePuffer);
  for (int cnt = 1; cnt < 2 * flaenge; cnt += 1)
  {
    // Schreiben eines Messwertes in den Ringpuffer
    Serial.print("Fuege Wert hinzu: ");
    Serial.print(cnt);
    WertePuffer.addValue(&cnt);
    Serial.print(" => Puffer: ");
    lesePuffer(&WertePuffer);
  };
  /* ====================================================================================================
    Werte abwechselnd in den Puffer speichern und auslesen. a) ohne ueberlauf, b) mit ueberlauf
  */
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Werte abwechselnd in den Puffer speichern und auslesen.");
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("RingBuffer.getNewValue(&Wert) liest einen neuen Wert, sofern dieser im Puffer vorhanden ist.");
  Serial.println("RingBuffer.resetBuffer() setzt den Lesezeiger auf den Schreibzeiger und erklaert alle Werte im Puffer fuer ungueltig.");
  Serial.println("2. a) Werte werden rechtzeitig ausgelesen, sodass es nicht zum ueberlauf kommt.");
  Serial.println("----------------------------------------------------------------------------");
  int Wert;
  int nWerte;
  WertePuffer.clearBuffer(0);
  WertePuffer.resetBuffer(); 
  Serial.print("Ausgangszustand des Puffers:   ");
  lesePuffer(&WertePuffer);
  int nWertelst[] = {7,2,2,6,4,3};
  for (int loopCnt =0;loopCnt < 5; loopCnt +=1) {    
    //Werte schreiben
    nWerte = nWertelst[loopCnt];
    for (int cnt = 1; cnt <= nWerte; cnt += 1) {
      Serial.print("Fuege Wert hinzu: ");
      Serial.print(cnt);
      WertePuffer.addValue(&cnt);
      Serial.print(" => Puffer: ");
      lesePuffer(&WertePuffer);
    };
    //Werte lesen
    Serial.print("Lese alle (neuen) Werte im Puffer: ");
    while (WertePuffer.getNewValue(&Wert)) {
      Serial.print(Wert);
      Serial.print(". ");
    }
    Serial.println(" ");
  }
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Werte werden schneller geschrieben als ausgelesen, sodass es zum Pufferueberlauf und damit zum ueberschreiben von Werten kommt.");
  Serial.println("----------------------------------------------------------------------------");
  WertePuffer.clearBuffer(0);
  Serial.print("Ausgangszustand des Puffers:   ");
  lesePuffer(&WertePuffer);
  WertePuffer.dump();
  const int defnWerte[5] = {7,14,7,14,7};
  for (int loopCnt =0;loopCnt < 5; loopCnt +=1) {    
    nWerte = defnWerte[loopCnt];
    //Werte schreiben
    for (int cnt = 1; cnt <= nWerte; cnt += 1) {
      Serial.print("Fuege Wert hinzu: ");
      Serial.print(cnt);
      WertePuffer.addValue(&cnt);
      Serial.print(" => Puffer: ");
      lesePuffer(&WertePuffer);
      // WertePuffer.dump();
    };
    //Werte lesen
    Serial.print("Lese alle (neuen) Werte im Puffer: ");
    while (WertePuffer.getNewValue(&Wert)) {
      Serial.print(Wert);
      Serial.print(". ");
    }
    Serial.println(" ");
  }
  /* =========  Schritt 3  =========
    Haltefunktion des Puffers  
  */
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Der Puffer kann gleichzeitig als Halteglied verwendet werden. Wenn der Rueckgabewert von Funktion 4 ignoriert wird,");
  Serial.println("liefert die Funktion den letzten gueltigen Wert zurueck.");
  Serial.println("Funktion6 RingBuffer.getNewValue(&Wert)");
  Serial.println("----------------------------------------------------------------------------");
  WertePuffer.clearBuffer(0);
  Serial.print("Ausgangszustand des Puffers:   ");
  lesePuffer(&WertePuffer);
  nWerte = 4;
  for (int cnt = 1; cnt <= nWerte; cnt += 1) {
    Serial.print("Fuege Wert hinzu: ");
    Serial.print(cnt);
    WertePuffer.addValue(&cnt);
    Serial.print(" => Puffer: ");
    lesePuffer(&WertePuffer);
  };
  //Werte lesen
  Serial.print("Lese Werte aus dem Puffer: ");
  for (int cnt = 1; cnt <= nWerte+11; cnt += 1) {
    WertePuffer.getNewValue(&Wert);
    Serial.print(Wert);
    Serial.print(". ");
  }
  Serial.println(" ");
  /* ====================================================================================================
    Synchrones Auslesen des ganzen Puffers
  */
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Der Puffer dient auch als Wertespeicher innerhalb von Filterfunktionen.");
  Serial.println("Dabei werden in der Regel Werte innerhalb eines determinierten Ablaufs in den Puffer geschrieben und anschliessend ");
  Serial.println("gezielt aus dem Puffer gelesen und unmittelbar verarbeitet.");
  Serial.println("Um Rechenzeit zu sparen werden in diesem Fall keine Kopien der Daten beim Lesen erzeugt und die Verwaltung");
  Serial.println("der Werte im Puffer liegt in der Hand des Nutzers, bzw. der aufrufenden Funktion.");
  Serial.println("Im Falle der Auswertung des Filters ist es haeufig so, dass je Programmzyklus genau ein Messwert je Messignal erfasst ");
  Serial.println("wird und innerhalb des Filters mit einer definierten Anzahl an vorhergehenden Werten verrechnet wird.");
  Serial.println("RingBuffer.getNextValue(&Wert) liest dann den naechsten Wert, RingBuffer.getprevValue(&Wert) liest den vorangehenden Wert,");
  Serial.println("ausgehend von einem separaten (freien) Lesezeiger, der mit RingBuffer.resetFReadPointer() auf den Schreibzeiger");
  Serial.println("und mit setFReadPointerToRead() auf den Lesezeiger ausgerichtet wird.");
  Serial.println("Das Beispiel schreibt 5 Einsen in den Puffer. Anschliessend werden alle Werte im Puffer gelesen und der Mittelwert berechnet.");
  Serial.println("Im folgenden Beispiel werden nur die 5 letzten Werte gelesen und daraus der Mittelwert berechnet.");
  Serial.println("----------------------------------------------------------------------------");
  WertePuffer.clearBuffer(0);
  Serial.print("Ausgangszustand des Puffers:   ");
  lesePuffer(&WertePuffer);
  nWerte = 5;
  for (int cnt = 1; cnt <= nWerte; cnt += 1) {
    Serial.print("Fuege Wert hinzu: ");
    Serial.print(cnt);
    WertePuffer.addValue(&cnt);
    Serial.print(" => Puffer: ");
    lesePuffer(&WertePuffer);
  };
  //Alle Werte im Puffer lesen
  float average = 0;
  WertePuffer.resetFReadPointer();
  Serial.print("Lese Werte aus dem Puffer: ");
  for (int cnt = 1; cnt <= flaenge; cnt += 1) {
    Wert = *WertePuffer.getNextValue();
    average += Wert;
    Serial.print(Wert);
    Serial.print(". ");
  }
  Serial.print(" Mittelwert aus allen Werten im Puffer: ");
  Serial.print(average/flaenge);
  Serial.println(".");

  //Die letzten 5 Werte im Puffer (rueckwaerts) lesen
  average = 0;
  WertePuffer.resetFReadPointer();
  Serial.print("Lese Werte aus dem Puffer: ");
  for (int cnt = 1; cnt <= 5; cnt += 1) {
    Wert = *WertePuffer.getPrevValue();
    average += Wert;
    Serial.print(Wert);
    Serial.print(". ");
  }
  Serial.print(" Mittelwert aus den letzten 5 Werten: ");
  Serial.print(average/5);
  Serial.println(".");
  /* ====================================================================================================
    Eine bestimmte Anzahl von Werten ab einem defineirten Punkt auslesen.
  */
  Serial.println("----------------------------------------------------------------------------");
  Serial.println("Im vorigen Beispiel war die Reihenfolge, in der die Werte aus dem Puffer geselen wurden unerheblich.");
  Serial.println("Falls das nicht der Fall ist, so kann der Lesezeiger durch die ueberhabe eines offsets mit RingBuffer.setFReadPointerToRead(offset)");
  Serial.println("gezielt ausgerichtet werden um von dort aus in der gewuenschten Richtung die gewuenschte Anzahl von Werten lesen zu koennen.");
  WertePuffer.resetBuffer();
//  WertePuffer.setFReadPointerToRead();
  Serial.println();
//  WertePuffer.dump();
  WertePuffer.setFReadPointerToRead(-4); // Lesezeiger 4 werte vor dem Schreibzeiger ausrichten
  Serial.println();
//  WertePuffer.dump();
  Serial.print("Lese 3 Werte aus dem Puffer beginnend mit dem 4. juengsten Wert: ");
  for (int cnt = 1; cnt <= 7; cnt += 1) {
    Wert = *WertePuffer.getNextValue();
    Serial.print(Wert);
    Serial.print(". ");
  }
  Serial.println("Ende.");  
}
  
  void loop()
  {
  }
