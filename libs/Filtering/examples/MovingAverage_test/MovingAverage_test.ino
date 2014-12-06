/*
Beispiel für Moving Average Filter
Das Filter wird mit Werten, die zufällig um 100 schwanken beaufschlagt.
Anfangswert für das Beispiel ist ein geleertes Filter, d.h. während der ersten Werte 
zeichnet sich ein einschwingverhalten ab, während die Letzten Werte den Glättungseffekt wiederspiegeln.
Norman Butzek, 2014
*/
#include <filtering.h>
const int flaenge = 10;
int Werte[flaenge];
MovingAverage MAPuffer(flaenge,00);  // Erzeuge ein Objekt Ringpuffer

void lesePuffer(MovingAverage *Puffer)
{
  // Auslesen der Werte im Ringpuffer
  int *aktPuffer=NULL;
  aktPuffer=  Puffer->getBuffer();
  for (int idx = 0; idx < flaenge; idx++) {   
    Serial.print(aktPuffer[idx]); Serial.print(", ");
  }
  Serial.println(".");
}

void setup()
{
  Serial.begin(9600);
  randomSeed(analogRead(0)); 
  Serial.println("Test des MovingAverage Filters");
  Serial.println("Das Programm testet das moving average Filter.");
  Serial.println("Der Puffer ist zu Beginn mit Nullen initialisiert und wird mit Werten die um +-10 um die Zahl 100 schwanken");
  Serial.println("beschrieben. Waehrend der ersten Werte, ist deutlich der Einschwingvorgang des Filters zu erkennen,");
  Serial.println("was daran liegt, dass zu Beginn das Ergebnis noch stark von den Nullen im Filter dominiert wird, und");
  Serial.println("diese erst allmählich aus dem Puffer verschwinden. Sobald alle Werte des Filters beschrieben wurden");
  Serial.println("zeigt sich deutlich die glaettende Wirkung. Da diese von der Filterlänge abhängt, lohnt es sich damit etwas herumzuspielen.");
  Serial.println("--------------------");  
  for (int cnt=1; cnt<2*flaenge; cnt+=1) {
    Serial.print("Eingangswert: ");   
    int neuWert = 100+random(-10,11); // Signal+10/zufälliges Rauschen
    Serial.print(neuWert);
    Serial.print(" => Ausgangswert: ");   
    Serial.println(MAPuffer.filterMA(&neuWert));
  // Wenn man es sich genauer anschauen möchte kommentiert man die beiden folgenden Zeilen ein.
  // MAPuffer.dump();
  // lesePuffer(&MAPuffer);
  };
}

void loop()
{
  Serial.print("Eingangswert: ");   
  int neuWert = 100+random(-10,11); // Signal+10/zufälliges Rauschen
  Serial.print(neuWert);
  Serial.print(" => Ausangswert: ");   
  Serial.println(MAPuffer.filterMA(&neuWert));
}


