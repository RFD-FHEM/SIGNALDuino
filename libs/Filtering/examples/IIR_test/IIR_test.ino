#include <filtering.h>
/*
Einfaches Testprogramm um den IIR-Filter der 
filtering-Klasse zu testen.
Das Programm zeigt aber nur grundsätzlich die einzelnen Schritte
und das Verhalten mit realen Parametern muss noch getestet werden.
*/
int ordnung =3;
int fparams[] = {0,4,3}, bparams[] = {2,1};

IIR fltr(ordnung, fparams, bparams, 0);

void setup() {
  Serial.begin(9600);
  Serial.println("...");
  Serial.print("Fparams: ");
  //Parameter übergeben
  // fparams ausgeben
  for(int idx = 0; idx < ordnung; idx++) {
    Serial.print(fltr._fparams[idx]);
    Serial.print(";");
  }
    Serial.print(" Bparams: ");
  // bparams ausgeben
  for(int idx = 0; idx < ordnung; idx++) {
    Serial.print(fltr._bparams[idx]);
    Serial.print(";");
  }
  Serial.println(".");
  Serial.println("--------------------");
  Serial.println("Test des IIR-Filters");
  for (int cnt=1; cnt<10; cnt++) {
    Serial.print("Eingangswert: ");   
    int neuWert = cnt;
    Serial.print(neuWert);
    Serial.print(" => Ausangswert: ");   
    Serial.println(fltr.filterIIR(&neuWert));
  };
}

void loop() {
}
