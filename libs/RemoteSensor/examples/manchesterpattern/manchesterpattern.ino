/*
*   Example Sketch for testing the patternDecoder lib and some sublibs
*   Copyright (C) 2014  .Butzek
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


#define PROGNAME               "example-patter-manchester"
#define PROGVERS               "0.2"

#define BAUDRATE               9600
#include <patternDecoder.h>
#include <bitstore.h>
//Decoder
ManchesterpatternDetector ManchesterDetect(true);
patternDetector rtzDetect;

OSV2Decoder osv2Dec (&ManchesterDetect);
ASDecoder   asDec  (&ManchesterDetect);

int sample_OSV2_data[] = {852, -1112, 356, -624, 840, -632, -1116, 840, -1104, 848, -1112, 352, -628, 836, -1124, 828, -644, -1112, 840, -1124, 832, -1116, -624, 840, -1120, 832, -1120, 836, -636, -1124, 832, -1116, -628, 840, -624, 352, -1124, 832, -1120, 836, -1116, 840, -1108, 844, -1104, 852, -1104, 364, -608, 856, -1100, 852, -1108, 848, -624, 352, -1092, 376, -604, 860, -624, 356, -1108, 356, -608, 860, -624, 352, -1092, 376, -604, 860, -604, 372, -1096, 368, -604, 864, -608, 368, -1104, 852, -1092, 372, -612, 852, -608, 368, -1092, 376, -612, 852, -3568, 864, -1080, 872, -1080, 876, -1088, 864, -1088, 868, -1092, 856, -1096, 860, -1084, 872, -1084, 868, -1100, 856, -1084, 868, -1096, 860, -1084, 868, -1080, 872, -1092, 864, -1088, 864, -1092, 864, -608, 368, -1088, 380, -592, 872, -600, 376, -1080, 388, -600, 860, -1092, 864, -600, 376, -1092, 372, -604, 864, -1088, 868, -600, 372, -1092, 864, -1084, 380, -600, 868, -1084, 868, -1088, 868, -596, 380, -1092, 376, -596, 868, -1088, 868, -1088, 864, -1092, 860, -620, 356, -1088, 868, -1088, 380, -604, 860, -608, 368, -1104, 360, -600, 868, -596, 384, -1084, 864, -1100, 364, -608, 860, -1092, 860, -1096, 860, -1096, 856, -600, 380, -1088, 860, -1092, 376, -616, 852, -612, 364, -1084, 872, -1084, 868, -1088, 376, -624, 844, -1088, 868, -596, 376, -1088, 868, -1084, 868, -1084, 384, -608, 856, -1096, 856, -1092, 864, -604, 372, -1088, 868, -1076, 392, -596, 864, -592, 388, -1084, 868, -1092, 860, -1092, 864, -1096, 860, -1092, 860, -1096, 368, -596, 868, -1088, 868, -1084, 868, -608, 372, -1092, 372, -600, 868, -604, 372, -1088, 376, -600, 864, -612, 368, -1088, 376, -608, 856, -600, 376, -1088, 380, -604, 864, -604, 372, -1080, 872, -1096, 372, -596, 868, -600, 376, -1084, 380, -604, 860, -32001, -540, -15084, -4280, -1824, -13332, -9112, -1880, -2448, -2552, -8176, -1552, -8304, -11572, -1028, -11124, -424, -8744, -524, -2728};
int sample_rtz_data[] = {512, -3880, 512, -1924, 512, -9200, 576, -1920, 516, -3880, 512, -1924, 512, -3880, 516, -1936, 512, -3880, 512, -3880, 516, -3876, 516, -3880, 512, -1924, 512, -1920, 528, -3880, 516, -3876, 516, -3880, 512, -1924, 512, -3880, 512, -1936, 512, -1928, 508, -1924, 524, -1924, 508, -3884, 512, -3880, 512, -3880, 512, -1940, 512, -1920, 512, -1924, 524, -3880, 512, -3880, 516, -1920, 512, -3880, 512, -1936, 512, -1924, 512, -1936, 512, -1924, 508, -3880, 512, -1940, 508, -9184, 576, -1936, 512, -3876, 516, -1920, 512, -3880, 528, -1920, 516, -3876, 512, -3880, 512, -3880, 528, -3876, 516, -1920, 512, -1924, 524, -3880, 512, -3880, 512, -3880, 512, -1920, 528, -3880, 512, -1920, 512, -1940, 508, -1924, 508, -1940, 508, -3884, 508, -3880, 512, -3880, 512, -1936, 512, -1924, 508, -1940, 508, -3880, 512, -3880, 512, -1936, 512, -3880, 512, -1920, 512, -1936, 512, -1924, 508, -1940, 508, -3880};

// AS Data
//SB:10110011
//B1:00000110  =0x06
//B2:10000000  =0x80
//B3:11001000  =0xc8
//B4:10000000  =0x80
//                                        | Start                                                                                                                                                                                                                                                                                                                                                                                         |Ends
int sample_AS_data[] = {-764, -524, -536, 1776, -1508, 956, -684, 1712, -784, 844, -1568, 924, -720, 1712, -808, 828, -812, 816, -820, 812, -824, 808, -1608, 880, -756, 1652, -1620, 1656, -832, 796, -844, 788, -844, 784, -844, 792, -844, 788, -844, 788, -1632, 864, -772, 1636, -856, 780, -1628, 1636, -856, 772, -860, 776, -1636, 1632, -864, 772, -860, 772, -864, 772, -860, 768, -860, 776, -860, 776, -860, 772, -780, 776, -3908, 1640, -1628, 864, -768, 1640, -852, 784, -1624, 864, -772, 1676, -848, 788, -844, 792, -844, 784, -848, 788, -1620, 868, -772, 1636, -1632, 1644, -848, 780, -852, 784, -852, 780, -848, 784, -852, 784, -844, 788, -1632, 864, -768, 1640, -852, 784, -1624, 1640, -852, 776, -852, 788, -1636, 1632, -852, 784, -852, 776, -856, 780, -852, 784, -848, 784, -852, 780, -856, 776, -776, 780};

void detect_mc()
{
  static bool state;
  uint16_t len = sizeof(sample_AS_data)/sizeof(sample_AS_data[0]);
  for (uint16_t i=0;i<=len;++i)
  {
      //Serial.print("#");Serial.println(i);
      state = ManchesterDetect.detect(&sample_AS_data[i]); //OSV2
      delay(400);
      if (state)
      {
        Serial.println("Mc Message dekodiert");
        ManchesterDetect.reset();
        delay(1000);
      }
  }
  if (!state)
  {
    Serial.println("Message nicht dekodiert");
    delay(1000);
   }

}

void decode_mc()
{
  static bool state;
  uint16_t len = sizeof(sample_AS_data)/sizeof(sample_AS_data[0]);
  for (uint16_t i=0;i<=len;++i)
  {
      //Serial.print("#");Serial.println(i);
      //delay(1);
      if (ManchesterDetect.detect(&sample_AS_data[i]))
      {
        Serial.println("Manchester erkannt");
        state = osv2Dec.decode(); //OSV2 Decoder
        if (state)
        {
            Serial.println("OSV2 Message dekodiert");
            //ManchesterDetect.reset();
            delay(1000);
        }
        state = asDec.decode(); //OSV2 Decoder
        if (state)
        {
            Serial.println("AS Message dekodiert");
            //ManchesterDetect.reset();
            delay(1000);
        }
        ManchesterDetect.reset();

      }

      //delay(100);

/*
      state = asDec.decode(&sample_OSV2_data[i]); // AS Decoder
      //delay(100);
      if (state)
      {
        Serial.println("AS Message dekodiert");
        ManchesterDetect.reset();
        delay(1000);
      }
*/
  }
  if (!state)
  {
    //Serial.println("Message nicht dekodiert");
    delay(1000);
   }

}



void detect_rtz()
{
  static bool state;
  uint16_t len = sizeof(sample_rtz_data)/sizeof(sample_rtz_data[0]);
  for (uint16_t i=0;i<=len;++i)
  {
      //Serial.print("#");Serial.println(i);
      state = rtzDetect.detect(&sample_rtz_data[i]); //OSV2
      //delay(100);
      if (state)
      {
        Serial.println("RTZ Message dekodiert");
        rtzDetect.reset();
        delay(1000);
      }
  }
  if (!state)
  {
    Serial.println("Message nicht dekodiert");
    delay(1000);
   }

}



uint8_t msg=0;



void setup() {
  Serial.begin(BAUDRATE);
  delay(2000);
  Serial.println("Startup:");
/*
  Serial.println("Bittest, storing for numbers in one byte (8 bits):");
  BitStore store(1);
  for (uint8_t idx=0;idx<10;idx++){
    store.addValue((idx%2)==0); // Adds Value 0-9
    //Serial.print("Adding:");Serial.println((idx%2)==0);
  }
  //Serial.println(msg,BIN);
  Serial.println("Store entry:");
  Serial.print(store.datastore[0]);Serial.print(store.datastore[1]);Serial.println("");
  Serial.println("Read stored numbers fom byte: ");
  for (uint8_t idx=0;idx<10;idx++){
    Serial.println(store.getValue(idx),DEC); // Prints pos 0-9
  }
  Serial.print("Read one byte complete: ");
  Serial.println(store.getByte(0));
  return;

*/
  Serial.print("Len Input data (Manchester): ");
  //Serial.println(sizeof(sample_OSV2_data)/sizeof(sample_OSV2_data[0]));
  Serial.println(sizeof(sample_AS_data)/sizeof(sample_AS_data[0]));
  //detect_mc();
  decode_mc();

/*
  Serial.print("Len Input data (RTZ): ");
  Serial.println(sizeof(sample_rtz_data)/sizeof(sample_rtz_data[0]));
  detect_rtz();
*/
//  randomize_signal();
//  detect();

}

void loop() {
  //}
  delay(1);
}
