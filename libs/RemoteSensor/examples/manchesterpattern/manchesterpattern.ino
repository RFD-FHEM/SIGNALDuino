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
*
*   This Sketch will only run complete on a Arduino Mega or in a Simulator which has enouth ram
*/


#define PROGNAME               "example-patter-manchester"
#define PROGVERS               "0.3"

#define BAUDRATE               57600
#include <patternDecoder.h>
#include <bitstore.h>
//Decoder
ManchesterpatternDetector ManchesterDetect(true);
patternDetector ooDetect;
patternDecoder  ooDecode;

OSV2Decoder osv2Dec (&ManchesterDetect);
ASDecoder   asDec  (&ManchesterDetect);
// OSV2 Data hex : DADC539E18277055                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Detector ends here
int sample_OSV2_data[] = {852, -1112, 356, -624, 840, -632, -1116, 840, -1104, 848, -1112, 352, -628, 836, -1124, 828, -644, -1112, 840, -1124, 832, -1116, -624, 840, -1120, 832, -1120, 836, -636, -1124, 832, -1116, -628, 840, -624, 352, -1124, 832, -1120, 836, -1116, 840, -1108, 844, -1104, 852, -1104, 364, -608, 856, -1100, 852, -1108, 848, -624, 352, -1092, 376, -604, 860, -624, 356, -1108, 356, -608, 860, -624, 352, -1092, 376, -604, 860, -604, 372, -1096, 368, -604, 864, -608, 368, -1104, 852, -1092, 372, -612, 852, -608, 368, -1092, 376, -612, 852, -3568, 864, -1080, 872, -1080, 876, -1088, 864, -1088, 868, -1092, 856, -1096, 860, -1084, 872, -1084, 868, -1100, 856, -1084, 868, -1096, 860, -1084, 868, -1080, 872, -1092, 864, -1088, 864, -1092, 864, -608, 368, -1088, 380, -592, 872, -600, 376, -1080, 388, -600, 860, -1092, 864, -600, 376, -1092, 372, -604, 864, -1088, 868, -600, 372, -1092, 864, -1084, 380, -600, 868, -1084, 868, -1088, 868, -596, 380, -1092, 376, -596, 868, -1088, 868, -1088, 864, -1092, 860, -620, 356, -1088, 868, -1088, 380, -604, 860, -608, 368, -1104, 360, -600, 868, -596, 384, -1084, 864, -1100, 364, -608, 860, -1092, 860, -1096, 860, -1096, 856, -600, 380, -1088, 860, -1092, 376, -616, 852, -612, 364, -1084, 872, -1084, 868, -1088, 376, -624, 844, -1088, 868, -596, 376, -1088, 868, -1084, 868, -1084, 384, -608, 856, -1096, 856, -1092, 864, -604, 372, -1088, 868, -1076, 392, -596, 864, -592, 388, -1084, 868, -1092, 860, -1092, 864, -1096, 860, -1092, 860, -1096, 368, -596, 868, -1088, 868, -1084, 868, -608, 372, -1092, 372, -600, 868, -604, 372, -1088, 376, -600, 864, -612, 368, -1088, 376, -608, 856, -600, 376, -1088, 380, -604, 864, -604, 372, -1080, 872, -1096, 372, -596, 868, -600, 376, -1084, 380, -604, 860, -32001, -540, -15084, -4280, -1824, -13332, -9112, -1880, -2448, -2552, -8176, -1552, -8304, -11572, -1028, -11124, -424, -8744, -524, -2728};

const uint8_t rand_data_size=50;

int random_data[rand_data_size];  // Array for some Random Data

int sample_onoff_data[]=
// Logilink:
 {512, -9200, 576, -1920, 516, -3880, 512, -1924, 512, -3880, 516, -1936, 512, -3880, 512, -3880, 516, -3876, 516, -3880, 512, -1924, 512, -1920, 528, -3880, 516, -3876, 516, -3880, 512, -1924, 512, -3880, 512, -1936, 512, -1928, 508, -1924, 524, -1924, 508, -3884, 512, -3880, 512, -3880, 512, -1940, 512, -1920, 512, -1924, 524, -3880, 512, -3880, 516, -1920, 512, -3880, 512, -1936, 512, -1924, 512, -1936, 512, -1924, 508, -3880, 512, -1940, 508};//, -9184, 576, -1936, 512, -3876, 516, -1920, 512, -3880, 528, -1920, 516, -3876, 512, -3880, 512, -3880, 528, -3876, 516, -1920, 512, -1924, 524, -3880, 512, -3880, 512, -3880, 512, -1920, 528, -3880, 512, -1920, 512, -1940, 508, -1924, 508, -1940, 508, -3884, 508, -3880, 512, -3880, 512, -1936, 512, -1924, 508, -1940, 508, -3880, 512, -3880, 512, -1936, 512, -3880, 512, -1920, 512, -1936, 512, -1924, 508, -1940, 508, -3880};
// ITTX Data:
//{-1012, -3556, -2364, 152, -108, -1448, -5604, -3944, 188, -3304, -768, -1212, -2156, -568, -636, -460, -704, 100, -496, -1324, -1052, 112, -5852, -1484, -716, -300, -2692, -380, 1264, -1012, 1260, -1004, 1256, -1016, 1240, -1032, 468, -1020, 1252, -1020, 488, -1024, 1232, -1032, 1236, -1020, 1236, -1036, 1236, -1036, 1228, -1040, 460, -1028, 1240, -1036, 1232, -1032, 1228, -1040, 1232, -1044, 1220, -1040, 460, -1032, 472, -1036, 1224, -1040, 464, -1044, 1224, -1036, 468, -1048, 1212, -1040, 1224, -1032, 476, -1036, 1232, -1024, 1236, -1044, 456, -1052, 460, -1036, 1228, -1044, 1220, -1040, 460, -1036, 1240, -1024, 468, -1032, 1248, -1024, 1232, -1028, 476, -1028, 1232, -1032, 476, -1036, 1224, -1048, 1216, -1040, 468, 464, -843};
// OSV2 Manchester:
// {852, -1112, 356, -624, 840, -632, -1116, 840, -1104, 848, -1112, 352, -628, 836, -1124, 828, -644, -1112, 840, -1124, 832, -1116, -624, 840, -1120, 832, -1120, 836, -636, -1124, 832, -1116, -628, 840, -624, 352, -1124, 832, -1120, 836, -1116, 840, -1108, 844, -1104, 852, -1104, 364, -608, 856, -1100, 852, -1108, 848, -624, 352, -1092, 376, -604, 860, -624, 356, -1108, 356, -608, 860, -624, 352, -1092, 376, -604, 860, -604, 372, -1096, 368, -1092, 376, -612, 852, -3568, 864, -1080, 872, -1080, 876, -1088, 864, -1088, 868, -1092, 856, -1096, 860, -1084, 872, -1084, 868, -1100, 856, -1084, 868, -1096, 860, -1084, 868, -1080, 872, -1092, 864, -1088, 864, -1092, 864, -608, 368, -1088, 380, -592, 872, -600, 376, -1080, 388, -600, 860, -1092, 864, -600, 376, -1092, 372, -604, 864, -1088, 868, -600, 372, -1092, 864, -1084, 380, -600, 868, -1084, 868, -1088, 868, -596, 380, -1092, 376, -596, 868, -1088, 868, -1088, 864, -1092, 860, -620, 356, -1088, 868, -1088, 380, -604, 860, -608, 368, -1104, 360, -600, 868, -596, 384, -1084, 864, -1100, 364, -608, 860, -1092, 860, -1096, 860, -1096, 856, -600, 380, -1088, 860, -1092, 376, -616, 852, -612, 364, -1084, 872, -1084, 868, -1088, 376, -624, 844, -1088, 868, -596, 376, -1088, 868, -1084, 868, -1084, 384, -608, 856, -1096, 856, -1092, 864, -604, 372, -1088, 868, -1076, 392, -596, 864, -592, 388, -1084, 868, -1092, 860, -1092, 864, -1096, 860, -1092, 860, -1096, 368, -596, 868, -1088, 868, -1084, 868, -608, 372, -1092, 372, -600, 868, -604, 372, -1088, 376, -600, 864, -612, 368, -1088, 376, -608, 856, -600, 376, -1088, 380, -604, 864, -604, 372, -1080, 872, -1096, 372, -596, 868, -600, 376, -1084, 380, -604, 860, -32001, -540, -15084, -4280, -1824, -13332, -9112, -1880, -2448, -2552, -8176, -1552, -8304, -11572, -1028, -11124, -424, -8744, -524, -2728};

// AS Data
//SB:10110011
//B1:00000110  =0x06
//B2:10000000  =0x80
//B3:11001000  =0xc8
//B4:10000000  =0x80
//                                        | Start                                                                                                                                                                                                                                                                                                                                                                                         |Ends
int sample_AS_data[] = {-764, -524, -536, 1776, -1508, 956, -684, 1712, -784, 844, -1568, 924, -720, 1712, -808, 828, -812, 816, -820, 812, -824, 808, -1608, 880, -756, 1652, -1620, 1656, -832, 796, -844, 788, -844, 784, -844, 792, -844, 788, -844, 788, -1632, 864, -772, 1636, -856, 780, -1628, 1636, -856, 772, -860, 776, -1636, 1632, -864, 772, -860, 772, -864, 772, -860, 768, -860, 776, -860, 776, -860, 772, -780, 776, -3908, 1640, -1628, 864, -768, 1640, -852, 784, -1624, 864, -772, 1676, -848, 788, -844, 792, -844, 784, -848, 788, -1620, 868, -772, 1636, -1632, 1644, -848, 780, -852, 784, -852, 780, -848, 784, -852, 784, -844, 788, -1632, 864, -768, 1640, -852, 784, -1624, 1640, -852, 776, -852, 788, -1636, 1632, -852, 784, -852, 776, -856, 780, -852, 784, -848, 784, -852, 780, -856, 776, -776, 780};


int *pulsedata = sample_OSV2_data;
uint16_t lendata = sizeof(sample_OSV2_data)/sizeof(sample_OSV2_data[0]);

uint32_t signalduration =0;

void init_random_data()
{
    for (uint8_t i=0; i<rand_data_size; i++)
    {
        random_data[i] = random(-600,600);
    }

}

class DecodeOOK {
  protected:
    byte total_bits, bits, flip, state, pos, data[25];

    virtual char decode (word width) = 0;

  public:

    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK () {
      resetDecoder();
    }

    bool nextPulse (word width) {
      if (state != DONE)

        switch (decode(width)) {
          case -1: resetDecoder(); break;
          case 1:  done(); break;
        }
      return isDone();
    }

    bool isDone () const {
      return state == DONE;
    }

    const byte* getData (byte& count) const {
      count = pos;
      return data;
    }

    void resetDecoder () {
      total_bits = bits = pos = flip = 0;
      state = UNKNOWN;
    }

    // add one bit to the packet data buffer

    virtual void gotBit (char value) {
      total_bits++;
      byte *ptr = data + pos;
      *ptr = (*ptr >> 1) | (value << 7);

      if (++bits >= 8) {
        bits = 0;
        if (++pos >= sizeof data) {
          resetDecoder();
          return;
        }
      }
      state = OK;
    }

    // store a bit using Manchester encoding
    void manchester (char value) {
      flip ^= value; // manchester code, long pulse flips the bit
      gotBit(flip);
    }




    void done () {
      while (bits)
        gotBit(0); // padding
      state = DONE;
    }
};

class OregonDecoderV2 : public DecodeOOK {
  public:

    OregonDecoderV2() {}

    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
      Serial.print((value ? 0x01 : 00));
      if (!(total_bits & 0x01))
      {
        data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
      }
      total_bits++;
      pos = total_bits >> 4;
      if (pos >= sizeof data) {
        resetDecoder();
        return;
      }
      state = OK;
    }

    virtual char decode (word width) {
      if (200 <= width && width < 1200) {
        //Serial.print("Dauer="); Serial.println(width);
        //Serial.println(width);
        byte w = width >= 700;

        switch (state) {
          case UNKNOWN:
            if (w != 0) {
              // Long pulse
              ++flip;
            } else if (w == 0 && 24 <= flip) {
              // Short pulse, start bit
              Serial.print("flip="); Serial.println(flip);
              flip = 0;
              state = T0;
            } else {
              // Reset decoder
              return -1;
            }
            break;
          case OK:
            if (w == 0) {
              // Short pulse
              state = T0;
            } else {
              // Long pulse
              manchester(1);
            }
            break;
          case T0:
            if (w == 0) {
              // Second short pulse
              manchester(0);
            } else {
              // Reset decoder
              return -1;
            }
            break;
        }
      } else if (width >= 2500  && pos >= 8) {
        return 1;
      } else {
        return -1;
      }
      return 0;
    }
};


void detect_mc()
{
   bool state;
  uint16_t len = sizeof(sample_OSV2_data)/sizeof(sample_OSV2_data[0]);
  for (uint16_t i=0;i<=len;++i)
  {
      //Serial.print("#");Serial.println(i);
      state = ManchesterDetect.detect(&sample_OSV2_data[i]); //OSV2
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
  bool state;

  for (uint16_t i=0;i<=lendata;++i)
  {
      //Serial.print("#");Serial.println(i);
      //delay(1);
      if (ManchesterDetect.detect(&pulsedata[i]))
      {
        Serial.println("Manchester erkannt");
        state = osv2Dec.decode(); //OSV2 Decoder
        delay(2000);
        if (state)
        {
            Serial.println("OSV2 Message dekodiert");
            Serial.println(osv2Dec.getMessageHexStr());
            //ManchesterDetect.reset();
            delay(1000);
        }
        state = asDec.decode(); //OSV2 Decoder
        if (state)
        {
            Serial.println("AS Message dekodiert");
        	Serial.println(asDec.getMessageHexStr());

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




void decode_osv2_jeelib()
{
    String message = "";
    OregonDecoderV2 orscV2;
    for (uint16_t i=0; i<=lendata; ++i)
    {
        if (orscV2.nextPulse(abs(pulsedata[i])) )
        {
            uint8_t len;
            const byte* data = orscV2.getData(len);
            byte bdata;
            byte idx;

            Serial.print("Binary ("); Serial.print(len); Serial.print(") : ");
            for (idx=0;idx<len;++idx);
            {
                Serial.println("");
                Serial.println(idx);
                for (byte p=0;p<8;p++)
                {

                    bdata = data[idx];
    //                Serial.print(bdata >> p & 0x1);
                }
            }
            Serial.println(idx);

            char tmp[36]="";
            char len_field[2]="";
            uint8_t tmp_len = 0;
            //strcat(tmp, "OSV2:");
            //tmp_len = 5;
#ifdef DEBUG
            Serial.print("HEXStream");
#endif

            for (byte i = 0; i < len; ++i)
            {
                Serial.print(data[i] >> 4, HEX);
                Serial.print(data[i] & 0x0F, HEX);
                Serial.print("=");
                Serial.print(data[i],BIN);
                Serial.print(",");
                tmp_len += snprintf(tmp + tmp_len, 36, "%02X", data[i]);
            }

#ifdef DEBUG
            Serial.println(" ");
            Serial.println("Length:");
#endif
            message=(String(len*8, HEX));
            message.concat(tmp);
            Serial.println(message);
            orscV2.resetDecoder();
        }

    }

}

void detect_onoff()
{
  static bool state;
  uint16_t len = sizeof(sample_onoff_data)/sizeof(sample_onoff_data[0]);
  for (uint16_t i=0;i<=len;++i)
  {
      //Serial.print("#");Serial.println(i);
      state = ooDetect.detect(&sample_onoff_data[i]);
      //delay(100);
      if (state)
      {
        Serial.println("ON-OFF Message dekodiert");
        ooDetect.reset();
        delay(1000);
      }
  }
  if (!state)
  {
    ooDetect.processMessage();
    Serial.println("Message nicht dekodiert");
    delay(1000);
   }

}


void decode_onoff()
{
  bool state;
  uint16_t len = sizeof(sample_onoff_data)/sizeof(sample_onoff_data[0]);
  init_random_data();
  for ( uint8_t i=0;i<=rand_data_size;i++)
  {
       state = ooDecode.decode(&random_data[i]);  // simulate some noise
  }
  for (uint8_t repeat=0;repeat<=2;repeat++)
  {
      uint16_t i=0;
      if (repeat==0) i=20;      // Simulate that we have not retrieved all from the first message
      for (;i<=len;++i)
      {
          //Serial.print("#");Serial.println(i);
          state = ooDecode.decode(&sample_onoff_data[i]);
          //delay(100);
          signalduration+=abs(sample_onoff_data[i]);
          if (state)
          {
            Serial.println("ON-OFF Message dekodiert");
            ooDecode.reset();
            //delay(1000);
          }
      }
  }
  init_random_data();
  for ( uint8_t i=0;i<=rand_data_size;i++)
  {
      state =  ooDecode.decode(&random_data[i]);
  }

  if (!state)
  {
    ooDecode.processMessage();
    Serial.println("Message nicht dekodiert");
    //delay(1000);
   }

}



uint8_t msg=0;



void setup() {
  randomSeed(A4);
  Serial.begin(BAUDRATE);
  init_random_data();
  delay(2000);
  Serial.println("Startup:");

	// Simple ON OFF Data
  Serial.print("Len Input data (onoff signal): ");
  Serial.println(sizeof(sample_onoff_data)/sizeof(sample_onoff_data[0]));
  Serial.println("Detecting pattern ");
  //detect_onoff();
  uint32_t start_time=micros();
  decode_onoff();
  uint32_t end_time=micros();
/*
	// OSV2 Data

  Serial.print("Len Input data (Manchester): ");
  Serial.println(lendata);
  Serial.println("Decoding protocol:");
  decode_mc();

	// AS Data
  Serial.print("Len Input data (Manchester): ");
  pulsedata = sample_AS_data;
  Serial.println("Decoding protocol:");
  lendata = sizeof(sample_AS_data)/sizeof(sample_AS_data[0]);
  Serial.println(lendata);
  //detect_mc();

  decode_mc();


  //Serial.print("Decoding with jeelib: "); // For testing the results
  //decode_osv2_jeelib();
//  randomize_signal();
//  detect();
*/
  uint32_t duration= end_time-start_time;
  Serial.print("Detection Time =");  Serial.print(duration);  Serial.println(" micro seconds");
  Serial.print("Signal Time is=");  Serial.print(signalduration);  Serial.println(" micro seconds");




}

void loop() {
  //}
  delay(1);
}




