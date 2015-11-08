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
patternDetector ooDetect;
patternDecoder  ooDecode;

// OSV2 Data hex : DADC539E18277055                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Detector ends here
int sample_OSV2_data[] = { 852, -1112, 356, -624, 840, -632, -1116, 840, -1104, 848, -1112, 352, -628, 836, -1124, 828, -644, -1112, 840, -1124, 832, -1116, -624, 840, -1120, 832, -1120, 836, -636, -1124, 832, -1116, -628, 840, -624, 352, -1124, 832, -1120, 836, -1116, 840, -1108, 844, -1104, 852, -1104, 364, -608, 856, -1100, 852, -1108, 848, -624, 352, -1092, 376, -604, 860, -624, 356, -1108, 356, -608, 860, -624, 352, -1092, 376, -604, 860, -604, 372, -1096, 368, -604, 864, -608, 368, -1104, 852, -1092, 372, -612, 852, -608, 368, -1092, 376, -612, 852, -3568, 864, -1080, 872, -1080, 876, -1088, 864, -1088, 868, -1092, 856, -1096, 860, -1084, 872, -1084, 868, -1100, 856, -1084, 868, -1096, 860, -1084, 868, -1080, 872, -1092, 864, -1088, 864, -1092, 864, -608, 368, -1088, 380, -592, 872, -600, 376, -1080, 388, -600, 860, -1092, 864, -600, 376, -1092, 372, -604, 864, -1088, 868, -600, 372, -1092, 864, -1084, 380, -600, 868, -1084, 868, -1088, 868, -596, 380, -1092, 376, -596, 868, -1088, 868, -1088, 864, -1092, 860, -620, 356, -1088, 868, -1088, 380, -604, 860, -608, 368, -1104, 360, -600, 868, -596, 384, -1084, 864, -1100, 364, -608, 860, -1092, 860, -1096, 860, -1096, 856, -600, 380, -1088, 860, -1092, 376, -616, 852, -612, 364, -1084, 872, -1084, 868, -1088, 376, -624, 844, -1088, 868, -596, 376, -1088, 868, -1084, 868, -1084, 384, -608, 856, -1096, 856, -1092, 864, -604, 372, -1088, 868, -1076, 392, -596, 864, -592, 388, -1084, 868, -1092, 860, -1092, 864, -1096, 860, -1092, 860, -1096, 368, -596, 868, -1088, 868, -1084, 868, -608, 372, -1092, 372, -600, 868, -604, 372, -1088, 376, -600, 864, -612, 368, -1088, 376, -608, 856, -600, 376, -1088, 380, -604, 864, -604, 372, -1080, 872, -1096, 372, -596, 868, -600, 376, -1084, 380, -604, 860, -32001, -540, -15084, -4280, -1824, -13332, -9112, -1880, -2448, -2552, -8176, -1552, -8304, -11572, -1028, -11124, -424, -8744, -524, -2728};

const uint8_t rand_data_size=30;

int random_data[rand_data_size];  // Array for some Random Data

int sample_onoff_data[]=
// Logilink:
 {512, -9200, 576, -1920, 516, -3880, 512, -1924, 512, -3880, 516, -1936, 512, -3880, 512, -3880, 516, -3876, 516, -3880, 512, -1924, 512, -1920, 528, -3880, 516, -3876, 516, -3880, 512, -1924, 512, -3880, 512, -1936, 512, -1928, 508, -1924, 524, -1924, 508, -3884, 512, -3880, 512, -3880, 512, -1940, 512, -1920, 512, -1924, 524, -3880, 512, -3880, 516, -1920, 512, -3880, 512, -1936, 512, -1924, 512, -1936, 512, -1924, 508, -3880, 512, -1940, 508};//, -9184, 576, -1936, 512, -3876, 516, -1920, 512, -3880, 528, -1920, 516, -3876, 512, -3880, 512, -3880, 528, -3876, 516, -1920, 512, -1924, 524, -3880, 512, -3880, 512, -3880, 512, -1920, 528, -3880, 512, -1920, 512, -1940, 508, -1924, 508, -1940, 508, -3884, 508, -3880, 512, -3880, 512, -1936, 512, -1924, 508, -1940, 508, -3880, 512, -3880, 512, -1936, 512, -3880, 512, -1920, 512, -1936, 512, -1924, 508, -1940, 508, -3880};
// ITTX Data:
//{-1012, -3556, -2364, 152, -108, -1448, -5604, -3944, 188, -3304, -768, -1212, -2156, -568, -636, -460, -704, 100, -496, -1324, -1052, 112, -5852, -1484, -716, -300, -2692, -380, 1264, -1012, 1260, -1004, 1256, -1016, 1240, -1032, 468, -1020, 1252, -1020, 488, -1024, 1232, -1032, 1236, -1020, 1236, -1036, 1236, -1036, 1228, -1040, 460, -1028, 1240, -1036, 1232, -1032, 1228, -1040, 1232, -1044, 1220, -1040, 460, -1032, 472, -1036, 1224, -1040, 464, -1044, 1224, -1036, 468, -1048, 1212, -1040, 1224, -1032, 476, -1036, 1232, -1024, 1236, -1044, 456, -1052, 460, -1036, 1228, -1044, 1220, -1040, 460, -1036, 1240, -1024, 468, -1032, 1248, -1024, 1232, -1028, 476, -1028, 1232, -1032, 476, -1036, 1224, -1048, 1216, -1040, 468, 464, -843};

// AS Data
//SB:10110011
//B1:00000110  =0x06
//B2:10000000  =0x80
//B3:11001000  =0xc8
//B4:10000000  =0x80
//                                        | Start                                                                                                                                                                                                                                                                                                                                                                                         |Ends
int sample_AS_data[] = {-764, -524, -536, 1776, -1508, 956, -684, 1712, -784, 844, -1568, 924, -720, 1712, -808, 828, -812, 816, -820, 812, -824, 808, -1608, 880, -756, 1652, -1620, 1656, -832, 796, -844, 788, -844, 784, -844, 792, -844, 788, -844, 788, -1632, 864, -772, 1636, -856, 780, -1628, 1636, -856, 772, -860, 776, -1636, 1632, -864, 772, -860, 772, -864, 772, -860, 768, -860, 776, -860, 776, -860, 772, -780, 776, -3908, 1640, -1628, 864, -768, 1640, -852, 784, -1624, 864, -772, 1676, -848, 788, -844, 792, -844, 784, -848, 788, -1620, 868, -772, 1636, -1632, 1644, -848, 780, -852, 784, -852, 780, -848, 784, -852, 784, -844, 788, -1632, 864, -768, 1640, -852, 784, -1624, 1640, -852, 776, -852, 788, -1636, 1632, -852, 784, -852, 776, -856, 780, -852, 784, -848, 784, -852, 780, -856, 776, -776, 780};


// This array can be filled with output from signalduino
//id=738, channel=2, temp=12.3 :  unknown Protocol
//uint8_t signal_Stream[]= {  0,3,0,2,0,1,0,2,0,2,0,2,0,1,0,1,0,1,0,2,0,1,0,1,0,2,0,1,0,1,0,1,0,1,0,1,0,2,0,2,0,2,0,2,0,1,0,2,0,2,0,2,0,2,0,2,0,2,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,3};
//int patternData[]=  { 476, -980, -1956, -4016,100  };//{-100,-988,488,-1960,-4016};

// Kaku Swirtch Protocol bits: 00111000111000111111111010001001
//uint8_t signal_Stream[]={1,5,1,0,1,3,1,0,1,3,1,2,1,0,1,2,1,0,1,2,1,0,1,0,1,2,1,0,1,2,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,0,1,3,1,0,1,3,1,0,1,2,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,2,1,0,1,0,1,3,1,2,1,0,1,0,1,3,1,0,1,3,1,0,1,3,1,2,1,0,1,0,1,3,1,0,1,3,1,2,1,0,1,4};
//int patternData[]={-232,308,-904,-1092,-9464,-2548,1589};

// Demo Signal like from einhell example
//int patternData[]={390,-1184,-401,1122,-20012};
//uint8_t signal_Stream []={0,2,3,2,3,2,3,1,0,1,0,2,3,2,3,1,0,2,3,1,0,1,0,1,0,1,0,1,0,2,3,2,3,2,3,1,0,1,0,2,3,1,0,1,0,1,0,2,3,2,3,2,3,2,3,2,3,2,3,1,0,2,0,4};

// Demo Signal like from RF803E

//int patternData[]={291,-506,0,-926,695,-1758};
//uint8_t signal_Stream []={0,1,0,3,0,1,0,1,0,1,0,1,0,1,0,1,0,1,4,1,0,1,0,1,0,1,0,1,0,1,0,1,0,3,4,3,4,1,0,1,0,3,4,1,4,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,5,0,1,0,1,0,3,4,3,4,3,0,1,4,3,0,1,0,1,4,3,0,1,4,3,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,4,3,0,1,4,1,0,1,0,1,0,1,0,3,4,1,0,3,0,1,0,1,0,1,0,1,0,1,0,1,0,1,4,1,0,1,0,1,0,1,0,1,0,1,0,1,0,3,4,3,4,1,0,1,0,3,4,1,4,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,5,0,1,0,1,0,3,4,3,4,3,0,1,4,3,0,1,0,1,4,3,0,1};

// Demo Signal like from Oregon NR868
int patternData[]={
-1072,577,-521,1128,-1801
};

uint8_t signal_Stream []= {
0,1,2,1,2,3,0,3,2,1,2,1,0,3,0,3,4,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,3,2,1,0,3,0,1,2,1,2,3,0,3,2,1,2,1,0,3,0,3,4,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,3,2,1,0,3,0,1,2,1,2,3,0,3,2,1,2,1,0,3,0,3,4,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,3,2,1,0,3,0,1,2,1,2,3,0,3,2,1,2,1,0,3,0,3,4,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,3,2,1,0,3,0,1,2,1,2,3,0,3,2,1,2,1,0,3,0,3,4,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,0,

};

//uint8_t signal_Stream []={ 0,1,0,1,0,2,0,1,0,1,0,1,0,1,0,1,0,1,0,1,3,1,0,1,0,1,0,1,0,1,0,1,0,1,3,1,3,1,3,2,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,3,1,0,1,0,1,0,1,0,1,0,1,0,1,0,2,0,1,0,1,0,1,0,1,0,1,0,1,0,1,3,1,0,1,0,1,0,1,0,1,0,1,0,1,3,1,3,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,};

//MS;P0=390;P1=-1184;P2=-401;P3=1122;P4=12754;P5=-20012;P6=1371;D=02323232310232310231010101010231010102310101010232323232323102302305023232323102323102310101010102310101023101010102323232323231023023;CP=0;SP=5;


int *pulsedata =NULL;
uint16_t lendata=0;


uint32_t signalduration =0;

void init_random_data()
{

    for (uint8_t i=0; i<rand_data_size; i++)
    {
        random_data[i] = rand() >> 5;//random(1,800);
        if (i % 2 == 0)
        {
          random_data[i]=random_data[i]*-1;
        }

        //delay(1);
      //  Serial.print(',');Serial.print(random_data[i]);
    }

    //Serial.println("");

}



void detect_onoff()
{
  static bool state;
  for (uint16_t i=0;i<=lendata;++i)
  {
      //Serial.print("#");Serial.println(i);
      state = ooDetect.detect(&pulsedata[i]);
      //delay(100);
      if (state)
      {
        Serial.println("Message dekodiert");
        //ooDetect.reset();
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
  signalduration=0;

  for ( uint8_t i=0;i<rand_data_size;i++)
  {
       state = ooDecode.decode(&random_data[i]);  // simulate some noise
  }

  uint16_t i=0;

  for (uint8_t repeat=0;repeat<=2;repeat++)
  {
      i=0;
      if (repeat==0) i=20;      // Simulate that we have not retrieved all from the first message
      for (;i<=lendata;++i)
      {
          //Serial.print("#");Serial.println(i);
          state = ooDecode.decode(&pulsedata[i]);
          //delay(100);
          signalduration+=abs(pulsedata[i]);
          if (state)
          {
            Serial.println("Message dekodiert");
            //ooDecode.reset();
            //delay(1000);
          }
      }
  }
  for ( uint8_t i=0;i<rand_data_size;i++)
  {
      state =  ooDecode.decode(&random_data[i]);
  }


}


void decode_signalstream()
{
    uint16_t len = sizeof(signal_Stream)/sizeof(signal_Stream[0]);
    init_random_data();
    signalduration=0;
    bool state;

    for ( uint8_t i=0;i<rand_data_size;i++)
    {
        //state =  ooDecode.decode(&random_data[i]);
    }

    for (uint8_t j=0;j<1;++j) {

        uint16_t i=0;

        for (;i<lendata;++i)
        {
          //Serial.print("#");Serial.println(i);
          state = ooDecode.decode(&patternData[signal_Stream[i]]);
          //delay(100);
          signalduration+=abs(patternData[signal_Stream[i]]);
        }
    }

    init_random_data();
    for ( uint8_t i=0;i<rand_data_size;i++)
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
  randomSeed(A0);
  Serial.begin(BAUDRATE);
  init_random_data();
  delay(2000);
  Serial.println("Startup:");
	// Simple ON OFF Data
  //detect_onoff();
  uint32_t start_time=0;
  uint32_t end_time=0;
  uint32_t duration=0;



    //   Oregon Scientific V2 protocol regression test
/*
  pulsedata = sample_OSV2_data;
  lendata = sizeof(sample_OSV2_data)/sizeof(sample_OSV2_data[0]);
  Serial.println("");
  Serial.println("--------------------------------------------------------");
  Serial.print("Len Input data (OSV2 Manchester): ");
  Serial.println(lendata);
  Serial.println("Detecting pattern ");
  init_random_data(); signalduration=0;
  start_time=micros();
  decode_onoff();
  end_time=micros();
  duration= end_time-start_time;
  Serial.print("Detection Time =");  Serial.print(duration);  Serial.println(" micro seconds");
  Serial.print("Signal Time is=");  Serial.print(signalduration);  Serial.println(" micro seconds");
  Serial.println("--------------------------------------------------------");
*/

    //   regression test, working with Signaldata and not pulsedata
  Serial.println("");
  Serial.println("--------------------------------------------------------");
  Serial.print("Len Input data (signal data protocol): ");
  lendata = sizeof(signal_Stream)/sizeof(signal_Stream[0]);
  Serial.println(lendata);
  Serial.println("Detecting pattern ");
  init_random_data(); signalduration=0;
  start_time=micros();
  decode_signalstream();
  end_time=micros();
  duration= end_time-start_time;
  Serial.print("Detection Time =");  Serial.print(duration);  Serial.println(" micro seconds");
  Serial.print("Signal Time is=");  Serial.print(signalduration);  Serial.println(" micro seconds");
  Serial.println("--------------------------------------------------------");
  Serial.println("");


return;

    //   M0 Logilink protocol puls pause regression test
  pulsedata = sample_onoff_data;
  lendata = sizeof(sample_onoff_data)/sizeof(sample_onoff_data[0]);
  Serial.println("");
  Serial.println("--------------------------------------------------------");
  Serial.print("Len Input data (Logilink protocol): ");
  Serial.println(lendata);
  Serial.println("Detecting pattern ");
  Serial.println("");
  init_random_data(); signalduration=0;
  start_time=micros();
  decode_onoff();
  end_time=micros();
  duration= end_time-start_time;
  Serial.print("Detection Time =");  Serial.print(duration);  Serial.println(" micro seconds");
  Serial.print("Signal Time is=");  Serial.print(signalduration);  Serial.println(" micro seconds");
  Serial.println("--------------------------------------------------------");
  Serial.println("");



//compr: 4->2;7*-1020->32*-1036 idx:-1028 idx2:-1;



}

void loop() {
  //}
  delay(1);
}




