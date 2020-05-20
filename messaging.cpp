#include "messaging.h"
#include <arduino.h>
#include "GastroMarket.h"

byte message[20];

void Send_Data()
{
  //Sending data to smartphone
#define STANDARD_MESSAGE 1


  message[0] = '$';
  message[1] = '>';
  message[2] = STANDARD_MESSAGE;            //Type of message
  message[3] = (LeftINCounter) & 0xFF;                //Person counter IN left side
  message[4] = (LeftINCounter >> 8) & 0xFF;                 //Person counter IN left side
  message[5] = (LeftOUTCounter) & 0xFF;           //Person counter OUT left side
  message[6] = (LeftOUTCounter >> 8) & 0xFF;              //Person counter OUT left side
  message[7] = (RightINCounter) & 0xFF;            //Person counter IN right side
  message[8] = (RightINCounter >> 8) & 0xFF;           //Person counter IN right side
  message[9] = (RightOUTCounter) & 0xFF;          //Person counter OUT right side
  message[10] = (RightOUTCounter >> 8) & 0xFF;         //Person counter OUT right side
  message[11] = Left38Raw;                   //raw data from sensor
  message[12] = Left56Raw;                   //raw data from sensor
  message[13] = Right38Raw;                  //raw data from sensor
  message[14] = Right56Raw;                  //raw data from sensor
  message[15] = BatteryPercentage;            //raw voltage dividor battery 
  message[16] = PIRSensor;                    // PIR sensor
  message[17] = HandSensor;

  byte checksumValue = 0;
  for (int i = 2; i < 19; i++)checksumValue ^= message[i];
  message[19] =  checksumValue;    //Checksum

 Serial.write(message, sizeof(message));

  Left38Raw = false;
  Left56Raw = false;
  Right38Raw = false;
  Right56Raw = false;
}


void parseRXData(byte * dataToParse)
{
  byte input_string[18];
  for (uint8_t i = 0; i <= 17; i++)
  {
    input_string[i] = dataToParse[i];
  }
  {
    byte checksumValue = 0;
    for (int i = 0; i < 17; i++)checksumValue ^= input_string[i];

    ///==================- CHECKSUM RX OK -==================================
    if (input_string[17] == checksumValue)
    {
      #define RESET_MSG 0
      if (input_string[0] == RESET_MSG) //reset counters
      {
         LeftINCounter = 0;
         LeftOUTCounter = 0;
         RightINCounter = 0;
         RightOUTCounter = 0;
         HandSensor = 0;
      }
      
      #define MSG_SET_DOSE 1
      if (input_string[0] == MSG_SET_DOSE) 
      {
        DesinfectantDose = input_string[1]*10;
      }
      #define MSG_RUN_VENTILATION 2
      if(input_string[0] = RUN_VENTILATION)
      {
        RunPumpVentilation = true;
      }
      

    }//CHECKSUM RX MESSAGE
  }
}

void Get_Data()
{
  //Receive message from smart daevice
  uint8_t bytesInBuffer = Serial.available();
  if (bytesInBuffer > 0)
  {
    byte readBuffer[20] = {0};

    if (Serial.find("$<")) 
    {
      Serial.readBytes(readBuffer, 18);
      parseRXData(readBuffer);
    }

  }
}
