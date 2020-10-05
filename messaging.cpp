#include "messaging.h"
#include <arduino.h>
#include "GastroMarket.h"

byte message[20] = {0};

#define BUTTON_PRESSED 0
#define HAND_SENSOR 1
#define TURNING_OFF 2


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
  message[11] = (byte)(Left38Raw | (Left56Raw << 4));
  message[12] = (byte)(Right38Raw | (Right56Raw << 4));
  
 if(buttonPressed) message[13] |= 1 << BUTTON_PRESSED; 
 else message[13] &= ~(1 << BUTTON_PRESSED);
 if(HandSensor) message[13] |= 1 << HAND_SENSOR; 
 else message[13] &= ~(1 << HAND_SENSOR);
 if(turningOff)  message[13] |= 1 << TURNING_OFF; 
 else  message[13] &= ~(1 << TURNING_OFF);
  message[14] = ambient_temperature;
  message[15] = BatteryPercentage;            //raw voltage divider battery
  message[16] = SonarDistance;                    //Sonar Distance in cm divided by 10
  message[17] = (object_temperature) & 0xFF;
  message[18] = (object_temperature >> 8) & 0xFF; 

  byte checksumValue = 0;
  for (int i = 2; i < 19; i++)checksumValue ^= message[i];
  message[19] =  checksumValue;    //Checksum

  Serial.write(message, sizeof(message));

  Left38Raw = false;
  Left56Raw = false;
  Right38Raw = false;
  Right56Raw = false;
  buttonPressed = false;
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
        DesinfectantDose = input_string[1]*48; // 1ml is 480ms of pump run
       
      }
#define MSG_RUN_VENTILATION 2
      if (input_string[0] == MSG_RUN_VENTILATION)
      {
        byte on = input_string[1];
        RunPumpVentilation = on;
        PumpVentilation(on);
      }
#define MSG_BLUE_LED 3
      if (input_string[0] == MSG_BLUE_LED)
      {
        byte on = input_string[1];
        FlashBlueLed = on;
        ControlBlueLed(on);
      }
#define MSG_IGNORE_SENSOR 4

 if (input_string[0] == MSG_IGNORE_SENSOR)
      {
         RightSensorEnabled = input_string[1];
         LeftSensorEnabled = input_string[2];
      }

#define MSG_RED_TEMP_LED 5 

      if (input_string[0] == MSG_RED_TEMP_LED)
      {
        byte on = input_string[1];
        ControlRedLedTemp(on);
      }

#define MSG_RED_INFO_LED 6

      if (input_string[0] == MSG_RED_INFO_LED)
      {
        byte on = input_string[1];
        ControlRedInfoHand(on);
      }
#define MSG_SERVICE_ENTER 7

      if (input_string[0] == MSG_SERVICE_ENTER)
      {
        serviceEnter = input_string[1];
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
