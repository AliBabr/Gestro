#include <PinChangeInt.h>
#include "GastroMarket.h"
#include "messaging.h"
#include <TimerOne.h>

bool Sensor38Left = 0;
bool Sensor38Right = 0;
bool Sensor56Left = 0;
bool Sensor56Right = 0;

bool Sensor38Left_last = 0;
bool Sensor38Right_last = 0;
bool Sensor56Left_last = 0;
bool Sensor56Right_last = 0;

bool PIRSensor = 0;

uint64_t currentTime = 0;


uint16_t LeftINCounter;
uint16_t RightINCounter;

uint16_t LeftOUTCounter;
uint16_t RightOUTCounter;

uint64_t Left38FirstHitTime = 0;
uint64_t Left56FirstHitTime = 0;
uint64_t Right38FirstHitTime = 0;
uint64_t Right56FirstHitTime = 0;

uint64_t Left38Timeout = 0;
uint64_t Left56Timeout = 0;
uint64_t Right38Timeout = 0;
uint64_t Right56Timeout = 0;

uint64_t  LeftPersonTimeout = 0;
uint64_t  RightPersonTimeout = 0;

uint64_t SendTime = 0;

byte Left38Raw = 0;
byte Left56Raw = 0;
byte Right38Raw = 0;
byte Right56Raw = 0;

byte BatteryPercentage = 0;



void setup() {
  // put your setup code here, to run once:
pinMode(LED_38_KHZ_LEFT,OUTPUT); //TIMER2
pinMode(LED_38_KHZ_RIGHT, OUTPUT); //TIMER2

pinMode(LED_56_KHZ_LEFT,OUTPUT); // TIMER1
pinMode(LED_56_KHZ_RIGHT, OUTPUT); //TIMER1


pinMode(REL_1,OUTPUT); 
pinMode(REL_2, OUTPUT); 
pinMode(REL_3,OUTPUT); 
pinMode(REL_4, OUTPUT); 


pinMode(SENSOR_38_KHZ_LEFT, INPUT);
pinMode(SENSOR_38_KHZ_RIGHT, INPUT);

pinMode(SENSOR_56_KHZ_LEFT, INPUT);
pinMode(SENSOR_56_KHZ_RIGHT, INPUT);
pinMode(PIR_SENSOR,INPUT);

attachPinChangeInterrupt(SENSOR_38_KHZ_LEFT, read38Left, FALLING);
attachPinChangeInterrupt(SENSOR_38_KHZ_RIGHT, read38Right, FALLING);
attachPinChangeInterrupt(SENSOR_56_KHZ_LEFT, read56Left, FALLING);
attachPinChangeInterrupt(SENSOR_56_KHZ_RIGHT, read56Right, FALLING);

Serial.begin(9600);

write38();

write56();

//Timer1.initialize(17); //17 us 58 KHz
 Timer1.pwm(LED_56_KHZ_LEFT, 512);
}

void stopTransducer()
{
    TCCR2A = _BV(COM2A1) | _BV(COM2B1);
}

//OCR2A = 142 //56 KHz
//OCR2A = 210 //38 KHz
void write38()
{
  TCCR2A = _BV(COM2A0) | _BV(WGM21) | _BV(COM2B0);
  TCCR2B = _BV(CS20);
  OCR2A = 210;
  OCR2B = 210;
}

void stop38()
{
  TCCR2A = _BV(COM2A1) | _BV(COM2B1);
}


void write56()
{

 Timer1.initialize(18); //17 us 58 KHz
}

void stop56()
{
  
}


void read38Left()
{
   if(Sensor38Left == false)
   { 
     Sensor38Left = true;
     Left38FirstHitTime = millis();
     Left38Timeout = Left38FirstHitTime + SENSOR_TIMEOUT;
   }
}


void read38Right()
{
  if(Sensor38Right == false)
   {
      Sensor38Right = true;
      Right38FirstHitTime = millis();
      Right38Timeout = Right38FirstHitTime + SENSOR_TIMEOUT;
   }
}

void read56Left()
{ 
  if(Sensor56Left == false)
  {
  Sensor56Left = true;
  Left56FirstHitTime = millis();
  Left56Timeout = Left56FirstHitTime + SENSOR_TIMEOUT;
  }
}

void read56Right()
{
  if(Sensor56Right == false)
  {
    Sensor56Right = true;
    Right56FirstHitTime = millis();
    Right56Timeout = Right56FirstHitTime + SENSOR_TIMEOUT;
  }
}


void checkTimeouts()
{
   if(Left38Timeout < currentTime)
   {
      Sensor38Left = false;
   }
      if(Right38Timeout < currentTime)
   {
      Sensor38Right = false;
   }
      if(Left56Timeout < currentTime)
   {
      Sensor56Left = false;
   }
      if(Right56Timeout < currentTime)
   {
      Sensor56Right = false;
   }

   if((RightPersonTimeout > currentTime))
   {
    Sensor38Right = false;
    Sensor56Right = false;

   }
   
   if((LeftPersonTimeout > currentTime))
   {
    Sensor38Left = false;
    Sensor56Left = false;
    
   }
}

 
void checkDirection()
{
  if(Sensor38Left && Sensor56Left && (LeftPersonTimeout < currentTime))
  {
    if(Left38FirstHitTime > Left56FirstHitTime) LeftINCounter++;
    else LeftOUTCounter++;
    LeftPersonTimeout = currentTime + DELAY_BETWEEN_PERSONS;

  }

  if(Sensor38Right && Sensor56Right && (RightPersonTimeout < currentTime))
  {
    if(Right38FirstHitTime > Right56FirstHitTime) RightINCounter++;
    else RightOUTCounter++;

    RightPersonTimeout = currentTime + DELAY_BETWEEN_PERSONS;
  }
}

void checkRange()
{
    if(!digitalRead(SENSOR_38_KHZ_LEFT))
    {
      Left38Raw = true;
       #ifdef DEBUG 
       Serial.println("00000     ");
       #endif
    }
    if(!digitalRead(SENSOR_56_KHZ_LEFT))
    {
      Left56Raw = true;
       #ifdef DEBUG 
       Serial.println("     00000");
       #endif
    }
    if(!digitalRead(SENSOR_38_KHZ_RIGHT))
    {
      Right38Raw = true;
    }
    if(!digitalRead(SENSOR_56_KHZ_RIGHT))
    {
      Right56Raw = true;
    }
}

void checkPIR()
{
      PIRSensor = digitalRead(PIR_SENSOR);
//   if(digitalRead(PIR_SENSOR))Serial.println("++++++++++++++++++++++++++++");
//      else Serial.println(" ");
}
//#define DEBUG
void loop() 
{
      currentTime = millis();
      checkTimeouts();
      checkDirection();
      Get_Data();
  
      if(SendTime < currentTime)
      {
        checkRange();
        checkPIR();
        SendTime = currentTime + 1000/SENDING_FREQUENCY_HZ;
        #ifndef DEBUG 
          Send_Data(); 
        #endif  
        
       
      
      }
}
