#include <PinChangeInt.h>
#include "GastroMarket.h"
#include "messaging.h"
#include <TimerOne.h>
#include <AceButton.h>
#include  "thermometer.h"
using namespace ace_button;

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
uint64_t FilterTime = 0;

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

uint64_t PumpTimeout = 0;
uint64_t PumpCoolDown = 0;
uint64_t PumpVentilationTimeout = 0;

uint64_t SendTime = 0;

byte Left38Raw = 0;
byte Left56Raw = 0;
byte Right38Raw = 0;
byte Right56Raw = 0;

byte BatteryPercentage = 0;

byte HandSensor = 0;
byte RunPumpVentilation = 0;
byte FlashBlueLed = 0;
volatile uint16_t DesinfectantDose = 400; //in millis

bool LeftSensorEnabled = true;
bool RightSensorEnabled = true;

bool buttonPressed = false;
bool turningOff = false;

bool sleepMode = false;

bool serviceEnter = false;

byte SonarDistance = 0;
uint16_t object_temperature = 0;
byte ambient_temperature = 0;

void handleEvent(AceButton*, uint8_t, uint8_t);
AceButton button(BUTTON_PIN);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_38_KHZ_LEFT, OUTPUT); //TIMER2
  pinMode(LED_38_KHZ_RIGHT, OUTPUT); //TIMER2

  pinMode(LED_56_KHZ_LEFT, OUTPUT); // TIMER1
  pinMode(LED_56_KHZ_RIGHT, OUTPUT); //TIMER1

  // pinMode(LED_56_KHZ_HAND, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SENSOR_56_KHZ_HAND, INPUT_PULLUP);

  //pinMode(REL_1, OUTPUT);
  //pinMode(REL_2, OUTPUT);
  pinMode(REL_3, OUTPUT);
  //pinMode(REL_4, OUTPUT);

  pinMode(TURN_ON_PIN, OUTPUT);
  pinMode(RED_LED_HAND, OUTPUT);  
  pinMode(RED_LED_TEMP, OUTPUT);
  pinMode(CHARGING_ON, OUTPUT);
  
  digitalWrite(TURN_ON_PIN, 1);

  pinMode(SENSOR_38_KHZ_LEFT, INPUT_PULLUP);
  pinMode(SENSOR_38_KHZ_RIGHT, INPUT_PULLUP);

  pinMode(SENSOR_56_KHZ_LEFT, INPUT_PULLUP);
  pinMode(SENSOR_56_KHZ_RIGHT, INPUT_PULLUP);
  //pinMode(PIR_SENSOR, INPUT_PULLUP);


  //pinMode(TURN_ON_BUTTON, INPUT);
   // Button uses the built-in pull up register.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

 thermometer_setup();

#ifndef DEBUG
  for(int i = 0; i < 20;i++)
  {
     BatteryCheck();
  }
#endif

  digitalWrite(CHARGING_ON, 1);

// Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);
  buttonConfig->setLongPressDelay(2000);


  attachPinChangeInterrupt(SENSOR_38_KHZ_LEFT, read38Left, FALLING);
  attachPinChangeInterrupt(SENSOR_38_KHZ_RIGHT, read38Right, FALLING);
  attachPinChangeInterrupt(SENSOR_56_KHZ_LEFT, read56Left, FALLING);
  attachPinChangeInterrupt(SENSOR_56_KHZ_RIGHT, read56Right, FALLING);

  Serial.begin(9600);

  write38();
  write56();
  
//starting sequency LED
  digitalWrite(BLUE_LED, true);
  delay(200);
  digitalWrite(BLUE_LED, false);
  delay(200);
  digitalWrite(BLUE_LED, true);
  delay(200);
  digitalWrite(BLUE_LED, false);
  delay(200);
  digitalWrite(BLUE_LED, true);
  delay(200);
  digitalWrite(BLUE_LED, false);


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
  Timer1.initialize(18); //18 us 56 KHz
  Timer1.pwm(LED_56_KHZ_LEFT, 512);
  Timer1.pwm(LED_56_KHZ_RIGHT, 512);
  // Timer1.pwm(LED_56_KHZ_HAND, 512);
}

void stop56()
{

}




void read38Left()
{
  if (Sensor38Left == false)
  {
    Sensor38Left = true;
    Left38FirstHitTime = micros64();
    Left38Timeout = Left38FirstHitTime + SENSOR_TIMEOUT;
  }
}

void read38Right()
{
  if (Sensor38Right == false)
  {
    Sensor38Right = true;
    Right38FirstHitTime = micros64();
    Right38Timeout = Right38FirstHitTime + SENSOR_TIMEOUT;
  }
}

void read56Left()
{
  if (Sensor56Left == false)
  {
    Sensor56Left = true;
    Left56FirstHitTime = micros64();
    Left56Timeout = Left56FirstHitTime + SENSOR_TIMEOUT;
  }
}

void read56Right()
{
  if (Sensor56Right == false)
  {
    Sensor56Right = true;
    Right56FirstHitTime = micros64();
    Right56Timeout = Right56FirstHitTime + SENSOR_TIMEOUT;
  }
}

void checkTimeouts()
{
  if (Left38Timeout < currentTime)
  {
    Sensor38Left = false;
  }
  if (Right38Timeout < currentTime)
  {
    Sensor38Right = false;
  }
  if (Left56Timeout < currentTime)
  {
    Sensor56Left = false;
  }
  if (Right56Timeout < currentTime)
  {
    Sensor56Right = false;
  }

  if ((RightPersonTimeout > currentTime))
  {
    Sensor38Right = false;
    Sensor56Right = false;

  }

  if ((LeftPersonTimeout > currentTime))
  {
    Sensor38Left = false;
    Sensor56Left = false;
  }
}


void checkDirection()
{
  if (Sensor38Left && Sensor56Left && (LeftPersonTimeout < currentTime))
  {

    if(LeftSensorEnabled)
    {
      if(!serviceEnter)
      {
        if (Left38FirstHitTime > Left56FirstHitTime) LeftINCounter++;
        else LeftOUTCounter++;
      }
      
    }
    serviceEnter = false;
 
    LeftPersonTimeout = currentTime + DELAY_BETWEEN_PERSONS;
  }

  if (Sensor38Right && Sensor56Right && (RightPersonTimeout < currentTime))
  {
    if(RightSensorEnabled)
    {
      if(!serviceEnter)
      {
        if (Right38FirstHitTime > Right56FirstHitTime) RightINCounter++;
        else RightOUTCounter++;
      }
      
    }

    serviceEnter = false;
    
    RightPersonTimeout = currentTime + DELAY_BETWEEN_PERSONS;
  }
}

static uint16_t Left38Sum;
static uint8_t Left38Filt = 0;

static uint32_t Left56Sum;
static uint16_t Left56Filt = 0;

static uint16_t Right38Sum;
static uint8_t Right38Filt = 0;

static uint32_t Right56Sum;
static uint16_t Right56Filt = 0;


void checkRange()
{
  bool left38 = digitalRead(SENSOR_38_KHZ_LEFT);
  bool left56 = digitalRead(SENSOR_56_KHZ_LEFT);
  bool right38 = digitalRead(SENSOR_38_KHZ_RIGHT);
  bool right56 = digitalRead(SENSOR_56_KHZ_RIGHT);
  bool hand56 = digitalRead(SENSOR_56_KHZ_HAND);

  if (!left38)
  {
    if (Left38Raw < 15) Left38Raw++;
  }

  if (!left56)
  {
    if (Left56Raw < 15) Left56Raw++;
  }

  if (!right38)
  {
    if (Right38Raw < 15) Right38Raw++;
  }

  if (!right56)
  {
    if (Right56Raw < 15) Right56Raw++;
  }
}


#define FILTER_POWER 2
void FilterRawData()
{
  Left38Sum = Left38Sum - Left38Filt + Left38Raw;
  Left38Filt = Left38Sum >> (FILTER_POWER);

  Left56Sum = Left56Sum - Left56Filt + Left56Raw;
  Left56Filt = Left56Sum >> (FILTER_POWER);

  Right38Sum = Right38Sum - Right38Filt + Right38Raw;
  Right38Filt = Right38Sum >> (FILTER_POWER);

  Right56Sum = Right56Sum - Right56Filt + Right56Raw;
  Right56Filt = Right56Sum >> (FILTER_POWER);
}


void CheckHandSensor()
{
  unsigned long dose =  (unsigned long)DesinfectantDose ;
  byte isHand = 0;
  if (PumpCoolDown < currentTime) //
  {
    isHand = !digitalRead(SENSOR_56_KHZ_HAND);

  }
  else isHand = 0;

  if (!digitalRead(SENSOR_56_KHZ_HAND) ) PumpCoolDown = currentTime + PUMP_COOLDOWN;

  if (isHand && PumpTimeout < currentTime)
  {
    HandSensor = true;
    PumpTimeout = currentTime + dose;
  }

  if (PumpTimeout > currentTime)
  {
    digitalWrite(REL_3, true);
    //PumpCoolDown = currentTime + ONE_SEC_USEC;
    digitalWrite(BLUE_LED, true);
  }
  else
  {
    digitalWrite(REL_3, false);
    if (!FlashBlueLed)  digitalWrite(BLUE_LED, false);
  }
}

void ControlBlueLed(uint8_t state)
{
  digitalWrite(BLUE_LED, state);
}

void ControlRedLedTemp(uint8_t state)
{
  digitalWrite(RED_LED_TEMP, state);
}
void ControlRedInfoHand(uint8_t state)
{
  digitalWrite(RED_LED_HAND, state);
}


#define VENTILATION_TIME 5000 //300
#define ONE_CYCLE_DELAY 100
void PumpVentilation(uint8_t state)
{
  if (RunPumpVentilation)
  {
    digitalWrite(REL_3, state);
  }
  else digitalWrite(REL_3, state);
}

static uint32_t BatterySum = FULL;
static uint16_t BatteryFilt = FULL;
void BatteryCheck()
{
  static int lowVoltageCounter = 0;
  double batteryRaw = analogRead(BATTERY_SENSOR);
  uint16_t rawBatteryPercentage = 0;
  rawBatteryPercentage = (uint16_t)(((batteryRaw-EMPTY)/(FULL-EMPTY))*100)*10;
  
  BatterySum = BatterySum - BatteryFilt + rawBatteryPercentage;
  BatteryFilt = BatterySum >> (5);

  BatteryPercentage = (byte)(BatteryFilt/10.0);
  if(batteryRaw <=EMPTY)
  {
    lowVoltageCounter++;
  }
  else lowVoltageCounter = 0;
  if(lowVoltageCounter > 5000)
  {
      EmptyBatteryWarningAnimation();
     digitalWrite(TURN_ON_PIN, 0);
     delay(1000);
  } 
}

void EmptyBatteryWarningAnimation()
{
    digitalWrite(RED_LED_HAND, 1);
    delay(200);
    digitalWrite(RED_LED_HAND, 0);
    delay(200);
     digitalWrite(RED_LED_HAND, 1);
    delay(200);
    digitalWrite(RED_LED_HAND, 0);
    delay(200);
    digitalWrite(RED_LED_HAND, 1);
    delay(200);
    digitalWrite(RED_LED_HAND, 0);
    delay(200);
    digitalWrite(RED_LED_HAND, 1);
    delay(200);
    digitalWrite(RED_LED_HAND, 0);
    delay(200);
}



//void TurnOffCheck()
//{
//  volatile int  turn_off_cycle = 0;
//  while (!digitalRead(TURN_ON_BUTTON))
//  {  digitalWrite(BLUE_LED, true);
//   
//    //buttonPressed = true;
//    
//    turn_off_cycle++;
//    delay(50);
////    if(turn_off_cycle > 40)
////    {
////       if (digitalRead(TURN_ON_BUTTON))
////        {
////           if(sleepMode) sleepMode = true;
////           else sleepMode = false;
////           digitalWrite(BLUE_LED, false);
////        }
////     
////    }
//    if (turn_off_cycle > 90) //5second hold button
//      //int bootload_start_LED = 0;
//      digitalWrite(BLUE_LED, true);
//      while (1)
//      {
//        delay(10);
//        if (digitalRead(TURN_ON_BUTTON))
//        {
//          delay(100);
//          digitalWrite(BLUE_LED, false);
//          digitalWrite(TURN_ON_PIN, LOW);
//        }
//        //delay(50);
//      }
//  }
//}

// The event handler for the button.
void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t buttonState) {


  // Control the LED only for the Pressed and Released events.
  // Notice that if the MCU is rebooted while the button is pressed down, no
  // event is triggered and the LED remains off.
  switch (eventType) {
    case AceButton::kEventPressed:
      digitalWrite(BLUE_LED, true);
      FlashBlueLed = true;
      break;
    case AceButton::kEventReleased:
         buttonPressed = true;
         FlashBlueLed = false;
      break;
     case AceButton::kEventDoubleClicked:
        buttonPressed = true;
     break;
     case AceButton::kEventLongPressed:

             if(turningOff == false)
             {
                turningOff = true;
                Send_Data();
             }
            
            delay(100);
            digitalWrite(BLUE_LED, false);
            delay(100);
            digitalWrite(TURN_ON_PIN, LOW);
      break;

  }
}


static uint32_t SonarSum;
static uint64_t SonarFilt = 0;

#define FILTER_POWER_SONAR 2
#define MAX_RANG (502.0) //cm
#define  ADC_SOLUTION      (1023.0)//ADC accuracy of Arduino UNO is 10bit

void checkSonar()
{
  //read the value from the sensor:
 uint32_t sensity_t = analogRead(SENSOR_SONAR);

  SonarSum = SonarSum - SonarFilt + sensity_t;
  SonarFilt = SonarSum >> (FILTER_POWER_SONAR);

  SonarDistance = (byte)(((sensity_t * MAX_RANG)/ADC_SOLUTION)/2.0);

 
}

uint64_t micros64() {
  static uint32_t low32, high32;
  uint32_t new_low32 = micros();
  if (new_low32 < low32) high32++;
  low32 = new_low32;
  return (uint64_t) high32 << 32 | low32;
}
//#define DEBUG
void loop()
{
  currentTime = millis();

  if(!sleepMode)
  {
    checkTimeouts();
    checkDirection();
    Get_Data();
    checkRange();
    FilterRawData();
    if (!RunPumpVentilation) CheckHandSensor();
    // PumpVentilation();
    
    if (SendTime < currentTime)
    {
      //checkPIR();
      checkSonar();
      measure_temperature();
      SendTime = currentTime + 1000 / SENDING_FREQUENCY_HZ;
  
      Left38Raw = Left38Filt;
      Left56Raw = Left56Filt;
      Right38Raw = Right38Filt;
      Right56Raw = Right56Filt;
  
  #ifndef DEBUG
    if(!turningOff) Send_Data();
  #endif
  
      Left38Raw = 0;
      Left56Raw = 0;
      Right38Raw = 0;
      Right56Raw = 0;
      HandSensor = 0;
    }
  }// if not in sleep mode
  
    BatteryCheck();
   // TurnOffCheck();
    button.check();
}
