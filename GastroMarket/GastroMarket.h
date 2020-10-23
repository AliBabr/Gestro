#ifndef CGASTROMARKET_H
#define CGASTROMARKET_H

#define LED_38_KHZ_LEFT 11 //Left 38 kHz IR LED pin
#define LED_38_KHZ_RIGHT 3 //Right 38 kHz IR LED pin

#define LED_56_KHZ_LEFT 9    //Left 56 kHz IR LED pin
#define LED_56_KHZ_RIGHT 10  //Right 56 kHz IR LED pin

#define SENSOR_56_KHZ_LEFT A0  //Left 56 kHz IR RECEIVER pin
#define SENSOR_56_KHZ_RIGHT A1 //Right 56 kHz IR RECEIVER pin

#define SENSOR_38_KHZ_LEFT A2   //Left 38 kHz IR RECEIVER pin
#define SENSOR_38_KHZ_RIGHT A3  //Right 38 kHz IR RECEIVER pin

#define SENSOR_56_KHZ_HAND 4    //Hand 56 kHz IR RECEIVER pin
#define SENSOR_SONAR A6         //Sonar analog input
#define BATTERY_SENSOR A7       //Battery voltage divider
#define BLUE_LED 5              //Blue LED in hand sensor
#define PUMP_RELAY 8            //Pump relay output
#define RED_LED_HAND 7           //Red LED hand
#define RED_LED_TEMP 13         // Red LED temp
#define CHARGING_ON 6           //USB tablet charging control pin

#define TURN_ON_PIN 12          //MOSFET for turn on control pin



// The pin number attached to the button.
const int BUTTON_PIN = 2;


#define SENSOR_TIMEOUT 1500   //SENSOR_TIMEOUT must be < DELAY_BETWEEN_PERSONS
#define DELAY_BETWEEN_PERSONS 1000  //Minimal time between two persons 
#define PUMP_COOLDOWN 300           //minimal time between two consecutive hand desinfections

#define SENDING_FREQUENCY_HZ 4      //frequency of sending messages to app


#define FULL 252.0 //RAW analog battery value for full battery
#define EMPTY 204 //RAW analog battery value for empty battery

extern uint64_t currentTime;

extern uint16_t LeftINCounter;
extern uint16_t RightINCounter;

extern uint16_t LeftOUTCounter;
extern uint16_t RightOUTCounter;

extern byte Left38Raw;
extern byte Left56Raw;
extern byte Right38Raw;
extern byte Right56Raw;

extern byte BatteryPercentage;

extern byte SonarDistance;
extern byte message[20];
extern byte HandSensor;
extern volatile uint16_t DesinfectantDose;
extern byte RunPumpVentilation;
extern byte FlashBlueLed;

extern bool sleepMode;

extern bool LeftSensorEnabled;
extern bool RightSensorEnabled;

extern bool buttonPressed;
extern bool turningOff;

extern uint16_t object_temperature;

extern byte ambient_temperature;

extern bool serviceEnter;


void ControlBlueLed(uint8_t state);
void ControlRedLedTemp(uint8_t state);
void ControlRedInfoHand(uint8_t state);
void PumpVentilation(uint8_t state);
#endif
