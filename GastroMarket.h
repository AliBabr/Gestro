#ifndef CGASTROMARKET_H
#define CGASTROMARKET_H

#define LED_38_KHZ_LEFT 11
#define LED_38_KHZ_RIGHT 3

#define LED_56_KHZ_LEFT 9
#define LED_56_KHZ_RIGHT 10

#define SENSOR_56_KHZ_LEFT A0
#define SENSOR_56_KHZ_RIGHT A1

#define SENSOR_38_KHZ_LEFT A2
#define SENSOR_38_KHZ_RIGHT A3

#define PIR_SENSOR 6
#define SENSOR_56_KHZ_HAND A4
#define BATTERY_SENSOR A5
#define BLUE_LED 5
#define REL_3 8

#define TURN_ON_PIN 12 
#define TURN_ON_BUTTON 2


#define SENSOR_TIMEOUT 1500   //SENSOR_TIMEOUT must be < DELAY_BETWEEN_PERSONS
#define DELAY_BETWEEN_PERSONS 1000

#define PUMP_COOLDOWN 700

#define SENDING_FREQUENCY_HZ 4

#define REL_1 4
#define REL_2 7
#define REL_3 8
#define REL_4 12 


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
extern bool PIRSensor;
extern byte message[20];
extern byte HandSensor;
extern volatile uint16_t DesinfectantDose;
extern byte RunPumpVentilation;
extern byte FlashBlueLed;

void ControlBlueLed(uint8_t state);
void PumpVentilation(uint8_t state);
#endif
