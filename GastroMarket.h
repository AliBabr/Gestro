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

#define LED_56_KHZ_HAND 6
#define SENSOR_56_KHZ_HAND A4
#define BLUE_LED 5

#define PIR_SENSOR 7

#define SENSOR_TIMEOUT 500000   //SENSOR_TIMEOUT must be < DELAY_BETWEEN_PERSONS
#define DELAY_BETWEEN_PERSONS 1000000

#define SENDING_FREQUENCY_HZ 4

#define REL_1 4
#define REL_2 7
#define REL_3 8
#define REL_4 12 

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
extern uint16_t DesinfectantDose;
extern byte RunPumpVentilation;
#endif
