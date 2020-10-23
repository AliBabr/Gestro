#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

#include <Adafruit_MLX90614.h>
#include "thermometer.h"
#include  "GastroMarket.h"

//IRTherm therm; // Create an IRTherm object to interact with throughout
#define BODY_EMISIVITY 0.98 //Body emisivity of human skin

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void thermometer_setup() {
  //therm.begin(); // Initialize thermal IR sensor
 // therm.setUnit(TEMP_C); // Set the library's units to Farenheit

  delay(100);
  // Call setEmissivity() to configure the MLX90614's
  // emissivity compensation:
 // therm.setEmissivity(BODY_EMISIVITY);
  delay(100);
   mlx.begin(); 
}

double tempObjectC = 0;
double tempAmbientC = 0;

void measure_temperature() {

//  if (therm.read()) // On success, read() will return 1, on fail 0.
//  {
//    tempObjectC = therm.object() - (3.21 - 3) * 0.6; //supply voltage compensation
//    object_temperature = uint16_t(tempObjectC * 10);
//
//
//    tempAmbientC = therm.ambient() - (3.21 - 3) * 0.6; //supply voltage compensation
//    tempAmbientC = tempAmbientC;
//    ambient_temperature = byte(tempAmbientC);
//
//  }


    tempObjectC = mlx.readObjectTempC() - (3.21 - 3) * 0.6; //supply voltage compensation according datasheet of MLX90614(3.21V measured on voltage stabilisator on thermometer module)
    object_temperature = uint16_t(tempObjectC * 10);


    tempAmbientC = mlx.readAmbientTempC() - (3.21 - 3) * 0.6; //supply voltage compensation according datasheet of MLX90614(3.21V measured on voltage stabilisator on thermometer module)
    tempAmbientC = tempAmbientC;
    ambient_temperature = byte(tempAmbientC);


}
