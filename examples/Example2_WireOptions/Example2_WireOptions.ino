/******************************************************************************
  Read all the regigsters of the BME280
  BME280 Arduino and Teensy example
  Marshall Taylor @ SparkFun Electronics
  May 20, 2015

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14348 - Qwiic Combo Board
  https://www.sparkfun.com/products/13676 - BME280 Breakout Board

  This example shows how to connect to different I2C address and push to different Wire ports.

  The BME280 has two I2C addresses: 0x77 (jumper open) or 0x76 (jumper closed)

  This sketch configures the BME280 to read all measurements.  The sketch also
  displays the BME280's physical memory and what the driver perceives the
  calibration words to be.

*/

#include "Wire.h"

#include "SparkFunBME280.h"
BME280 mySensorA; //Global sensor object

void setup()
{
  Serial.begin(9600);
  Serial.println("Example showing alternate I2C addresses");

  //Other I2C ports are available but it depends on your platform. 
  //For example: Wire1, Wire2, softWire, etc

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  //Wire1.begin();
  //Wire1.setClock(400000);

  mySensorA.setI2CAddress(0x77); //The default for the SparkFun Environmental Combo board is 0x77 (jumper open).
  //If you close the jumper it is 0x76
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  mySensorA.beginI2C(Wire); //Connect to this sensor using the Wire port

  //mySensorB.setI2CAddress(0x76); //Connect to a second sensor
  //mySensorB.beginI2C(Wire1); //Connect to this sensor using the Wire1 port
}

void loop()
{
  Serial.print("Humidity: ");
  Serial.print(mySensorA.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");
  Serial.print(mySensorA.readFloatPressure(), 0);

  Serial.print(" Alt: ");
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensorA.readFloatAltitudeFeet(), 1);

  Serial.print(" Temp: ");
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensorA.readTempF(), 2);

  //Serial.print(" TempB: ");
  //Serial.print(mySensorB.readTempF(), 2);

  Serial.println();

  delay(50);
}

