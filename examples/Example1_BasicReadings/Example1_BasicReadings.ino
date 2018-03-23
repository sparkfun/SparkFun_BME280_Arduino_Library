/*
  Get basic environmental readings from the BME280
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 9th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

  This example shows how to obtain humidity, pressure, and current temperature from the BME280. It
  assumes an I2C connection.

  Resources:
  Uses Wire.h for I2C operation
  Uses SPI.h for SPI operation

  Development environment specifics:
  Arduino IDE 1.8.5

*/

#include "Wire.h"

#include "SparkFunBME280.h"
BME280 mySensor; //Global sensor object

void setup()
{
  Serial.begin(9600);
  while(!Serial); //Wait for terminal to open
  Serial.println("Reading basic values from BME280");

  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The chip did not respond. Please check wiring.");
    while(1); //Freeze
  }
}

void loop()
{
  Serial.print("Humidity: ");
  Serial.print(mySensor.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");
  Serial.print(mySensor.readFloatPressure(), 0);

  Serial.print(" Alt: ");
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor.readFloatAltitudeFeet(), 1);

  Serial.print(" Temp: ");
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensor.readTempF(), 2);

  Serial.println();

  delay(50);
}
