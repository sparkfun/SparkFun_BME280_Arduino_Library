/*
  Communicate with BME280s with different I2C addresses
  Nathan Seidle @ SparkFun Electronics
  March 23, 2015

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14348 - Qwiic Combo Board
  https://www.sparkfun.com/products/13676 - BME280 Breakout Board

  This example shows how to connect two sensors on the same I2C bus.

  The BME280 has two I2C addresses: 0x77 (jumper open) or 0x76 (jumper closed)

  Hardware connections:
  BME280 -> Arduino
  GND -> GND
  3.3 -> 3.3
  SDA -> A4
  SCL -> A5
*/

#include <Wire.h>

#include "SparkFunBME280.h"
SparkFunBME280I2C mySensorA; //Uses default I2C address 0x77
SparkFunBME280I2C mySensorB; //Uses I2C address 0x76 (jumper closed)

void setup()
{
  Serial.begin(115200);
  Serial.println("Example showing alternate I2C addresses");

  Wire.begin();

  //If you close the jumper it is 0x76

  if(mySensorA.begin(0x77) == false) Serial.println("Sensor A connect failed");

  if(mySensorB.begin(0x76) == false) Serial.println("Sensor B connect failed");
}

void loop()
{
  Serial.print("HumidityA: ");
  Serial.print(mySensorA.readFloatHumidity(), 0);

  Serial.print(" PressureA: ");
  Serial.print(mySensorA.readFloatPressure(), 0);

  Serial.print(" TempA: ");
  //Serial.print(mySensorA.readTempC(), 2);
  Serial.print(mySensorA.readTempF(), 2);

  Serial.print(" HumidityB: ");
  Serial.print(mySensorB.readFloatHumidity(), 0);

  Serial.print(" PressureB: ");
  Serial.print(mySensorB.readFloatPressure(), 0);

  Serial.print(" TempB: ");
  //Serial.print(mySensorB.readTempC(), 2);
  Serial.print(mySensorB.readTempF(), 2);

  Serial.println();

  delay(50);
}

