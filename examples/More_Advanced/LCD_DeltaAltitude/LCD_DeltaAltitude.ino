/******************************************************************************
LCD_DeltaAltitude.ino
BME280 Arduino and Teensy example
Marshall Taylor @ SparkFun Electronics
Oct 19, 2015
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

This sketch configures a BME280 to measure height changes, with a button
to zero.

Hardware connections:
BME280 -> Arduino
GND -> GND
3.3 -> 3.3
SDA -> A4
SCL -> A5

Button (Normally open) between A4 and A5.

LCD:
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital 6
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

To use, run the sketch, then press and hold the button until the data on the
serial monitor shows zero.  Then, move the sensor to a new altitude and watch
how the output changes!

Note: For most accurate results keep the sensor orientation and temperature the
same between measurements.

Resources:
Uses Wire.h for I2C operation
Uses SPI.h for SPI operation
Included CircularBuffer class for averaging

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include <stdint.h>
#include "SparkFunBME280.h"
#include "CircularBuffer.h"

#include "Wire.h"
#include "SPI.h"
#include <LiquidCrystal.h>

//Global sensor object
SparkFunBME280I2C mySensor;
uint8_t buttonGroundPin = A0;
uint8_t buttonSensePin = A2;

CircularBuffer myBuffer(50);
//int icounter = 0;
float reference = 0;

LiquidCrystal lcd(7,6,5,4,3,2);

void setup()
{
	//Configure the button IO
	pinMode( buttonSensePin, INPUT_PULLUP );
	pinMode( buttonGroundPin, OUTPUT );
	digitalWrite( buttonGroundPin, 0 );
	
	//***Driver settings********************************//
	//The I2C address is passed to .begin(). Can be 0x77(default) or 0x76
	//For SPI, use the SparkFunBME280SPI class and pass the chip select pin to .begin()


	//***Operation settings*****************************//
	
	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	mySensor.settings.runMode = 3; //Forced mode
	
	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	mySensor.settings.tStandby = 0;
	
	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	mySensor.settings.filter = 4;
	
	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 5;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 5;
	
	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 0;
	
	Serial.begin(57600);
	Serial.print("Program Started\n");
	Wire.begin();
	Serial.print("Starting BME280... chip ID: 0x");
	//Calling .begin() causes the settings to be loaded
	delay(10);//Needs > 2 ms to start up
	mySensor.begin(0x77);
	Serial.println(mySensor.chipID(), HEX);

	Serial.println();
	
	lcd.begin(16, 2);
	
	lcd.clear();

}

void loop()
{
	//Get the local temperature!  Do this for calibration
	mySensor.readTempC();
	
	//Check button.  No debounce, so hold for operation
	if( digitalRead( buttonSensePin ) == 0 )
	{
		//Set reference altitude.
		reference = mySensor.readFloatAltitudeFeet();
	}
	
	float lastAlt = mySensor.readFloatAltitudeFeet() - reference;
	myBuffer.pushElement(lastAlt);
	float tempAlt = myBuffer.averageLast(8); //Do an average of the latest samples
	
	lcd.setCursor(0,0);
	lcd.print("Altitude:       ");
	
	myBuffer.pushElement(lastAlt);
	
	lcd.setCursor(0,1);
	if(tempAlt >= 0)
	{
		lcd.print(" ");
	}
	lcd.print(tempAlt);
	lcd.print(" ft       ");
	delay(200);

}
