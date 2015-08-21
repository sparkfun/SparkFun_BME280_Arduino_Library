/******************************************************************************
<filename>
<Title>
<name @ SparkFun Electronics>
<original creation date>
<github repository address>
<multiline verbose description of file functionality>

This file reads and writes arbitrary values to the I2C bus, targeting the BME280



Resources:
<additional library requirements>
Development environment specifics:
<arduino/development environment version>
<hardware version>
<etc>
This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include <stdint.h>
#include "SparkFunBME280.h"

#include "Wire.h"
#include "SPI.h"

//Global sensor object
BME280 mySensor;


float t_fine;

void setup()
{
	//Tell the driver what sensor modes to use here!
	
	//commInterface can be I2C_MODE or SPI_MODE
	mySensor.settings.commInterface = I2C_MODE;
	
	//Specify chipSelectPin using arduino pin names
	mySensor.settings.chipSelectPin = 10;
	
	//Specify I2C address.  Can be 0x77(default) or 0x76
	mySensor.settings.I2CAddress = 0x77;
	
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
	mySensor.settings.filter = 0;
	
	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;
	
	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;
	
	Serial.begin(57600);
	Serial.print("Program Started\n");
	Serial.print("Starting BME280... result of .begin(): 0x");
	
	//Calling .begin() causes the settings to be loaded
	Serial.println(mySensor.begin(), HEX);

//Debug chaff
	/*
	Serial.print("ID(0xD0): 0x");
	Serial.println(mySensor.readRegister(0xD0), HEX);
	Serial.print("Reset register(0xE0): 0x");
	Serial.println(mySensor.readRegister(0xE0), HEX);
	Serial.print("ctrl_meas(0xF4): 0x");
	Serial.println(mySensor.readRegister(0xF4), HEX);
	Serial.print("ctrl_hum(0xF2): 0x");
	Serial.println(mySensor.readRegister(0xF2), HEX);
		
	Serial.println();

	Serial.print("\nDisplaying concatenated calibration words\n");
	Serial.print("dig_T1, uint16: ");
	Serial.println(mySensor.calibration.dig_T1);
	Serial.print("dig_T2, int16: ");
	Serial.println(mySensor.calibration.dig_T2);
	Serial.print("dig_T3, int16: ");
	Serial.println(mySensor.calibration.dig_T3);
	
	Serial.print("dig_P1, uint16: ");
	Serial.println(mySensor.calibration.dig_P1);
	Serial.print("dig_P2, int16: ");
	Serial.println(mySensor.calibration.dig_P2);
	Serial.print("dig_P3, int16: ");
	Serial.println(mySensor.calibration.dig_P3);
	Serial.print("dig_P4, int16: ");
	Serial.println(mySensor.calibration.dig_P4);
	Serial.print("dig_P5, int16: ");
	Serial.println(mySensor.calibration.dig_P5);
	Serial.print("dig_P6, int16: ");
	Serial.println(mySensor.calibration.dig_P6);
	Serial.print("dig_P7, int16: ");
	Serial.println(mySensor.calibration.dig_P7);
	Serial.print("dig_P8, int16: ");
	Serial.println(mySensor.calibration.dig_P8);
	Serial.print("dig_P9, int16: ");
	Serial.println(mySensor.calibration.dig_P9);
	
	Serial.print("dig_H1, uint8: ");
	Serial.println(mySensor.calibration.dig_H1);
	Serial.print("dig_H2, int16: ");
	Serial.println(mySensor.calibration.dig_H2);
	Serial.print("dig_H3, uint8: ");
	Serial.println(mySensor.calibration.dig_H3);
	Serial.print("dig_H4, int16: ");
	Serial.println(mySensor.calibration.dig_H4);
	Serial.print("dig_H5, int16: ");
	Serial.println(mySensor.calibration.dig_H5);
	Serial.print("dig_H6, uint8: ");
	Serial.println(mySensor.calibration.dig_H6);
	*/
}

void loop()
{
//Debug chaff
	/*
	Serial.print("ID: 0x");
	Serial.println(mySensor.readRegister(0xD0), HEX);
	Serial.print("Status: 0x");
	Serial.println(mySensor.readRegister(0xF3), HEX);
	Serial.print("tempMSB: 0x");
	Serial.println(mySensor.readRegister(BME280_TEMPERATURE_MSB_REG), HEX);
	Serial.print("tempLSB: 0x");
	Serial.println(mySensor.readRegister(BME280_TEMPERATURE_LSB_REG), HEX);
	Serial.print("tempXLSB: 0x");
	Serial.println(mySensor.readRegister(BME280_TEMPERATURE_XLSB_REG), HEX);
	*/

	Serial.print("Temperature: ");
	Serial.print(mySensor.readTempC(), 2);
	Serial.println(" degrees C");

	Serial.print("Temperature: ");
	Serial.print(mySensor.readTempF(), 2);
	Serial.println(" degrees F");

	Serial.print("Pressure: ");
	Serial.print(mySensor.readFloatPressure(), 2);
	Serial.println(" Pa");

	Serial.print("Altitude: ");
	Serial.print(mySensor.readFloatAltitudeMeters(), 2);
	Serial.println("m");

	Serial.print("Altitude: ");
	Serial.print(mySensor.readFloatAltitudeFeet(), 2);
	Serial.println("ft");	

	Serial.print("%RH: ");
	Serial.print(mySensor.readFloatHumidity(), 2);
	Serial.println(" %");
	
	Serial.println();
	
	delay(1000);

}
