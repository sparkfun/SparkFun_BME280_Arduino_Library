/******************************************************************************
SparkFunBME280.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
//See SparkFunBME280.h for additional topology notes.

#include "SparkFunBME280.h"
#include "stdint.h"
#include <math.h>

#include "Wire.h"
#include "SPI.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
BME280::BME280( void )
{
	//Construct with these default settings if nothing is specified

	//Select interface mode
	settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE

	//Select address for I2C.  Does nothing for SPI
	settings.I2CAddress = 0x77; //Ignored for SPI_MODE

	//Select CS pin for SPI.  Does nothing for I2C
	settings.chipSelectPin = 10;

	settings.runMode = 0;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BME280::begin()
{
	delay(2);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

	//Check the settings structure values to determine how to setup the device
	switch (settings.commInterface)
	{

	case I2C_MODE:
		if(_i2cPort != 0) _i2cPort->begin();
		break;

	case SPI_MODE:
		// start the SPI library:
		SPI.begin();
		#ifdef ARDUINO_ARCH_ESP32
		SPI.setFrequency(1000000);
		// Data is read and written MSb first.
		SPI.setBitOrder(SPI_MSBFIRST);
		// Like the standard arduino/teensy comment below, mode0 seems wrong according to standards
		// but conforms to the timing diagrams when used for the ESP32
		SPI.setDataMode(SPI_MODE0);
		#else
		// Maximum SPI frequency is 10MHz, could divide by 2 here:
		SPI.setClockDivider(SPI_CLOCK_DIV32);
		// Data is read and written MSb first.
		SPI.setBitOrder(MSBFIRST);
		// Data is captured on rising edge of clock (CPHA = 0)
		// Base value of the clock is HIGH (CPOL = 1)
		// This was SPI_MODE3 for RedBoard, but I had to change to
		// MODE0 for Teensy 3.1 operation
		SPI.setDataMode(SPI_MODE3);
		#endif
		// initalize the  data ready and chip select pins:
		pinMode(settings.chipSelectPin, OUTPUT);
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((int8_t)readRegister(BME280_DIG_H6_REG));

	settings.runMode = 3; //  3, Normal mode
	settings.tStandby = 0; //  0, 0.5ms
	settings.filter = 0; //  0, filter off

	setPressureOverSample(1); //Default
	setHumidityOverSample(1); //Default
	setTempOverSample(1); //Default
	
	setMode(MODE_NORMAL); //Go!
	
	return(readRegister(BME280_CHIP_ID_REG)); //Should return 0x60
}

//Begin comm with BME280 over I2C
bool BME280::beginI2C(TwoWire &wirePort)
{
	_i2cPort = &wirePort;
	_i2cPort->begin(); //The caller can begin their port and set the speed. We just confirm it here otherwise it can be hard to debug.
	
	settings.commInterface = I2C_MODE;
	//settings.I2CAddress = 0x77; //We assume user has set the I2C address using setI2CAddress()
	
	if(begin() == 0x60) return(true); //Begin normal init with these settings. Should return chip ID of 0x60
	return(false);
}

//Set the mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BME280::setMode(uint8_t mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
}

//Gets the current mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
uint8_t BME280::getMode()
{
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	return(controlData & 0b11111100); //Mask out all but bits 1 and 0
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void BME280::setTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid over sampling values
void BME280::setPressureOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid over sampling values
void BME280::setHumidityOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_HUMIDITY_REG);
	controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	writeRegister(BME280_CTRL_HUMIDITY_REG, controlData);

	setMode(originalMode); //Return to the original user's choice
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the humidty, pressure, and temp oversample functions
uint8_t BME280::checkSampleValue(uint8_t userValue)
{
	switch(userValue) 
	{
		case(0): break; //Valid
		case(1): break; //Valid
		case(2): break; //Valid
		case(4): break; //Valid
		case(8): break; //Valid
		case(16): break; //Valid
		default: 
			userValue = 1; //Default to 1x
			break; //Good
	}
	return(userValue);	
}

//Set the global setting for the I2C address we want to communicate with
//Default is 0x77
void BME280::setI2CAddress(uint8_t address)
{
	settings.I2CAddress = address; //Set the I2C address for this device
}

//Strictly resets.  Run .begin() afterwards
void BME280::reset( void )
{
	writeRegister(BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float BME280::readFloatPressure( void )
{

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	int32_t adc_P = ((uint32_t)readRegister(BME280_PRESSURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_PRESSURE_LSB_REG) << 4) | ((readRegister(BME280_PRESSURE_XLSB_REG) >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	return (float)p_acc / 256.0;
	
}

//Sets the internal variable _referencePressure so the 
void BME280::setReferencePressure(float refPressure)
{
	_referencePressure = rePressure;
}

//Return the local reference pressure
float BME280::getReferencePressure()
{
	return(_referencePressure);
}



float BME280::readFloatAltitudeMeters( void )
{
	float heightOutput = 0;
	
	heightOutput = ((float)-45846.2)*(pow(((float)readFloatPressure()/(float)_referencePressure), 0.190263) - (float)1);
	return heightOutput;
	
}

float BME280::readFloatAltitudeFeet( void )
{
	float heightOutput = 0;
	
	heightOutput = readFloatAltitudeMeters() * 3.28084;
	return heightOutput;
	
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280::readFloatHumidity( void )
{
	
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
	int32_t adc_H = ((uint32_t)readRegister(BME280_HUMIDITY_MSB_REG) << 8) | ((uint32_t)readRegister(BME280_HUMIDITY_LSB_REG));
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;

}



//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

float BME280::readTempC( void )
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
	int32_t adc_T = ((uint32_t)readRegister(BME280_TEMPERATURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_TEMPERATURE_LSB_REG) << 4) | ((readRegister(BME280_TEMPERATURE_XLSB_REG) >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100;
	
	return output;
}

float BME280::readTempF( void )
{
	float output = readTempC();
	output = (output * 9) / 5 + 32;

	return output;
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void BME280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	switch (settings.commInterface)
	{

	case I2C_MODE:
		_i2cPort->beginTransmission(settings.I2CAddress);
		_i2cPort->write(offset);
		_i2cPort->endTransmission();

		// request bytes from slave device
		_i2cPort->requestFrom(settings.I2CAddress, length);
		while ( (_i2cPort->available()) && (i < length))  // slave may send less than requested
		{
			c = _i2cPort->read(); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		while ( i < length ) // slave may send less than requested
		{
			c = SPI.transfer(0x00); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}

}

uint8_t BME280::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	switch (settings.commInterface) {

	case I2C_MODE:
		_i2cPort->beginTransmission(settings.I2CAddress);
		_i2cPort->write(offset);
		_i2cPort->endTransmission();

		_i2cPort->requestFrom(settings.I2CAddress, numBytes);
		while ( _i2cPort->available() ) // slave may send less than requested
		{
			result = _i2cPort->read(); // receive a byte as a proper uint8_t
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		result = SPI.transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
	return result;
}

int16_t BME280::readRegisterInt16( uint8_t offset )
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	return output;
}

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	switch (settings.commInterface)
	{
	case I2C_MODE:
		//Write the byte
		_i2cPort->beginTransmission(settings.I2CAddress);
		_i2cPort->write(offset);
		_i2cPort->write(dataToWrite);
		_i2cPort->endTransmission();
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset & 0x7F);
		// send a value of 0 to read the first byte returned:
		SPI.transfer(dataToWrite);
		// decrement the number of bytes left to read:
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
}
