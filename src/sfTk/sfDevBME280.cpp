/**
 * @file sfDevBME280.cpp
 * @brief Implementation file for the platform independent SparkFun BME280 device driver
 *
 * Originally written by Marshall Taylor @ SparkFun Electronics, May 20, 2015
 *
 * @author SparkFun Electronics
 * @date 2015-2025
 * @copyright Copyright (c) 2015-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#include "sfDevBME280.h"
#include <math.h>

// Number of bytes in the measurement burst read - pressure (3), temperature (3), humidity (2)
#define kBME280MeasurementBurstSize 8

// Calibration data is stored in two blocks: 0x88 to 0xA1 and 0xE1 to 0xE7
#define kBME280CalibBlock1Start BME280_DIG_T1_LSB_REG
#define kBME280CalibBlock1Size 26
#define kBME280CalibBlock2Start BME280_DIG_H2_LSB_REG
#define kBME280CalibBlock2Size 7

// Value written to the reset register to perform a soft reset
#define kBME280SoftResetValue 0xB6

// The measuring bit in the status register
#define kBME280StatusMeasuring (1 << 3)

// In SPI mode, bit 7 of the register address is the read/write flag - it's cleared for a write
#define kBME280SPIWriteMask 0x7F
#define kBME280I2CWriteMask 0xFF

//--------------------------------------------------------------------------------------------------
// Constructor -- Specifies the default configuration
//
sfDevBME280::sfDevBME280()
    : t_fine{0}, _theBus{nullptr}, _regWriteMask{kBME280I2CWriteMask}, _referencePressure{101325.0}
{
    // These are deprecated settings
    settings.runMode = MODE_NORMAL;
    settings.tStandby = 0; // 0.5ms
    settings.filter = 0;   // Filter off
    settings.tempOverSample = 1;
    settings.pressOverSample = 1;
    settings.humidOverSample = 1;
    settings.tempCorrection = 0.f; // correction of temperature - added to the result
}

//--------------------------------------------------------------------------------------------------
// Begin the BME280 device. Requires a bus object to communicate with the device.
//
sfTkError_t sfDevBME280::begin(sfTkIBus *theBus)
{
    if (theBus == nullptr)
        return ksfTkErrBusNotInit;

    _theBus = theBus;
    _regWriteMask = theBus->type() == ksfTkBusTypeSPI ? kBME280SPIWriteMask : kBME280I2CWriteMask;

    sftk_delay_ms(2); // Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

    // Check communication with IC before anything else
    if (!isConnected())
        return ksfTkErrFail;

    sfTkError_t rc = readCalibration();
    if (rc != ksfTkErrOk)
        return rc;

    // Most of the time the sensor will be init with default values
    // But in case user has old/deprecated code, use the settings.x values
    setStandbyTime(settings.tStandby);
    setFilter(settings.filter);
    setPressureOverSample(settings.pressOverSample); // Default of 1x oversample
    setHumidityOverSample(settings.humidOverSample); // Default of 1x oversample
    setTempOverSample(settings.tempOverSample);      // Default of 1x oversample

    return setMode(MODE_NORMAL); // Go!
}

//--------------------------------------------------------------------------------------------------
// Read all the compensation data from the device - range 0x88:A1, 0xE1:E7
//
sfTkError_t sfDevBME280::readCalibration(void)
{
    uint8_t buf1[kBME280CalibBlock1Size];
    uint8_t buf2[kBME280CalibBlock2Size];
    size_t nRead;

    sfTkError_t rc = _theBus->readRegister((uint8_t)kBME280CalibBlock1Start, buf1, sizeof(buf1), nRead);
    if (rc != ksfTkErrOk)
        return rc;

    rc = _theBus->readRegister((uint8_t)kBME280CalibBlock2Start, buf2, sizeof(buf2), nRead);
    if (rc != ksfTkErrOk)
        return rc;

    calibration.dig_T1 = (uint16_t)((buf1[1] << 8) | buf1[0]);
    calibration.dig_T2 = (int16_t)((buf1[3] << 8) | buf1[2]);
    calibration.dig_T3 = (int16_t)((buf1[5] << 8) | buf1[4]);

    calibration.dig_P1 = (uint16_t)((buf1[7] << 8) | buf1[6]);
    calibration.dig_P2 = (int16_t)((buf1[9] << 8) | buf1[8]);
    calibration.dig_P3 = (int16_t)((buf1[11] << 8) | buf1[10]);
    calibration.dig_P4 = (int16_t)((buf1[13] << 8) | buf1[12]);
    calibration.dig_P5 = (int16_t)((buf1[15] << 8) | buf1[14]);
    calibration.dig_P6 = (int16_t)((buf1[17] << 8) | buf1[16]);
    calibration.dig_P7 = (int16_t)((buf1[19] << 8) | buf1[18]);
    calibration.dig_P8 = (int16_t)((buf1[21] << 8) | buf1[20]);
    calibration.dig_P9 = (int16_t)((buf1[23] << 8) | buf1[22]);

    calibration.dig_H1 = buf1[25];
    calibration.dig_H2 = (int16_t)((buf2[1] << 8) | buf2[0]);
    calibration.dig_H3 = buf2[2];
    // H4 and H5 are signed 12 bit values - the MSB register holds the signed upper 8 bits
    calibration.dig_H4 = (int16_t)(((int16_t)(int8_t)buf2[3] * 16) | (buf2[4] & 0x0F));
    calibration.dig_H5 = (int16_t)(((int16_t)(int8_t)buf2[5] * 16) | ((buf2[4] >> 4) & 0x0F));
    calibration.dig_H6 = (int8_t)buf2[6];

    return ksfTkErrOk;
}

//--------------------------------------------------------------------------------------------------
// Return the chip ID of the device. 0 on error
//
uint8_t sfDevBME280::chipID(void)
{
    uint8_t id;
    if (_theBus == nullptr || _theBus->readRegister((uint8_t)BME280_CHIP_ID_REG, id) != ksfTkErrOk)
        return 0;

    return id;
}

//--------------------------------------------------------------------------------------------------
// Is a BME280 or BMP280 connected?
//
bool sfDevBME280::isConnected(void)
{
    uint8_t id = chipID();

    return id == kBME280ChipID || id == kBMP280ChipID;
}

//--------------------------------------------------------------------------------------------------
// Helper - read/modify/write the bits in mask of the given register.
//
sfTkError_t sfDevBME280::updateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    uint8_t regValue;
    sfTkError_t rc = _theBus->readRegister(reg, regValue);
    if (rc != ksfTkErrOk)
        return rc;

    regValue = (regValue & ~mask) | (value & mask);

    return _theBus->writeRegister((uint8_t)(reg & _regWriteMask), regValue);
}

//--------------------------------------------------------------------------------------------------
// Set the mode bits in the ctrl_meas register
//  Mode 00 = Sleep
//  01 and 10 = Forced
//  11 = Normal mode
//
sfTkError_t sfDevBME280::setMode(uint8_t mode)
{
    if (mode > 0b11)
        mode = MODE_SLEEP; // Error check. Default to sleep mode

    return updateRegisterBits(BME280_CTRL_MEAS_REG, 0b00000011, mode);
}

//--------------------------------------------------------------------------------------------------
// Gets the current mode bits in the ctrl_meas register
//
uint8_t sfDevBME280::getMode(void)
{
    uint8_t controlData;
    if (_theBus == nullptr || _theBus->readRegister((uint8_t)BME280_CTRL_MEAS_REG, controlData) != ksfTkErrOk)
        return MODE_SLEEP;

    return (controlData & 0b00000011); // Clear bits 7 through 2
}

//--------------------------------------------------------------------------------------------------
// Set the standby bits in the config register
// tStandby can be:
//   0, 0.5ms
//   1, 62.5ms
//   2, 125ms
//   3, 250ms
//   4, 500ms
//   5, 1000ms
//   6, 10ms
//   7, 20ms
//
sfTkError_t sfDevBME280::setStandbyTime(uint8_t timeSetting)
{
    if (timeSetting > 0b111)
        timeSetting = 0; // Error check. Default to 0.5ms

    return updateRegisterBits(BME280_CONFIG_REG, 0b11100000, timeSetting << 5); // Align with bits 7/6/5
}

//--------------------------------------------------------------------------------------------------
// Set the filter bits in the config register
// filter can be off or number of FIR coefficients to use:
//   0, filter off
//   1, coefficients = 2
//   2, coefficients = 4
//   3, coefficients = 8
//   4, coefficients = 16
//
sfTkError_t sfDevBME280::setFilter(uint8_t filterSetting)
{
    if (filterSetting > 0b111)
        filterSetting = 0; // Error check. Default to filter off

    return updateRegisterBits(BME280_CONFIG_REG, 0b00011100, filterSetting << 2); // Align with bits 4/3/2
}

//--------------------------------------------------------------------------------------------------
// Helper - set an over sample value. The config is only writeable in sleep mode, so the device is placed
// in sleep mode, the value set, and the original mode restored.
//
sfTkError_t sfDevBME280::setOverSample(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t overSampleAmount)
{
    overSampleAmount = checkSampleValue(overSampleAmount); // Error check

    uint8_t originalMode = getMode(); // Get the current mode so we can go back to it at the end

    sfTkError_t rc = setMode(MODE_SLEEP);
    if (rc != ksfTkErrOk)
        return rc;

    rc = updateRegisterBits(reg, mask, overSampleAmount << shift);

    // Return to the original user's choice - even if the update failed
    sfTkError_t rcMode = setMode(originalMode);

    return rc != ksfTkErrOk ? rc : rcMode;
}

//--------------------------------------------------------------------------------------------------
// Set the temperature oversample value
// 0 turns off temp sensing
// 1 to 16 are valid over sampling values
//
sfTkError_t sfDevBME280::setTempOverSample(uint8_t overSampleAmount)
{
    // osrs_t - bits 7, 6, 5
    return setOverSample(BME280_CTRL_MEAS_REG, 0b11100000, 5, overSampleAmount);
}

//--------------------------------------------------------------------------------------------------
// Set the pressure oversample value
// 0 turns off pressure sensing
// 1 to 16 are valid over sampling values
//
sfTkError_t sfDevBME280::setPressureOverSample(uint8_t overSampleAmount)
{
    // osrs_p - bits 4, 3, 2
    return setOverSample(BME280_CTRL_MEAS_REG, 0b00011100, 2, overSampleAmount);
}

//--------------------------------------------------------------------------------------------------
// Set the humidity oversample value
// 0 turns off humidity sensing
// 1 to 16 are valid over sampling values
//
sfTkError_t sfDevBME280::setHumidityOverSample(uint8_t overSampleAmount)
{
    // osrs_h - bits 2, 1, 0
    return setOverSample(BME280_CTRL_HUMIDITY_REG, 0b00000111, 0, overSampleAmount);
}

//--------------------------------------------------------------------------------------------------
// Validates an over sample value
// Allowed values are 0 to 16
// These are used in the humidty, pressure, and temp oversample functions
//
uint8_t sfDevBME280::checkSampleValue(uint8_t userValue)
{
    switch (userValue)
    {
    case 0:
        return 0;
    case 1:
        return 1;
    case 2:
        return 2;
    case 4:
        return 3;
    case 8:
        return 4;
    case 16:
        return 5;
    default:
        return 1; // Default to 1x
    }
}

//--------------------------------------------------------------------------------------------------
// Check the measuring bit and return true while device is taking measurement
//
bool sfDevBME280::isMeasuring(void)
{
    uint8_t stat;
    if (_theBus == nullptr || _theBus->readRegister((uint8_t)BME280_STAT_REG, stat) != ksfTkErrOk)
        return false;

    return (stat & kBME280StatusMeasuring) != 0;
}

//--------------------------------------------------------------------------------------------------
// Strictly resets.  Run .begin() afterwards
//
sfTkError_t sfDevBME280::reset(void)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    return _theBus->writeRegister((uint8_t)(BME280_RST_REG & _regWriteMask), (uint8_t)kBME280SoftResetValue);
}

//--------------------------------------------------------------------------------------------------
// Read all sensor registers as a burst. See BME280 Datasheet section 4. Data readout
// tempScale = 0 for Celsius scale (default setting)
// tempScale = 1 for Fahrenheit scale
//
sfTkError_t sfDevBME280::readAllMeasurements(BME280_SensorMeasurements *measurements, uint8_t tempScale)
{
    if (measurements == nullptr)
        return ksfTkErrInvalidParam;

    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    uint8_t dataBurst[kBME280MeasurementBurstSize];
    size_t nRead;
    sfTkError_t rc = _theBus->readRegister((uint8_t)BME280_MEASUREMENTS_REG, dataBurst, sizeof(dataBurst), nRead);
    if (rc != ksfTkErrOk)
        return rc;

    // Temperature first - it sets t_fine, which is used by the pressure and humidity compensation
    float temperature = readTempFromBurst(dataBurst);
    measurements->temperature = tempScale == 0 ? temperature : (temperature * 9) / 5 + 32;

    readFloatPressureFromBurst(dataBurst, measurements);
    readFloatHumidityFromBurst(dataBurst, measurements);

    return ksfTkErrOk;
}

//--------------------------------------------------------------------------------------------------
// Pressure Section
//--------------------------------------------------------------------------------------------------

// Returns pressure in Pa. The calculation results in an unsigned 32 bit integer in Q24.8 format (24 integer bits
// and 8 fractional bits). Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
float sfDevBME280::compensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p_acc;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibration.dig_P5) << 17);
    var2 = var2 + (((int64_t)calibration.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calibration.dig_P3) >> 8) + ((var1 * (int64_t)calibration.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration.dig_P1) >> 33;
    if (var1 == 0)
        return 0; // avoid exception caused by division by zero

    p_acc = 1048576 - adc_P;
    p_acc = (((p_acc << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calibration.dig_P9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
    var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
    p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7) << 4);

    return (float)p_acc / 256.0;
}

sfTkError_t sfDevBME280::readFloatPressure(float &pressure)
{
    uint8_t buffer[3];
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    size_t nRead;
    sfTkError_t rc = _theBus->readRegister((uint8_t)BME280_PRESSURE_MSB_REG, buffer, sizeof(buffer), nRead);
    if (rc != ksfTkErrOk)
        return rc;

    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

    pressure = compensatePressure(adc_P);

    return ksfTkErrOk;
}

float sfDevBME280::readFloatPressure(void)
{
    float pressure;
    return readFloatPressure(pressure) == ksfTkErrOk ? pressure : NAN;
}

void sfDevBME280::readFloatPressureFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements)
{
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

    measurements->pressure = compensatePressure(adc_P);
}

//--------------------------------------------------------------------------------------------------
// Sets the internal variable _referencePressure so the altitude is calculated properly.
// This is also known as "sea level pressure" and is in Pascals. The value is probably
// within 10% of 101325. This varies based on the weather:
// https://en.wikipedia.org/wiki/Atmospheric_pressure#Mean_sea-level_pressure
//
// if you are concerned about accuracy or precision, make sure to pull the
// "sea level pressure"value from a trusted source like NOAA.
//
void sfDevBME280::setReferencePressure(float refPressure)
{
    _referencePressure = refPressure;
}

// Return the local reference pressure
float sfDevBME280::getReferencePressure(void)
{
    return _referencePressure;
}

float sfDevBME280::readFloatAltitudeMeters(void)
{
    // Getting height from a pressure reading is called the "international barometric height formula".
    // The magic value of 44330.77 was adjusted in issue #30.
    // There's also some discussion of it here: https://www.sparkfun.com/tutorials/253
    // This calculation is NOT designed to work on non-Earthlike planets such as Mars or Venus;
    // see NRLMSISE-00. That's why it is the "international" formula, not "interplanetary".
    // Sparkfun is not liable for incorrect altitude calculations from this
    // code on those planets. Interplanetary selfies are welcome, however.
    return ((float)-44330.77) *
           (pow(((float)readFloatPressure() / (float)_referencePressure), 0.190263) - (float)1); // Corrected, see issue 30
}

float sfDevBME280::readFloatAltitudeFeet(void)
{
    return readFloatAltitudeMeters() * 3.28084;
}

//--------------------------------------------------------------------------------------------------
// Humidity Section
//--------------------------------------------------------------------------------------------------

// Returns humidity in %RH. The calculation results in an unsigned 32 bit integer in Q22.10 format (22 integer and
// 10 fractional bits). Output value of “47445” represents 47445/1024 = 46.333 %RH
float sfDevBME280::compensateHumidity(int32_t adc_H)
{
    int32_t var1;
    var1 = (t_fine - ((int32_t)76800));
    var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
              ((int32_t)16384)) >>
             15) *
            (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) *
                 (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >>
                10) +
               ((int32_t)2097152)) *
                  ((int32_t)calibration.dig_H2) +
              8192) >>
             14));
    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);

    return (float)(var1 >> 12) / 1024.0;
}

sfTkError_t sfDevBME280::readFloatHumidity(float &humidity)
{
    uint8_t buffer[2];
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    size_t nRead;
    sfTkError_t rc = _theBus->readRegister((uint8_t)BME280_HUMIDITY_MSB_REG, buffer, sizeof(buffer), nRead);
    if (rc != ksfTkErrOk)
        return rc;

    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);

    humidity = compensateHumidity(adc_H);

    return ksfTkErrOk;
}

float sfDevBME280::readFloatHumidity(void)
{
    float humidity;
    return readFloatHumidity(humidity) == ksfTkErrOk ? humidity : NAN;
}

void sfDevBME280::readFloatHumidityFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements)
{
    int32_t adc_H = ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7]);

    measurements->humidity = compensateHumidity(adc_H);
}

//--------------------------------------------------------------------------------------------------
// Temperature Section
//--------------------------------------------------------------------------------------------------

void sfDevBME280::setTemperatureCorrection(float corr)
{
    settings.tempCorrection = corr;
}

// Returns temperature in DegC. The calculation has a resolution of 0.01 DegC - a value of “5123” equals 51.23 DegC.
// t_fine carries the fine temperature as a member value - used by the pressure and humidity compensation.
float sfDevBME280::compensateTemperature(int32_t adc_T)
{
    // By datasheet, calibrate
    int64_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)calibration.dig_T1 << 1))) * ((int32_t)calibration.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t)calibration.dig_T1))) >> 12) *
            ((int32_t)calibration.dig_T3)) >>
           14;
    t_fine = var1 + var2;
    float output = (t_fine * 5 + 128) >> 8;

    return output / 100 + settings.tempCorrection;
}

sfTkError_t sfDevBME280::readTempC(float &tempC)
{
    uint8_t buffer[3];
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    size_t nRead;
    sfTkError_t rc = _theBus->readRegister((uint8_t)BME280_TEMPERATURE_MSB_REG, buffer, sizeof(buffer), nRead);
    if (rc != ksfTkErrOk)
        return rc;

    int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

    tempC = compensateTemperature(adc_T);

    return ksfTkErrOk;
}

float sfDevBME280::readTempC(void)
{
    float tempC;
    return readTempC(tempC) == ksfTkErrOk ? tempC : NAN;
}

float sfDevBME280::readTempF(void)
{
    return (readTempC() * 9) / 5 + 32;
}

float sfDevBME280::readTempFromBurst(uint8_t buffer[])
{
    int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);

    return compensateTemperature(adc_T);
}

//--------------------------------------------------------------------------------------------------
// Dew point Section
//--------------------------------------------------------------------------------------------------

// Returns Dew point in DegC
double sfDevBME280::dewPointC(void)
{
    double celsius = readTempC();
    double humidity = readFloatHumidity();
    // (1) Saturation Vapor Pressure = ESGG(T)
    double RATIO = 373.15 / (273.15 + celsius);
    double RHS = -7.90298 * (RATIO - 1);
    RHS += 5.02808 * log10(RATIO);
    RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
    RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
    RHS += log10(1013.246);
    // factor -3 is to adjust units - Vapor Pressure SVP * humidity
    double VP = pow(10, RHS - 3) * humidity;
    // (2) DEWPOINT = F(Vapor Pressure)
    double T = log(VP / 0.61078); // temp var
    return (241.88 * T) / (17.558 - T);
}

// Returns Dew point in DegF
double sfDevBME280::dewPointF(void)
{
    return (dewPointC() * 1.8 + 32); // Convert C to F
}
