/**
 * @file sfDevBME280.h
 * @brief Header file for the platform independent SparkFun BME280 device driver
 *
 * This file contains the declaration of the sfDevBME280 class, which implements the operation of the
 * Bosch BME280 (and BMP280) temperature, pressure and humidity sensor. The class communicates with the
 * device using a SparkFun Toolkit bus object (sfTkIBus), making it independent of the bus type (I2C or SPI)
 * and of the platform it is running on.
 *
 * Originally written by Marshall Taylor @ SparkFun Electronics, May 20, 2015
 *
 * @author SparkFun Electronics
 * @date 2015-2025
 * @copyright Copyright (c) 2015-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

// include the sparkfun toolkit headers
#include <sfTk/sfToolkit.h>

// Bus interfaces
#include <sfTk/sfTkIBus.h>
#include <sfTk/sfTkISPI.h>

/** @brief Device mode - sleep. No measurements are performed */
#define MODE_SLEEP 0b00
/** @brief Device mode - forced. A single measurement is performed, then the device returns to sleep */
#define MODE_FORCED 0b01
/** @brief Device mode - normal. Measurements are performed continuously */
#define MODE_NORMAL 0b11

// Register names:
#define BME280_DIG_T1_LSB_REG 0x88
#define BME280_DIG_T1_MSB_REG 0x89
#define BME280_DIG_T2_LSB_REG 0x8A
#define BME280_DIG_T2_MSB_REG 0x8B
#define BME280_DIG_T3_LSB_REG 0x8C
#define BME280_DIG_T3_MSB_REG 0x8D
#define BME280_DIG_P1_LSB_REG 0x8E
#define BME280_DIG_P1_MSB_REG 0x8F
#define BME280_DIG_P2_LSB_REG 0x90
#define BME280_DIG_P2_MSB_REG 0x91
#define BME280_DIG_P3_LSB_REG 0x92
#define BME280_DIG_P3_MSB_REG 0x93
#define BME280_DIG_P4_LSB_REG 0x94
#define BME280_DIG_P4_MSB_REG 0x95
#define BME280_DIG_P5_LSB_REG 0x96
#define BME280_DIG_P5_MSB_REG 0x97
#define BME280_DIG_P6_LSB_REG 0x98
#define BME280_DIG_P6_MSB_REG 0x99
#define BME280_DIG_P7_LSB_REG 0x9A
#define BME280_DIG_P7_MSB_REG 0x9B
#define BME280_DIG_P8_LSB_REG 0x9C
#define BME280_DIG_P8_MSB_REG 0x9D
#define BME280_DIG_P9_LSB_REG 0x9E
#define BME280_DIG_P9_MSB_REG 0x9F
#define BME280_DIG_H1_REG 0xA1
#define BME280_CHIP_ID_REG 0xD0 // Chip ID
#define BME280_RST_REG 0xE0     // Softreset Reg
#define BME280_DIG_H2_LSB_REG 0xE1
#define BME280_DIG_H2_MSB_REG 0xE2
#define BME280_DIG_H3_REG 0xE3
#define BME280_DIG_H4_MSB_REG 0xE4
#define BME280_DIG_H4_LSB_REG 0xE5
#define BME280_DIG_H5_MSB_REG 0xE6
#define BME280_DIG_H6_REG 0xE7
#define BME280_CTRL_HUMIDITY_REG 0xF2    // Ctrl Humidity Reg
#define BME280_STAT_REG 0xF3             // Status Reg
#define BME280_CTRL_MEAS_REG 0xF4        // Ctrl Measure Reg
#define BME280_CONFIG_REG 0xF5           // Configuration Reg
#define BME280_MEASUREMENTS_REG 0xF7     // Measurements register start
#define BME280_PRESSURE_MSB_REG 0xF7     // Pressure MSB
#define BME280_PRESSURE_LSB_REG 0xF8     // Pressure LSB
#define BME280_PRESSURE_XLSB_REG 0xF9    // Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG 0xFA  // Temperature MSB
#define BME280_TEMPERATURE_LSB_REG 0xFB  // Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG 0xFC // Temperature XLSB
#define BME280_HUMIDITY_MSB_REG 0xFD     // Humidity MSB
#define BME280_HUMIDITY_LSB_REG 0xFE     // Humidity LSB

/** @brief Default I2C address of the BME280 (address jumper open) */
const uint8_t kBME280DefaultAddress = 0x77;

/** @brief Alternate I2C address of the BME280 (address jumper closed) */
const uint8_t kBME280AlternateAddress = 0x76;

/** @brief Chip ID returned by a BME280 */
const uint8_t kBME280ChipID = 0x60;

/** @brief Chip ID returned by a BMP280 */
const uint8_t kBMP280ChipID = 0x58;

/**
 * @brief Settings used by the driver.
 *
 * The application can set these values directly. The settings are sent to the sensor when begin() is called.
 * Calling the individual setter methods (setFilter(), setTempOverSample(), etc) after begin() is the preferred
 * method of changing the device configuration.
 */
struct BME280_SensorSettings
{
  public:
    /** Device run mode (deprecated - use setMode()) */
    uint8_t runMode;
    /** Standby time setting (deprecated - use setStandbyTime()) */
    uint8_t tStandby;
    /** IIR Filter setting (deprecated - use setFilter()) */
    uint8_t filter;
    /** Temperature over sample setting (deprecated - use setTempOverSample()) */
    uint8_t tempOverSample;
    /** Pressure over sample setting (deprecated - use setPressureOverSample()) */
    uint8_t pressOverSample;
    /** Humidity over sample setting (deprecated - use setHumidityOverSample()) */
    uint8_t humidOverSample;
    /** Correction of temperature - added to the result */
    float tempCorrection;
};

/**
 * @brief Calibration constants read from the device. These are used by the driver to compensate the raw
 * measurements.
 */
struct SensorCalibration
{
  public:
    uint16_t dig_T1; ///< Temperature compensation value T1
    int16_t dig_T2;  ///< Temperature compensation value T2
    int16_t dig_T3;  ///< Temperature compensation value T3

    uint16_t dig_P1; ///< Pressure compensation value P1
    int16_t dig_P2;  ///< Pressure compensation value P2
    int16_t dig_P3;  ///< Pressure compensation value P3
    int16_t dig_P4;  ///< Pressure compensation value P4
    int16_t dig_P5;  ///< Pressure compensation value P5
    int16_t dig_P6;  ///< Pressure compensation value P6
    int16_t dig_P7;  ///< Pressure compensation value P7
    int16_t dig_P8;  ///< Pressure compensation value P8
    int16_t dig_P9;  ///< Pressure compensation value P9

    uint8_t dig_H1; ///< Humidity compensation value H1
    int16_t dig_H2; ///< Humidity compensation value H2
    uint8_t dig_H3; ///< Humidity compensation value H3
    int16_t dig_H4; ///< Humidity compensation value H4
    int16_t dig_H5; ///< Humidity compensation value H5
    int8_t dig_H6;  ///< Humidity compensation value H6
};

/**
 * @brief Holds a set of measurements read from the device in a single burst read.
 */
struct BME280_SensorMeasurements
{
  public:
    float temperature; ///< Temperature - in C or F depending on the requested scale
    float pressure;    ///< Pressure in Pa
    float humidity;    ///< Relative humidity in %
};

/**
 * @class sfDevBME280
 * @brief Platform independent driver class for the BME280 temperature, pressure and humidity sensor.
 *
 * This class implements the functionality of the BME280 sensor. Communication with the device is performed
 * using a SparkFun Toolkit bus object (sfTkIBus), which is passed to the begin() method.
 *
 * Usage:
 * - Call begin() with an initialized bus object to setup the sensor.
 * - Use the configuration methods to set the mode, over sampling, filter and standby time.
 * - Use the read methods to obtain temperature, pressure, humidity, altitude and dew point values.
 */
class sfDevBME280
{
  public:
    /**
     * @brief Constructor - sets the default settings for the device
     */
    sfDevBME280();

    /**
     * @brief Initializes the BME280 device using the provided bus.
     *
     * Verifies the chip ID, reads the calibration data from the device, applies the values in settings
     * and places the device in normal mode.
     *
     * @param theBus Pointer to the bus interface (sfTkIBus) to use for communication.
     * @return sfTkError_t ksfTkErrOk on success, otherwise an error code
     */
    sfTkError_t begin(sfTkIBus *theBus = nullptr);

    /**
     * @brief Checks if a BME280 or BMP280 is connected and responding.
     *
     * @return true if the device returns a valid chip ID, false otherwise.
     */
    bool isConnected(void);

    /**
     * @brief Returns the chip ID of the connected device.
     *
     * @return uint8_t The chip ID - 0x60 for a BME280, 0x58 for a BMP280. 0 on a communication error.
     */
    uint8_t chipID(void);

    /**
     * @brief Get the current mode of the device: sleep, forced, or normal
     *
     * @return uint8_t MODE_SLEEP, MODE_FORCED or MODE_NORMAL
     */
    uint8_t getMode(void);

    /**
     * @brief Set the mode of the device
     *
     * @param mode The mode to set - MODE_SLEEP, MODE_FORCED or MODE_NORMAL
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t setMode(uint8_t mode);

    /**
     * @brief Set the temperature over sample value
     *
     * @param overSampleAmount 0 turns off temperature sensing, 1, 2, 4, 8 or 16 are valid over sampling values
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t setTempOverSample(uint8_t overSampleAmount);

    /**
     * @brief Set the pressure over sample value
     *
     * @param overSampleAmount 0 turns off pressure sensing, 1, 2, 4, 8 or 16 are valid over sampling values
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t setPressureOverSample(uint8_t overSampleAmount);

    /**
     * @brief Set the humidity over sample value
     *
     * @param overSampleAmount 0 turns off humidity sensing, 1, 2, 4, 8 or 16 are valid over sampling values
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t setHumidityOverSample(uint8_t overSampleAmount);

    /**
     * @brief Set the standby time between measurements when in normal mode
     *
     * @param timeSetting 0 = 0.5ms, 1 = 62.5ms, 2 = 125ms, 3 = 250ms, 4 = 500ms, 5 = 1000ms, 6 = 10ms, 7 = 20ms
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t setStandbyTime(uint8_t timeSetting);

    /**
     * @brief Set the IIR filter coefficient
     *
     * @param filterSetting 0 = filter off, 1 = 2 coefficients, 2 = 4, 3 = 8, 4 = 16
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t setFilter(uint8_t filterSetting);

    /**
     * @brief Sets the local sea level reference pressure, in Pa, used for altitude calculations.
     *
     * The value is probably within 10% of 101325 and varies based on the weather. If accuracy is a
     * concern, use a value from a trusted source like NOAA.
     *
     * @param refPressure The reference pressure in Pa
     */
    void setReferencePressure(float refPressure);

    /**
     * @brief Returns the sea level reference pressure in Pa
     *
     * @return float The reference pressure in Pa
     */
    float getReferencePressure(void);

    /**
     * @brief Checks the measuring bit of the status register.
     *
     * @return true while the device is taking a measurement
     */
    bool isMeasuring(void);

    /**
     * @brief Performs a soft reset of the device. Call begin() afterwards.
     *
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t reset(void);

    /**
     * @brief Reads all measurements (temperature, pressure, humidity) in a single burst read.
     *
     * See section 4 (Data readout) of the BME280 datasheet.
     *
     * @param[out] measurements Pointer to the structure that receives the values
     * @param tempScale 0 for Celsius (default), 1 for Fahrenheit
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t readAllMeasurements(BME280_SensorMeasurements *measurements, uint8_t tempScale = 0);

    /**
     * @brief Reads the pressure in Pa
     *
     * @note The pressure compensation uses the temperature value from the last temperature read.
     *
     * @param[out] pressure The pressure in Pa
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t readFloatPressure(float &pressure);

    /**
     * @brief Reads the pressure in Pa
     *
     * @note The pressure compensation uses the temperature value from the last temperature read.
     *
     * @return float The pressure in Pa, NAN on error
     */
    float readFloatPressure(void);

    /**
     * @brief Returns the altitude in meters, calculated from the current pressure and the reference pressure
     *
     * @return float The altitude in meters
     */
    float readFloatAltitudeMeters(void);

    /**
     * @brief Returns the altitude in feet, calculated from the current pressure and the reference pressure
     *
     * @return float The altitude in feet
     */
    float readFloatAltitudeFeet(void);

    /**
     * @brief Calculates the pressure from the data of a burst read.
     *
     * @param buffer The 8 bytes read from BME280_MEASUREMENTS_REG
     * @param[out] measurements The pressure field is set
     */
    void readFloatPressureFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

    /**
     * @brief Reads the relative humidity in %
     *
     * @note The humidity compensation uses the temperature value from the last temperature read.
     *
     * @param[out] humidity The relative humidity in %
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t readFloatHumidity(float &humidity);

    /**
     * @brief Reads the relative humidity in %
     *
     * @note The humidity compensation uses the temperature value from the last temperature read.
     *
     * @return float The relative humidity in %, NAN on error
     */
    float readFloatHumidity(void);

    /**
     * @brief Calculates the humidity from the data of a burst read.
     *
     * @param buffer The 8 bytes read from BME280_MEASUREMENTS_REG
     * @param[out] measurements The humidity field is set
     */
    void readFloatHumidityFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

    /**
     * @brief Set a correction value that is added to the temperature result
     *
     * @param corr The temperature correction in C
     */
    void setTemperatureCorrection(float corr);

    /**
     * @brief Reads the temperature in C
     *
     * @param[out] tempC The temperature in C
     * @return sfTkError_t ksfTkErrOk on success
     */
    sfTkError_t readTempC(float &tempC);

    /**
     * @brief Reads the temperature in C
     *
     * @return float The temperature in C, NAN on error
     */
    float readTempC(void);

    /**
     * @brief Reads the temperature in F
     *
     * @return float The temperature in F, NAN on error
     */
    float readTempF(void);

    /**
     * @brief Calculates the temperature, in C, from the data of a burst read.
     *
     * @param buffer The 8 bytes read from BME280_MEASUREMENTS_REG
     * @return float The temperature in C
     */
    float readTempFromBurst(uint8_t buffer[]);

    /**
     * @brief Returns the dew point in C
     *
     * From Pavel-Sayekat: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/pull/6/files
     *
     * @return double The dew point in C
     */
    double dewPointC(void);

    /**
     * @brief Returns the dew point in F
     *
     * @return double The dew point in F
     */
    double dewPointF(void);

    /** Settings used by the driver */
    BME280_SensorSettings settings;

    /** Calibration constants read from the device */
    SensorCalibration calibration;

    /** Fine temperature value - set when the temperature is read, used by pressure and humidity compensation */
    int32_t t_fine;

  protected:
    /**
     * Pointer to the bus interface used to communicate with the device
     */
    sfTkIBus *_theBus;

  private:
    /** Mask applied to a register address on a write - clears the read/write bit in SPI mode */
    uint8_t _regWriteMask;

    sfTkError_t readCalibration(void);
    sfTkError_t updateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value);
    sfTkError_t setOverSample(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t overSampleAmount);
    uint8_t checkSampleValue(uint8_t userValue);

    float compensateTemperature(int32_t adc_T);
    float compensatePressure(int32_t adc_P);
    float compensateHumidity(int32_t adc_H);

    float _referencePressure;
};
