/**
 * @file SparkFunBME280.h
 * @brief SparkFun BME280 Arduino Library header file
 *
 * This file implements the Arduino interface for the BME280 temperature, pressure and humidity sensor. A
 * separate class is provided for each communication bus: SparkFunBME280I2C and SparkFunBME280SPI.
 *
 * The device logic is implemented in the platform independent sfDevBME280 class, which uses the
 * SparkFun Toolkit for communication.
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

// helps to keep the Toolkit header before the tk calls
// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevBME280.h"
// clang-format on

#include <SPI.h>
#include <Wire.h>

#ifndef BME280_SPI_CLOCK
#ifdef ARDUINO_ARCH_ESP32
#define BME280_SPI_CLOCK 1000000
#else
#define BME280_SPI_CLOCK 500000
#endif
#endif

#ifndef BME280_SPI_MODE
#define BME280_SPI_MODE SPI_MODE0
#endif

/**
 * @brief Arduino class for the BME280 temperature, pressure and humidity sensor - using I2C communication
 *
 * This class sets up an I2C bus using the SparkFun Toolkit and passes it to the sfDevBME280 class, which
 * implements the device logic.
 *
 * @see sfDevBME280
 */
class SparkFunBME280I2C : public sfDevBME280
{
  public:
    /**
     * @brief Begin communication with the device over I2C
     *
     * @note The Wire port must be started (Wire.begin()) by the caller.
     *
     * @param address The I2C address of the device - kBME280DefaultAddress (0x77, jumper open) or
     *        kBME280AlternateAddress (0x76, jumper closed)
     * @param wirePort The Wire port to use - default is Wire
     * @return true on success, false otherwise
     */
    bool begin(const uint8_t address = kBME280DefaultAddress, TwoWire &wirePort = Wire)
    {
        // Setup Arduino I2C bus
        _theI2CBus.init(wirePort, address);

        // Begin the sensor
        return sfDevBME280::begin(&_theI2CBus) == ksfTkErrOk;
    }

  private:
    sfTkArdI2C _theI2CBus;
};

/**
 * @brief Arduino class for the BME280 temperature, pressure and humidity sensor - using SPI communication
 *
 * This class sets up a SPI bus using the SparkFun Toolkit and passes it to the sfDevBME280 class, which
 * implements the device logic.
 *
 * @see sfDevBME280
 */
class SparkFunBME280SPI : public sfDevBME280
{
  public:
    /**
     * @brief Begin communication with the device over SPI
     *
     * @param csPin The chip select pin
     * @param spiPort The SPI port to use - default is SPI
     * @param spiSettings The SPI settings to use - defaults are set by BME280_SPI_CLOCK and BME280_SPI_MODE
     * @return true on success, false otherwise
     */
    bool begin(uint8_t csPin, SPIClass &spiPort = SPI,
               SPISettings spiSettings = SPISettings(BME280_SPI_CLOCK, MSBFIRST, BME280_SPI_MODE))
    {
        // start the SPI port and initialize the chip select pin
        spiPort.begin();
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);

        // Setup Arduino SPI bus
        _theSPIBus.init(spiPort, spiSettings, csPin);

        // Begin the sensor
        return sfDevBME280::begin(&_theSPIBus) == ksfTkErrOk;
    }

  private:
    sfTkArdSPI _theSPIBus;
};

// for backwards compatibility
/**
 * @brief Deprecated I2C class for the BME280 - supports the I2C interface of version 2.x of this library
 *
 * @deprecated This class is deprecated for version 3.0 of this library. Use SparkFunBME280I2C or
 * SparkFunBME280SPI instead.
 */
class BME280 : public SparkFunBME280I2C
{
  public:
    /**
     * @brief Set the I2C address used by beginI2C(). Use 0x76 if the address jumper is closed.
     *
     * @param address The I2C address
     */
    void setI2CAddress(uint8_t address)
    {
        _address = address;
    }

    /**
     * @brief Begin communication with the device over I2C, using the address set by setI2CAddress()
     *
     * @param wirePort The Wire port to use - default is Wire
     * @return true on success, false otherwise
     */
    bool beginI2C(TwoWire &wirePort = Wire)
    {
        return begin(_address, wirePort);
    }

  private:
    uint8_t _address = kBME280DefaultAddress;
};
