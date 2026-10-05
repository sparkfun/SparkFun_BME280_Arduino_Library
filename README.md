SparkFun BME280 Arduino Library
========================================

![SparkFun BME280 Breakout](https://cdn.sparkfun.com//assets/parts/1/2/3/2/9/14348-01.jpg)

[*SparkFun CCS811/BME280 Combo (SEN-14348)*](https://www.sparkfun.com/products/14348)

![GitHub License](https://img.shields.io/github/license/sparkfun/SparkFun_BME280_Arduino_Library)
![Release](https://img.shields.io/github/v/release/sparkfun/SparkFun_BME280_Arduino_Library)
![Release Date](https://img.shields.io/github/release-date/sparkfun/SparkFun_BME280_Arduino_Library)
![Documentation - build](https://img.shields.io/github/actions/workflow/status/sparkfun/SparkFun_BME280_Arduino_Library/build-deploy-ghpages.yml?label=doc%20build)
[![Compile Test](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/actions/workflows/test-compile-sketch.yml/badge.svg)](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/actions/workflows/test-compile-sketch.yml)
![GitHub issues](https://img.shields.io/github/issues/sparkfun/SparkFun_BME280_Arduino_Library)

This library allows the user to:

* Read pressure in Pa
* Read temperature in C
* Read humidity in %RH

It also provides the following mathematical functions based from the above.

* Read temperature in F
* Read altitude in meters
* Read altitude in feet

Some examples include a circular buffer class for averaging.

> [!NOTE]
> Version 3.0 and above of this library uses the [SparkFun Toolkit](https://github.com/sparkfun/SparkFun_Toolkit) for communication with the sensor (I2C and SPI). The SparkFun Toolkit is installed automatically when this library is installed using the Arduino Library Manager. If installing manually, the SparkFun Toolkit library must also be installed.

The device logic is implemented in the platform independent ``sfDevBME280`` class (``src/sfTk``). A separate Arduino class is provided for each communication bus, and each class sets up its bus and passes it to ``sfDevBME280``:

* ``SparkFunBME280I2C`` - I2C, using a ``TwoWire`` port. ``begin(address = 0x77, wirePort = Wire)``
* ``SparkFunBME280SPI`` - SPI. ``begin(csPin, spiPort = SPI, spiSettings)``

Changes from version 2.x:

* The ``BME280`` class is deprecated. It's kept for backwards compatibility as an I2C only class that supports ``setI2CAddress()`` and ``beginI2C()``. For SPI, use ``SparkFunBME280SPI``.
* The communication fields of ``settings`` (``commInterface``, ``I2CAddress``, ``chipSelectPin`` and ``spiSettings``) were removed - the bus is set by the class used and the parameters passed to ``begin()``. The SPI clock and mode default to the ``BME280_SPI_CLOCK``/``BME280_SPI_MODE`` defines.
* ``begin()`` returns ``true`` on success. Use ``chipID()`` to read the chip ID.
* Software I2C (SoftwareWire) support was removed.
* The register access methods (``readRegister()``, ``readRegisterRegion()``, ``readRegisterInt16()`` and ``writeRegister()``) were removed. To access device registers directly, use a SparkFun Toolkit bus object - see ``Example5_ReadAllRegisters``.
* Setter methods (``setMode()``, ``setFilter()``, etc.) now return an error code (``sfTkError_t``) instead of ``void``.
* The ``readTempC()``, ``readFloatPressure()`` and ``readFloatHumidity()`` methods return ``NAN`` if the sensor can't be read.

Thanks to:
* calvin1564 for [fixing a name collision](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/pull/34)
* LukaHitH for adding [BMP280 support](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/pull/23)
* cdonate for adding [burst read](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/pull/50)
* jdavid for integrating [math.h](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/pull/49)

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/docs** - Doxygen configuration used to generate the library's API documentation.
* **/src** - Source files for the library (.cpp, .h).
* **/src/sfTk** - Platform independent implementation of the BME280 driver, built on the SparkFun Toolkit.
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Documentation
--------------

* **[Library Documentation](https://docs.sparkfun.com/SparkFun_BME280_Arduino_Library)** - API documentation for this library, generated using Doxygen.
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **Product Repositories:** (including hardware files)
  * [SparkFun BME280 Breakout](https://github.com/sparkfun/BME280-Breakout-Board) - Original product repository
  * [SparkFun Environmental Combo Breakout - CCS811/BME280 (Qwiic)](https://www.sparkfun.com/products/14348)
  * [SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic)](https://github.com/sparkfun/Qwiic_Atmospheric_Sensor_Breakout_BME280)
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-bme280-breakout-hookup-guide)** - Basic hookup guide for the SparkFun BME280 Breakout.

Products that use this Library 
---------------------------------

* [*SEN-15440*](https://www.sparkfun.com/products/15440)
* [*SEN-14348*](https://www.sparkfun.com/products/14348)
* [*SEN-13676*](https://www.sparkfun.com/products/13676)

Version History
---------------

* [V_1.0.0](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/V_1.0.0) - Public release.
* [V_1.1.0](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/V_1.1.0) - Pulled request to add decimal precision to readFloatPressure() and readFloatHumidity().
* [V_1.2.0](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/V_1.2.0) - Added ESP32 architecture directives to modify begin operation.
* [V_2.0.0](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/V_2.0.0) - Major update. Generalized wire port. Functionalized oversampling, standby time, etc. Added setMode.
* V_3.0.0 - Ported to use the SparkFun Toolkit. Added Doxygen documentation and GitHub workflows.


License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
