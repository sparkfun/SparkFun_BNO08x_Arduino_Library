/**
 * @file SparkFun_BNO08x_Arduino_Library.h
 * @brief Arduino-specific implementation for the SparkFun BNO08x IMU.
 *
 * @details
 * This file provides the Arduino-specific implementation of the BNO08x driver classes. The classes
 * inherit from the platform independent sfDevBNO08x class and implement the I2C and SPI communication
 * using the SparkFun Toolkit Arduino bus classes, plus the INT and RST pin handling.
 *
 * Key features:
 * - Arduino I2C and SPI initialization
 * - INT and RST pin handling
 * - Connection verification
 * - Toolkit integration
 *
 * @section Class Classes
 * - SfeBNO08xArdI2C: BNO08x connected over I2C
 * - SfeBNO08xArdSPI: BNO08x connected over SPI
 * - BNO08x: Compatibility class providing the begin()/beginSPI() interface of earlier releases
 *
 * @section Dependencies Dependencies
 * - Arduino.h
 * - SparkFun_Toolkit.h
 * - sfDevBNO08x.h
 *
 * @author SparkFun Electronics
 * @date 2025
 * @copyright Copyright (c) 2023-2025, SparkFun Electronics Inc. All rights reserved.
 *
 * @section License License
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library
 */

#pragma once

// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevBNO08x.h"
#include <Arduino.h>
// clang-format on

///////////////////////////////////////////////////////////////////////////////
// Bus classes
//
// The BNO08x SHTP protocol reads data without first sending a register address,
// which the Toolkit bus interface does not provide. These classes extend the
// Toolkit Arduino buses with that raw read.
///////////////////////////////////////////////////////////////////////////////

/**
 * @class SfeBNO08xI2CBus
 * @brief Toolkit Arduino I2C bus with a register-less read.
 */
class SfeBNO08xI2CBus : public sfTkArdI2C
{
  public:
    /**
     * @brief Read bytes from the device in a single I2C transaction, with no register address.
     *
     * @param data Buffer to read into
     * @param length Number of bytes to read. Must not exceed bufferChunkSize().
     * @return ksfTkErrOk on success, otherwise an error code.
     */
    sfTkError_t readData(uint8_t *data, size_t length)
    {
        if (!_i2cPort)
            return ksfTkErrBusNotInit;

        if ((size_t)_i2cPort->requestFrom((int)address(), (int)length, (int)true) != length)
            return ksfTkErrBusUnderRead;

        for (size_t i = 0; i < length; i++)
            data[i] = _i2cPort->read();

        return ksfTkErrOk;
    }
};

/**
 * @class SfeBNO08xSPIBus
 * @brief Toolkit Arduino SPI bus with a register-less read.
 */
class SfeBNO08xSPIBus : public sfTkArdSPI
{
  public:
    using sfTkArdSPI::init;

    /**
     * @brief Initialize the bus with the BNO08x SPI settings (mode 3, MSB first) and deselect the device.
     *
     * @param csPin Chip select pin
     * @param spiPortSpeed SPI clock speed in Hz - limited to 3MHz, the BNO08x maximum
     * @param spiPort SPIClass instance to use for SPI communication
     * @return ksfTkErrOk on success, otherwise an error code.
     */
    sfTkError_t init(uint8_t csPin, uint32_t spiPortSpeed, SPIClass &spiPort)
    {
        if (spiPortSpeed > 3000000)
            spiPortSpeed = 3000000; // BNO08x max is 3MHz

        SPISettings spiSettings(spiPortSpeed, MSBFIRST, SPI_MODE3);

        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH); // Deselect the BNO08x

        return init(spiPort, spiSettings, csPin, true);
    }

    /**
     * @brief Read bytes from the device in a single SPI transaction, clocking out zeros.
     *
     * @param data Buffer to read into
     * @param length Number of bytes to read
     * @return ksfTkErrOk on success, otherwise an error code.
     */
    sfTkError_t readData(uint8_t *data, size_t length)
    {
        if (!_spiPort)
            return ksfTkErrBusNotInit;

        memset(data, 0, length);

        _spiPort->beginTransaction(_sfeSPISettings);
        digitalWrite(cs(), LOW);

        _spiPort->transfer(data, length);

        digitalWrite(cs(), HIGH);
        _spiPort->endTransaction();

        return ksfTkErrOk;
    }
};

///////////////////////////////////////////////////////////////////////////////
// Device classes
///////////////////////////////////////////////////////////////////////////////

/**
 * @class SfeBNO08xArdBase
 * @brief Arduino base for the BNO08x classes - INT/RST pin handling.
 *
 * @details
 * Implements the GPIO hooks of sfDevBNO08x using the Arduino pin API.
 */
class SfeBNO08xArdBase : public sfDevBNO08x
{
  protected:
    SfeBNO08xArdBase() : _intPin{-1}, _rstPin{-1}
    {
    }

    /**
     * @brief Set the INT and RST pins. The INT pin is configured as an input with pullup.
     *
     * @param intPin INT pin, or -1 if not connected
     * @param rstPin RST pin, or -1 if not connected
     * @param rstPinMode Initial pin mode for the RST pin
     */
    void setPins(int8_t intPin, int8_t rstPin, uint8_t rstPinMode)
    {
        _intPin = intPin;
        if (_intPin != -1)
            pinMode(_intPin, INPUT_PULLUP);

        _rstPin = rstPin;
        if (_rstPin != -1)
            pinMode(_rstPin, rstPinMode);
    }

    bool hasInterruptPin(void) override
    {
        return _intPin != -1;
    }

    bool interruptAsserted(void) override
    {
        return digitalRead(_intPin) == LOW;
    }

    bool hasResetPin(void) override
    {
        return _rstPin != -1;
    }

    void setResetPin(bool high) override
    {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, high ? HIGH : LOW);
    }

  private:
    int8_t _intPin;
    int8_t _rstPin;
};

/**
 * @class SfeBNO08xArdI2C
 * @brief Arduino I2C implementation for the BNO08x IMU.
 *
 * Example usage:
 * @code
 * SfeBNO08xArdI2C myIMU;
 * if (myIMU.begin(kBNO08xDefaultAddress, Wire, BNO08X_INT, BNO08X_RST)) {
 *     // Sensor initialized successfully
 * }
 * @endcode
 *
 * @note This class uses the Arduino Wire library for I2C communication
 *
 * @see sfDevBNO08x
 */
class SfeBNO08xArdI2C : public SfeBNO08xArdBase
{
  public:
    SfeBNO08xArdI2C()
    {
    }

    /**
     * @brief Initializes the BNO08x with I2C communication.
     *
     * @param address I2C address of the device (default: kBNO08xDefaultAddress)
     * @param wirePort TwoWire instance to use for I2C communication (default: Wire)
     * @param intPin INT pin, or -1 if not connected (connecting it is recommended)
     * @param rstPin RST pin, or -1 if not connected (connecting it is recommended)
     *
     * @return true If initialization successful
     * @return false If any initialization step fails
     */
    bool begin(const uint8_t address = kBNO08xDefaultAddress, TwoWire &wirePort = Wire, int8_t intPin = -1,
               int8_t rstPin = -1)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        setPins(intPin, rstPin, INPUT_PULLUP);

        setCommunicationBus(&_theI2CBus);

        // Limit transfers to what the Wire buffer can hold
        setMaxTransferSize(_theI2CBus.bufferChunkSize());

        // Give the device a chance to come up before checking for it
        waitForInterrupt();

        if (!isConnected())
            return false;

        return sfDevBNO08x::begin();
    }

    /**
     * @brief Checks if the BNO08x is connected and responding.
     *
     * @return true If device responds to a ping at the current I2C address
     */
    bool isConnected(void)
    {
        return _theI2CBus.ping() == ksfTkErrOk;
    }

    /**
     * @brief Gets the currently configured I2C address of the BNO08x.
     *
     * @return uint8_t The current I2C address
     */
    uint8_t getDeviceAddress(void)
    {
        return _theI2CBus.address();
    }

  protected:
    sfTkError_t readBusData(uint8_t *data, size_t length) override
    {
        return _theI2CBus.readData(data, length);
    }

  private:
    SfeBNO08xI2CBus _theI2CBus;
};

/**
 * @class SfeBNO08xArdSPI
 * @brief Arduino SPI implementation for the BNO08x IMU.
 *
 * Example usage:
 * @code
 * SfeBNO08xArdSPI myIMU;
 * if (myIMU.begin(BNO08X_CS, BNO08X_INT, BNO08X_RST)) {
 *     // Sensor initialized successfully
 * }
 * @endcode
 *
 * @note The INT pin is required when using SPI.
 *
 * @see sfDevBNO08x
 */
class SfeBNO08xArdSPI : public SfeBNO08xArdBase
{
  public:
    SfeBNO08xArdSPI()
    {
    }

    /**
     * @brief Initializes the BNO08x with SPI communication.
     *
     * @param csPin Chip select pin
     * @param intPin INT pin
     * @param rstPin RST pin
     * @param spiPortSpeed SPI clock speed in Hz (default: 1MHz, max: 3MHz)
     * @param spiPort SPIClass instance to use for SPI communication (default: SPI)
     *
     * @return true If initialization successful
     * @return false If any initialization step fails
     */
    bool begin(uint8_t csPin, uint8_t intPin, uint8_t rstPin, uint32_t spiPortSpeed = 1000000,
               SPIClass &spiPort = SPI)
    {
        if (_theSPIBus.init(csPin, spiPortSpeed, spiPort) != ksfTkErrOk)
            return false;

        setPins(intPin, rstPin, OUTPUT);

        setCommunicationBus(&_theSPIBus);

        return sfDevBNO08x::begin();
    }

  protected:
    sfTkError_t readBusData(uint8_t *data, size_t length) override
    {
        return _theSPIBus.readData(data, length);
    }

  private:
    SfeBNO08xSPIBus _theSPIBus;
};

/**
 * @class BNO08x
 * @brief Compatibility class with the interface of earlier releases of this library.
 *
 * @details
 * Provides begin() for I2C (from SfeBNO08xArdI2C) and beginSPI() for SPI on a single class. New
 * code should use SfeBNO08xArdI2C or SfeBNO08xArdSPI.
 */
class BNO08x : public SfeBNO08xArdI2C
{
  public:
    BNO08x() : _usingSPI{false}
    {
    }

    /// @brief Initializes the BNO08x with I2C communication. See SfeBNO08xArdI2C::begin().
    bool begin(const uint8_t address = kBNO08xDefaultAddress, TwoWire &wirePort = Wire, int8_t intPin = -1,
               int8_t rstPin = -1)
    {
        _usingSPI = false;
        return SfeBNO08xArdI2C::begin(address, wirePort, intPin, rstPin);
    }

    /// @brief Initializes the BNO08x with SPI communication. See SfeBNO08xArdSPI::begin().
    bool beginSPI(uint8_t user_CSPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed = 1000000,
                  SPIClass &spiPort = SPI)
    {
        if (_theSPIBus.init(user_CSPin, spiPortSpeed, spiPort) != ksfTkErrOk)
            return false;

        _usingSPI = true;

        setPins(user_INTPin, user_RSTPin, OUTPUT);

        setCommunicationBus(&_theSPIBus);

        return sfDevBNO08x::begin();
    }

    /// @brief Retained for compatibility - the library no longer prints debug output.
    void enableDebugging(Stream &debugPort = Serial)
    {
        (void)debugPort;
    }

  protected:
    sfTkError_t readBusData(uint8_t *data, size_t length) override
    {
        return _usingSPI ? _theSPIBus.readData(data, length) : SfeBNO08xArdI2C::readBusData(data, length);
    }

  private:
    bool _usingSPI;
    SfeBNO08xSPIBus _theSPIBus;
};
