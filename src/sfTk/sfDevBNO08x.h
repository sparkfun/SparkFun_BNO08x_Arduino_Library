/**
 * @file sfDevBNO08x.h
 * @brief Header file for the SparkFun BNO08x IMU - platform independent driver.
 *
 * This file contains the class definition, constants and report IDs for interacting with the BNO08x IMU.
 *
 * @details
 * sfDevBNO08x is a comms-agnostic driver for the BNO08x that uses the SparkFun Toolkit. It wraps the
 * CEVA Sensor Hub (SH2) driver, providing the SH2 Hardware Abstraction Layer (HAL) on top of a
 * Toolkit bus object (sfTkIBus).
 *
 * The BNO08x communicates using the SHTP packet protocol, not a register map. Two capabilities this
 * requires are not part of the Toolkit bus interface, so they are provided by the platform layer
 * through protected virtual methods:
 * - A raw, register-less read of bytes from the device (readBusData())
 * - Access to the INT and RST GPIO pins (hasInterruptPin(), interruptAsserted(), hasResetPin(), setResetPin())
 *
 * @note The CEVA SH2 driver keeps its state in static variables, so only one BNO08x device can be
 * active at a time.
 *
 * @author SparkFun Electronics
 * @date 2025
 * @copyright Copyright (c) 2023-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @section Classes Classes
 * - sfDevBNO08x
 *
 * @section Repository Repository
 * https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library
 *
 * @section Credits Credits
 * Originally written by Nathan Seidle, adjusted by Pete Lewis to use the CEVA Sensor Hub driver
 * (https://github.com/ceva-dsp/sh2), with HAL code based on the Adafruit BNO08x Arduino Library
 * (https://github.com/adafruit/Adafruit_BNO08x).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

// include the sparkfun toolkit headers
#include <sfTk/sfToolkit.h>

// Bus interfaces
#include <sfTk/sfTkIBus.h>

// CEVA Sensor Hub driver
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

///////////////////////////////////////////////////////////////////////////////
// I2C Addressing
///////////////////////////////////////////////////////////////////////////////

const uint8_t kBNO08xDefaultAddress = 0x4B; // Default I2C address on the SparkFun breakout
const uint8_t kBNO08xAltAddress = 0x4A;     // I2C address when the ADR jumper is closed

// Legacy name for the default address
#define BNO08x_DEFAULT_ADDRESS kBNO08xDefaultAddress

///////////////////////////////////////////////////////////////////////////////
// SHTP / Report Definitions
///////////////////////////////////////////////////////////////////////////////

// All the ways we can configure or talk to the BNO08x, figure 34, page 36 reference manual
// These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER SH2_ACCELEROMETER
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED SH2_GYROSCOPE_CALIBRATED
#define SENSOR_REPORTID_MAGNETIC_FIELD SH2_MAGNETIC_FIELD_CALIBRATED
#define SENSOR_REPORTID_LINEAR_ACCELERATION SH2_LINEAR_ACCELERATION
#define SENSOR_REPORTID_ROTATION_VECTOR SH2_ROTATION_VECTOR
#define SENSOR_REPORTID_GRAVITY SH2_GRAVITY
#define SENSOR_REPORTID_UNCALIBRATED_GYRO SH2_GYROSCOPE_UNCALIBRATED
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR SH2_GYRO_INTEGRATED_RV
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER SH2_STEP_COUNTER
#define SENSOR_REPORTID_STABILITY_CLASSIFIER SH2_STABILITY_CLASSIFIER
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER SH2_PERSONAL_ACTIVITY_CLASSIFIER
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

// Command IDs from section 6.4, page 42
// These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z 0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

///////////////////////////////////////////////////////////////////////////////

/**
 * @class sfDevBNO08x
 * @brief Platform independent driver for the BNO08x IMU.
 *
 * @details
 * This class implements the BNO08x functionality on top of the CEVA SH2 driver, using a SparkFun
 * Toolkit bus for writes and the platform hooks (see the protected section) for raw reads and GPIO.
 *
 * A platform layer subclasses this class, implements readBusData(), optionally implements the
 * GPIO hooks, sets the communication bus and then calls begin().
 */
class sfDevBNO08x
{
  public:
    sfDevBNO08x();

    /// @brief Initialize the BNO08x through the specified bus. This resets the device, opens the SH2
    /// driver and reads the product IDs.
    /// @param theBus Pointer to the bus object. If nullptr, the bus set by setCommunicationBus() is used.
    /// @return True if successful, false if it fails.
    bool begin(sfTkIBus *theBus = nullptr);

    /// @brief Sets the communication bus to the specified bus.
    /// @param theBus Bus to set as the communication device.
    void setCommunicationBus(sfTkIBus *theBus);

    /// @brief Set the maximum number of bytes moved in a single bus transaction. SHTP transfers larger
    /// than this are split. Defaults to SH2_HAL_MAX_TRANSFER_IN (no splitting).
    /// @param maxSize Maximum transfer size, in bytes. Must be larger than the 4 byte SHTP header.
    void setMaxTransferSize(size_t maxSize);

    /// @brief The product IDs returned by the sensor during begin()
    sh2_ProductIds_t prodIds;

    /// @brief The most recent sensor report, filled in by getSensorEvent()
    sh2_SensorValue_t sensorValue;

    /// @brief Reset the device using the RST pin, if the platform provides one.
    void hardwareReset(void);

    /// @brief Check if the sensor has reported a reset. Reading this clears the flag.
    /// @return True if a reset occurred since the last call.
    bool wasReset(void);

    /// @brief Returns the reset cause reported in the product IDs.
    uint8_t getResetReason(void);

    /// @brief Enable the given report type
    /// @param sensorId The report ID to enable
    /// @param interval_us The update interval for reports, in microseconds
    /// @param sensorSpecific Config settings specific to the sensor/report ID (e.g. activities to enable
    /// for the personal activity classifier)
    /// @return True if successful, false if it fails.
    bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us = 10000, uint32_t sensorSpecific = 0);

    /// @brief Service the bus and fill sensorValue with a new report if one is available.
    /// @return True if a new report was received.
    bool getSensorEvent(void);

    /// @brief The sensor ID of the most recent report.
    uint8_t getSensorEventID(void);

    /// @brief Service the SH2 driver, processing any pending traffic from the device.
    bool serviceBus(void);

    /// @brief Reset the device using the executable channel.
    bool softReset(void);

    /// @brief Use the executable channel to turn the BNO08x on.
    bool modeOn(void);

    /// @brief Use the executable channel to put the BNO08x to sleep.
    bool modeSleep(void);

    /// @brief Given a Q value, converts a fixed point value to a floating point number.
    float qToFloat(int16_t fixedPointValue, uint8_t qPoint);

    // Report enable helpers - the time between reports is in milliseconds.
    bool enableRotationVector(uint16_t timeBetweenReports = 10);
    bool enableGeomagneticRotationVector(uint16_t timeBetweenReports = 10);
    bool enableGameRotationVector(uint16_t timeBetweenReports = 10);
    bool enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
    bool enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
    bool enableAccelerometer(uint16_t timeBetweenReports = 10);
    bool enableLinearAccelerometer(uint16_t timeBetweenReports = 10);
    bool enableGravity(uint16_t timeBetweenReports = 10);
    bool enableGyro(uint16_t timeBetweenReports = 10);
    bool enableUncalibratedGyro(uint16_t timeBetweenReports = 10);
    bool enableMagnetometer(uint16_t timeBetweenReports = 10);
    bool enableTapDetector(uint16_t timeBetweenReports);
    bool enableStepCounter(uint16_t timeBetweenReports = 10);
    bool enableStabilityClassifier(uint16_t timeBetweenReports = 10);
    bool enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable);
    bool enableRawAccelerometer(uint16_t timeBetweenReports = 10);
    bool enableRawGyro(uint16_t timeBetweenReports = 10);
    bool enableRawMagnetometer(uint16_t timeBetweenReports = 10);
    bool enableGyroIntegratedRotationVector(uint16_t timeBetweenReports = 10);

    void getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy);
    float getQuatI(void);
    float getQuatJ(void);
    float getQuatK(void);
    float getQuatReal(void);
    float getQuatRadianAccuracy(void);
    uint8_t getQuatAccuracy(void);

    float getGameQuatI(void);
    float getGameQuatJ(void);
    float getGameQuatK(void);
    float getGameQuatReal(void);

    void getAccel(float &x, float &y, float &z, uint8_t &accuracy);
    float getAccelX(void);
    float getAccelY(void);
    float getAccelZ(void);
    uint8_t getAccelAccuracy(void);

    void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
    float getLinAccelX(void);
    float getLinAccelY(void);
    float getLinAccelZ(void);
    uint8_t getLinAccelAccuracy(void);

    void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
    float getGyroX(void);
    float getGyroY(void);
    float getGyroZ(void);
    uint8_t getGyroAccuracy(void);

    void getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, uint8_t &accuracy);
    float getUncalibratedGyroX(void);
    float getUncalibratedGyroY(void);
    float getUncalibratedGyroZ(void);
    float getUncalibratedGyroBiasX(void);
    float getUncalibratedGyroBiasY(void);
    float getUncalibratedGyroBiasZ(void);
    uint8_t getUncalibratedGyroAccuracy(void);

    float getGyroIntegratedRVI(void);
    float getGyroIntegratedRVJ(void);
    float getGyroIntegratedRVK(void);
    float getGyroIntegratedRVReal(void);
    float getGyroIntegratedRVangVelX(void);
    float getGyroIntegratedRVangVelY(void);
    float getGyroIntegratedRVangVelZ(void);

    void getMag(float &x, float &y, float &z, uint8_t &accuracy);
    float getMagX(void);
    float getMagY(void);
    float getMagZ(void);
    uint8_t getMagAccuracy(void);

    void getGravity(float &x, float &y, float &z, uint8_t &accuracy);
    float getGravityX(void);
    float getGravityY(void);
    float getGravityZ(void);
    uint8_t getGravityAccuracy(void);

    bool setCalibrationConfig(uint8_t sensors);
    bool saveCalibration(void);

    bool tareNow(bool zAxis = false, sh2_TareBasis_t basis = SH2_TARE_BASIS_ROTATION_VECTOR);
    bool saveTare(void);
    bool clearTare(void);

    /// @brief Returns the tap detector flags. Reading clears the value so each tap is seen once.
    uint8_t getTapDetector(void);
    uint64_t getTimeStamp(void);
    uint16_t getStepCount(void);
    uint8_t getStabilityClassifier(void);
    uint8_t getActivityClassifier(void);
    uint8_t getActivityConfidence(uint8_t activity);

    int16_t getRawAccelX(void);
    int16_t getRawAccelY(void);
    int16_t getRawAccelZ(void);

    int16_t getRawGyroX(void);
    int16_t getRawGyroY(void);
    int16_t getRawGyroZ(void);

    int16_t getRawMagX(void);
    int16_t getRawMagY(void);
    int16_t getRawMagZ(void);

    /// @brief Roll (rotation around the x-axis) from the rotation vector, in radians.
    float getRoll(void);
    /// @brief Pitch (rotation around the y-axis) from the rotation vector, in radians.
    float getPitch(void);
    /// @brief Yaw/heading (rotation around the z-axis) from the rotation vector, in radians.
    float getYaw(void);

  protected:
    /// @brief Platform hook: read length bytes from the device in a single bus transaction, with no
    /// register address. The Toolkit bus interface does not provide a register-less read.
    /// @param data Buffer to read into
    /// @param length Number of bytes to read
    /// @return ksfTkErrOk on success, otherwise an error code.
    virtual sfTkError_t readBusData(uint8_t *data, size_t length) = 0;

    /// @brief Platform hook: true if the INT pin is connected.
    virtual bool hasInterruptPin(void)
    {
        return false;
    }

    /// @brief Platform hook: true if the INT pin is asserted (low), signaling the device has data or
    /// is ready for a transfer.
    virtual bool interruptAsserted(void)
    {
        return true;
    }

    /// @brief Platform hook: true if the RST pin is connected.
    virtual bool hasResetPin(void)
    {
        return false;
    }

    /// @brief Platform hook: drive the RST pin to the given level.
    virtual void setResetPin(bool high)
    {
        (void)high;
    }

    /// @brief Wait for the device to assert INT, if an INT pin is connected. If it is not asserted
    /// within ~500ms the device is hardware reset.
    /// @return True if INT was asserted, or no INT pin is connected.
    bool waitForInterrupt(void);

  private:
    // The SH2 driver passes a pointer to the HAL structure to each HAL function. The structure is
    // wrapped so the HAL functions can get back to the owning device object.
    struct sfDevBNO08xHal
    {
        sh2_Hal_t hal; // must be first
        sfDevBNO08x *device;
    };

    // SH2 HAL functions
    static int halOpen(sh2_Hal_t *self);
    static void halClose(sh2_Hal_t *self);
    static int halRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    static int halWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    static uint32_t halGetTimeUs(sh2_Hal_t *self);

    // SH2 callbacks
    static void asyncEventHandler(void *cookie, sh2_AsyncEvent_t *pEvent);
    static void sensorEventHandler(void *cookie, sh2_SensorEvent_t *pEvent);

    static sfDevBNO08x *deviceFromHal(sh2_Hal_t *self);

    int open(void);
    int read(uint8_t *pBuffer, unsigned len);
    int write(uint8_t *pBuffer, unsigned len);

    bool enableReportMs(sh2_SensorId_t sensorId, uint16_t timeBetweenReports, uint32_t sensorSpecific = 0);

    sfTkIBus *_theBus; // Pointer to bus device.
    sfDevBNO08xHal _hal;
    size_t _maxTransferSize;
    bool _resetOccurred;
    bool _eventReceived;
    uint8_t _tapDetector;
};
