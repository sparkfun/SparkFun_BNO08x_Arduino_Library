/**
 * @file sfDevBNO08x.cpp
 * @brief Implementation file for the SparkFun BNO08x IMU - platform independent driver.
 *
 * @details
 * This file implements the sfDevBNO08x class: the SH2 Hardware Abstraction Layer (HAL) on top of a
 * SparkFun Toolkit bus, and the sensor configuration and data access methods.
 *
 * @section Implementation Implementation Details
 * - SHTP transfers are read in chunks no larger than the max transfer size. Each chunk after the
 *   first is a new bus transaction, which the device prefixes with a repeated 4 byte SHTP header.
 * - Timing uses the Toolkit sftk_ticks_ms()/sftk_delay_ms() functions.
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

#include "sfDevBNO08x.h"

// For the I2C bus type ID
#include <sfTk/sfTkII2C.h>

#include <math.h>
#include <string.h>

// Size of the SHTP header that starts every transfer
const size_t ksfBNO08xSHTPHeaderLen = 4;

// Number of 1ms polls of the INT pin before giving up and resetting the device
const uint16_t ksfBNO08xIntWaitMs = 500;

// Soft reset packet, sent on the executable channel when opening an I2C connection
static const uint8_t ksfBNO08xSoftResetPacket[] = {5, 0, 1, 0, 1};

//--------------------------------------------------------------------------------------------
sfDevBNO08x::sfDevBNO08x()
    : _theBus{nullptr}, _maxTransferSize{SH2_HAL_MAX_TRANSFER_IN}, _resetOccurred{false}, _eventReceived{false},
      _tapDetector{0}
{
    memset(&prodIds, 0, sizeof(prodIds));
    memset(&sensorValue, 0, sizeof(sensorValue));

    _hal.hal.open = halOpen;
    _hal.hal.close = halClose;
    _hal.hal.read = halRead;
    _hal.hal.write = halWrite;
    _hal.hal.getTimeUs = halGetTimeUs;
    _hal.device = this;
}

//--------------------------------------------------------------------------------------------
bool sfDevBNO08x::begin(sfTkIBus *theBus)
{
    if (theBus != nullptr)
        setCommunicationBus(theBus);

    if (_theBus == nullptr)
        return false;

    hardwareReset();

    if (sh2_open(&_hal.hal, asyncEventHandler, this) != SH2_OK)
        return false;

    memset(&prodIds, 0, sizeof(prodIds));
    if (sh2_getProdIds(&prodIds) != SH2_OK)
        return false;

    sh2_setSensorCallback(sensorEventHandler, this);

    return true;
}

//--------------------------------------------------------------------------------------------
void sfDevBNO08x::setCommunicationBus(sfTkIBus *theBus)
{
    _theBus = theBus;
}

//--------------------------------------------------------------------------------------------
void sfDevBNO08x::setMaxTransferSize(size_t maxSize)
{
    if (maxSize > ksfBNO08xSHTPHeaderLen)
        _maxTransferSize = maxSize;
}

//--------------------------------------------------------------------------------------------
// HAL implementation
//--------------------------------------------------------------------------------------------

sfDevBNO08x *sfDevBNO08x::deviceFromHal(sh2_Hal_t *self)
{
    return reinterpret_cast<sfDevBNO08xHal *>(self)->device;
}

int sfDevBNO08x::halOpen(sh2_Hal_t *self)
{
    return deviceFromHal(self)->open();
}

void sfDevBNO08x::halClose(sh2_Hal_t *self)
{
    (void)self;
}

int sfDevBNO08x::halRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)t_us;
    return deviceFromHal(self)->read(pBuffer, len);
}

int sfDevBNO08x::halWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    return deviceFromHal(self)->write(pBuffer, len);
}

uint32_t sfDevBNO08x::halGetTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return sftk_ticks_ms() * 1000;
}

//--------------------------------------------------------------------------------------------
bool sfDevBNO08x::waitForInterrupt(void)
{
    if (!hasInterruptPin())
        return true;

    for (uint16_t i = 0; i < ksfBNO08xIntWaitMs; i++)
    {
        if (interruptAsserted())
            return true;
        sftk_delay_ms(1);
    }

    hardwareReset();

    return false;
}

//--------------------------------------------------------------------------------------------
int sfDevBNO08x::open(void)
{
    waitForInterrupt();

    // Over SPI, the device signals it is ready via INT - nothing more to do.
    if (_theBus->type() != ksfTkBusTypeI2C)
        return 0;

    // Over I2C, send a soft reset to start from a known state.
    for (uint8_t attempts = 0; attempts < 5; attempts++)
    {
        if (_theBus->writeData(ksfBNO08xSoftResetPacket, sizeof(ksfBNO08xSoftResetPacket)) == ksfTkErrOk)
        {
            sftk_delay_ms(300);
            return 0;
        }
        sftk_delay_ms(30);
    }

    return -1;
}

//--------------------------------------------------------------------------------------------
// Read one SHTP transfer into pBuffer. Returns the transfer length, or 0 if nothing was read.
int sfDevBNO08x::read(uint8_t *pBuffer, unsigned len)
{
    if (!waitForInterrupt())
        return 0;

    uint8_t header[ksfBNO08xSHTPHeaderLen];
    if (readBusData(header, sizeof(header)) != ksfTkErrOk)
        return 0;

    // Transfer length includes the header; the top bit flags a continuation.
    uint16_t packetSize = ((uint16_t)header[0] | (uint16_t)header[1] << 8) & ~0x8000;

    if (packetSize == 0 || packetSize > len)
        return 0;

    uint16_t remaining = packetSize;
    bool firstRead = true;

    while (remaining > 0)
    {
        if (!waitForInterrupt())
            return 0;

        // The first chunk includes the header as part of the transfer. Later chunks are new bus
        // transactions that start with a repeated header, which is read over the tail of the data
        // already received and then restored.
        size_t readSize;
        uint8_t *readDest;
        uint8_t saved[ksfBNO08xSHTPHeaderLen];

        if (firstRead)
        {
            readSize = remaining < _maxTransferSize ? remaining : _maxTransferSize;
            readDest = pBuffer;
        }
        else
        {
            readSize = remaining + ksfBNO08xSHTPHeaderLen;
            if (readSize > _maxTransferSize)
                readSize = _maxTransferSize;
            readDest = pBuffer - ksfBNO08xSHTPHeaderLen;
            memcpy(saved, readDest, sizeof(saved));
        }

        sfTkError_t status = readBusData(readDest, readSize);

        if (!firstRead)
            memcpy(readDest, saved, sizeof(saved));

        if (status != ksfTkErrOk)
            return 0;

        uint16_t cargoRead = firstRead ? readSize : readSize - ksfBNO08xSHTPHeaderLen;
        pBuffer += cargoRead;
        remaining -= cargoRead;
        firstRead = false;
    }

    return packetSize;
}

//--------------------------------------------------------------------------------------------
// Write a transfer to the device. Returns the number of bytes accepted, or 0 if not written.
int sfDevBNO08x::write(uint8_t *pBuffer, unsigned len)
{
    size_t writeSize = len < _maxTransferSize ? len : _maxTransferSize;

    if (!waitForInterrupt())
        return 0;

    if (_theBus->writeData(pBuffer, writeSize) != ksfTkErrOk)
        return 0;

    return writeSize;
}

//--------------------------------------------------------------------------------------------
// SH2 callbacks
//--------------------------------------------------------------------------------------------

void sfDevBNO08x::asyncEventHandler(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    if (pEvent->eventId == SH2_RESET)
        static_cast<sfDevBNO08x *>(cookie)->_resetOccurred = true;
}

void sfDevBNO08x::sensorEventHandler(void *cookie, sh2_SensorEvent_t *pEvent)
{
    sfDevBNO08x *device = static_cast<sfDevBNO08x *>(cookie);

    if (sh2_decodeSensorEvent(&device->sensorValue, pEvent) != SH2_OK)
    {
        device->sensorValue.timestamp = 0;
        return;
    }

    if (device->sensorValue.sensorId == SH2_TAP_DETECTOR)
        device->_tapDetector = device->sensorValue.un.tapDetector.flags;

    device->_eventReceived = true;
}

//--------------------------------------------------------------------------------------------
// Device control
//--------------------------------------------------------------------------------------------

void sfDevBNO08x::hardwareReset(void)
{
    if (!hasResetPin())
        return;

    setResetPin(true);
    sftk_delay_ms(10);
    setResetPin(false);
    sftk_delay_ms(10);
    setResetPin(true);
    sftk_delay_ms(10);
}

bool sfDevBNO08x::wasReset(void)
{
    bool x = _resetOccurred;
    _resetOccurred = false;

    return x;
}

uint8_t sfDevBNO08x::getResetReason(void)
{
    return prodIds.entry[0].resetCause;
}

bool sfDevBNO08x::serviceBus(void)
{
    sh2_service();
    return true;
}

bool sfDevBNO08x::softReset(void)
{
    return sh2_devReset() == SH2_OK;
}

bool sfDevBNO08x::modeOn(void)
{
    return sh2_devOn() == SH2_OK;
}

bool sfDevBNO08x::modeSleep(void)
{
    return sh2_devSleep() == SH2_OK;
}

//--------------------------------------------------------------------------------------------
bool sfDevBNO08x::getSensorEvent(void)
{
    _eventReceived = false;

    sh2_service();

    return _eventReceived;
}

uint8_t sfDevBNO08x::getSensorEventID(void)
{
    return sensorValue.sensorId;
}

//--------------------------------------------------------------------------------------------
bool sfDevBNO08x::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us, uint32_t sensorSpecific)
{
    sh2_SensorConfig_t config;

    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = sensorSpecific;
    config.reportInterval_us = interval_us;

    if (!waitForInterrupt())
        return false;

    return sh2_setSensorConfig(sensorId, &config) == SH2_OK;
}

bool sfDevBNO08x::enableReportMs(sh2_SensorId_t sensorId, uint16_t timeBetweenReports, uint32_t sensorSpecific)
{
    return enableReport(sensorId, (uint32_t)timeBetweenReports * 1000, sensorSpecific);
}

bool sfDevBNO08x::enableRotationVector(uint16_t timeBetweenReports)
{
    return enableReportMs(SH2_ROTATION_VECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableGeomagneticRotationVector(uint16_t timeBetweenReports)
{
    return enableReportMs(SH2_GEOMAGNETIC_ROTATION_VECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableGameRotationVector(uint16_t timeBetweenReports)
{
    return enableReportMs(SH2_GAME_ROTATION_VECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableAccelerometer(uint16_t timeBetweenReports)
{
    return enableReportMs(SH2_ACCELEROMETER, timeBetweenReports);
}

bool sfDevBNO08x::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

bool sfDevBNO08x::enableGravity(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
}

bool sfDevBNO08x::enableGyro(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, timeBetweenReports);
}

bool sfDevBNO08x::enableUncalibratedGyro(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_UNCALIBRATED_GYRO, timeBetweenReports);
}

bool sfDevBNO08x::enableMagnetometer(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

bool sfDevBNO08x::enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableTapDetector(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);
}

bool sfDevBNO08x::enableStepCounter(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

bool sfDevBNO08x::enableStabilityClassifier(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

bool sfDevBNO08x::enableRawAccelerometer(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

bool sfDevBNO08x::enableRawGyro(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

bool sfDevBNO08x::enableRawMagnetometer(uint16_t timeBetweenReports)
{
    return enableReportMs(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

bool sfDevBNO08x::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable)
{
    return enableReportMs(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

//--------------------------------------------------------------------------------------------
// Calibration and tare
//--------------------------------------------------------------------------------------------

bool sfDevBNO08x::setCalibrationConfig(uint8_t sensors)
{
    return sh2_setCalConfig(sensors) == SH2_OK;
}

bool sfDevBNO08x::saveCalibration(void)
{
    return sh2_saveDcdNow() == SH2_OK;
}

bool sfDevBNO08x::tareNow(bool zAxis, sh2_TareBasis_t basis)
{
    return sh2_setTareNow(zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, basis) == SH2_OK;
}

bool sfDevBNO08x::saveTare(void)
{
    return sh2_persistTare() == SH2_OK;
}

bool sfDevBNO08x::clearTare(void)
{
    return sh2_clearTare() == SH2_OK;
}

//--------------------------------------------------------------------------------------------
// Data access
//--------------------------------------------------------------------------------------------

float sfDevBNO08x::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    return ldexpf((float)fixedPointValue, -qPoint);
}

// Quaternion to Euler conversion
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440
float sfDevBNO08x::getRoll(void)
{
    float dqw = getQuatReal();
    float dqx = getQuatI();
    float dqy = getQuatJ();
    float dqz = getQuatK();

    float norm = sqrtf(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    float t0 = +2.0f * (dqw * dqx + dqy * dqz);
    float t1 = +1.0f - 2.0f * (dqx * dqx + ysqr);

    return atan2f(t0, t1);
}

float sfDevBNO08x::getPitch(void)
{
    float dqw = getQuatReal();
    float dqx = getQuatI();
    float dqy = getQuatJ();
    float dqz = getQuatK();

    float norm = sqrtf(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float t2 = +2.0f * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    return asinf(t2);
}

float sfDevBNO08x::getYaw(void)
{
    float dqw = getQuatReal();
    float dqx = getQuatI();
    float dqy = getQuatJ();
    float dqz = getQuatK();

    float norm = sqrtf(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    float t3 = +2.0f * (dqw * dqz + dqx * dqy);
    float t4 = +1.0f - 2.0f * (ysqr + dqz * dqz);

    return atan2f(t3, t4);
}

// Rotation vector
void sfDevBNO08x::getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy)
{
    i = sensorValue.un.rotationVector.i;
    j = sensorValue.un.rotationVector.j;
    k = sensorValue.un.rotationVector.k;
    real = sensorValue.un.rotationVector.real;
    radAccuracy = sensorValue.un.rotationVector.accuracy;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getQuatI(void)
{
    return sensorValue.un.rotationVector.i;
}

float sfDevBNO08x::getQuatJ(void)
{
    return sensorValue.un.rotationVector.j;
}

float sfDevBNO08x::getQuatK(void)
{
    return sensorValue.un.rotationVector.k;
}

float sfDevBNO08x::getQuatReal(void)
{
    return sensorValue.un.rotationVector.real;
}

float sfDevBNO08x::getQuatRadianAccuracy(void)
{
    return sensorValue.un.rotationVector.accuracy;
}

uint8_t sfDevBNO08x::getQuatAccuracy(void)
{
    return sensorValue.status;
}

// Game rotation vector
float sfDevBNO08x::getGameQuatI(void)
{
    return sensorValue.un.gameRotationVector.i;
}

float sfDevBNO08x::getGameQuatJ(void)
{
    return sensorValue.un.gameRotationVector.j;
}

float sfDevBNO08x::getGameQuatK(void)
{
    return sensorValue.un.gameRotationVector.k;
}

float sfDevBNO08x::getGameQuatReal(void)
{
    return sensorValue.un.gameRotationVector.real;
}

// Accelerometer
void sfDevBNO08x::getAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
    x = sensorValue.un.accelerometer.x;
    y = sensorValue.un.accelerometer.y;
    z = sensorValue.un.accelerometer.z;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getAccelX(void)
{
    return sensorValue.un.accelerometer.x;
}

float sfDevBNO08x::getAccelY(void)
{
    return sensorValue.un.accelerometer.y;
}

float sfDevBNO08x::getAccelZ(void)
{
    return sensorValue.un.accelerometer.z;
}

uint8_t sfDevBNO08x::getAccelAccuracy(void)
{
    return sensorValue.status;
}

// Linear acceleration, i.e. minus gravity
void sfDevBNO08x::getLinAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
    x = sensorValue.un.linearAcceleration.x;
    y = sensorValue.un.linearAcceleration.y;
    z = sensorValue.un.linearAcceleration.z;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getLinAccelX(void)
{
    return sensorValue.un.linearAcceleration.x;
}

float sfDevBNO08x::getLinAccelY(void)
{
    return sensorValue.un.linearAcceleration.y;
}

float sfDevBNO08x::getLinAccelZ(void)
{
    return sensorValue.un.linearAcceleration.z;
}

uint8_t sfDevBNO08x::getLinAccelAccuracy(void)
{
    return sensorValue.status;
}

// Gyroscope
void sfDevBNO08x::getGyro(float &x, float &y, float &z, uint8_t &accuracy)
{
    x = sensorValue.un.gyroscope.x;
    y = sensorValue.un.gyroscope.y;
    z = sensorValue.un.gyroscope.z;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getGyroX(void)
{
    return sensorValue.un.gyroscope.x;
}

float sfDevBNO08x::getGyroY(void)
{
    return sensorValue.un.gyroscope.y;
}

float sfDevBNO08x::getGyroZ(void)
{
    return sensorValue.un.gyroscope.z;
}

uint8_t sfDevBNO08x::getGyroAccuracy(void)
{
    return sensorValue.status;
}

// Uncalibrated gyroscope
void sfDevBNO08x::getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz,
                                      uint8_t &accuracy)
{
    x = sensorValue.un.gyroscopeUncal.x;
    y = sensorValue.un.gyroscopeUncal.y;
    z = sensorValue.un.gyroscopeUncal.z;
    bx = sensorValue.un.gyroscopeUncal.biasX;
    by = sensorValue.un.gyroscopeUncal.biasY;
    bz = sensorValue.un.gyroscopeUncal.biasZ;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getUncalibratedGyroX(void)
{
    return sensorValue.un.gyroscopeUncal.x;
}

float sfDevBNO08x::getUncalibratedGyroY(void)
{
    return sensorValue.un.gyroscopeUncal.y;
}

float sfDevBNO08x::getUncalibratedGyroZ(void)
{
    return sensorValue.un.gyroscopeUncal.z;
}

float sfDevBNO08x::getUncalibratedGyroBiasX(void)
{
    return sensorValue.un.gyroscopeUncal.biasX;
}

float sfDevBNO08x::getUncalibratedGyroBiasY(void)
{
    return sensorValue.un.gyroscopeUncal.biasY;
}

float sfDevBNO08x::getUncalibratedGyroBiasZ(void)
{
    return sensorValue.un.gyroscopeUncal.biasZ;
}

uint8_t sfDevBNO08x::getUncalibratedGyroAccuracy(void)
{
    return sensorValue.status;
}

// Gravity
void sfDevBNO08x::getGravity(float &x, float &y, float &z, uint8_t &accuracy)
{
    x = sensorValue.un.gravity.x;
    y = sensorValue.un.gravity.y;
    z = sensorValue.un.gravity.z;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getGravityX(void)
{
    return sensorValue.un.gravity.x;
}

float sfDevBNO08x::getGravityY(void)
{
    return sensorValue.un.gravity.y;
}

float sfDevBNO08x::getGravityZ(void)
{
    return sensorValue.un.gravity.z;
}

uint8_t sfDevBNO08x::getGravityAccuracy(void)
{
    return sensorValue.status;
}

// Magnetometer
void sfDevBNO08x::getMag(float &x, float &y, float &z, uint8_t &accuracy)
{
    x = sensorValue.un.magneticField.x;
    y = sensorValue.un.magneticField.y;
    z = sensorValue.un.magneticField.z;
    accuracy = sensorValue.status;
}

float sfDevBNO08x::getMagX(void)
{
    return sensorValue.un.magneticField.x;
}

float sfDevBNO08x::getMagY(void)
{
    return sensorValue.un.magneticField.y;
}

float sfDevBNO08x::getMagZ(void)
{
    return sensorValue.un.magneticField.z;
}

uint8_t sfDevBNO08x::getMagAccuracy(void)
{
    return sensorValue.status;
}

// Gyro integrated rotation vector
float sfDevBNO08x::getGyroIntegratedRVI(void)
{
    return sensorValue.un.gyroIntegratedRV.i;
}

float sfDevBNO08x::getGyroIntegratedRVJ(void)
{
    return sensorValue.un.gyroIntegratedRV.j;
}

float sfDevBNO08x::getGyroIntegratedRVK(void)
{
    return sensorValue.un.gyroIntegratedRV.k;
}

float sfDevBNO08x::getGyroIntegratedRVReal(void)
{
    return sensorValue.un.gyroIntegratedRV.real;
}

float sfDevBNO08x::getGyroIntegratedRVangVelX(void)
{
    return sensorValue.un.gyroIntegratedRV.angVelX;
}

float sfDevBNO08x::getGyroIntegratedRVangVelY(void)
{
    return sensorValue.un.gyroIntegratedRV.angVelY;
}

float sfDevBNO08x::getGyroIntegratedRVangVelZ(void)
{
    return sensorValue.un.gyroIntegratedRV.angVelZ;
}

// Classifiers, counters and timestamp
uint8_t sfDevBNO08x::getTapDetector(void)
{
    uint8_t previousTapDetector = _tapDetector;
    _tapDetector = 0; // Reset so user code sees exactly one tap
    return previousTapDetector;
}

uint16_t sfDevBNO08x::getStepCount(void)
{
    return sensorValue.un.stepCounter.steps;
}

uint8_t sfDevBNO08x::getStabilityClassifier(void)
{
    return sensorValue.un.stabilityClassifier.classification;
}

uint8_t sfDevBNO08x::getActivityClassifier(void)
{
    return sensorValue.un.personalActivityClassifier.mostLikelyState;
}

uint8_t sfDevBNO08x::getActivityConfidence(uint8_t activity)
{
    return sensorValue.un.personalActivityClassifier.confidence[activity];
}

uint64_t sfDevBNO08x::getTimeStamp(void)
{
    return sensorValue.timestamp;
}

// Raw MEMS values
int16_t sfDevBNO08x::getRawAccelX(void)
{
    return sensorValue.un.rawAccelerometer.x;
}

int16_t sfDevBNO08x::getRawAccelY(void)
{
    return sensorValue.un.rawAccelerometer.y;
}

int16_t sfDevBNO08x::getRawAccelZ(void)
{
    return sensorValue.un.rawAccelerometer.z;
}

int16_t sfDevBNO08x::getRawGyroX(void)
{
    return sensorValue.un.rawGyroscope.x;
}

int16_t sfDevBNO08x::getRawGyroY(void)
{
    return sensorValue.un.rawGyroscope.y;
}

int16_t sfDevBNO08x::getRawGyroZ(void)
{
    return sensorValue.un.rawGyroscope.z;
}

int16_t sfDevBNO08x::getRawMagX(void)
{
    return sensorValue.un.rawMagnetometer.x;
}

int16_t sfDevBNO08x::getRawMagY(void)
{
    return sensorValue.un.rawMagnetometer.y;
}

int16_t sfDevBNO08x::getRawMagZ(void)
{
    return sensorValue.un.rawMagnetometer.z;
}
