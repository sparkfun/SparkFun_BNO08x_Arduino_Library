/*
  This is a library written for the BNO08x
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the 
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert 
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit 
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  The BNO08x IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO08x and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library

  Development environment specifics:
  Arduino IDE 2.1.1

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Some of this library was based off of the Adafruit BNO08x Arduino Library.
  More specifically, the code layers connecting to the HillCrest/Ceva Driver.
  Their original work can be found here:
  https://github.com/adafruit/Adafruit_BNO08x
  Thank you Adafruit and your developers for all your hard work put into your Library!
*/

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <SPI.h>

//The default I2C address for the BNO08x on the SparkFun breakout is 0x4B. 0x4A is also possible.
#define BNO08x_DEFAULT_ADDRESS 0x4B

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH 32

//#else

//The catch-all default is 32
//#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//All the ways we can configure or talk to the BNO08x, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
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

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
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
#define TARE_AXIS_Z   0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

class BNO08x
{
public:
	boolean begin(uint8_t deviceAddress = BNO08x_DEFAULT_ADDRESS, TwoWire &wirePort = Wire, int8_t user_INTPin = -1, int8_t user_RSTPin = -1); //By default use the default I2C addres, and use Wire port
	boolean beginSPI(uint8_t user_CSPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed = 1000000, SPIClass &spiPort = SPI);
	boolean isConnected();

    sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor
	sh2_SensorValue_t sensorValue;

    void hardwareReset(void);
    bool wasReset(void); //Returns true if the sensor has reported a reset. Reading this will unflag the reset.

	uint8_t getResetReason(); // returns prodIds->resetCause

    bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000, uint32_t sensorSpecific = 0);
    bool getSensorEvent();
	uint8_t getSensorEventID();

	void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

	bool softReset();	  //Try to reset the IMU via software
	bool serviceBus(void);	
	uint8_t resetReason(); //Query the IMU for the reason it last reset
	bool modeOn();	  //Use the executable channel to turn the BNO on
	bool modeSleep();	  //Use the executable channel to put the BNO to sleep

	float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number

	bool enableRotationVector(uint16_t timeBetweenReports = 10);
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
	float getQuatI();
	float getQuatJ();
	float getQuatK();
	float getQuatReal();
	float getQuatRadianAccuracy();
	uint8_t getQuatAccuracy();

	void getAccel(float &x, float &y, float &z, uint8_t &accuracy);
	float getAccelX();
	float getAccelY();
	float getAccelZ();
	uint8_t getAccelAccuracy();

	void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	uint8_t getLinAccelAccuracy();

	void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
	float getGyroX();
	float getGyroY();
	float getGyroZ();
	uint8_t getGyroAccuracy();

	void getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, uint8_t &accuracy);
	float getUncalibratedGyroX();
	float getUncalibratedGyroY();
	float getUncalibratedGyroZ();
	float getUncalibratedGyroBiasX();
	float getUncalibratedGyroBiasY();
	float getUncalibratedGyroBiasZ();
	uint8_t getUncalibratedGyroAccuracy();

	float getGyroIntegratedRVI();
	float getGyroIntegratedRVJ();
	float getGyroIntegratedRVK();
	float getGyroIntegratedRVReal();
	float getGyroIntegratedRVangVelX();
	float getGyroIntegratedRVangVelY();
	float getGyroIntegratedRVangVelZ();

	void getMag(float &x, float &y, float &z, uint8_t &accuracy);
	float getMagX();
	float getMagY();
	float getMagZ();
	uint8_t getMagAccuracy();

	void getGravity(float &x, float &y, float &z, uint8_t &accuracy);
	float getGravityX();
	float getGravityY();
	float getGravityZ();
	uint8_t getGravityAccuracy();

	// void calibrateAccelerometer();
	// void calibrateGyro();
	// void calibrateMagnetometer();
	// void calibratePlanarAccelerometer();
	// void calibrateAll();
	// void endCalibration();
	// void saveCalibration();
	// void requestCalibrationStatus(); //Sends command to get status
	// boolean calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status

	bool tareNow(bool zAxis=false, sh2_TareBasis_t basis=SH2_TARE_BASIS_ROTATION_VECTOR);
	bool saveTare();
	bool clearTare();

	bool setReorientation(sh2_Quaternion_t *orientation);
	
	uint8_t getTapDetector();
	uint64_t getTimeStamp();
	uint16_t getStepCount();
	uint8_t getStabilityClassifier();
	uint8_t getActivityClassifier();
	uint8_t getActivityConfidence(uint8_t activity);

	int16_t getRawAccelX();
	int16_t getRawAccelY();
	int16_t getRawAccelZ();

	int16_t getRawGyroX();
	int16_t getRawGyroY();
	int16_t getRawGyroZ();

	int16_t getRawMagX();
	int16_t getRawMagY();
	int16_t getRawMagZ();

	float getRoll();
	float getPitch();
	float getYaw();

//	void sendCommand(uint8_t command);
//	void sendCalibrateCommand(uint8_t thingToCalibrate);

	//Metadata functions
	// int16_t getQ1(uint16_t recordID);
	// int16_t getQ2(uint16_t recordID);
	// int16_t getQ3(uint16_t recordID);
	// float getResolution(uint16_t recordID);
	// float getRange(uint16_t recordID);
	// uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
	// void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
	// bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

	//Global Variables
	// uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	// uint8_t shtpData[MAX_PACKET_SIZE];
	// uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	// uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	// uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

//	unsigned long _spiPortSpeed; //Optional user defined port speed
//	uint8_t _cs;				 //Pins needed for SPI

private:

	Stream *_debugPort;			 //The stream to send debug messages to if enabled. Usually Serial.
	boolean _printDebug = false; //Flag to print debugging variables

	//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
	uint8_t tapDetector;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;
	int16_t gravity_Q1 = 8;

protected:
	virtual bool _init(int32_t sensor_id = 0);
	sh2_Hal_t _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};