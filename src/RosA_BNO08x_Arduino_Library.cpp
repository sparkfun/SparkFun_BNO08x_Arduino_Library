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
  Arduino IDE 1.8.5

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Some of this library was based off of the Adafruit BNO08x Arduino Library.
  More specifically, the code layers connecting to the HillCrest/Ceva Driver.
  Their original work can be found here:
  https://github.com/adafruit/Adafruit_BNO08x
  Thank you Adafruit and your developers for all your hard work put into your Library!
*/

#include "RosA_BNO08x_Arduino_Library.h"

int8_t _int_pin = -1, _reset_pin = -1;
static TwoWire *_i2cPort = NULL;		//The generic connection to user's chosen I2C hardware
static SPIClass *_spiPort = NULL;  		//The generic connection to user's chosen SPI hardware
static uint8_t _deviceAddress = BNO08x_DEFAULT_ADDRESS; //Keeps track of I2C address. setI2CAddress changes this.
unsigned long _spiPortSpeed = 3000000; //Optional user defined port speed
uint8_t _cs;				 //Pin needed for SPI


static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(sh2_Hal_t *self);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hal_hardwareReset(void);

static bool i2c_write(const uint8_t *buffer, size_t len, bool stop = true,
        const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);

static bool i2c_read(uint8_t *buffer, size_t len, bool stop = true);
static bool _i2c_read(uint8_t *buffer, size_t len, bool stop);

static bool hal_wait_for_int(void);
static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
static void spihal_close(sh2_Hal_t *self);
static int spihal_open(sh2_Hal_t *self);

static bool spi_read(uint8_t *buffer, size_t len, uint8_t sendvalue = 0xFF);
static bool spi_write(const uint8_t *buffer, size_t len,
			const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
			

size_t _maxBufferSize = 32;
size_t maxBufferSize();		

//Initializes the sensor with basic settings using I2C
//Returns false if sensor is not detected
boolean BNO08x::begin(uint8_t deviceAddress, TwoWire &wirePort, int8_t user_INTPin, int8_t user_RSTPin)
{
  	_deviceAddress = deviceAddress;
  	_i2cPort = &wirePort;

	// if user passes in an INT pin, then lets set that up.
	if(user_INTPin != -1)
	{
		_int_pin = user_INTPin;
		pinMode(_int_pin, INPUT_PULLUP);
	}

	// if user passes in an RESET pin, then lets set that up.
	if(user_RSTPin != -1)
	{
		_reset_pin = user_RSTPin;
		pinMode(_reset_pin, INPUT_PULLUP);
	}

  	if(_int_pin != -1) hal_wait_for_int();

  	if (isConnected() == false) // Check for sensor by verifying ACK response
    	return (false); 

	Serial.println(F("I2C address found"));
    //delay(1000);

    _HAL.open = i2chal_open;
    _HAL.close = i2chal_close;
    _HAL.read = i2chal_read;
    _HAL.write = i2chal_write;
    _HAL.getTimeUs = hal_getTimeUs;

    return _init();
}

//Initializes the sensor with basic settings using SPI
//Returns false if sensor is not detected
boolean BNO08x::beginSPI(uint8_t user_CSPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	//Get user settings
	_spiPort = &spiPort;
	_spiPortSpeed = spiPortSpeed;
	if (_spiPortSpeed > 3000000)
		_spiPortSpeed = 3000000; //BNO08x max is 3MHz

	_cs = user_CSPin;
	_int_pin = user_INTPin;
	_reset_pin = user_RSTPin;

	pinMode(_cs, OUTPUT);
	pinMode(_int_pin, INPUT_PULLUP);
	pinMode(_reset_pin, OUTPUT);

	digitalWrite(_cs, HIGH); //Deselect BNO08x

	_spiPort->begin(); //Turn on SPI hardware

	_HAL.open = spihal_open;
	_HAL.close = spihal_close;
	_HAL.read = spihal_read;
	_HAL.write = spihal_write;
	_HAL.getTimeUs = hal_getTimeUs;

    return _init();
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void BNO08x::enableDebugging(Stream &debugPort)
{
	_debugPort = &debugPort;
	_printDebug = true;
}

// Quaternion to Euler conversion
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440
// Return the roll (rotation around the x-axis) in Radians
float BNO08x::getRoll()
{
	float dqw = getQuatReal();
	float dqx = getQuatI();
	float dqy = getQuatJ();
	float dqz = getQuatK();

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;

	// roll (x-axis rotation)
	float t0 = +2.0 * (dqw * dqx + dqy * dqz);
	float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
	float roll = atan2(t0, t1);

	return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float BNO08x::getPitch()
{
	float dqw = getQuatReal();
	float dqx = getQuatI();
	float dqy = getQuatJ();
	float dqz = getQuatK();

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	//float ysqr = dqy * dqy;

	// pitch (y-axis rotation)
	float t2 = +2.0 * (dqw * dqy - dqz * dqx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float pitch = asin(t2);

	return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float BNO08x::getYaw()
{
	float dqw = getQuatReal();
	float dqx = getQuatI();
	float dqy = getQuatJ();
	float dqz = getQuatK();

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;

	// yaw (z-axis rotation)
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = atan2(t3, t4);

	return (yaw);
}

//Gets the full quaternion
//i,j,k,real output floats
void BNO08x::getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy)
{
	i = qToFloat(rawQuatI, rotationVector_Q1);
	j = qToFloat(rawQuatJ, rotationVector_Q1);
	k = qToFloat(rawQuatK, rotationVector_Q1);
	real = qToFloat(rawQuatReal, rotationVector_Q1);
	radAccuracy = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
	accuracy = quatAccuracy;
}

//Return the rotation vector quaternion I
float BNO08x::getQuatI()
{
	// float quat = qToFloat(rawQuatI, rotationVector_Q1);
	// if (_printDebug == true)
	// {
	// 	if ((quat < -1.0) || (quat > 1.0))
	// 	{
	// 		_debugPort->print(F("getQuatI: quat: ")); // Debug the occasional non-unitary Quat
	// 		_debugPort->print(quat, 2);
	// 		_debugPort->print(F(" rawQuatI: "));
	// 		_debugPort->print(rawQuatI);
	// 		_debugPort->print(F(" rotationVector_Q1: "));
	// 		_debugPort->println(rotationVector_Q1);
	// 	}
	// }
	// return (quat);
	return _sensor_value->un.rotationVector.i;
}

//Return the rotation vector quaternion J
float BNO08x::getQuatJ()
{
	float quat = qToFloat(rawQuatJ, rotationVector_Q1);
	if (_printDebug == true)
	{
		if ((quat < -1.0) || (quat > 1.0)) // Debug the occasional non-unitary Quat
		{
			_debugPort->print(F("getQuatJ: quat: "));
			_debugPort->print(quat, 2);
			_debugPort->print(F(" rawQuatJ: "));
			_debugPort->print(rawQuatJ);
			_debugPort->print(F(" rotationVector_Q1: "));
			_debugPort->println(rotationVector_Q1);
		}
	}
	//return (quat);
	return _sensor_value->un.rotationVector.j;
}

//Return the rotation vector quaternion K
float BNO08x::getQuatK()
{
	float quat = qToFloat(rawQuatK, rotationVector_Q1);
	if (_printDebug == true)
	{
		if ((quat < -1.0) || (quat > 1.0)) // Debug the occasional non-unitary Quat
		{
			_debugPort->print(F("getQuatK: quat: "));
			_debugPort->print(quat, 2);
			_debugPort->print(F(" rawQuatK: "));
			_debugPort->print(rawQuatK);
			_debugPort->print(F(" rotationVector_Q1: "));
			_debugPort->println(rotationVector_Q1);
		}
	}
	//return (quat);
	return _sensor_value->un.rotationVector.k;
}

//Return the rotation vector quaternion Real
float BNO08x::getQuatReal()
{
	return _sensor_value->un.rotationVector.real;
}

//Return the rotation vector radian accuracy
float BNO08x::getQuatRadianAccuracy()
{
	return _sensor_value->un.rotationVector.accuracy;
}

//Return the rotation vector sensor event report status accuracy
uint8_t BNO08x::getQuatAccuracy()
{
	return _sensor_value->status;
}

//Return the game rotation vector quaternion I
float BNO08x::getGameQuatI()
{
	return _sensor_value->un.gameRotationVector.i;
}

//Return the game rotation vector quaternion J
float BNO08x::getGameQuatJ()
{
	return _sensor_value->un.gameRotationVector.j;
}

//Return the game rotation vector quaternion K
float BNO08x::getGameQuatK()
{
	return _sensor_value->un.gameRotationVector.k;
}

//Return the game rotation vector quaternion Real
float BNO08x::getGameQuatReal()
{
	return _sensor_value->un.gameRotationVector.real;
}

//Gets the full acceleration
//x,y,z output floats
void BNO08x::getAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawAccelX, accelerometer_Q1);
	y = qToFloat(rawAccelY, accelerometer_Q1);
	z = qToFloat(rawAccelZ, accelerometer_Q1);
	accuracy = accelAccuracy;
}

//Return the acceleration component
float BNO08x::getAccelX()
{
	return _sensor_value->un.accelerometer.x;
}

//Return the acceleration component
float BNO08x::getAccelY()
{
	return _sensor_value->un.accelerometer.y;
}

//Return the acceleration component
float BNO08x::getAccelZ()
{
	return _sensor_value->un.accelerometer.z;
}

//Return the acceleration component
uint8_t BNO08x::getAccelAccuracy()
{
	return _sensor_value->status;
}

// linear acceleration, i.e. minus gravity

//Gets the full lin acceleration
//x,y,z output floats
void BNO08x::getLinAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	y = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	z = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	accuracy = accelLinAccuracy;
}

//Return the acceleration component
float BNO08x::getLinAccelX()
{
	return _sensor_value->un.linearAcceleration.x;
}

//Return the acceleration component
float BNO08x::getLinAccelY()
{
	return _sensor_value->un.linearAcceleration.y;
}

//Return the acceleration component
float BNO08x::getLinAccelZ()
{
	return _sensor_value->un.linearAcceleration.z;
}

//Return the acceleration component
uint8_t BNO08x::getLinAccelAccuracy()
{
	return _sensor_value->status;
}

//Gets the full gyro vector
//x,y,z output floats
void BNO08x::getGyro(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawGyroX, gyro_Q1);
	y = qToFloat(rawGyroY, gyro_Q1);
	z = qToFloat(rawGyroZ, gyro_Q1);
	accuracy = gyroAccuracy;
}

//Return the gyro component
float BNO08x::getGyroX()
{
	return _sensor_value->un.gyroscope.x;
}

//Return the gyro component
float BNO08x::getGyroY()
{
	return _sensor_value->un.gyroscope.y;
}

//Return the gyro component
float BNO08x::getGyroZ()
{
	return _sensor_value->un.gyroscope.z;
}

//Return the gyro component
uint8_t BNO08x::getGyroAccuracy()
{
	return (gyroAccuracy);
}

//Gets the full uncalibrated gyro vector
//x,y,z,bx,by,bz output floats
void BNO08x::getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, uint8_t &accuracy)
{
	x = _sensor_value->un.gyroscopeUncal.x;
	y = _sensor_value->un.gyroscopeUncal.y;
	z = _sensor_value->un.gyroscopeUncal.z;
	bx = _sensor_value->un.gyroscopeUncal.biasX;
	by = _sensor_value->un.gyroscopeUncal.biasY;
	bz = _sensor_value->un.gyroscopeUncal.biasZ;
	accuracy = _sensor_value->status;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroX()
{
	return _sensor_value->un.gyroscopeUncal.x;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroY()
{
	return _sensor_value->un.gyroscopeUncal.y;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroZ()
{
	return _sensor_value->un.gyroscopeUncal.z;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroBiasX()
{
	return _sensor_value->un.gyroscopeUncal.biasX;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroBiasY()
{
	return _sensor_value->un.gyroscopeUncal.biasY;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroBiasZ()
{
	return _sensor_value->un.gyroscopeUncal.biasZ;
}

//Return the gyro component
uint8_t BNO08x::getUncalibratedGyroAccuracy()
{
	return (UncalibGyroAccuracy);
}

//Gets the full gravity vector
//x,y,z output floats
void BNO08x::getGravity(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(gravityX, gravity_Q1);
	y = qToFloat(gravityX, gravity_Q1);
	z = qToFloat(gravityX, gravity_Q1);
	accuracy = gravityAccuracy;
}

float BNO08x::getGravityX()
{
	return _sensor_value->un.gravity.x;
}

//Return the gravity component
float BNO08x::getGravityY()
{
	return _sensor_value->un.gravity.y;
}

//Return the gravity component
float BNO08x::getGravityZ()
{
	return _sensor_value->un.gravity.z;
}

uint8_t BNO08x::getGravityAccuracy()
{
	return _sensor_value->status;
}

//Gets the full mag vector
//x,y,z output floats
void BNO08x::getMag(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawMagX, magnetometer_Q1);
	y = qToFloat(rawMagY, magnetometer_Q1);
	z = qToFloat(rawMagZ, magnetometer_Q1);
	accuracy = magAccuracy;
}

//Return the magnetometer component
float BNO08x::getMagX()
{
	return _sensor_value->un.magneticField.x;
}

//Return the magnetometer component
float BNO08x::getMagY()
{
	return _sensor_value->un.magneticField.y;
}

//Return the magnetometer component
float BNO08x::getMagZ()
{
	return _sensor_value->un.magneticField.z;
}

//Return the mag component
uint8_t BNO08x::getMagAccuracy()
{
	return _sensor_value->status;
}

// Return Gyro Integrated Rotation Vector i
float BNO08x::getGyroIntegratedRVI()
{
	return _sensor_value->un.gyroIntegratedRV.i;
}

// Return Gyro Integrated Rotation Vector j
float BNO08x::getGyroIntegratedRVJ()
{
	return _sensor_value->un.gyroIntegratedRV.j;
}

// Return Gyro Integrated Rotation Vector k
float BNO08x::getGyroIntegratedRVK()
{
	return _sensor_value->un.gyroIntegratedRV.k;
}

// Return Gyro Integrated Rotation Vector real
float BNO08x::getGyroIntegratedRVReal()
{
	return _sensor_value->un.gyroIntegratedRV.real;
}

// Return Gyro Integrated Rotation Vector angVelX
float BNO08x::getGyroIntegratedRVangVelX()
{
	return _sensor_value->un.gyroIntegratedRV.angVelX;
}

// Return Gyro Integrated Rotation Vector angVelY
float BNO08x::getGyroIntegratedRVangVelY()
{
	return _sensor_value->un.gyroIntegratedRV.angVelY;
}

// Return Gyro Integrated Rotation Vector angVelZ
float BNO08x::getGyroIntegratedRVangVelZ()
{
	return _sensor_value->un.gyroIntegratedRV.angVelZ;
}

//Return the tap detector
uint8_t BNO08x::getTapDetector()
{
//	uint8_t previousTapDetector = tapDetector;
//	tapDetector = 0; //Reset so user code sees exactly one tap
//	return (previousTapDetector);
	return _sensor_value->un.tapDetector.flags;
}
//Return shake detector
uint16_t BNO08x::getShakeDetector()
{
	return _sensor_value->un.shakeDetector.shake;
}


//Return the step count
uint16_t BNO08x::getStepCount()
{
	return _sensor_value->un.stepCounter.steps;
}

//Return the stability classifier
uint8_t BNO08x::getStabilityClassifier()
{
	return _sensor_value->un.stabilityClassifier.classification;
}

//Return the activity classifier
uint8_t BNO08x::getActivityClassifier()
{
	return _sensor_value->un.personalActivityClassifier.mostLikelyState;
}

//Return the activity confindence
uint8_t BNO08x::getActivityConfidence(uint8_t activity)
{
	return _sensor_value->un.personalActivityClassifier.confidence[activity];
}

//Return the time stamp
uint64_t BNO08x::getTimeStamp()
{
	return _sensor_value->timestamp;
}

//Return raw mems value for the accel
int16_t BNO08x::getRawAccelX()
{
	return _sensor_value->un.rawAccelerometer.x;
}
//Return raw mems value for the accel
int16_t BNO08x::getRawAccelY()
{
	return _sensor_value->un.rawAccelerometer.y;
}
//Return raw mems value for the accel
int16_t BNO08x::getRawAccelZ()
{
	return _sensor_value->un.rawAccelerometer.z;
}

//Return raw mems value for the gyro
int16_t BNO08x::getRawGyroX()
{
	return _sensor_value->un.rawGyroscope.x;
}
int16_t BNO08x::getRawGyroY()
{
	return _sensor_value->un.rawGyroscope.y;
}
int16_t BNO08x::getRawGyroZ()
{
	return _sensor_value->un.rawGyroscope.z;
}

//Return raw mems value for the mag
int16_t BNO08x::getRawMagX()
{
	return _sensor_value->un.rawMagnetometer.x;
}
int16_t BNO08x::getRawMagY()
{
	return _sensor_value->un.rawMagnetometer.y;
}
int16_t BNO08x::getRawMagZ()
{
	return _sensor_value->un.rawMagnetometer.z;
}

// //Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
// //Q1 is used for all sensor data calculations
// int16_t BNO08x::getQ1(uint16_t recordID)
// {
// 	//Q1 is always the lower 16 bits of word 7
// 	//uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
// 	//return (q);
// }

// //Given a record ID, read the Q2 value from the metaData record in the FRS
// //Q2 is used in sensor bias
// int16_t BNO08x::getQ2(uint16_t recordID)
// {
// 	//Q2 is always the upper 16 bits of word 7
// 	//uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
// 	//return (q);
// }

// //Given a record ID, read the Q3 value from the metaData record in the FRS
// //Q3 is used in sensor change sensitivity
// int16_t BNO08x::getQ3(uint16_t recordID)
// {
// 	//Q3 is always the upper 16 bits of word 8
// 	//uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
// 	//return (q);
// }

// //Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
// float BNO08x::getResolution(uint16_t recordID)
// {
// 	//The resolution Q value are 'the same as those used in the sensor's input report'
// 	//This should be Q1.
// 	int16_t Q = getQ1(recordID);

// 	//Resolution is always word 2
// 	//uint32_t value = readFRSword(recordID, 2); //Get word 2

// 	float resolution = qToFloat(value, Q);

// 	return (resolution);
// }

// //Given a record ID, read the range value from the metaData record in the FRS for a given sensor
// float BNO08x::getRange(uint16_t recordID)
// {
// 	//The resolution Q value are 'the same as those used in the sensor's input report'
// 	//This should be Q1.
// 	int16_t Q = getQ1(recordID);

// 	//Range is always word 1
// 	//uint32_t value = readFRSword(recordID, 1); //Get word 1

// 	float range = qToFloat(value, Q);

// 	return (range);
// }

bool BNO08x::serviceBus(void)
{
  sh2_service();
  return true;
}

//Send command to reset IC
bool BNO08x::softReset(void)
{
  int status = sh2_devReset();

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

//Set the operating mode to "On"
//(This one is for @jerabaul29)
bool BNO08x::modeOn(void)
{
  int status = sh2_devOn();

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

//Set the operating mode to "Sleep"
//(This one is for @jerabaul29)
bool BNO08x::modeSleep(void)
{
  int status = sh2_devSleep();

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO08x::getResetReason()
{
	return prodIds.entry[0].resetCause;
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO08x::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{

	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

//Sends the packet to enable the rotation vector
bool BNO08x::enableRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the geomagnetic rotation vector
bool BNO08x::enableGeomagneticRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO08x::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
bool BNO08x::enableGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO08x::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO08x::enableAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO08x::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);	
}

//Sends the packet to enable the gravity vector
bool BNO08x::enableGravity(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GRAVITY, timeBetweenReports);	
}

//Sends the packet to enable the gyro
bool BNO08x::enableGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, timeBetweenReports);		
}

//Sends the packet to enable the uncalibrated gyro
bool BNO08x::enableUncalibratedGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_UNCALIBRATED_GYRO, timeBetweenReports);		
}

//Sends the packet to enable the magnetometer
bool BNO08x::enableMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);		
}

//Sends the packet to enable the high refresh-rate gyro-integrated rotation vector
bool BNO08x::enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);		
}

//Sends the packet to enable the tap detector
bool BNO08x::enableTapDetector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);		
}

//Sends the packet to enable the shake detector
bool BNO08x::enableShakeDetector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_SHAKE_DETECTOR, timeBetweenReports);		
}

//Sends the packet to enable the step counter
bool BNO08x::enableStepCounter(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);		
}

//Sends the packet to enable the Stability Classifier
bool BNO08x::enableStabilityClassifier(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO08x::enableRawAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO08x::enableRawGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO08x::enableRawMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);		
}

//Sends the packet to enable the various activity classifiers
bool BNO08x::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

// See 2.2 of the Calibration Procedure document 1000-4044
// Set the desired sensors to have active dynamic calibration
bool BNO08x::setCalibrationConfig(uint8_t sensors)
{
  int status = sh2_setCalConfig(sensors);

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

bool BNO08x::tareNow(bool zAxis, sh2_TareBasis_t basis)
{
  int status = sh2_setTareNow(zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, basis);

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

bool BNO08x::saveTare()
{
  int status = sh2_persistTare();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool BNO08x::clearTare()
{
  int status = sh2_clearTare();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

// //This tells the BNO08x to begin calibrating
// //See page 50 of reference manual and the 1000-4044 calibration doc
// void BNO08x::sendCalibrateCommand(uint8_t thingToCalibrate)
// {
// 	/*shtpData[3] = 0; //P0 - Accel Cal Enable
// 	shtpData[4] = 0; //P1 - Gyro Cal Enable
// 	shtpData[5] = 0; //P2 - Mag Cal Enable
// 	shtpData[6] = 0; //P3 - Subcommand 0x00
// 	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
// 	shtpData[8] = 0; //P5 - Reserved
// 	shtpData[9] = 0; //P6 - Reserved
// 	shtpData[10] = 0; //P7 - Reserved
// 	shtpData[11] = 0; //P8 - Reserved*/

// 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
// 		shtpData[x] = 0;

// 	if (thingToCalibrate == CALIBRATE_ACCEL)
// 		shtpData[3] = 1;
// 	else if (thingToCalibrate == CALIBRATE_GYRO)
// 		shtpData[4] = 1;
// 	else if (thingToCalibrate == CALIBRATE_MAG)
// 		shtpData[5] = 1;
// 	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
// 		shtpData[7] = 1;
// 	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
// 	{
// 		shtpData[3] = 1;
// 		shtpData[4] = 1;
// 		shtpData[5] = 1;
// 	}
// 	else if (thingToCalibrate == CALIBRATE_STOP)
// 	{
// 		; //Do nothing, bytes are set to zero
// 	}

// 	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
// 	calibrationStatus = 1;

// 	//Using this shtpData packet, send a command
// 	sendCommand(COMMAND_ME_CALIBRATE);
// }

// //Request ME Calibration Status from BNO08x
// //See page 51 of reference manual
// void BNO08x::requestCalibrationStatus()
// {
// 	/*shtpData[3] = 0; //P0 - Reserved
// 	shtpData[4] = 0; //P1 - Reserved
// 	shtpData[5] = 0; //P2 - Reserved
// 	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
// 	shtpData[7] = 0; //P4 - Reserved
// 	shtpData[8] = 0; //P5 - Reserved
// 	shtpData[9] = 0; //P6 - Reserved
// 	shtpData[10] = 0; //P7 - Reserved
// 	shtpData[11] = 0; //P8 - Reserved*/

// 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
// 		shtpData[x] = 0;

// 	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

// 	//Using this shtpData packet, send a command
// 	sendCommand(COMMAND_ME_CALIBRATE);
// }

//This tells the BNO08x to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
bool BNO08x::saveCalibration()
{
  int status = sh2_saveDcdNow();
  if (status != SH2_OK) {
    return false;
  }
  return true;	
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool BNO08x::_init(int32_t sensor_id) {
  int status;

  hardwareReset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool BNO08x::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool BNO08x::getSensorEvent() {
  _sensor_value = &sensorValue;

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @param sensorSpecific config settings specific to sensor/reportID.
 * (e.g. enabling/disabling possible activities in personal activity classifier)
 * @return true: success false: failure
 */
bool BNO08x::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us,
							   	   uint32_t sensorSpecific) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = sensorSpecific;

  config.reportInterval_us = interval_us;

  if(_int_pin != -1) {
	if (!hal_wait_for_int()) {
      return 0;
  	}
  }
  
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

/****************************************
***************************************** I2C interface
*****************************************
*****************************************/

static int i2chal_open(sh2_Hal_t *self) {
  // Serial.println("I2C HAL open");

  if(_int_pin != -1) hal_wait_for_int();
  
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (i2c_write(softreset_pkt, 5)) {
      success = true;
      break;
    }
    delay(30);
  }
  if (!success)
    return -1;
  delay(300);
  return 0;
}

static void i2chal_close(sh2_Hal_t *self) {
  // Serial.println("I2C HAL close");
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("I2C HAL read");

  // uint8_t *pBufferOrig = pBuffer;

  if(_int_pin != -1) {
	if (!hal_wait_for_int()) {
    	return 0;
  	}
  }

  uint8_t header[4];
  if (!i2c_read(header, 4)) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  size_t i2c_buffer_max = maxBufferSize();

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    // Serial.print("Reading from I2C: "); Serial.println(read_size);
    // Serial.print("Remaining to read: "); Serial.println(cargo_remaining);

	if(_int_pin != -1) {
		if (!hal_wait_for_int()) {
			return 0;
		}
	}

    if (!i2c_read(i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }

  /*
  for (int i=0; i<packet_size; i++) {
    Serial.print(pBufferOrig[i], HEX);
    Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }
  Serial.println();
  */

  return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = maxBufferSize();

  /*
  Serial.print("I2C HAL write packet size: ");
  Serial.print(len);
  Serial.print(" & max buffer size: ");
  Serial.println(i2c_buffer_max);
  */

  uint16_t write_size = min(i2c_buffer_max, len);

  if(_int_pin != -1) {
	if (!hal_wait_for_int()) {
    	return 0;
  	}
  }

  if (!i2c_write(pBuffer, write_size)) {
    return 0;
  }

  return write_size;
}

/****************************************
***************************************** HAL interface
*****************************************
*****************************************/

static void hal_hardwareReset(void) {
  if (_reset_pin != -1) {
    // Serial.println("BNO08x Hardware reset");

    pinMode(_reset_pin, OUTPUT);
    digitalWrite(_reset_pin, HIGH);
    delay(10);
    digitalWrite(_reset_pin, LOW);
    delay(10);
    digitalWrite(_reset_pin, HIGH);
    delay(10);
  }
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = millis() * 1000;
  // Serial.printf("I2C HAL get time: %d\n", t);
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    //Serial.println("BNO08x - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void BNO08x::hardwareReset(void) { hal_hardwareReset(); }


//Return the sensorID
uint8_t BNO08x::getSensorEventID()
{
	return _sensor_value->sensorId;
}


//Returns true if I2C device ack's
boolean BNO08x::isConnected()
{
  	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
  	if (_i2cPort->endTransmission() != 0)
    	return (false); //Sensor did not ACK
  	return (true);
}

/****************************************
***************************************** I2C Write/Read Functions
*****************************************
*****************************************/

/*!
 *    @brief  Write a buffer or two to the I2C device. Cannot be more than
 * maxBufferSize() bytes.
 *    @param  buffer Pointer to buffer of data to write. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  len Number of bytes from buffer to write
 *    @param  prefix_buffer Pointer to optional array of data to write before
 * buffer. Cannot be more than maxBufferSize() bytes. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  prefix_len Number of bytes from prefix buffer to write
 *    @param  stop Whether to send an I2C STOP signal on write
 *    @return True if write was successful, otherwise false.
 */
bool i2c_write(const uint8_t *buffer, size_t len, bool stop,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len) {
  if ((len + prefix_len) > maxBufferSize()) {
    // currently not guaranteed to work if more than 32 bytes!
    // we will need to find out if some platforms have larger
    // I2C buffer sizes :/
    return false;
  }

  _i2cPort->beginTransmission(_deviceAddress);

  // Write the prefix data (usually an address)
  if ((prefix_len != 0) && (prefix_buffer != nullptr)) {
    if (_i2cPort->write(prefix_buffer, prefix_len) != prefix_len) {
      return false;
    }
  }

  // Write the data itself
  if (_i2cPort->write(buffer, len) != len) {
    return false;
  }

  if (_i2cPort->endTransmission(stop) == 0) {
    return true;
  } else {
    return false;
  }
}

/*!
 *    @brief  Read from I2C into a buffer from the I2C device.
 *    Cannot be more than maxBufferSize() bytes.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  stop Whether to send an I2C STOP signal on read
 *    @return True if read was successful, otherwise false.
 */
boolean i2c_read(uint8_t *buffer, size_t len, bool stop) {
  size_t pos = 0;
  while (pos < len) {
    size_t read_len =
        ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
    bool read_stop = (pos < (len - read_len)) ? false : stop;
    if (!_i2c_read(buffer + pos, read_len, read_stop))
      return false;
    pos += read_len;
  }
  return true;
}

boolean _i2c_read(uint8_t *buffer, size_t len, bool stop) {
#if defined(TinyWireM_h)
  size_t recv = _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len);
#elif defined(ARDUINO_ARCH_MEGAAVR)
  size_t recv = _i2cPort->requestFrom(_deviceAddress, len, stop);
#else
  size_t recv = _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len, (uint8_t)stop);
#endif

  if (recv != len) {
    // Not enough data available to fulfill our obligation!
    return false;
  }

  for (uint16_t i = 0; i < len; i++) {
    buffer[i] = _i2cPort->read();
  }
  return true;
}

  /*!   @brief  How many bytes we can read in a transaction
   *    @return The size of the Wire receive/transmit buffer */
size_t maxBufferSize() { return _maxBufferSize; }


/****************************************
***************************************** HAL SPI interface
*****************************************
*****************************************/

static int spihal_open(sh2_Hal_t *self) {
  // Serial.println("SPI HAL open");

  hal_wait_for_int();

  return 0;
}

static bool hal_wait_for_int(void) {
  for (int i = 0; i < 500; i++) {
    if (!digitalRead(_int_pin))
      return true;
    // Serial.print(".");
    delay(1);
  }
  // Serial.println("Timed out!");
  hal_hardwareReset();

  return false;
}

static void spihal_close(sh2_Hal_t *self) {
  // Serial.println("SPI HAL close");
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("SPI HAL read");

  uint16_t packet_size = 0;

  if (!hal_wait_for_int()) {
    return 0;
  }

  if (!spi_read(pBuffer, 4, 0x00)) {
    return 0;
  }

  // Determine amount to read
  packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  if (packet_size > len) {
    return 0;
  }

  if (!hal_wait_for_int()) {
    return 0;
  }

  if (!spi_read(pBuffer, packet_size, 0x00)) {
    return 0;
  }

  return packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  // Serial.print("SPI HAL write packet size: ");
  // Serial.println(len);

  if (!hal_wait_for_int()) {
    return 0;
  }

  spi_write(pBuffer, len);

  return len;
}

/****************************************
***************************************** SPI WRITE/READ Functions
*****************************************
*****************************************/

/*!
 *    @brief  Read from SPI into a buffer from the SPI device, with transaction
 * management.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  sendvalue The 8-bits of data to write when doing the data read,
 * defaults to 0xFF
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
static bool spi_read(uint8_t *buffer, size_t len, uint8_t sendvalue) {
  memset(buffer, sendvalue, len); // clear out existing buffer

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
	digitalWrite(_cs, LOW);

  _spiPort->transfer(buffer, len);

  digitalWrite(_cs, HIGH);
  _spiPort->endTransaction();

  return true;
}

/*!
 *    @brief  Write a buffer or two to the SPI device, with transaction
 * management.
 *    @param  buffer Pointer to buffer of data to write
 *    @param  len Number of bytes from buffer to write
 *    @param  prefix_buffer Pointer to optional array of data to write before
 * buffer.
 *    @param  prefix_len Number of bytes from prefix buffer to write
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
static bool spi_write(const uint8_t *buffer, size_t len,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len) {

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
	digitalWrite(_cs, LOW);

  // do the writing
#if defined(ARDUINO_ARCH_ESP32)
  if (_spiPort) {
    // if (prefix_len > 0) {
    //   _spiPort->transferBytes(prefix_buffer, nullptr, prefix_len);
    // }
    if (len > 0) {
      _spiPort->transferBytes(buffer, nullptr, len);
    }
  } else
#endif
  {
    // for (size_t i = 0; i < prefix_len; i++) {
    //   _spiPort->transfer(prefix_buffer[i]);
    // }
    for (size_t i = 0; i < len; i++) {
      _spiPort->transfer(buffer[i]);
    }
  }

  digitalWrite(_cs, HIGH);
  _spiPort->endTransaction();

  return true;
}
