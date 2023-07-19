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

#include "SparkFun_BNO08x_Arduino_Library.h"

static int8_t _int_pin, _reset_pin;
static TwoWire *_i2cPort = NULL;		//The generic connection to user's chosen I2C hardware
static uint8_t _deviceAddress = BNO08x_DEFAULT_ADDRESS; //Keeps track of I2C address. setI2CAddress changes this.

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

size_t _maxBufferSize = 32;
size_t maxBufferSize();		

//Initializes the sensor with basic settings
//Returns false if sensor is not detected
boolean BNO08x::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  	_deviceAddress = deviceAddress;
  	_i2cPort = &wirePort;

  	if (isConnected() == false) // Check for sensor by verifying ACK response
    	return (false); 

	Serial.println(F("I2C address found"));
    delay(1000);

    _HAL.open = i2chal_open;
    _HAL.close = i2chal_close;
    _HAL.read = i2chal_read;
    _HAL.write = i2chal_write;
    _HAL.getTimeUs = hal_getTimeUs;

    return _init();
}

boolean BNO08x::beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	_i2cPort = NULL; //This null tells the send/receive functions to use SPI

	//Get user settings
	_spiPort = &spiPort;
	_spiPortSpeed = spiPortSpeed;
	if (_spiPortSpeed > 3000000)
		_spiPortSpeed = 3000000; //BNO08x max is 3MHz

	_cs = user_CSPin;
	_wake = user_WAKPin;
	_int = user_INTPin;
	_rst = user_RSTPin;

	pinMode(_cs, OUTPUT);
	pinMode(_wake, OUTPUT);
	pinMode(_int, INPUT_PULLUP);
	pinMode(_rst, OUTPUT);

	digitalWrite(_cs, HIGH); //Deselect BNO08x

	//Configure the BNO08x for SPI communication
	digitalWrite(_wake, HIGH); //Before boot up the PS0/WAK pin must be high to enter SPI mode
	digitalWrite(_rst, LOW);   //Reset BNO08x
	delay(2);				   //Min length not specified in datasheet?
	digitalWrite(_rst, HIGH);  //Bring out of reset

	//Wait for first assertion of INT before using WAK pin. Can take ~104ms
	waitForSPI();

	//if(wakeBNO08x() == false) //Bring IC out of sleep after reset
	//  Serial.println("BNO08x did not wake up");

	_spiPort->begin(); //Turn on SPI hardware

	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When BNO08x first boots it broadcasts big startup packet
	//Read it and dump it
	waitForSPI(); //Wait for assertion of INT before reading advert message.
	receivePacket();

	//The BNO08x will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	waitForSPI(); //Wait for assertion of INT before reading Init response
	receivePacket();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	waitForSPI();
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			if (_printDebug == true)
			{
				_debugPort->print(F("SW Version Major: 0x"));
				_debugPort->print(shtpData[2], HEX);
				_debugPort->print(F(" SW Version Minor: 0x"));
				_debugPort->print(shtpData[3], HEX);
				uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
				_debugPort->print(F(" SW Part Number: 0x"));
				_debugPort->print(SW_Part_Number, HEX);
				uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
				_debugPort->print(F(" SW Build Number: 0x"));
				_debugPort->print(SW_Build_Number, HEX);
				uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
				_debugPort->print(F(" SW Version Patch: 0x"));
				_debugPort->println(SW_Version_Patch, HEX);
			}
			return (true);
		}
	}

	return (false); //Something went wrong
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void BNO08x::enableDebugging(Stream &debugPort)
{
	_debugPort = &debugPort;
	_printDebug = true;
}

//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO08x::dataAvailable(void)
{
	return (getReadings() != 0);
}

uint16_t BNO08x::getReadings(void)
{
	//If we have an interrupt pin connection available, check if data is available.
	//If int pin is not set, then we'll rely on receivePacket() to timeout
	//See issue 13: https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/issues/13
	if (_int != 255)
	{
		if (digitalRead(_int) == HIGH)
			return 0;
	}

	if (receivePacket() == true)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
		}
		else if(shtpHeader[2] == CHANNEL_GYRO)
		{
		return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
	}
	return 0;
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
uint16_t BNO08x::parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO08x responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return 0;
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
uint16_t BNO08x::parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
	if(shtpHeader[2] == CHANNEL_GYRO) {
		rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
		rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
		rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
		rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
		rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
		rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
		rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

		return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
	}

	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports
	uint16_t data6 = 0;

	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}
	if (dataLength - 5 > 13)
	{
		data6 = (uint16_t)shtpData[5 + 15] << 8 | shtpData[5 + 14];
	}


	//Store these generic values to their proper global variable
	if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
	{
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE_CALIBRATED)
	{	
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_UNCALIBRATED_GYRO)
	{
		UncalibGyroAccuracy = status;
		rawUncalibGyroX = data1;
		rawUncalibGyroY = data2;
		rawUncalibGyroZ = data3;
		rawBiasX  = data4;
		rawBiasY  = data5;
		rawBiasZ  = data6;
	}
	else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
	{
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;

		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		rawQuatRadianAccuracy = data5;
	}
	else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR)
	{
		tapDetector = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; //Bytes 8/9
	}
	else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; //Most likely state
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
	{
		memsRawAccelX = data1;
		memsRawAccelY = data2;
		memsRawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
	{
		memsRawGyroX = data1;
		memsRawGyroY = data2;
		memsRawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
	{
		memsRawMagX = data1;
		memsRawMagY = data2;
		memsRawMagZ = data3;
	}
	else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		if (_printDebug == true)
		{
			_debugPort->println(F("!"));
		}
		//The BNO08x responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			if (_printDebug == true)
			{
				_debugPort->println(F("ME Cal report found!"));
			}
			calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else if(shtpData[5] == SENSOR_REPORTID_GRAVITY)
	{
		gravityAccuracy = status;
		gravityX = data1;
		gravityY = data2;
		gravityZ = data3;
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
		return 0;
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return shtpData[5];
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
	x = qToFloat(rawUncalibGyroX, gyro_Q1);
	y = qToFloat(rawUncalibGyroY, gyro_Q1);
	z = qToFloat(rawUncalibGyroZ, gyro_Q1);
	bx = qToFloat(rawBiasX, gyro_Q1);
	by = qToFloat(rawBiasY, gyro_Q1);
	bz = qToFloat(rawBiasZ, gyro_Q1);
	accuracy = UncalibGyroAccuracy;
}
//Return the gyro component
float BNO08x::getUncalibratedGyroX()
{
	float gyro = qToFloat(rawUncalibGyroX, gyro_Q1);
	return (gyro);
}
//Return the gyro component
float BNO08x::getUncalibratedGyroY()
{
	float gyro = qToFloat(rawUncalibGyroY, gyro_Q1);
	return (gyro);
}
//Return the gyro component
float BNO08x::getUncalibratedGyroZ()
{
	float gyro = qToFloat(rawUncalibGyroZ, gyro_Q1);
	return (gyro);
}
//Return the gyro component
float BNO08x::getUncalibratedGyroBiasX()
{
	float gyro = qToFloat(rawBiasX, gyro_Q1);
	return (gyro);
}
//Return the gyro component
float BNO08x::getUncalibratedGyroBiasY()
{
	float gyro = qToFloat(rawBiasY, gyro_Q1);
	return (gyro);
}
//Return the gyro component
float BNO08x::getUncalibratedGyroBiasZ()
{
	float gyro = qToFloat(rawBiasZ, gyro_Q1);
	return (gyro);
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
	float x = qToFloat(gravityX, gravity_Q1);
	return x;
}

//Return the gravity component
float BNO08x::getGravityY()
{
	float y = qToFloat(gravityY, gravity_Q1);
	return y;
}

//Return the gravity component
float BNO08x::getGravityZ()
{
	float z = qToFloat(gravityZ, gravity_Q1);
	return z;
}

uint8_t BNO08x::getGravityAccuracy()
{
	return (gravityAccuracy);
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
	uint8_t previousTapDetector = tapDetector;
	tapDetector = 0; //Reset so user code sees exactly one tap
	return (previousTapDetector);
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

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t BNO08x::getQ1(uint16_t recordID)
{
	//Q1 is always the lower 16 bits of word 7
	uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
	return (q);
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO08x::getQ2(uint16_t recordID)
{
	//Q2 is always the upper 16 bits of word 7
	uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
	return (q);
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO08x::getQ3(uint16_t recordID)
{
	//Q3 is always the upper 16 bits of word 8
	uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
	return (q);
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO08x::getResolution(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = getQ1(recordID);

	//Resolution is always word 2
	uint32_t value = readFRSword(recordID, 2); //Get word 2

	float resolution = qToFloat(value, Q);

	return (resolution);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO08x::getRange(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = getQ1(recordID);

	//Range is always word 1
	uint32_t value = readFRSword(recordID, 1); //Get word 1

	float range = qToFloat(value, Q);

	return (range);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO08x::readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if (readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
		return (metaData[0]);						  //Return this one word

	return (0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO08x::frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
	shtpData[1] = 0;							//Reserved
	shtpData[2] = (readOffset >> 0) & 0xFF;		//Read Offset LSB
	shtpData[3] = (readOffset >> 8) & 0xFF;		//Read Offset MSB
	shtpData[4] = (recordID >> 0) & 0xFF;		//FRS Type LSB
	shtpData[5] = (recordID >> 8) & 0xFF;		//FRS Type MSB
	shtpData[6] = (blockSize >> 0) & 0xFF;		//Block size LSB
	shtpData[7] = (blockSize >> 8) & 0xFF;		//Block size MSB

	//Transmit packet on channel 2, 8 bytes
	sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool BNO08x::readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;

	//First we send a Flash Record System (FRS) request
	frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

	//Read bytes until FRS reports that the read is complete
	while (1)
	{
		//Now we wait for response
		while (1)
		{
			uint8_t counter = 0;
			while (receivePacket() == false)
			{
				if (counter++ > 100)
					return (false); //Give up
				delay(1);
			}

			//We have the packet, inspect it for the right contents
			//See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((((uint16_t)shtpData[13]) << 8) | shtpData[12]) == recordID)
					break; //This packet is one we are looking for
		}

		uint8_t dataLength = shtpData[1] >> 4;
		uint8_t frsStatus = shtpData[1] & 0x0F;

		uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

		//Record these words to the metaData array
		if (dataLength > 0)
		{
			metaData[spot++] = data0;
		}
		if (dataLength > 1)
		{
			metaData[spot++] = data1;
		}

		if (spot >= MAX_METADATA_SIZE)
		{
			if (_printDebug == true)
				_debugPort->println(F("metaData array over run. Returning."));
			return (true); //We have run out of space in our array. Bail.
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
		{
			return (true); //FRS status is read completed! We're done!
		}
	}
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
bool BNO08x::softReset(void)
{
	shtpData[0] = 1; //Reset

  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (sendPacket(CHANNEL_EXECUTABLE, 1)) {
      success = true;
      break;
    }
    delay(30);
  }
  if (!success)
    return 0;
  delay(300);
  return 1;

	// //Attempt to start communication with sensor
	// sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	// //Read all incoming data and flush it
	// delay(50);
	// while (receivePacket() == true)
	// 	; //delay(1);
	// delay(50);
	// while (receivePacket() == true)
	// 	; //delay(1);
}

//Set the operating mode to "On"
//(This one is for @jerabaul29)
void BNO08x::modeOn(void)
{
	shtpData[0] = 2; //On

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
}

//Set the operating mode to "Sleep"
//(This one is for @jerabaul29)
void BNO08x::modeSleep(void)
{
	shtpData[0] = 3; //Sleep

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
}

// Indicates if we've received a Reset Complete packet. Once it's been read, 
// the state will reset to false until another Reset Complete packet is found. 
bool BNO08x::hasReset() {
	if (_hasReset) {
		_hasReset = false;
		return true;
	}
	return false; 
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO08x::resetReason()
{
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return (shtpData[1]);
		}
	}

	return (0);
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

//Sends the commands to begin calibration of the accelerometer
void BNO08x::calibrateAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO08x::calibrateGyro()
{
	sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO08x::calibrateMagnetometer()
{
	sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO08x::calibratePlanarAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO08x::calibrateAll()
{
	sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO08x::endCalibration()
{
	sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
boolean BNO08x::calibrationComplete()
{
	if (calibrationStatus == 0)
		return (true);
	return (false);
}

void BNO08x::tareNow(bool zAxis, uint8_t rotationVectorBasis)
{
	sendTareCommand(TARE_NOW, zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, rotationVectorBasis);
}

void BNO08x::saveTare()
{
	sendTareCommand(TARE_PERSIST);
}

void BNO08x::clearTare()
{
	sendTareCommand(TARE_SET_REORIENTATION);
}

//Given a sensor's report ID, this tells the BNO08x to begin reporting the values
void BNO08x::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO08x to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO08x::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO08x::sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO08x to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO08x::sendCalibrateCommand(uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if (thingToCalibrate == CALIBRATE_STOP)
	{
		; //Do nothing, bytes are set to zero
	}

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	calibrationStatus = 1;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

void BNO08x::sendTareCommand(uint8_t command, uint8_t axis, uint8_t rotationVectorBasis)
{
	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[3] = command;
	
	if (command == TARE_NOW)
	{
		shtpData[4] = axis; // axis setting
		shtpData[5] = rotationVectorBasis; // rotation vector
	}
	
	//Using this shtpData packet, send a command
	sendCommand(COMMAND_TARE);
}

//Request ME Calibration Status from BNO08x
//See page 51 of reference manual
void BNO08x::requestCalibrationStatus()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO08x to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO08x::saveCalibration()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_DCD); //Save DCD command
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
boolean BNO08x::waitForI2C()
{
	for (uint16_t counter = 0; counter < 1000; counter++) //Don't got more than 255
	{
		int i2c_available_amount = _i2cPort->available();
		if (i2c_available_amount > 0)
		{
			if (_printDebug == true)
			{
		        _debugPort->print(F("I2C available"));
				_debugPort->println(i2c_available_amount);
	        }
			return (true);
		}
		delayMicroseconds(1000);
		if (_printDebug == true)
		    _debugPort->print(F("."));
	}

	if (_printDebug == true)
		_debugPort->println(F("I2C timeout"));
	return (false);
}

//Blocking wait for BNO08x to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
boolean BNO08x::waitForSPI()
{
	for (uint8_t counter = 0; counter < 125; counter++) //Don't got more than 255
	{
		if (digitalRead(_int) == LOW)
			return (true);
		if (_printDebug == true)
			_debugPort->println(F("SPI Wait"));
		delay(1);
	}

	if (_printDebug == true)
		_debugPort->println(F("SPI INT timeout"));
	return (false);
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
boolean BNO08x::receivePacket(void)
{
	if (_i2cPort == NULL) //Do SPI
	{
		if (digitalRead(_int) == HIGH)
			return (false); //Data is not available

		//Old way: if (waitForSPI() == false) return (false); //Something went wrong

		//Get first four bytes to find out how much data we need to read

		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Get the first four bytes, aka the packet header
		uint8_t packetLSB = _spiPort->transfer(0);
		uint8_t packetMSB = _spiPort->transfer(0);
		uint8_t channelNumber = _spiPort->transfer(0);
		uint8_t sequenceNumber = _spiPort->transfer(0); //Not sure if we need to store this or not

		//Store the header info
		shtpHeader[0] = packetLSB;
		shtpHeader[1] = packetMSB;
		shtpHeader[2] = channelNumber;
		shtpHeader[3] = sequenceNumber;

		//Calculate the number of data bytes in this packet
		uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
		dataLength &= ~(1 << 15); //Clear the MSbit.
		//This bit indicates if this package is a continuation of the last. Ignore it for now.
		//TODO catch this as an error and exit
		if (dataLength == 0)
		{
			//Packet is empty
			printHeader();
			return (false); //All done
		}
		dataLength -= 4; //Remove the header bytes from the data count

		//Read incoming data into the shtpData array
		for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
		{
			uint8_t incoming = _spiPort->transfer(0xFF);
			if (dataSpot < MAX_PACKET_SIZE)	//BNO08x can respond with upto 270 bytes, avoid overflow
				shtpData[dataSpot] = incoming; //Store data into the shtpData array
		}

		digitalWrite(_cs, HIGH); //Release BNO08x

		_spiPort->endTransaction();
		printPacket();
	}
	else //Do I2C
	{
		int requestFromReturn;
		requestFromReturn = _i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)4); //Ask for four bytes to find out how much data we need to read
		Serial.print("requestFromReturn:");
		Serial.println(requestFromReturn);

		if (waitForI2C() == false)
			return (false); //Error

		//Get the first four bytes, aka the packet header
		uint8_t packetLSB = _i2cPort->read();
		uint8_t packetMSB = _i2cPort->read();
		uint8_t channelNumber = _i2cPort->read();
		uint8_t sequenceNumber = _i2cPort->read(); //Not sure if we need to store this or not

		//Store the header info.
		shtpHeader[0] = packetLSB;
		shtpHeader[1] = packetMSB;
		shtpHeader[2] = channelNumber;
		shtpHeader[3] = sequenceNumber;

		//Calculate the number of data bytes in this packet
		uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
		dataLength &= ~(1 << 15); //Clear the MSbit.
		//This bit indicates if this package is a continuation of the last. Ignore it for now.
		//TODO catch this as an error and exit

		if (_printDebug == true)
		{
			_debugPort->print(F("receivePacket (I2C): dataLength is: "));
			_debugPort->println(dataLength);
			_debugPort->print(F("HEADER IS: "));
			_debugPort->print(shtpHeader[0]);
			_debugPort->print(F("\t"));
			_debugPort->print(shtpHeader[1]);
			_debugPort->print(F("\t"));
			_debugPort->print(shtpHeader[2]);
			_debugPort->print(F("\t"));
			_debugPort->print(shtpHeader[3]);
			_debugPort->println(F("\t"));
		}

		if (dataLength == 0)
		{
			//Packet is empty
			return (false); //All done
		}
		dataLength -= 4; //Remove the header bytes from the data count

		getData(dataLength);
	}

	// Quickly check for reset complete packet. No need for a seperate parser.
	// This function is also called after soft reset, so we need to catch this
	// packet here otherwise we need to check for the reset packet in multiple
	// places.
	if (shtpHeader[2] == CHANNEL_EXECUTABLE && shtpData[0] == EXECUTABLE_RESET_COMPLETE) 
	{
		_hasReset = true;
	} 

	return (true); //We're done!
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
boolean BNO08x::getData(uint16_t bytesRemaining)
{
	Serial.println("getData BEGINNING");
	uint16_t dataSpot = 0; //Start at the beginning of shtpData array

	//Setup a series of chunked 32 byte reads
	while (bytesRemaining > 0)
	{
		uint16_t numberOfBytesToRead = bytesRemaining;
		if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
			numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);
		
		Serial.print("numberOfBytesToRead");
		Serial.println(numberOfBytesToRead);

		_i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)(numberOfBytesToRead + 4));
		if (waitForI2C() == false)
			return (0); //Error

		//The first four bytes are header bytes and are throw away
		_i2cPort->read();
		_i2cPort->read();
		_i2cPort->read();
		_i2cPort->read();

		for (uint8_t x = 0; x < numberOfBytesToRead; x++)
		{
			uint8_t incoming = _i2cPort->read();
			if (dataSpot < MAX_PACKET_SIZE)
			{
				shtpData[dataSpot++] = incoming; //Store data into the shtpData array
			}
			else
			{
				//Do nothing with the data
			}
		}

		bytesRemaining -= numberOfBytesToRead;
	}
	printPacket();
	return (true); //Done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
boolean BNO08x::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	if (_i2cPort == NULL) //Do SPI
	{
		//Wait for BNO08x to indicate it is available for communication
		if (waitForSPI() == false)
			return (false); //Something went wrong

		//BNO08x has max CLK of 3MHz, MSB first,
		//The BNO08x uses CPOL = 1 and CPHA = 1. This is mode3
		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Send the 4 byte packet header
		_spiPort->transfer(packetLength & 0xFF);			 //Packet length LSB
		_spiPort->transfer(packetLength >> 8);				 //Packet length MSB
		_spiPort->transfer(channelNumber);					 //Channel number
		_spiPort->transfer(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_spiPort->transfer(shtpData[i]);
		}

		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
	}
	else //Do I2C
	{
		//if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

		_i2cPort->beginTransmission(_deviceAddress);

		//Send the 4 byte packet header
		_i2cPort->write(packetLength & 0xFF);			  //Packet length LSB
		_i2cPort->write(packetLength >> 8);				  //Packet length MSB
		_i2cPort->write(channelNumber);					  //Channel number
		_i2cPort->write(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_i2cPort->write(shtpData[i]);
		}

		uint8_t i2cResult = _i2cPort->endTransmission();

		if (i2cResult != 0)
		{
			if (_printDebug == true)
			{
				_debugPort->print(F("sendPacket(I2C): endTransmission returned: "));
				_debugPort->println(i2cResult);
			}
			return (false);
		}
	}

	return (true);
}

//Pretty prints the contents of the current shtp header and data packets
void BNO08x::printPacket(void)
{
	if (_printDebug == true)
	{
		uint16_t packetLength = (uint16_t)shtpHeader[1] << 8 | shtpHeader[0];

		//Print the four byte header
		_debugPort->print(F("Header:"));
		for (uint8_t x = 0; x < 4; x++)
		{
			_debugPort->print(F(" "));
			if (shtpHeader[x] < 0x10)
				_debugPort->print(F("0"));
			_debugPort->print(shtpHeader[x], HEX);
		}

		uint8_t printLength = packetLength - 4;
		if (printLength > 40)
			printLength = 40; //Artificial limit. We don't want the phone book.

		_debugPort->print(F(" Body:"));
		for (uint8_t x = 0; x < printLength; x++)
		{
			_debugPort->print(F(" "));
			if (shtpData[x] < 0x10)
				_debugPort->print(F("0"));
			_debugPort->print(shtpData[x], HEX);
		}

		if (packetLength & 1 << 15)
		{
			_debugPort->println(F(" [Continued packet] "));
			packetLength &= ~(1 << 15);
		}

		_debugPort->print(F(" Length:"));
		_debugPort->print(packetLength);

		_debugPort->print(F(" Channel:"));
		if (shtpHeader[2] == 0)
			_debugPort->print(F("Command"));
		else if (shtpHeader[2] == 1)
			_debugPort->print(F("Executable"));
		else if (shtpHeader[2] == 2)
			_debugPort->print(F("Control"));
		else if (shtpHeader[2] == 3)
			_debugPort->print(F("Sensor-report"));
		else if (shtpHeader[2] == 4)
			_debugPort->print(F("Wake-report"));
		else if (shtpHeader[2] == 5)
			_debugPort->print(F("Gyro-vector"));
		else
			_debugPort->print(shtpHeader[2]);

		_debugPort->println();
	}
}

//Pretty prints the contents of the current shtp header (only)
void BNO08x::printHeader(void)
{
	if (_printDebug == true)
	{
		//Print the four byte header
		_debugPort->print(F("Header:"));
		for (uint8_t x = 0; x < 4; x++)
		{
			_debugPort->print(F(" "));
			if (shtpHeader[x] < 0x10)
				_debugPort->print(F("0"));
			_debugPort->print(shtpHeader[x], HEX);
		}
		_debugPort->println();
	}
}


/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool BNO08x::_init(int32_t sensor_id) {
  int status;

  //hardwareReset();

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
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

/**************************************** I2C interface
 * ***********************************************************/

static int i2chal_open(sh2_Hal_t *self) {
  // Serial.println("I2C HAL open");
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
  if (!i2c_write(pBuffer, write_size)) {
    return 0;
  }

  return write_size;
}


/**************************************** HAL interface
 * ***********************************************************/

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
    Serial.println("BNO08x - Error decoding sensor event");
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