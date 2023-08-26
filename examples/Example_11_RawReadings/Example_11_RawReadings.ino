/*
  Using the BNO08x IMU

  This example shows how to output the raw accel, gryo and mag values.

  It outputs the timestampe/i/j/k/real/accuracy parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

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

  Hardware Connections:
  Plug the sensor into IoT RedBoard via QWIIC cable.
  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

// variables to store all our incoming values

// raw accel
uint16_t x;
uint16_t y;
uint16_t z;

// raw gyros
uint16_t gx;
uint16_t gy;
uint16_t gz;

// raw mags
uint16_t mx;
uint16_t my;
uint16_t mz;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  if (myIMU.begin() == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");

  if (myIMU.enableAccelerometer() == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("Could not enable accelerometer");
  }

  if (myIMU.enableRawAccelerometer() == true) {
    Serial.println(F("Raw Accelerometer enabled"));
  } else {
    Serial.println("Could not enable raw accelerometer");
  }

  if (myIMU.enableGyro() == true) {
    Serial.println(F("Gyro enabled"));
  } else {
    Serial.println("Could not enable gyro");
  }

  if (myIMU.enableRawGyro() == true) {
    Serial.println(F("Raw Gyro enabled"));
  } else {
    Serial.println("Could not enable raw gyro");
  }

  if (myIMU.enableMagnetometer() == true) {
    Serial.println(F("Magnetometer enabled"));
  } else {
    Serial.println("Could not enable Magnetometer");
  }

  if (myIMU.enableRawMagnetometer() == true) {
    Serial.println(F("Raw Magnetometer enabled"));
  } else {
    Serial.println("Could not enable Raw Magnetometer");
  }

  Serial.println(F("Raw MEMS readings enabled"));
  Serial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true)
  {

    // keep track of if we've recieved an updated value on any one of the
    // reports we're looking for.
    uint8_t reportID = myIMU.getSensorEventID();

    switch (reportID)
    {
    case SENSOR_REPORTID_RAW_ACCELEROMETER:
      x = myIMU.getRawAccelX();
      y = myIMU.getRawAccelY();
      z = myIMU.getRawAccelZ();
      break;
    case SENSOR_REPORTID_RAW_GYROSCOPE:
      gx = myIMU.getRawGyroX();
      gy = myIMU.getRawGyroY();
      gz = myIMU.getRawGyroZ();
      break;
    case SENSOR_REPORTID_RAW_MAGNETOMETER:
      mx = myIMU.getRawMagX();
      my = myIMU.getRawMagY();
      mz = myIMU.getRawMagZ();
      break;
    default:
      break;
    }

    // Only print data to terminal if one of the report IDs we want came
    // in on the bus
    if( (reportID == SENSOR_REPORTID_RAW_ACCELEROMETER) ||
        (reportID == SENSOR_REPORTID_RAW_GYROSCOPE) ||
        (reportID == SENSOR_REPORTID_RAW_MAGNETOMETER))
    {
      Serial.print(x);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(z);
      Serial.print("\t");

      Serial.print(gx);
      Serial.print("\t");
      Serial.print(gy);
      Serial.print("\t");
      Serial.print(gz);
      Serial.print("\t");

      Serial.print(mx);
      Serial.print("\t");
      Serial.print(my);
      Serial.print("\t");
      Serial.print(mz);
      Serial.print("\t");

      Serial.println();
    }
  }
}
