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
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT
  A5  --> RST

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BNO08x myIMU;

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
#define BNO08X_INT  A4
//#define BNO08X_INT  -1
#define BNO08X_RST  A5
//#define BNO08X_RST  -1

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

// variables to store all our incoming values

// raw accel
int16_t x;
int16_t y;
int16_t z;

// raw gyros
int16_t gx;
int16_t gy;
int16_t gz;

// raw mags
int16_t mx;
int16_t my;
int16_t mz;

unsigned long previousDebugMillis = 0;
#define DEBUG_INTERVAL_MILLISECONDS 30

void setup() {
  Serial.begin(115200);
  
  while(!Serial) delay(10); // Wait for Serial to become available.
  // Necessary for boards with native USB (like the SAMD51 Thing+).
  // For a final version of a project that does not need serial debug (or a USB cable plugged in),
  // Comment out this while loop, or it will prevent the remaining code from running.
  
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  //Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");

  if (myIMU.enableAccelerometer(1) == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("Could not enable accelerometer");
  }

  if (myIMU.enableRawAccelerometer(1) == true) {
    Serial.println(F("Raw Accelerometer enabled"));
  } else {
    Serial.println("Could not enable raw accelerometer");
  }

  if (myIMU.enableGyro(1) == true) {
    Serial.println(F("Gyro enabled"));
  } else {
    Serial.println("Could not enable gyro");
  }

  if (myIMU.enableRawGyro(1) == true) {
    Serial.println(F("Raw Gyro enabled"));
  } else {
    Serial.println("Could not enable raw gyro");
  }

  if (myIMU.enableMagnetometer(1) == true) {
    Serial.println(F("Magnetometer enabled"));
  } else {
    Serial.println("Could not enable Magnetometer");
  }

  if (myIMU.enableRawMagnetometer(1) == true) {
    Serial.println(F("Raw Magnetometer enabled"));
  } else {
    Serial.println("Could not enable Raw Magnetometer");
  }

  Serial.println(F("Raw MEMS readings enabled"));
  Serial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));
}

void loop() {
  delayMicroseconds(10);

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

    // Only print data to the terminal at a user defined interval
    // Each data type (accel or gyro or mag) is reported from the
    // BNO086 as separate messages.
    // To allow for all these separate messages to arrive, and thus
    // have updated data on all axis/types, 
    // The report intervals for each datatype must be much faster
    // than our debug interval.

    // time since last debug data printed to terminal
    int timeSinceLastSerialPrint = (millis() - previousDebugMillis);

    // Only print data to the terminal at a user deficed interval
    if(timeSinceLastSerialPrint > DEBUG_INTERVAL_MILLISECONDS)
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

      Serial.print(timeSinceLastSerialPrint);

      Serial.println();

      previousDebugMillis = millis();

    }
  }
}
