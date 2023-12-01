/*
  Using the BNO08x IMU

  This example shows how to adjust settings of the dynamic calibration of the 
  BNO08x. 
  
  The BNO08x allows you to turn on/off dynamic calibration for each sensor in 
  the IMU (accel, gyro, or mag).

  Please refer to the BNO08X data sheet Section 3 (page 37)
  https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/component_documentation/BNO080_085-Datasheet_v1.16.pdf

  Note, by default, dynamic calibration is enabled for accel and mag.
  Some special use cases may require turning on all or any special combo of sensor 
  dynamic calibration.
  
  After the calibration settings are set, this example will output the 
  x/y/z/accuracy of the mag and the i/j/k/real parts of the game rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  Note, the "simple calibration" feature, sh2_startCal(), is not available on 
  the BNO08x. See this issue for more info:
  https://github.com/ceva-dsp/sh2/issues/11

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

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
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

// mags
float mx;
float my;
float mz;
byte magAccuracy;

// quats
float quatI;
float quatJ;
float quatK;
float quatReal;

unsigned long previousDebugMicros = 0;
#define DEBUG_INTERVAL_MICROSECONDS 10000

void setup() {
  Serial.begin(115200);
  
  while(!Serial) delay(10); // Wait for Serial to become available.
  // Necessary for boards with native USB (like the SAMD51 Thing+).
  // For a final version of a project that does not need serial debug (or a USB cable plugged in),
  // Comment out this while loop, or it will prevent the remaining code from running.
  
  Serial.println();
  Serial.println("BNO08x Calibration Example");

  Wire.begin();

  //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  // Enable dynamic calibration for desired sensors (accel, gyro, and mag)
  // uncomment/comment out as needed to try various options
  if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_GYRO || SH2_CAL_MAG) == true) { // all three sensors
  //if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_MAG) == true) { // Default settings
  //if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL) == true) { // only accel
    Serial.println(F("Calibration Command Sent Successfully"));
  } else {
    Serial.println("Could not send Calibration Command. Freezing...");
    while(1) delay(10);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");

  if (myIMU.enableMagnetometer(1) == true) {
    Serial.println(F("Magnetometer enabled"));
    Serial.println(F("Output in form x, y, z, in uTesla"));
  } else {
    Serial.println("Could not enable magnetometer");
  }

  if (myIMU.enableGameRotationVector(1) == true) {
    Serial.println(F("Game Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real"));
  } else {
    Serial.println("Could not enable game rotation vector");
  }
}

void loop() {
  delayMicroseconds(10);

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {
    // is the event a report of the magnetometer?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
      mx = myIMU.getMagX();
      my = myIMU.getMagY();
      mz = myIMU.getMagZ();
      magAccuracy = myIMU.getMagAccuracy();
    }
    // is the event a report of the game rotation vector?
    else if (myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      quatI = myIMU.getGameQuatI();
      quatJ = myIMU.getGameQuatJ();
      quatK = myIMU.getGameQuatK();
      quatReal = myIMU.getGameQuatReal();
    } 
  }

  // Only print data to the terminal at a user defined interval
  // Each data type (accel or gyro or mag) is reported from the
  // BNO086 as separate messages.
  // To allow for all these separate messages to arrive, and thus
  // have updated data on all axis/types, 
  // The report intervals for each datatype must be much faster
  // than our debug interval.

  // time since last debug data printed to terminal
  unsigned long microsSinceLastSerialPrint = (micros() - previousDebugMicros);

  // Only print data to the terminal at a user deficed interval
  if(microsSinceLastSerialPrint > DEBUG_INTERVAL_MICROSECONDS)
  {
    Serial.print(mx, 2);
    Serial.print("\t\t");
    Serial.print(my, 2);
    Serial.print("\t\t");
    Serial.print(mz, 2);
    Serial.print("\t\t");
    printAccuracyLevel(magAccuracy);
    Serial.print("\t\t");

    Serial.print(quatI, 2);
    Serial.print("\t\t");
    Serial.print(quatJ, 2);
    Serial.print("\t\t");
    Serial.print(quatK, 2);
    Serial.print("\t\t");
    Serial.print(quatReal, 2);
    Serial.print("\t\t");

    Serial.print(microsSinceLastSerialPrint);
    Serial.println();
    previousDebugMicros = micros();
  }

  if(Serial.available())
  {
    byte incoming = Serial.read();

    if(incoming == 's')
    {
      // Saves the current dynamic calibration data (DCD) to memory
      // Note, The BNO08X stores updated Dynamic Calibration Data (DCD) to RAM 
      // frequently (every 5 seconds), so this command may not be necessary
      // depending on your application.
      if (myIMU.saveCalibration() == true) {
        Serial.println(F("Calibration data was saved successfully"));
      } else {
        Serial.println("Save Calibration Failure");
      }
    }
  }
}

//Given a accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}