/*
  Using the BNO08x IMU

  This example shows how to use two sensors on the same I2C bus.

  The sensors must be set up (using the ADDR jumpers) to different addresses:
  (0x4A and 0x4B)

  It outputs the i/j/k/real parts of the rotation vector from both sensors.
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

  Hardware Connections:
  "Daisey chain" two sensors to the IoT Redboard via QWIIC cables.
  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU1; //Open I2C ADR jumper - goes to address 0x4B
BNO08x myIMU2; //Closed I2C ADR jumper - goes to address 0x4A

// Sensor "1"'s data variables
float quatI;
float quatJ;
float quatK;
float quatReal;
float quatRadianAccuracy;

// Sensor "1"'s data variables
float quatI_2;
float quatJ_2;
float quatK_2;
float quatReal_2;
float quatRadianAccuracy_2;

// NewData is an update flag to know that new data has come in 
// and it's time to print to terminal.
bool newData = false;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  //Start 2 sensors
  if (myIMU1.begin(0x4B) == false) {
    Serial.println("BNO08x not detected at default I2C address (0x4B). Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found at 0x4B!");

  delay(3000);

  if (myIMU2.begin(0x4A) == false) {
    Serial.println("BNO08x not detected at I2C address (0x4A). Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found at 0x4A!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  //setReports1();
  setReports2();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports1(void) {
  Serial.println("Setting desired reports");
  if (myIMU1.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled for myIMU1"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector on myIMU1");
  }
}

// Here is where you define the sensor outputs you want to receive
void setReports2(void) {
  Serial.println("Setting desired reports");
  if (myIMU2.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled for my IMU2"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector on myIMU2");
  }
}

void loop() {
  delay(10);

  // if (myIMU1.wasReset()) {
  //   Serial.print("sensor1 was reset ");
  //   setReports1();
  // }

  if (myIMU2.wasReset()) {
    Serial.print("sensor2 was reset ");
    setReports2();
  }

  // Has a new event come in on the Sensor Hub Bus?
  // if (myIMU1.getSensorEvent() == true) {
  //   // is it the correct sensor data we want?
  //   if (myIMU1.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
  //     newData = true;
  //     quatI = myIMU1.getQuatI();
  //     quatJ = myIMU1.getQuatJ();
  //     quatK = myIMU1.getQuatK();
  //     quatReal = myIMU1.getQuatReal();
  //     quatRadianAccuracy = myIMU1.getQuatRadianAccuracy();
  //   }
  // }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU2.getSensorEvent() == true) {
    // is it the correct sensor data we want?
    if (myIMU2.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      newData = true;
      quatI_2 = myIMU2.getQuatI();
      quatJ_2 = myIMU2.getQuatJ();
      quatK_2 = myIMU2.getQuatK();
      quatReal_2 = myIMU2.getQuatReal();
      quatRadianAccuracy_2 = myIMU2.getQuatRadianAccuracy();
    }
  } 

  if (newData == true) {
    //Serial.print("First:");
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));

    //Serial.print("Second:");
    Serial.print(quatI_2, 2);
    Serial.print(F(","));
    Serial.print(quatJ_2, 2);
    Serial.print(F(","));
    Serial.print(quatK_2, 2);
    Serial.print(F(","));
    Serial.print(quatReal_2, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy_2, 2);
    Serial.print(F(","));

    Serial.println();

    newData = false; // clear flag so we only print again when new data arrives.
  }
}
