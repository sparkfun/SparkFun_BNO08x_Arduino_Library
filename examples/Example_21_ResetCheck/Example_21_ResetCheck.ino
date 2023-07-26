/*
  Using the BNO08x IMU

  This example shows how check for a "Reset Complete" message from the sensor,
  which is helpful when used in tandem with requestResetReason() and 
  getResetReason(). The sensor will be reset each time 25 readings are received 
  to demonstrate. 

  It outputs the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  By: @mattbradford83
  Date: 1 August 2021

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
  Plug the sensor into IoT Redboard via QWIIC cable.
  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

int cyclecount = 0;

#define BNO08X_ADDR 0x4B  // SparkFun BNO080 Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  if (myIMU.begin(BNO08X_ADDR) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  Serial.print(F(" Reason: "));
  Serial.println(myIMU.getResetReason());

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableGyro() == true) {
    Serial.println(F("Gyro enabled"));
    Serial.println(F("Output in form x, y, z, in radians per second"));
  } else {
    Serial.println("Could not enable gyro");
  }
}

void loop() {
  delay(1);
  myIMU.serviceBus();

  // One of these will appear at the very start because of the power on reset.
  // Use requestResetReason() and getResetReason() for the difference between
  // different resets.
  if (myIMU.wasReset()) {
    Serial.println(" ------------------ BNO08x has reset. ------------------ ");
    //Serial.println(myIMU.getResetReason());
    // if( myIMU.requestResetReason() == true ) {
    //     Serial.print(F(" Reason: "));
    //     Serial.println(myIMU.getResetReason());
    //   }
    // else {
    //   Serial.println("Reset Reason Request failed.");
    // }

      for (int n = 0; n < myIMU.prodIds.numEntries; n++) {
        Serial.print("ResetCause ");
    Serial.print(myIMU.prodIds.entry[n].resetCause);
    Serial.print("Part ");
    Serial.print(myIMU.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(myIMU.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(myIMU.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(myIMU.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(myIMU.prodIds.entry[n].swBuildNumber);
  }

    setReports();  // We'll need to re-enable reports after any reset.
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {

      cyclecount++;

      Serial.print(F("["));
      if (cyclecount < 10) Serial.print(F("0"));
      Serial.print(cyclecount);
      Serial.print(F("] "));

      float x = myIMU.getGyroX();
      float y = myIMU.getGyroY();
      float z = myIMU.getGyroZ();

      Serial.print(x, 2);
      Serial.print(F(","));
      Serial.print(y, 2);
      Serial.print(F(","));
      Serial.print(z, 2);
      Serial.print(F(","));

      Serial.println();

      if (cyclecount == 25) {
        myIMU.softReset();
        cyclecount = 0;
      }
    }
  }
}
