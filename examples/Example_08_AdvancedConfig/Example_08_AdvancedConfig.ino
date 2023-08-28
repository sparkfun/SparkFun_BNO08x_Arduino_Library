/*
  Using the BNO08x IMU

  This example shows how to configure the sensor with different I2C address
  (0x4A) and different Wire ports (Wire1, on SDA:25, SCL:17).

  Note, for this example to work, you must close the address jumper on the
  bottom of the PCB, and wire up SDA/SCL to their newly defined pins (25/17).

  It will output the i/j/k/real parts of the rotation vector.
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

  IoT RedBoard --> BNO08x
  3.3V       -->    3.3V
  GND        -->    GND
  SCL1 (D17) -->    SCL
  SDA1 (D25) -->    SDA

  Note: Make sure to close the ADR jumper on the back of the board as well.

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

// define some pins for our new I2C port, aka "Wire1"
// on a SparkFun ESP32 IoT REdboard, these can be most pins, except those that
// are only inputs.
#define I2C_SDA1 25
#define I2C_SCL1 17

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire1.begin(I2C_SDA1, I2C_SCL1, 100000);

  // Wire1.begin(); // Some Arduino boards have pre-defined Wire1 SDA/SCL pins,
  // When using these, you don't have to supply the pin arguments, and they
  // are usually labled on the board as "SDA1" and "SCL1".

  // The first argument of our BNO08x begin() function is the I2C address of the
  // sensor, either 0x4B (default) or 0x4A. The second is the TwoWire I2C port
  // to use. Wire, Wire1, etc.
  if (myIMU.begin(0x4A, Wire1) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  // Wire1.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

      float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.print(F(","));
      Serial.print(quatReal, 2);
      Serial.print(F(","));
      Serial.print(quatRadianAccuracy, 2);

      Serial.println();
    }
  }
}
