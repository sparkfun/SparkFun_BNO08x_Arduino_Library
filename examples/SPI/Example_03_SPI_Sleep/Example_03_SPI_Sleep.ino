/*
  Using the BNO08x IMU

  This example shows how to use the sleep features of the sensor.

  It toggles the sleep mode on/off every 5 seconds.

  When awake, this example outputs the i/j/k/real parts of the rotation vector.
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
  D5  --> CS
  PICO --> SI
  POCI --> SO
  SCK  --> SCK
  A4  --> INT
  A5  --> RST
  3V3  --> 3V3
  GND  --> GND

  BNO08x "mode" pins set for SPI:
  PSO --> 3V3
  PS1 --> 3V3

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

// For SPI, we need some extra pins defined:
// Note, these can be other GPIO if you like.
#define BNO08X_CS   5
#define BNO08X_INT  A4
#define BNO08X_RST  A5

// If using the MicroMod Machine Learning Carrier Board, use these pins:
// for more info see https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/issues/19
//#define BNO08X_CS   PWM0
//#define BNO08X_INT  D0 // Note, D0 is also the CS pin for the camera, so you'll need to change that if you're using both.
// a good alternative for D0 is PWM1, (A0 and A1 are only inputs on the carrier board).
//#define BNO08X_RST  D1

unsigned long lastMillis = 0;  // Keep track of time
bool lastPowerState = true;    // Toggle between "On" and "Sleep"

void setup() {
  Serial.begin(115200);
  
  while(!Serial) delay(10); // Wait for Serial to become available.
  // Necessary for boards with native USB (like the SAMD51 Thing+).
  // For a final version of a project that does not need serial debug (or a USB cable plugged in),
  // Comment out this while loop, or it will prevent the remaining code from running.
  
  Serial.println();
  Serial.println("BNO08x Sleep Example");

  if (myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  setReports();

  Serial.println("Reading events");
  delay(100);

  lastMillis = millis();  // Keep track of time
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
  delay(100); // This delay allows enough time for the BNO086 to accept the new 
              // configuration and clear its reset status  
}

void loop() {
  delay(10);

  if (lastPowerState)  // Are we "On"?
  {

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
        Serial.print(F(","));

        Serial.println();
      }
    }
  }

  //Check if it is time to change the power state
  if (millis() > (lastMillis + 5000))  // Change state every 5 seconds
  {
    lastMillis = millis();  // Keep track of time

    if (lastPowerState)  // Are we "On"?
    {
      Serial.println("Putting BNO08x to sleep...");
      myIMU.modeSleep();  // Put BNO to sleep
    } else {
      Serial.println("Waking up BNO08x");
      myIMU.modeOn();  // Turn BNO back on
    }

    lastPowerState ^= 1;  // Invert lastPowerState (using ex-or)
  }
}
