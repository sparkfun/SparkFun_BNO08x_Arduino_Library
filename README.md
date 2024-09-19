SparkFun VR IMU BNO08X Arduino Library
===========================================================
Added shaker detection function to the main .cpp and .h file

[![SparkFun VR IMU Breakout - BNO086 (Qwiic)](https://cdn.sparkfun.com/r/600-600/assets/parts/2/3/0/2/7/22857-SEN_SparkFun_VR_IMU_Breakout-BNO086_Qwiic-_01.jpg)](https://www.sparkfun.com/products/22857)

[*SparkFun VR IMU Breakout - BNO086 (Qwiic)(SEN-22857)*](https://www.sparkfun.com/products/22857)

An Arduino Library for the BNO08x IMU combination triple axis accelerometer/gyro/magnetometer packaged with an ARM Cortex M0+ running powerful algorithms.

The BNO08x IMU has a combination triple axis accelerometer/gyro/magnetometer packaged with an ARM&copy; Cortex&trade; M0+ running powerful algorithms. This enables the BNO08x Inertial Measurement Unit (IMU) to produce accurate rotation vector headings with an error of 2 degrees or less. It's what we've been waiting for: all the sensor data is combined into meaningful, accurate IMU information.

This IC was designed to be implemented in Android based cellular phones to handle all the computations necessary for virtual reality goggles using only your phone. The sensor is quite powerful but with power comes a complex interface. We've written an I<sup>2</sup>C based library that provides the rotation vector (the reading most folks want from an IMU) as well as raw acceleration, gyro, and magnetometer readings. The sensor is capable of communicating over SPI and UART as well!

In addition the BNO08x IMU provides a built-in step counter, tap detector, activity classifier (are you running, walking, or sitting still?), and a shake detector. We are duly impressed.

This Arduino Library was built off the [original library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library) written by Nathan Seidle ([SparkFun](http://www.sparkfun.com)).

Thanks to all those who have helped improve this library!

This BNO086 Library - CEVA-DSP Sh2 Driver-based improvements:
* SFSailor for providing code for Geomagnetic Rotation Vector [issue #10](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/issues/10)
* LazaroFilm for adding setReorientation() [PR15](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/pull/15)
* rah2501 for fixing an INT/RST pin bug [PR13](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/pull/13)

Original BNO080 Library improvements:
* blendmaster for adding [Linear Accel report](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/4)
* per1234 for fixing our [keywords file](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/12)
* fm4dd for typo - [PR 19](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/19)
* tstellanova for heading accuracy correction - [PR 40](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/40)
* badVibes for gyro integrated rotation vector support - [PR 41](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/41)
* Filimindji for AR/VR Stabilized RotationVector and AR/VR Stabilized GameRotationVector support - [PR 46](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/46)
* ya-mouse for the getreadings improvements - [PR 55](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/55)
* Guillaume for the read-multiple-values helper functions and the interrupt example - [PR56](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/56) & [PR59](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/59)
* aedancullen for the tap detector - [PR 64](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/64)
* mattbradford83 for the hasReset code and example - [PR 92](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/pull/92)



Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.
* **[CONTRIBUTING.md](./CONTRIBUTING.md)** - guidance on how to contribute to this library.

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Hookup Guide](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/)** - Basic hookup guide for the SparkFun VR IMU Breakout - BNO086.
* **[Product Repository](https://github.com/sparkfun/SparkFun_VR_IMU_Breakout_BNO086_QWIIC)** - Main repository for the BNO086 (including hardware files)

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

_<COLLABORATION CREDIT>_
