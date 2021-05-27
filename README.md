## Setup
Configure the Arduino IDE to use the Arduino Nano 33 BLE board in Board Manager: Arduino nRF528x Boards (Mbed OS)
Using the Arduino IDE or another dependancy manager like Platformio install the following:
The Madgwick filter library with header MadgwickAHRS.h [https://www.arduino.cc/reference/en/libraries/madgwick/]
The IMU interface library (programs are currently for the BLE):
- The Arduino nano 33 BLE uses LSM9DS1 with header Arduino_LSM9DS1.h [https://www.arduino.cc/en/Reference/ArduinoLSM9DS1]
- The Arduino nano 33 IOT uses LSM6DS3 with header Arduino_LSM6DS3.h [https://www.arduino.cc/en/Reference/ArduinoLSM6DS3]


### IMUFilter
Program that uses the Madgwick filter to determine position from the IMU data.
Visualizer program in Visualizer. To run, install processing [https://processing.org/download/]
The visualizer used on the website is similar to this visualizer code but converted to JavaScript.


### CellularTest
Basic test of cellular modules capabilities with list of basic commands and resource commented.
The Arduino nano 33 BLE has two Serial interfaces accessed with "Serial" and "Serial1".


### CalibrationTesting
Original code for getting data from the IMU and working with that data.
Useful as a reference for working with the IMU.
Code for Arduino IOT Cloud commented and can probably remain unused [https://www.arduino.cc/en/IoT/HomePage]


## Next Steps
- Use the basic Serial communication in CellularTest to extend IMUFilter
- Send http commands over Serial to POST position data to the website
- When the website has position data, further expansions can be done from there


## Resources
Good description of Sim800l module and serial commands:
https://lastminuteengineers.com/sim800l-gsm-module-arduino-tutorial/

