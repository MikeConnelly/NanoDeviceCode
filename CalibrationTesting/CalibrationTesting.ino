// Madgwick - Version: Latest 
#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>

// unused Arduino Cloud variables
//#include "thingProperties.h"
/* 
  Arduino IoT Cloud Variables description

  CloudAngle oriXiot;
  CloudLength posXiot;
  CloudLength posYiot;
  CloudLength posZiot;
*/

// define global variables
Madgwick filter;
float roll, pitch, yaw;
float calX, calY, calZ;
float accX, accY, accZ;
float velX, velY, velZ;
float prevVelX, prevVelY, prevVelZ;
float posX, posY, posZ;
float gyroX, gyroY, gyroZ;
float gyroAvgX, gyroAvgY, gyroAvgZ;
float measurementTime, prevMeasurementTime;
int noAccXCount, noAccYCount, noAccZCount;
int SAMPLE_CONSTANT = 4;
int CALIBRATION_CONSTANT = 2;
int ACCELERATION_CUTOFF_CONSTANT = 2;
float NOISE_MARGIN = 0.2;

// function to help calculate position from accelerometer
bool withinErrorMargin(float var, float val) {
  return (var <= val) && (var >= (-1*val));
}

// function to help calculate position from accelerometer
bool outOfRange(float var, float val, float margin) {
  return (var > (val + margin)) || (var < (val - margin));
}

// function to calibrate the IMU accelerometer and gyroscope
void calibrate() {
  int count = 0;
  float sampleX, sampleY, sampleZ;
  float gyroSampleX, gyroSampleY, gyroSampleZ;
  int gyroCount = 0;
  calX = 0, calY = 0, calZ = 0;
  gyroAvgX = 0, gyroAvgY = 0, gyroAvgZ = 0;
  
  while (count < CALIBRATION_CONSTANT) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(sampleX, sampleY, sampleZ);
      calX = calX + sampleX;
      calY = calY + sampleY;
      calZ = calZ + sampleZ;
      count++;
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyroSampleX, gyroSampleY, gyroSampleZ);
      gyroAvgX = gyroAvgX + gyroSampleX;
      gyroAvgY = gyroAvgY + gyroSampleY;
      gyroAvgZ = gyroAvgZ + gyroSampleZ;
      gyroCount++;
    }
  }
  calX = calX / CALIBRATION_CONSTANT;
  calY = calY / CALIBRATION_CONSTANT;
  calZ = calZ / CALIBRATION_CONSTANT;
  gyroAvgX = gyroAvgX / gyroCount;
  gyroAvgY = gyroAvgY / gyroCount;
  gyroAvgZ = gyroAvgZ / gyroCount;
}


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // unused Arduino cloud setup
  // Defined in thingProperties.h
  //initProperties();
  // Connect to Arduino IoT Cloud
  //ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  //setDebugMessageLevel(2);
  //ArduinoCloud.printDebugInfo();

  if (!IMU.begin()) {
    Serial.println("starting IMU failed");
    while(1);
  }

  // set global variable default values
  velX = 0, velY = 0, velZ = 0;
  prevVelX = 0, prevVelY = 0, prevVelZ = 0;
  posX = 0, posY = 0, posZ = 0;
  gyroAvgX = 0, gyroAvgY = 0, gyroAvgZ = 0;
  measurementTime = 0, prevMeasurementTime = 0;
  noAccXCount = 0, noAccYCount = 0, noAccZCount = 0;
}

void loop() {
  //ArduinoCloud.update();
  calibrate();
  float dMeasurementTime;
  float avgVelX, avgVelY, avgVelZ;
  float sampleX, sampleY, sampleZ;
  float gyroSampleX, gyroSampleY, gyroSampleZ;
  int count = 0;
  int gyroCount = 0;
  accX = 0;
  accY = 0;
  accZ = 0;
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;

  while (count < SAMPLE_CONSTANT) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(sampleX, sampleY, sampleZ);
      accX = accX + sampleX - calX;
      accY = accY + sampleY - calY;
      accZ = accZ + sampleZ - calZ;
      count++;
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyroSampleX, gyroSampleY, gyroSampleZ);
      gyroX = gyroX + gyroSampleX - gyroAvgX;
      gyroY = gyroY + gyroSampleY - gyroAvgY;
      gyroZ = gyroZ + gyroSampleZ - gyroAvgZ;
      gyroCount++;
      /*Serial.print("gyro: ");
      Serial.print(gyroX);
      Serial.print(" ");
      Serial.print(gyroY);
      Serial.print(" ");
      Serial.println(gyroZ);*/
      
      if (outOfRange(gyroX, 0, 75) || outOfRange(gyroY, 0, 75) || outOfRange(gyroZ, 0, 75)) {
        calibrate();
      }
    }
  }
  
  measurementTime = millis() * 0.001;
  dMeasurementTime = measurementTime - prevMeasurementTime;
  
  accX = accX / SAMPLE_CONSTANT;
  accY = accY / SAMPLE_CONSTANT;
  accZ = accZ / SAMPLE_CONSTANT;

  accX = accX * 9.8;
  accY = accY * 9.8;
  accZ = accZ * 9.8;

  if (withinErrorMargin(accX, NOISE_MARGIN)) {
    accX = 0;
    noAccXCount++;
  } else {
    noAccXCount = 0;
  }
  if (withinErrorMargin(accY, NOISE_MARGIN)) {
    accY = 0;
    noAccYCount++;
  } else {
    noAccYCount = 0;
  }
  if (withinErrorMargin(accZ, NOISE_MARGIN)) {
    accZ = 0;
    noAccZCount++;
  } else {
    noAccZCount = 0;
  }

  velX = prevVelX + (accX * dMeasurementTime);
  velY = prevVelY + (accY * dMeasurementTime);
  velZ = prevVelZ + (accZ * dMeasurementTime);

  if (noAccXCount == ACCELERATION_CUTOFF_CONSTANT) { velX = 0; }
  if (noAccYCount == ACCELERATION_CUTOFF_CONSTANT) { velY = 0; }
  if (noAccZCount == ACCELERATION_CUTOFF_CONSTANT) { velZ = 0; }

  avgVelX = (velX + prevVelX) / 2;
  avgVelY = (velY + prevVelY) / 2;
  avgVelZ = (velZ + prevVelZ) / 2;
  
  posX = posX + (avgVelX * dMeasurementTime);
  posY = posY + (avgVelY * dMeasurementTime);
  posZ = posZ + (avgVelZ * dMeasurementTime);
 
  prevVelX = velX;
  prevVelY = velY;
  prevVelZ = velZ;
  prevMeasurementTime = measurementTime;

  Serial.print("pos: ");
  Serial.print(posX);
  Serial.print(" ");
  Serial.print(posY);
  Serial.print(" ");
  Serial.println(posZ);
  //posXiot = posX;
  //posYiot = posY;
  //posZiot = posZ;
  // device sleeping can get to 0.96 microA, 5-6mW
  // full startup time can be up to 50ms, cell devices for arduino, cell to iot service
}
