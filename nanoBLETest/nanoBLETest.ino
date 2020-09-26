#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

BLEService accelerationService("180C");
BLEService velocityService("180D");
BLEService positionService("180E");
BLEFloatCharacteristic accXChar("2A10", BLERead | BLENotify);
BLEFloatCharacteristic accYChar("2A11", BLERead | BLENotify);
BLEFloatCharacteristic accZChar("2A12", BLERead | BLENotify);
BLEFloatCharacteristic velXChar("2A13", BLERead | BLENotify);
BLEFloatCharacteristic velYChar("2A14", BLERead | BLENotify);
BLEFloatCharacteristic velZChar("2A15", BLERead | BLENotify);
BLEFloatCharacteristic posXChar("2A16", BLERead | BLENotify);
BLEFloatCharacteristic posYChar("2A17", BLERead | BLENotify);
BLEFloatCharacteristic posZChar("2A18", BLERead | BLENotify);

float accOffsetX;
float accOffsetY;
float accOffsetZ;
float prevAccX;
float prevAccY;
float prevAccZ;
float prevVelX;
float prevVelY;
float prevVelZ;
float velX;
float velY;
float velZ;
float posX;
float posY;
float posZ;
float prevMeasurementTime;
bool first;

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("starting BLE failed");
    while (1);
  }
  if (!IMU.begin()) {
    Serial.println("starting IMU failed");
    while(1);
  }

  accOffsetX = 0;
  accOffsetY = 9.8;
  accOffsetZ = 0;
  prevAccX = 0;
  prevAccY = 0;
  prevAccZ = 0;
  prevVelX = 0;
  prevVelY = 0;
  prevVelZ = 0;
  velX = 0;
  velY = 0;
  velZ = 0;
  posX = 0;
  posY = 0;
  posZ = 0;
  prevMeasurementTime = 0;
  first = true;

  BLE.setLocalName("Nano33BLE");
  //BLE.setAdvertisedService(greetingService);
  //greetingService.addCharacteristic(greetingCharacteristic);
  //BLE.addService(greetingService);
  BLE.setAdvertisedService(accelerationService);
  BLE.setAdvertisedService(velocityService);
  BLE.setAdvertisedService(positionService);
  accelerationService.addCharacteristic(accXChar);
  accelerationService.addCharacteristic(accYChar);
  accelerationService.addCharacteristic(accZChar);
  velocityService.addCharacteristic(velXChar);
  velocityService.addCharacteristic(velYChar);
  velocityService.addCharacteristic(velZChar);
  positionService.addCharacteristic(posXChar);
  positionService.addCharacteristic(posYChar);
  positionService.addCharacteristic(posZChar);
  BLE.addService(accelerationService);
  BLE.addService(velocityService);
  BLE.addService(positionService);
  
  BLE.advertise();
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  float accX, accY, accZ, measurementTime, dMeasurementTime;
  // float dvX, dvY, dvZ, dpX, dpY, dpZ = 0;
  float avgAccX, avgAccY, avgAccZ, avgVelX, avgVelY, avgVelZ = 0;
  // float accsX[20], accsY[20], accsZ[20];

  if (central) {
    if (first) { delay(500); }
    
    Serial.print("Connected to central MAC: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
      }
      
      measurementTime = millis() * 0.001;
      dMeasurementTime = measurementTime - prevMeasurementTime;

      accX = (accX*9.8) + accOffsetX;
      accY = (accY*9.8) + accOffsetY;
      accZ = (accZ*9.8) + accOffsetZ;
      
      Serial.println(accX);

      avgAccX = (accX + prevAccX) / 2;
      avgAccY = (accY + prevAccY) / 2;
      avgAccZ = (accZ + prevAccZ) / 2;
      /*
      dvX = accX * dMeasurementTime;
      dvY = accY * dMeasurementTime;
      dvZ = accZ * dMeasurementTime;
      */
      velX = prevVelX + (avgAccX * dMeasurementTime);
      velY = prevVelY + (avgAccY * dMeasurementTime);
      velZ = prevVelZ + (avgAccZ * dMeasurementTime);
/*
      dpX = velX * dMeasurementTime;
      dpY = velY * dMeasurementTime;
      dpZ = velZ * dMeasurementTime;
*/
      avgVelX = (velX + prevVelX) / 2;
      avgVelY = (velY + prevVelY) / 2;
      avgVelZ = (velZ + prevVelZ) / 2;
      
      posX = posX + (avgVelX * dMeasurementTime);
      posY = posY + (avgVelY * dMeasurementTime);
      posZ = posZ + (avgVelZ * dMeasurementTime);

      prevMeasurementTime = measurementTime;
      prevAccX = accX;
      prevAccY = accY;
      prevAccZ = accZ;
      prevVelX = velX;
      prevVelY = velY;
      prevVelZ = velZ;

      accXChar.writeValue(accX);
      accYChar.writeValue(accY);
      accZChar.writeValue(accZ);

      velXChar.writeValue(velX);
      velYChar.writeValue(velY);
      velZChar.writeValue(velZ);

      posXChar.writeValue(posX);
      posYChar.writeValue(posY);
      posZChar.writeValue(posZ);
      
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central MAC: ");
    Serial.println(central.address());
    first = false;
  }
}
