#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

String greeting = "Hello World!";
String farewell = "Goodbye World!";
BLEService greetingService("180C");
BLECharacteristic greetingCharacteristic("2A56", BLERead | BLENotify, "Hello World!");

float accOffsetX;
float accOffsetY;
float accOffsetZ;
float posX;
float posY;
float posZ;
bool first;

float relativeAcceleration(float acc, float offset) {
  if ((acc > 0.0 && offset > 0.0) || (acc < 0.0 && offset < 0.0)) {
    return acc - offset;
  } else {
    return acc + offset;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("starting BLE failed");
    while (1);
  }
  if (!IMU.begin()) {
    Serial.println("starting IMU failed");
    while(1);
  }

  first = true;

  BLE.setLocalName("Nano33BLE");
  BLE.setAdvertisedService(greetingService);
  greetingService.addCharacteristic(greetingCharacteristic);
  BLE.addService(greetingService);
  /*greetingCharacteristic.setValue(greeting);*/

  BLE.advertise();
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();
  float accX, accY, accZ, gyrX, gyrY, gyrZ, relAccX, relAccY, relAccZ gyrSample;
  if (first) { delay(500); }
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }

  if (central) {
    byte greetBytes[13];
    byte byeBytes[15];
    
    Serial.print("Connected to central MAC: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      farewell.getBytes(byeBytes, sizeof(byeBytes));
      greetingCharacteristic.writeValue(byeBytes, sizeof(byeBytes));
      delay(1000);
      greeting.getBytes(greetBytes, sizeof(greetBytes));
      greetingCharacteristic.writeValue(greetBytes, sizeof(greetBytes));
      delay(1000);
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central MAC: ");
    Serial.println(central.address());
  }
  first = false;
}
