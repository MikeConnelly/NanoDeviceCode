
// have to start position at 0, 0, 0
// accelerometer starts at 0, 1, 0 or something
// use gyroscope data to rotate position
// adjust acceleration correction based on position
// it won't be accurate but it might be good enough?

// use SF to get yaw, pitch, roll and use those /2pi radians to multiply by gravity?

float accOffsetX;
float accOffsetY;
float accOffsetZ;
float posX;
float posY;
float posZ;
int count;

float relativeAcceleration(float acc, float offset) {
  if ((acc > 0.0 && offset > 0.0) || (acc < 0.0 && offset < 0.0)) {
    return acc - offset;
  } else {
    return acc + offset;
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to init IMU");
  }
  count = 0;
}

void loop() {
  float accX, accY, accZ, gyrX, gyrY, gyrZ, relAccX, relAccY, relAccZ gyrSample;
  if (count == 0) { delay(500); }
  if (count > 10000) {return;}
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    if (count == 0) {
      Serial.println("acc setup");
      accOffsetX = accX;
      accOffsetY = accY;
      accOffsetZ = accZ;
      Serial.printf("%f\t%f\t%f\t%f\n", accX, accY, accZ, IMU.accelerationSampleRate());
    } else {
      IMU.readGyroscope(gyrX, gyrY, gyrZ);
      gyrSample = IMU.gyroscopeSampleRate();
      // rotated by gyrX * gyrSample
      
      relAccX = relativeAcceleration(accX, accOffsetX) * 9.8;
      relAccY = relativeAcceleration(accY, accOffsetY) * 9.8;
      relAccZ = relativeAcceleration(accZ, accOffsetZ) * 9.8;
      Serial.printf("%f\t%f\t%f\t%fHz\n", gyrX, gyrY, gyrZ, IMU.accelerationSampleRate());
      Serial.printf("%f\t%f\t%f\t%fHz\n", relAccX, relAccY, relAccZ, IMU.accelerationSampleRate());
      Serial.printf("%f\t%f\t%f\n", accOffsetX, accOffsetY, accOffsetZ);
    }
  }
  count++;
}
