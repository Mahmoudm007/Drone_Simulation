#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

#include <Servo.h>
Servo ESC1;     // create servo object to control ESC1
Servo ESC2;     // create servo object to control ESC2
Servo ESC3;     // create servo object to control ESC3
Servo ESC4;     // create servo object to control ESC4

int count;
float potValue1 = 50;  // value from the analog pin for ESC1
float potValue2 = 50;  // value from the analog pin for ESC2
float potValue3 = 50;  // value from the analog pin for ESC3
float potValue4 = 50;  // value from the analog pin for ESC4

float accel_y_d = -1.0; // desired pitch
float tolerance = 0.1, delta = 0.2;

void setup() {
  // Attach the ESCs
  ESC1.attach(15, 1000, 2000); // D8 for ESC1
  ESC2.attach(16, 1000, 2000); // D0 for ESC2
  ESC3.attach(12, 1000, 2000); // D6 for ESC3
  ESC4.attach(14, 1000, 2000); // D5 for ESC4

  Serial.begin(115200);
  
  // MPU setup
  while (!Serial)
    delay(10);
  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  ESC1.writeMicroseconds(map(potValue1, 0, 180, 1000, 2000));
  ESC2.writeMicroseconds(map(potValue2, 0, 180, 1000, 2000));
  ESC3.writeMicroseconds(map(potValue3, 0, 180, 1000, 2000));
  ESC4.writeMicroseconds(map(potValue4, 0, 180, 1000, 2000));
  delay(3000); // Wait 3 seconds to arm the drone

  for (count = 0; count < 180; count++) {
    ESC1.writeMicroseconds(map(potValue1, 0, 180, 1000, 2000));
    ESC2.writeMicroseconds(map(potValue2, 0, 180, 1000, 2000));
    ESC3.writeMicroseconds(map(potValue3, 0, 180, 1000, 2000));
    ESC4.writeMicroseconds(map(potValue4, 0, 180, 1000, 2000));

    Serial.print("potValue1= "); Serial.print(potValue1);
    Serial.print("  potValue2= "); Serial.println(potValue2);
    Serial.print("potValue3= "); Serial.print(potValue3);
    Serial.print("  potValue4= "); Serial.println(potValue4);
    Serial.print("count= "); Serial.println(count);
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // MPU readings
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");
    Serial.println("");

    // Control of motor speed to meet the desired pitch
    if ((a.acceleration.y > (accel_y_d + tolerance)) || (a.acceleration.y < (accel_y_d - tolerance))) {
      potValue1 += delta;
      potValue2 -= delta;
      potValue3 += delta;
      potValue4 -= delta;

      // Ensure values don't exceed limits
      if (potValue1 < 20) potValue1 = 20;
      if (potValue2 < 20) potValue2 = 20;
      if (potValue3 < 20) potValue3 = 20;
      if (potValue4 < 20) potValue4 = 20;
    }
    delay(500);
  }
}
