#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// Adafruit_MPU6050 mpu;
Servo ESC;

int count;
float potValue = 100; // value from the analog pin
float accel_y_d = -1.0; // desired pitch
float tolerance = 0.1, delta = 0.2;

void setup() {
  // Attach the ESC on pin 13
  ESC.attach(16, 1000, 2000); // D0 (pin, min pulse width, max pulse width in microseconds)
  Serial.begin(115200);

  // // MPU setup
  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Serial.println("Adafruit MPU6050 test!");

  // // Try to initialize!
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("MPU6050 Found!");

  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Serial.print("Accelerometer range set to: ");
  // switch (mpu.getAccelerometerRange()) {
  //   case MPU6050_RANGE_2_G:
  //     Serial.println("+-2G");
  //     break;
  //   case MPU6050_RANGE_4_G:
  //     Serial.println("+-4G");
  //     break;
  //   case MPU6050_RANGE_8_G:
  //     Serial.println("+-8G");
  //     break;
  //   case MPU6050_RANGE_16_G:
  //     Serial.println("+-16G");
  //     break;
  // }
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Serial.print("Gyro range set to: ");
  // switch (mpu.getGyroRange()) {
  //   case MPU6050_RANGE_250_DEG:
  //     Serial.println("+- 250 deg/s");
  //     break;
  //   case MPU6050_RANGE_500_DEG:
  //     Serial.println("+- 500 deg/s");
  //     break;
  //   case MPU6050_RANGE_1000_DEG:
  //     Serial.println("+- 1000 deg/s");
  //     break;
  //   case MPU6050_RANGE_2000_DEG:
  //     Serial.println("+- 2000 deg/s");
  //     break;
  // }

  // mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // Serial.print("Filter bandwidth set to: ");
  // switch (mpu.getFilterBandwidth()) {
  //   case MPU6050_BAND_260_HZ:
  //     Serial.println("260 Hz");
  //     break;
  //   case MPU6050_BAND_184_HZ:
  //     Serial.println("184 Hz");
  //     break;
  //   case MPU6050_BAND_94_HZ:
  //     Serial.println("94 Hz");
  //     break;
  //   case MPU6050_BAND_44_HZ:
  //     Serial.println("44 Hz");
  //     break;
  //   case MPU6050_BAND_21_HZ:
  //     Serial.println("21 Hz");
  //     break;
  //   case MPU6050_BAND_10_HZ:
  //     Serial.println("10 Hz");
  //     break;
  //   case MPU6050_BAND_5_HZ:
  //     Serial.println("5 Hz");
  //     break;
  // }

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  // ESC.write(potValue); // Send the signal to the ESC
  // delay(1000); // wait 3 seconds to arm the drone

  // for (count = 0; count < 180; count++) {
  //   ESC.write(potValue); // less than 180

  //   Serial.print("potValue = ");
  //   Serial.print(potValue);
  //   Serial.print(", count = ");
  //   Serial.println(count);

    // MPU
    // sensors_event_t a, g, temp;
    // mpu.getEvent(&a, &g, &temp);
    // Serial.print("Acceleration Y: ");
    // Serial.print(a.acceleration.y);
    // Serial.println(" m/s^2");

    // Control of motor speed to meet the desired pitch
    // if ((a.acceleration.y > (accel_y_d + tolerance)) || (a.acceleration.y < (accel_y_d - tolerance))) {
    //   potValue += delta;
    //   if (potValue > 180) potValue = 180;
    for(int p = potValue; p >= 150; p+=10){

      ESC.write(p);
      Serial.print(p);
      delay(1000);
    }
    // } else {
    //   potValue -= delta;
    //   if (potValue < 20) potValue = 20;
    //   ESC.write(potValue);
    // }
    // delay(500);
  }
// }
