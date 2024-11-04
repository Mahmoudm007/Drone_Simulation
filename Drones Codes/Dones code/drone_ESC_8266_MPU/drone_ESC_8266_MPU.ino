#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

#include <Servo.h>

Servo ESC1;     // create servo object to control the ESC
Servo ESC2;     // create servo object to control the ESC
Servo ESC3;     // create servo object to control the ESC
Servo ESC4;     // create servo object to control the ESC



int count;
float potValue1=50;  // value from the analog pin
float potValue2=50;
float potValue3=50;
float potValue4=50;

float accel_x_d= -1.0;//desired roll
float accel_y_d= -1.0;//desired pitch
float tolerance=0.1, delta=0.2;




void setup() {
  // Attach the ESC on pin 13,9
  ESC1.attach(0,1000,2000); // 0 is D3, 9(pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(2,1000,2000); //2 is D4, 9(pin, min pulse width, max pulse width in microseconds)  
  ESC3.attach(14,1000,2000); //14 is D5, 9(pin, min pulse width, max pulse width in microseconds)  
  ESC4.attach(12,1000,2000); //12 is D6, 9(pin, min pulse width, max pulse width in microseconds)  
  Serial.begin(115200);
  ////////////////MPU setup
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // infinite loop
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

  Serial.println("////////////////////////////////////////////////////////////////////////////////////////////");
  delay(100);
  ///////////////////////
}




void loop() {
  /* Get new sensor events with the readings */
    ESC1.write(potValue1);    // Send the signal to the ESC'
    delay(1000);
    ESC2.write(potValue2);
    delay(1000);
    ESC3.write(potValue3);
    delay(1000);
    ESC4.write(potValue4);
    delay(1000);//wait 3 seconds to arm the drone
  
  for(count=0;count<180;count+=10)
  {
    // potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
    //potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC1.write(potValue1);    // Send the signal to the ESC'
    delay(1000);
    ESC2.write(potValue2);
    delay(1000);
    ESC3.write(potValue3);
    delay(1000);
    ESC4.write(potValue4);
    delay(1000);//wait 3 seconds to arm the drone
    // analogWrite(9, 255);// 

    Serial.print("potValue1 = ");Serial.println(potValue1);
    Serial.print("potValue2 = ");Serial.println(potValue2);
    Serial.print("potValue3 = ");Serial.println(potValue3);
    Serial.print("potValue4 = ");Serial.println(potValue4);
    Serial.print("count= ");Serial.println(count);
  //   //////////MPU ////////////

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
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
  ///////////// control of motor speed to meet the desired pitch //////////
  if((a.acceleration.y>(accel_y_d+tolerance))||(a.acceleration.y>(accel_y_d-tolerance))){
    potValue1+=delta;
    ESC1.write(potValue1);
    potValue2-=delta;
    if( potValue2<20) potValue2=20;
    ESC2.write(potValue2);
  }
  if((a.acceleration.y<=(accel_y_d+tolerance))||(a.acceleration.y<=(accel_y_d-tolerance))){
    potValue2+=delta;
    ESC2.write(potValue2);
    potValue1-=delta;
    if( potValue1<20) potValue1=20;
    ESC1.write(potValue1);
  }

    if((a.acceleration.x>(accel_x_d+tolerance))||(a.acceleration.x>(accel_x_d-tolerance))){
    potValue3+=delta;
    ESC3.write(potValue3);
    potValue4-=delta;
    if( potValue4<20) potValue4=20;
    ESC4.write(potValue4);
  }

    if((a.acceleration.x>(accel_x_d+tolerance))||(a.acceleration.x>(accel_x_d-tolerance))){
    potValue4+=delta;
    ESC4.write(potValue4);
    potValue3-=delta;
    if( potValue3<20) potValue3=20;
    ESC3.write(potValue3);
  }
  ///////////////////////////
    delay(500);
  }
}
