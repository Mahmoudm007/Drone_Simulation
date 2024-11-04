// https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/#google_vignette
/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
     used with Arduino Uno and atmega 2650
*/
// starts rotating at potValue=40
// we need 2 seconds for the esc to start
// when it stops beeping, the drone is ready
// working !!!
// use with control for drone coursep


#include <Servo.h>
Servo ESC1;     // create servo object to control the ESC
Servo ESC2;     // create servo object to control the ESC


int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 13,9
  ESC1.attach(2,1000,2000); // 2 is D4, 9(pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(16,1000,2000); //16 is D0, 9(pin, min pulse width, max pulse width in microseconds)  
  Serial.begin(115200);
}

void loop() {
  for(potValue=100;potValue<180;potValue++)
  {
    //potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
    //potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC1.write(potValue);    // Send the signal to the ESC
    ESC2.write(potValue);
    //analogWrite(9, 255);// 

    Serial.print("potValue= ");
    Serial.println(potValue);
    delay(3000);
  }
}
