#include <Servo.h>

Servo myServo;

int potPin = A0;
int servoPin = 9;  
int normalLedPin = 10;
int abnormalLedPin = 11;

int prevPotValue = 0;
int prevServoAngle = 0;

int potThreshold = 20;
int servoThreshold = 10;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  pinMode(normalLedPin, OUTPUT);
  pinMode(abnormalLedPin, OUTPUT);
}

void loop() {
  int potValue = analogRead(potPin);
  int angle = map(potValue, 0, 1023, 0, 180);

  myServo.write(angle);

  if (abs(angle - prevServoAngle) > servoThreshold || abs(potValue - prevPotValue) > potThreshold) {
    Serial.print("Abnormal Reading! ");
    Serial.print("Servo Angle: ");
    Serial.print(angle);
    Serial.print(", Potentiometer Value: ");
    Serial.println(potValue);

    digitalWrite(abnormalLedPin, HIGH);  // Turn on abnormal LED
    digitalWrite(normalLedPin, LOW);     // Turn off normal LED
  } else {
    Serial.print("Servo Angle: ");
    Serial.print(angle);
    Serial.print(", Potentiometer Value: ");
    Serial.println(potValue);

    digitalWrite(normalLedPin, HIGH);    // Turn on normal LED
    digitalWrite(abnormalLedPin, LOW);   // Turn off abnormal LED
  }

  prevServoAngle = angle;
  prevPotValue = potValue;

  delay(15); // Delay for servo stability
}
