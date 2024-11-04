#include <Servo.h>
#include <LiquidCrystal_I2C.h>

// Global Variables
int LDRValue = 0;
int sensorStatus = 0;

// objects
Servo servoMotor_up_down;            // Create a servo object
Servo servoMotor_right_left;         // Create a servo object
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x3F for a 16 chars and 2 line display


// pins
#define LDR_PIN 6
#define LASER_PIN 13
#define EMPTY_PUMP_RELAY 2
#define PROBE_PUMP_RELAY 3
// on off
#define OFF LOW
#define ON HIGH
#define OFF_PROBE_PUMP HIGH
#define ON_PROBE_PUMP LOW

// angles
#define up_angle 70
#define down_angle 5
#define water_tank 10
#define reagent_tank 65
#define sample_tank 150
// delays
#define up_down_delay 3000
#define right_left_delay 3000
#define probe_pump_delay 1000
#define empty_pump_delay 3000

// IR sensor
#define homeSensor 9

// Water level sensor
#define waterLevelPower_sample 7
#define waterLevelPinReading_sample A0

#define waterLevelPower_mixer 8
#define waterLevelPinReading_mixer A1


// water level Value for storing water level
int waterLevelval_sample = 0;
int waterLevelval_mixer = 0;


void empty(void);
void servoMotor(int tank);
void concentration_calculation(void);
int readWaterSensor_sample();
int readWaterSensor_mixer();


void setup(void) {
  // Servo ------------
  servoMotor_up_down.attach(8);     // Attach the servo to pin 8
  servoMotor_right_left.attach(5);  // Attach the servo to pin 5

  servoMotor_up_down.write(up_angle);
  servoMotor_right_left.write(water_tank);

  // Laser --------
  pinMode(LASER_PIN, OUTPUT);  // Define the digital output interface pin 13

  // IR sensor (home sensor) ---------
  pinMode(homeSensor, INPUT);

  // Water level sensor
  pinMode(waterLevelPower_sample, OUTPUT);
	pinMode(waterLevelPower_mixer, OUTPUT);
	digitalWrite(waterLevelPower_sample, LOW);
	digitalWrite(waterLevelPower_mixer, LOW);

  //   ------ LCD INITIALIZATION
  lcd.init();
  lcd.clear();
  lcd.backlight();  // Make sure backlight is on

  // Print a message on both lines of the LCD.
  lcd.setCursor(2, 0);  // Set cursor to character 2 on line 0
  lcd.print("Welcome");

  lcd.setCursor(2, 1);  // Move cursor to character 2 on line 1
  lcd.print("ZOMA LAB");

  //    ----------------> Relay
  pinMode(EMPTY_PUMP_RELAY, OUTPUT);
  pinMode(PROBE_PUMP_RELAY, OUTPUT);
  digitalWrite(EMPTY_PUMP_RELAY, OFF);
  digitalWrite(PROBE_PUMP_RELAY, OFF_PROBE_PUMP);

  Serial.begin(9600);
}

void loop(void) {
// IR sensor "Home sensor" code for detect the distance of the probe to start the program
  /*
  if (sensorStatus == 1) // Check if the pin high or not
      {
         Serial.println("Motion Ended!"); // print Motion Detected! on the serial monitor window
     }
     else
       {
           Serial.println("Motion Detected!"); // print Motion Ended! on the serial monitor window
        }
  */




  // Water level sensor reading
  /*
  int water_level_sample = readWaterSensor_sample();
	int water_level_mixer = readWaterSensor_mixer();
	
	Serial.print("Water level of sample: ");
	Serial.println(water_level_sample);
  
  Serial.print("Water level of mixer: ");
	Serial.println(water_level_mixer);

	delay(1000);
  */



  /* Servo Motor*/
  // servoMotor(sample_tank);  //move to sample

  // servoMotor(reagent_tank);  //move to reagent

  // concentration_calculation();

  empty();  // empty the mixer

  // servoMotor(water_tank);  //move to water

  empty();  // empty the mixer/

  // lcd.clear();  // clear display



  digitalWrite(PROBE_PUMP_RELAY, ON_PROBE_PUMP);  // start the probe pump
  Serial.println("probe pump on");
  delay(probe_pump_delay);  // wait

  digitalWrite(PROBE_PUMP_RELAY, OFF_PROBE_PUMP);  // stop the probe pump
  Serial.println("probe pump off");
  delay(probe_pump_delay);  // wait

  LDRValue = analogRead(LDR_PIN);  //get the LDR value
  Serial.println(LDRValue);

}

void concentration_calculation(void) {
  digitalWrite(LASER_PIN, ON);  // Turn on the laser head
  delay(1000);                  // wait for one second

  LDRValue = analogRead(LDR_PIN);  //get the LDR value
  Serial.println(LDRValue);
  delay(2);

  lcd.clear();          // clear display
  lcd.setCursor(0, 0);  // move cursor to   (0, 0)
  lcd.print("concentration:");
  lcd.setCursor(0, 1);  // move cursor to   (2, 1)
  lcd.print(LDRValue);  // print message at (2, 1)
  delay(2000);          // display the above for two seconds

  digitalWrite(LASER_PIN, LOW);  // Turn off the laser head
}
void servoMotor(int tank) {

  servoMotor_right_left.write(tank);  // move the probe to the passing tank
  Serial.print("Servo Motor right / left = ");
  Serial.println(tank);

  delay(right_left_delay);  // wait

  servoMotor_up_down.write(down_angle);  // move the probe down
  Serial.print("Servo Motor up / down = ");
  Serial.println(down_angle);
  delay(up_down_delay);  // Wait

  digitalWrite(PROBE_PUMP_RELAY, ON_PROBE_PUMP);  // start the probe pump
  Serial.println("probe pump on");
  delay(probe_pump_delay);  // wait

  digitalWrite(PROBE_PUMP_RELAY, OFF_PROBE_PUMP);  // stop the probe pump
  Serial.println("probe pump off");
  delay(probe_pump_delay);  // wait

  servoMotor_up_down.write(up_angle);  // move the probe up
  Serial.print("Servo Motor up / down = ");
  Serial.println(up_angle);
  delay(up_down_delay);  // wait
}
void empty(void) {

  digitalWrite(EMPTY_PUMP_RELAY, ON);  // start the washing pump
  Serial.println("empty pump on");

  delay(empty_pump_delay);  // wait

  digitalWrite(EMPTY_PUMP_RELAY, LOW);  // stop the washing pump
  Serial.println("empty pump off");

  delay(empty_pump_delay);  // wait
}


//This is a function used to get the reading
int readWaterSensor_sample() {
	digitalWrite(waterLevelPower_sample, HIGH);	// Turn the sensor ON
	delay(50);							// wait 10 milliseconds
	waterLevelval_sample = analogRead(waterLevelPinReading_sample);		// Read the analog value form sensor
	digitalWrite(waterLevelPower_sample, LOW);		// Turn the sensor OFF
	return waterLevelval_sample;							// send current reading
}


int readWaterSensor_mixer() {
	digitalWrite(waterLevelPower_mixer, HIGH);	// Turn the sensor ON
	delay(50);							// wait 10 milliseconds
	waterLevelval_mixer = analogRead(waterLevelPinReading_mixer);		// Read the analog value form sensor
	digitalWrite(waterLevelPower_mixer, LOW);		// Turn the sensor OFF
	return waterLevelval_mixer;							// send current reading
}