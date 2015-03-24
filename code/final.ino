#include <Servo.h>
// -------------------- MOTORS --------------------
// 1500ms = Neutral
// 2500ms = Fast forward
// 500ms = Fast Reverse
// 200ms = Brake
Servo servo_RightMotor;
Servo servo_LeftMotor;


unsigned int Left_Motor_Speed;
unsigned int Right_Motor_Speed;
unsigned int Right_Motor_Stop = 1500;
unsigned int Left_Motor_Stop = 1500;
const int right_motor_address = 2;
const int left_motor_address = 3;
const int ultraSonic_In_address = 52;
const int ultraSonic_Out_address = 53;

// -------------------- VARIABLES --------------------

unsigned long Echo_Time;

// -------------------- STEP COUNTER --------------------
// NOTE: Step 0 is reserved for debugging

unsigned int step = 0;




void setup() {

Serial.begin(9600);
//Set up motors
pinMode(left_motor_address, OUTPUT);
servo_LeftMotor.attach(left_motor_address);
pinMode(right_motor_address, OUTPUT);
servo_RightMotor.attach(right_motor_address);

// set up ultrasonic
pinMode(ultraSonic_Out_address, INPUT);
pinMode(ultraSonic_In_address, OUTPUT);
}

void loop(){

  switch (step){

    case 0:
      // -------- TEST CODE HERE --------

    case 1:
      // Start phase: Robot is in the middle of the room
      // End phase: Robot  is parallel to table's edge

    case 2:
      servo_LeftMotor.writeMicroseconds(1800);
      servo_RightMotor.writeMicroseconds(1900);
      ping();

    case 3:

    case 4:

    case 5:

    case 6:

    case 7:

  }

}

// -------------------- SENSOR FUNCTIONS --------------------

// Ping Ultrasonic
// Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
void ping(){
digitalWrite(ultraSonic_In_address, HIGH);
delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
digitalWrite(ultraSonic_In_address, LOW);

// Use command pulseIn to listen to Ultrasonic_Data pin to record the
// time that it takes from when the Pin goes HIGH until it goes LOW
Echo_Time = pulseIn(ultraSonic_Out_address, HIGH, 10000);

// Print Sensor Readings
Serial.print("Time (microseconds): ");
Serial.print(Echo_Time, DEC);
Serial.print(", cm: ");
Serial.println(Echo_Time / 58); //divide time by 58 to get distance in cm
}

// -------------------- MOVEMENT FUNCTIONS --------------------

void pivotCounterClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_LeftMotor.writeMicroseconds(1250);
  servo_RightMotor.writeMicroseconds(1750);
  delay(1500);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);

}

void pivotClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_RightMotor.writeMicroseconds(1250);
  servo_LeftMotor.writeMicroseconds(1750);
  delay(1500);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
}

void pivotCounterClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_LeftMotor.writeMicroseconds(1250);
  servo_RightMotor.writeMicroseconds(1750);
  delay(1500);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);

}

void pivotClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_RightMotor.writeMicroseconds(1250);
  servo_LeftMotor.writeMicroseconds(1750);
  delay(1500);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
}