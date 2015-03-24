#include <Servo.h>
// Authors: Danny, Tin, Daniel
// Prepared for: MSE 2202B, Dr. Naish
// School: The University of Western Ontario

// --------------------------------------------------------------
// ------------------------- !VARIABLES -------------------------
// --------------------------------------------------------------


// -------------------- MOTORS --------------------

// 1500ms = Neutral
// 2500ms = Fast forward
// 500ms = Fast Reverse
// 200ms = Brake

Servo servo_RightMotor;
Servo servo_LeftMotor;
unsigned int left_motor_speed;
unsigned int right_motor_speed;
unsigned int right_motor_stop = 1500;
unsigned int left_motor_stop = 1500;
const int RIGHT_MOTOR_PIN = 2;
const int LEFT_MOTOR_PIN = 3;

// -------------------- SENSORS --------------------
const int FRONT_BOTTOM_LEVER_SWITCH_PIN = 4;
const int FRONT_TOP_LEVER_SWITCH_PIN = 5;

const int ULTRASONIC_IN_PIN = 52;
const int ULTRASONIC_OUT_PIN = 53;

// -------------------- VARIABLES --------------------

unsigned long echo_time;

unsigned long time_previous = 0; // Used for time functions, do not change
unsigned long time_elapsed = 0; // Used for time functions, do not change
boolean can_start_waiting = false; // Used for time functions, do not change

// -------------------- STEP COUNTER --------------------
// NOTE: Step 0 is reserved for debugging

unsigned int step = 0;


// ******************************************************************
// ************************* PROGRAM !SETUP *************************
// ******************************************************************
void setup() {

Serial.begin(9600);
// Set-up motors
pinMode(LEFT_MOTOR_PIN, OUTPUT);
servo_LeftMotor.attach(LEFT_MOTOR_PIN);
pinMode(RIGHT_MOTOR_PIN, OUTPUT);
servo_RightMotor.attach(RIGHT_MOTOR_PIN);

// Set-up buttons
pinMode(FRONT_TOP_LEVER_SWITCH_PIN, INPUT);
pinMode(FRONT_BOTTOM_LEVER_SWITCH_PIN, INPUT);

// Set-up ultrasonic
pinMode(ULTRASONIC_OUT_PIN, INPUT);
pinMode(ULTRASONIC_IN_PIN, OUTPUT);
}

// *****************************************************************
// ************************* PROGRAM !LOOP *************************
// *****************************************************************
void loop(){

  switch (step){

    // ==================== CASE 1-10 ====================

    case 0:
      // -------- THIS CASE RESERVED FOR TESTING --------
      // -------- PASTE TEST CODE HERE, SET STEP = 0 --------

    case 1:
      // Start case: Robot is in the middle of the room
      moveForward();
      if (hitWall()){
        backUp();
        pivotCounterClockwise();
      }
      // End case: Robot is parallel to the wall

    case 2:
      // Start case: Robot is parallel to the wall
      smartMoveForward();
      if (hitTable()){
        backUp();
        pivotCounterClockwise();
        step = 5; // !NOTICE: fix this, Jump to the case where you're at the table
      }
      else if(hitWall()){
        backUp();
        pivotCounterClockwise();
      }

      // End case: In front of the table

    case 3:
      servo_LeftMotor.writeMicroseconds(1800);
      servo_RightMotor.writeMicroseconds(1900);
      ping();

    case 4:

    case 5:

    case 6:

    case 7:

    case 8:

    case 9:

    case 10:

  // ==================== CASE 11-20 ====================

    case 11:

    case 12:

    case 13:

    case 14:

    case 15:

    case 16:

    case 17:

    case 18:

    case 19:

    case 20:

    // ==================== CASE 21-30 ====================

    case 21:

    case 22:

    case 23:

    case 24:

    case 25:

    case 26:

    case 27:

    case 28:

    case 29:

    case 30:

    // ==================== CASE 31-40 ====================

    case 31:

    case 32:

    case 33:

    case 34:

    case 35:

    case 36:

    case 37:

    case 38:

    case 39:

    case 40:

  }

}


// --------------------------------------------------------------
// ------------------------- !FUNCTIONS -------------------------
// --------------------------------------------------------------



// -------------------- SENSOR FUNCTIONS --------------------

// Ping ultrasonic
// Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
void ping(){
digitalWrite(ULTRASONIC_IN_PIN, HIGH);
delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
digitalWrite(ULTRASONIC_IN_PIN, LOW);

// Use command pulseIn to listen to ultrasonic_Data pin to record the
// time that it takes from when the Pin goes HIGH until it goes LOW
echo_time = pulseIn(ULTRASONIC_OUT_PIN, HIGH, 10000);

// Print Sensor Readings
Serial.print("Time (microseconds): ");
Serial.print(echo_time, DEC);
Serial.print(", cm: ");
Serial.println(echo_time / 58); //divide time by 58 to get distance in cm
}

boolean readTopFrontButton() {
  val = digitalRead(FRONT_TOP_LEVER_SWITCH_PIN);
  if (val == HIGH) {
    return true
  }
  else {
    return false
  }
}

boolean readBottomFrontButton() {
  val = digitalRead(FRONT_BOTTOM_LEVER_SWITCH_PIN);
  if (val == HIGH) {
    return true
  }
  else {
    return false
  }
}

boolean hitTable() {
  return (readBottomFrontButton && !readTopFrontButton)
}

boolean hitWall() {
  return (readBottomFrontButton && readTopFrontButton)
}


// -------------------- MOVEMENT FUNCTIONS --------------------

void smartMoveForward() {
  // Detect if you move too far away from the wall
}

void moveForward() {
  // !NOTICE motor speed set slow for debugging purposes
  servo_LeftMotor.writeMicroseconds(1700);
  servo_RightMotor.writeMicroseconds(1850);
}

void moveBackwards(){
  servo_LeftMotor.writeMicroseconds(1300);
  servo_RightMotor.writeMicroseconds(1300);
}

void backUp(){
  servo_LeftMotor.writeMicroseconds(1300);
  servo_RightMotor.writeMicroseconds(1300);
  delay(500);
  setNeutral();
  delay(100);
}

void setNeutral(){
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
}

void brake(){
  servo_LeftMotor.writeMicroseconds(200);
  servo_RightMotor.writeMicroseconds(200);
}

// Pivoting will turn the robot 90 degrees without moving
void pivotCounterClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_LeftMotor.writeMicroseconds(1250);
  servo_RightMotor.writeMicroseconds(1750);
  delay(1500);
  setNeutral();
  delay(200);
}

void pivotClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_RightMotor.writeMicroseconds(1250);
  servo_LeftMotor.writeMicroseconds(1750);
  delay(1500);
  setNeutral();
  delay(200);
}

// Turning will turn the robot 90 degrees with slight movement
// !NOTICE Need to test and fix these functions
void turnCounterClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_LeftMotor.writeMicroseconds(1250);
  servo_RightMotor.writeMicroseconds(1750);
  delay(1500);
  setNeutral();
  delay(200);

}

void turnClockwise() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  delay(200);
  servo_RightMotor.writeMicroseconds(1250);
  servo_LeftMotor.writeMicroseconds(1750);
  delay(1500);
  setNeutral();
  delay(200);
}


// -------------------- TIME FUNCTIONS --------------------

void startWaiting(){
  if(can_start_waiting){
    time_previous = millis();
  }
}

boolean waitMillisSecond(unsigned int interval){
  time_elapsed = millis();
  if ((time_elapsed - time_previous) > interval){
    can_start_waiting = false;
    return true; // Done waiting!
  }
  else {
    return false; // Not done waiting!
  }
}
