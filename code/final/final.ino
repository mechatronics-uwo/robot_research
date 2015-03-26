#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>

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

Servo servo_TopMotor;
Servo servo_RightMotor;
Servo servo_LeftMotor;


I2CEncoder encoder_TopMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_RightMotor;


// -------------------- SENSORS --------------------
const int TOP_MOTOR_PIN = 4;
const int RIGHT_MOTOR_PIN = 2;
const int LEFT_MOTOR_PIN = 3;

const int FRONT_BOTTOM_LEVER_SWITCH_PIN = 4;
const int FRONT_TOP_LEVER_SWITCH_PIN = 5;

const int ULTRASONIC_IN_PIN_FRONT = 52;
const int ULTRASONIC_OUT_PIN_FRONT = 53;

const int ULTRASONIC_IN_PIN_BACK = 48;
const int ULTRASONIC_OUT_PIN_BACK = 49;

// -------------------- VARIABLES --------------------

unsigned long echo_time[2];//0 is front timer, 1 is back timer

unsigned int Left_Motor_Speed;
unsigned int Right_Motor_Speed;
unsigned int Right_Motor_Stop = 1500;
unsigned int Left_Motor_Stop = 1500;
long leftEncoderStopTime = 0;
long rightEncoderStopTime = 0;
boolean val = false;
int Left_Motor_Offset = 0;
int Right_Motor_Offset = 0;

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

  Wire.begin();
  Serial.begin(9600);

  // Set-up motors
  pinMode(TOP_MOTOR_PIN)
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  servo_LeftMotor.attach(LEFT_MOTOR_PIN);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  servo_RightMotor.attach(RIGHT_MOTOR_PIN);


  // Set-up buttons
  pinMode(FRONT_TOP_LEVER_SWITCH_PIN, INPUT);
  pinMode(FRONT_BOTTOM_LEVER_SWITCH_PIN, INPUT);
  
  digitalWrite(FRONT_TOP_LEVER_SWITCH_PIN, HIGH);
  digitalWrite(FRONT_BOTTOM_LEVER_SWITCH_PIN, HIGH);

  // Set-up ultrasonic
  pinMode(ULTRASONIC_OUT_PIN_FRONT, INPUT);
  pinMode(ULTRASONIC_IN_PIN_FRONT, OUTPUT);
  
  pinMode(ULTRASONIC_OUT_PIN_BACK, INPUT);
  pinMode(ULTRASONIC_IN_PIN_BACK, OUTPUT);



  //Set up encoder
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false); // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true); // adjust for positive count when moving forward
  
  encoder_LeftMotor.zero();
  encoder_RightMotor.zero();
  encoder_TopMotor.zero();
}

// *****************************************************************
// ************************* PROGRAM !LOOP *************************
// *****************************************************************
void loop() {

  switch (step) {

    // ==================== CASE 1-10 ====================


    case 0:
      // RESERVED FOR TESTING, PASTE CODE HERE AND SET STEP = 0
      moveBackDistance(300);
      break;

    case 1:
      // Find the wall first
      moveForward(100);
      if (hitTable) {
        setNeutral();
        step = 3;
      }
      else if (hitWall) {
        setNeutral();
        step = 2;
      }
      break;

    case 2:
      // Start case: Wall is in front of robot, robot is parallel to it
      moveForward(300);
      if (hitWall()) {
        step++;
      }
      // End case: Robot is hit the wall
      break;
    case 3:

    case 4:

    case 5:

    case 6:

    case 7:

    case 8:

    case 9:

    case 10:

    // ==================== CASE 11-20 ====================

    case 11:/*
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
    */
    case 12:
      servo_LeftMotor.writeMicroseconds(1800);
      servo_RightMotor.writeMicroseconds(1900);
      ping();

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
      break;
  }

}


// --------------------------------------------------------------
// ------------------------- !FUNCTIONS -------------------------
// --------------------------------------------------------------



// -------------------- SENSOR FUNCTIONS --------------------

// Ping ultrasonic
// Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec

int ping() {
}

int frontPing() {
  //Front ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, LOW);

  unsigned long ping_time = pulseIn(ULTRASONIC_OUT_PIN_FRONT, HIGH, 10000);

  Serial.print("cm: ");
  Serial.print(ping_time / 58); //divide time by 58 to get

  return ping_time;
}

int backPing() {
  //Back ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_BACK, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ULTRASONIC_IN_PIN_BACK, LOW);

  // Use command pulseIn to listen to ultrasonic_Data pin to record the
  // time that it takes from when the Pin goes HIGH until it goes LOW
  echo_time[1] = pulseIn(ULTRASONIC_OUT_PIN_BACK, HIGH, 10000);

  // Print Sensor Readings
  //Serial.print("Time (microseconds): ");
  //Serial.print(echo_time[0], DEC);
  Serial.print("cm: ");
  Serial.print(echo_time[0] / 58); //divide time by 58 to get distance in cm
  Serial.print("cm: ");
  Serial.println(echo_time[1] / 58); //divide time by 58 to get distance in cm
  }
}

void getEncoderPos()
{                       
                        Serial.print("Rot: ");
			Serial.println(encoder_TopMotor.getRawPosition());
			Serial.print("Encoders L: ");
			Serial.print(encoder_LeftMotor.getRawPosition());
			Serial.print(", R: ");
			Serial.print(encoder_RightMotor.getRawPosition());
			
  
}

boolean hitTable() {

  if (FRONT_BOTTOM_LEVER_SWITCH_PIN == LOW && FRONT_TOP_LEVER_SWITCH_PIN == HIGH)
    return true;
  else
    return false;
}

boolean hitWall() {
  if (FRONT_BOTTOM_LEVER_SWITCH_PIN == LOW && FRONT_TOP_LEVER_SWITCH_PIN == LOW)
    return true;
    
  else
    return false;  
}


// -------------------- MOVEMENT FUNCTIONS --------------------

void allOfTin()
{
  currentReading = frontPing(); // update reading
  integral += currentReading; // add reading to integral

  output = pConstant * (currentReading - setPoint) + iConstant * integral + dConstant * (currentReading - lastReading);
  lastReading = currentReading; // update last reading

  // veer based on output
  if (output > 0)
    veerRight(200, abs(output));
  else if (output < 0)
    veerLeft(200, abs(output));
}


}
void moveForward(long speedFactor)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor), 1500, 2100);
  implementMotorSpeed();
}

void moveBackwards(long speedFactor)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop - speedFactor), 900, 1500);
  Right_Motor_Speed = constrain((Right_Motor_Stop - speedFactor), 900, 1500);
  implementMotorSpeed();
}
void moveBackDistance(long distance)
{
  leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
  while (leftEncoderStopTime + distance > encoder_LeftMotor.getRawPosition())
  {
    Left_Motor_Speed = constrain((Left_Motor_Stop - 300), 1500, 2100);
    Right_Motor_Speed = constrain((Right_Motor_Stop - 300), 1500, 2100);
    implementMotorSpeed();
  }

}


void backUp() {
  moveBackDistance(500);
}

void veerRight(long speedFactor, long intensity)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor + intensity), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor), 1500, 2100);
  implementMotorSpeed();
}

void veerLeft(long speedFactor, long intensity)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor + intensity), 1500, 2100);
  implementMotorSpeed();
}

void turnLeftOnSpot(long speedFactor)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop - speedFactor), 900, 1500);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor), 1500, 2100);
  implementMotorSpeed();
}
void turnRightOnSpot(long speedFactor)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop - speedFactor), 900, 1500);
  implementMotorSpeed();
}
void calcLeftTurn(long fullCircle, int angle) // full circle should be around 2800
{
  rightEncoderStopTime = encoder_RightMotor.getRawPosition();
  rightEncoderStopTime += (fullCircle * angle) / 360;
}
void calcRightTurn(long fullCircle, int angle)
{
  leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
  leftEncoderStopTime += (fullCircle * angle) / 360;
}

boolean doneLeftTurn()
{
  if (encoder_RightMotor.getRawPosition() > rightEncoderStopTime)
    return true;
  else
    return false;
}
boolean doneRightTurn()
{
  if (encoder_LeftMotor.getRawPosition() > leftEncoderStopTime)
    return true;
  else
    return false;
}
void implementMotorSpeed()
{	
  servo_LeftMotor.writeMicroseconds(constrain((Left_Motor_Speed + Left_Motor_Offset), 900, 2100));
  servo_RightMotor.writeMicroseconds(constrain((Right_Motor_Speed + Right_Motor_Offset), 900, 2100));
}

void setNeutral() {
  Left_Motor_Speed = 1500;
  Right_Motor_Speed = 1500;
  implementMotorSpeed();

}

void brake() {
  Left_Motor_Speed = 200;
  Right_Motor_Speed = 200;
  implementMotorSpeed();
}

// Pivoting will turn the robot 90 degrees without moving

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

void turnClockwise(long speedFactor) {
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = Left_Motor_Stop;
  implementMotorSpeed();
}


// -------------------- TIME FUNCTIONS --------------------


// Call startWaiting first, and then waitMilliSecond
void startWaiting() {
  if (can_start_waiting) {

    time_previous = millis();
  }
}

boolean waitMilliSecond(unsigned int interval) {

  time_elapsed = millis();
  if ((time_elapsed - time_previous) > interval) {
    can_start_waiting = false;
    return true; // Done waiting!
  }
  else {
    return false; // Not done waiting!
  }
}

