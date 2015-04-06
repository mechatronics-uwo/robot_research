#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>

// MSE 2202B Mechatronics Final Project
// Authors: Danny, Tin, Daniel
// Prepared for: Dr. Naish
// School: The University of Western Ontario
// Date: April 8th 2015

/*

INSTRUCTIONS

1. Place robot in the middle of the room, ideally with the robot's right side parallel to a nearby wall
2. Turn it on

*/

// --------------------------------------------------------------
// ------------------------- !VARIABLES -------------------------
// --------------------------------------------------------------


// -------------------- MOTORS --------------------

// 1500ms = Neutral
// 2500ms = Fast forward
// 500ms = Fast Reverse
// 200ms = Brake

Servo servo_RightMotor;
Servo servo_RotMotor;// Above 1500 is clockwise
Servo servo_LeftMotor;
Servo servo_ExtendMotor;// Above 1500 is retract
Servo servo_VerticleMotor;// Above 1500 is up
Servo servo_ClawMotor;// Above 1500 is open

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_RotMotor;
I2CEncoder encoder_LeftMotor;


// -------------------- SENSORS --------------------

const int LEFT_MOTOR_PIN = 3;
const int RIGHT_MOTOR_PIN = 2;
const int ROT_MOTOR_PIN = 6;
const int EXTEND_MOTOR_PIN = 7;
const int VERTICLE_MOTOR_PIN = 8;
const int CLAW_MOTOR_PIN = 9;

const int FRONT_BOTTOM_LEVER_SWITCH_PIN = 4;
const int FRONT_TOP_LEVER_SWITCH_PIN = 5;

const int TOP_BACK_LEVER_SWITCH_PIN = 10;
const int TOP_FRONT_LEVER_SWITCH_PIN = 11;

const int ARM_SWITCH_PIN = 12;
const int LOWER_ARM_SWITCH_PIN = 13;

const int ULTRASONIC_IN_PIN_FRONT = 52;
const int ULTRASONIC_OUT_PIN_FRONT = 53;

const int ULTRASONIC_IN_PIN_BACK = 50;
const int ULTRASONIC_OUT_PIN_BACK = 51;

const int ULTRASONIC_IN_PIN_ARM = 49;
const int ULTRASONIC_OUT_PIN_ARM = 48;

const int RIGHT_LIGHT_SENSOR = A0;
const int RIGHT_BOTTOM_LIGHT_SENSOR = A1;


// -------------------- VARIABLES --------------------
int light_value = 0;
int next_light_value = 0;

int count = 0;//counts the lights

// Motor variables
unsigned int Left_Motor_Speed;
unsigned int Right_Motor_Speed;
unsigned int Verticle_Motor_Speed;

unsigned int Right_Motor_Stop = 1500;
unsigned int Left_Motor_Stop = 1500;

long leftEncoderStopTime = 0;
long rightEncoderStopTime = 0;
long rotEncoderStopTime=0;

boolean loopStarted = false;// Start loop for turning
int Left_Motor_Offset = 0;
int Right_Motor_Offset = 30;

// Used for time functions, do not change
unsigned long time_previous = 0;
unsigned long time_elapsed = 0;
boolean can_start_waiting = false; // Time function flag


// -------------------- STAGE COUNTER --------------------
// NOTE: Stage 0 is reserved for debugging

unsigned int stage = 0;

// ******************************************************************
// ************************* PROGRAM !SETUP *************************
// ******************************************************************
void setup(){

  Wire.begin();
  Serial.begin(9600); // Start serial communication for debugging

  // Set-up motors
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  servo_LeftMotor.attach(LEFT_MOTOR_PIN);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  servo_RightMotor.attach(RIGHT_MOTOR_PIN);
  pinMode(ROT_MOTOR_PIN, OUTPUT);
  servo_RotMotor.attach(ROT_MOTOR_PIN);
  pinMode(EXTEND_MOTOR_PIN, OUTPUT);
  servo_ExtendMotor.attach(EXTEND_MOTOR_PIN);
  pinMode(VERTICLE_MOTOR_PIN, OUTPUT);
  servo_VerticleMotor.attach(VERTICLE_MOTOR_PIN);
  pinMode(CLAW_MOTOR_PIN, OUTPUT);
  servo_ClawMotor.attach(CLAW_MOTOR_PIN);

  // Set-up buttons
  pinMode(FRONT_TOP_LEVER_SWITCH_PIN, INPUT_PULLUP);
  pinMode(FRONT_BOTTOM_LEVER_SWITCH_PIN, INPUT_PULLUP);

  pinMode(TOP_BACK_LEVER_SWITCH_PIN, INPUT_PULLUP);
  pinMode(TOP_FRONT_LEVER_SWITCH_PIN, INPUT_PULLUP);

  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LOWER_ARM_SWITCH_PIN, INPUT_PULLUP);

  digitalWrite(FRONT_TOP_LEVER_SWITCH_PIN, HIGH);
  digitalWrite(FRONT_BOTTOM_LEVER_SWITCH_PIN, HIGH);

  // Set-up ultrasonic
  pinMode(ULTRASONIC_OUT_PIN_FRONT, INPUT);
  pinMode(ULTRASONIC_IN_PIN_FRONT, OUTPUT);

  pinMode(ULTRASONIC_OUT_PIN_BACK, INPUT);
  pinMode(ULTRASONIC_IN_PIN_BACK, OUTPUT);

  pinMode(ULTRASONIC_OUT_PIN_ARM, INPUT);
  pinMode(ULTRASONIC_IN_PIN_ARM, OUTPUT);


  //Set up encoders
  encoder_RotMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false); // adjust for positive count when moving forward

  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true); // adjust for positive count when moving forward

  encoder_RotMotor.zero();
  encoder_LeftMotor.zero();
  encoder_RightMotor.zero();
}

// *****************************************************************
// ************************* PROGRAM !LOOP *************************
// *****************************************************************
void loop() {

  switch (stage) {

    // ==================== STAGE 1-10 ====================
    case 0:
      // RESERVED FOR TESTING, PASTE CODE HERE AND SET STAGE = 0
      // stage = 1 // UNCOMMENT FOR PRODUCTION

      /*
      TESTING BACKLOG:
      */
//      
      detachArmMotors();
      delay(250);

      
      stage = 1;

    break;

    case 1:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is in the middle of the room, unaligned
      pivotAlign();
      delay(500);
      stage = 2;

      break;
      // End stage: robot is in the middle of the room, parallel to the wall or table

    case 2:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is in the middle of the room, parallel to the wall or table
      moveForward(200);
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 3;
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 4;
      }

      break;
      // End stage: Robot is parallel to the wall

    case 3:
      // Stage status: COMPLETE by Daniel
      // Start stage: Wall is in front of robot, robot is parallel to it
      smartMoveForwards();
      if (hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 4;
      }

      break;
      // End stage: Table is in front of robot, robot is parallel to it

    case 4:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is parallel to table, need to align itself to the table
      setNeutral();
      pivotAlign();
      setNeutral();
      stage = 5;

      break;
      // Robot is parallel to the table

    case 5:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is parallel to the table
      backUp();
      setNeutral();
      stage=6;

      break;
      // End stage: Robot is parallel to the table, all the way at the end

    case 6:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is either on the short or long side of the table
      if(detectLongSide()){
        setNeutral();
        stage = 8;
      }
      else{
        setNeutral();
        stage = 7;
      }

      break;
      // End stage: Robot is on the long end of the table, preparing to align itself and find the light

    case 7:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot has escaped the short edge of the table
      moveForwardDistance(300);
      delay(500);
      turnRightAngle(100);
      delay(500);
      stage = 8;

      break;
      // End stage: Robot is preparing to find the light

    case 8:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is preparing to find the light on the table
      while(!detectLight()){
        moveForward(130);
      }
      setNeutral();
      delay(1000);
      moveForwardDistance(300);
      delay(1000);
      setNeutral();
      stage = 9;

      break;
      // Robot is directly in front of the light of the long edge of the table, but needs to align itself at the correct distance

    case 9:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is by the long edge of the table, needs to align itself at the correct distance
      parallelPark();
      setNeutral();
      delay(1000);
      stage = 10;

      break;
      // End stage: Robot is now at the appropriate distane away from the table, needs to do a final parallel align check

    case 10:
      // Stage status: COMPLETE by Daniel
      // Start stage: Robot is now at the appropriate distane away from the table, needs to do a final parallel align check
      pivotAlign();
      delay(500);
      stage = 11;

      break;
      // End stage: Robot is now the correct distance away from the table, but needs to move to the edge

    // ==================== STAGE 11-20 ====================
    //Finding bottle

    case 11:
    // Stage status: COMPLETE by Daniel
    // Start stage: Robot is now the correct distance away from the table, but needs to move to the edge
      while(!detectLight()){
        moveBackwards(150);
      }
      setNeutral();

      stage = 12;

      break;
    // End stage: Robot is now parallel with the table, at the correct distance, and at the edge of the table.

    case 12:
      delay(500);
      pivotAlign();
      delay(500);
      setNeutral();
      attachArmMotors();
      delay(250);

      stage = 13;

      break;

    case 13:
    // Stage status: IN PROGRESS by Danny
    // Start stage: Robot is in the correct position with respect to the table. Robot needs to raise its arm
      raiseArm();
      delay(12000);
      stopArm();
      delay(1500);
      stage = 14;

      break;
    // End stage: Robot has its arm raised

    case 14:
      rotatePerpendicular();
      delay(3000);
      stage = 15;

      break;

    case 15:
      if(findBottleDanny()){
        stage = 16;
      }

      break;

    case 16:
//      zoomIntoBottle();
//      delay(1000);
      if (armPing() > 260){
        extendArm();
        openClaw();
      }
      else {
        stopHorizontalArm();
        delay(50);
        stopArm();
        delay(50);
        closeClaw();
        delay(3000);
        stopClaw();
        stage = 17;
      }
      break;


    case 17:
      pivotAlign();
      stage = 18;
      break;

    case 18:
      while(!hitTopFront()){
        retractArm();
      }
      stopHorizontalArm();
      delay(500);
      stage = 19;
      break;

    case 19:
      lowerArm();
      delay(1300);
      stopArm();
      delay(100);
      rotateParallel();
      delay(500);
      stopRotation();
      stage = 20;
      break;

    case 20:
      lowerArm();
      delay(6000);
      stopArm();
      detachArmMotors();
      delay(250);
      stage = 21;

      break;


    // ==================== STAGE 21-30 ====================

    case 21:
      smartMoveForwards();
      if (hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        count = 1;
      }
      else if(!detectObjectRight()){
        setNeutral();
        delay(1000);
        moveForwardDistance(300);
        delay(500);
        turnRightAngle(100);
        delay(1000);
        moveForwardDistance(800); // Set as 800 to clear table
      }
      else if((detectLight()) && (count == 1))
      {
        setNeutral();
        pivotAlign();
        setNeutral();
        moveForwardDistance(1000);
        turnRightAngle(90);
        moveBackDistance(500);
        stage = 22;
      }
      break;

    case 22:
      count = 0;
      moveForward(400);
      delay(6000);
      stage = 23;
      // if(hitWall()){
      //  setNeutral();
      //  moveBackDistance(300);
      //  turnLeftAngle(87);
      // }
      break;

    case 23:
      setNeutral();
      backUp();
      turnRightAngle(130);
      delay(1000);
      setNeutral();
      stage = 24;
      break;

    case 24:
      pivotAlign();
      delay(1000);
      stage = 25;
      break;

// *****************************************************************
// ************************** END OF EPIC **************************
// *****************************************************************

    case 25:
      moveForward(200);
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 26;
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 27;
      }
      break;

    case 26:
      smartMoveForwards();
      if (hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 27;
      }
      break;

    case 27:
      setNeutral();
      pivotAlign();
      setNeutral();
      delay(1000);
      stage = 28;
      break;

    case 28:
      backUp();
      setNeutral();
      stage = 29;
      break;

    case 29:
      // Need to test detectBottomLongSide
      if(detectBottomLongSide()){
        setNeutral();
        stage = 31;
      }
      else{
        setNeutral();
        stage = 30;
      }
      break;

    case 30:
      moveForwardDistance(300);
      delay(500);
      turnRightAngle(100);
      delay(500);
      stage = 31;

      break;


    // ==================== STAGE 31-40 ====================


    case 31:
      while(!detectBottomLight()){
        moveForward(130);
      }
      setNeutral();
      delay(1000);
      moveForwardDistance(300);
      delay(1000);
      setNeutral();
      stage = 32;

      break;

    case 32:
      parallelPark();
      setNeutral();
      delay(1000);
      stage = 33;

      break;

    case 33:
      pivotAlign();
      delay(500);
      stage = 34;

      break;

    case 34:
      while(!detectBottomLight()){
        moveBackwards(130);
      }
      setNeutral();

      stage = 35;
      break;

    case 35:
      delay(500);
      pivotAlign();
      delay(500);
      setNeutral();
      attachArmMotors();
      delay(250);

      stage = 36;
      break;

    case 36:
      while (!hitLowerArm()){
        lowerArm();
      }
      stopArm();
      delay(1000);
      raiseArm();
      delay(3500);
      stopArm();
      stage = 37;
      break;

    case 37:
      rotatePerpendicular();
      delay(4500);
      stopRotation();
      stage = 38;
      break;

    case 38:
      extendArm();
      delay(4000);
      stopHorizontalArm();
      stage = 40;
      break;

    case 39:
      stage = 40;
      break;

    case 40:
      openClaw();
      delay(3000);
      stopClaw();
      lowerArmUntilHit();
      raiseArm();
      delay(1500);
      stopArm();
      stage = 41;
      break;



    // ==================== STAGE 41-49 ====================

    case 41:
      while(!hitTopFront()){
        retractArm();
      }
      stopHorizontalArm();
      closeClaw();
      delay(3500);
      stopClaw();
      stage = 42;
      break;

    case 42:
      moveForwardDistance(700);
      delay(1000);
      stage = 43;
      break;

    case 43:
      if(findBottleDanny()){
        stage = 44;
      }
      break;

    case 44:
      if (armPing() > 260){
        extendArm();
        openClaw();
      }
      else {
        stopHorizontalArm();
        delay(1000);
        closeClaw();
        delay(3000);
        stopClaw();
        stage = 45;
      }
      break;

    case 45:
      pivotAlign();
      stage = 46;

      break;

    case 46:
      while(!hitTopFront()){
        retractArm();
      }
      stopHorizontalArm();
      delay(500);
      stage = 47;

      break;

    case 47:
      rotateParallel();
      delay(500);
      stopRotation();
      stage = 48;

      break;

    case 48:
      detachArmMotors();
      delay(250);
      stage = 49;

      break;

    case 49:
      smartMoveForwards();
      if (hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        count = 1;
      }
      else if(!detectObjectRight()){
        setNeutral();
        delay(1000);
        moveForwardDistance(300);
        delay(500);
        turnRightAngle(100);
        delay(1000);
        moveForwardDistance(800); // Set as 800 to clear table
      }
      else if((detectLight()) && (count == 1))
      {
        setNeutral();
        pivotAlign();
        setNeutral();
        moveForwardDistance(400);
        turnRightAngle(110);
        moveBackDistance(500);
        stage = 50;
      }
      break;

    case 50:
      count = 0;
      moveForward(400);
      delay(6000);
      stage = 51;
      break;


    // ==================== STAGE 51-60 ====================

    case 51:
      backUp();
      stage = 52;

      break;

    case 52:
      setNeutral();
      turnRightAngle(130);
      delay(1000);
      setNeutral();
      stage = 53;

      break;

    case 53:
      pivotAlign();
      delay(1000);
      stage = 54;
      break;

// *****************************************************************
// ************************** END OF EPIC **************************
// *****************************************************************

    case 54:
      moveForward(200);
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 56;
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 57;
      }
      break;

    case 55:
      stage = 56;
      break;

    case 56:
      smartMoveForwards();
      if (hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 57;
      }
      break;

    case 57:
      setNeutral();
      pivotAlign();
      setNeutral();
      delay(1000);
      stage = 58;
      break;

    case 58:
      backUp();
      setNeutral();
      stage = 59;
      break;

    case 59:
      if(detectLongSide()){
        setNeutral();
        stage = 61;
      }
      else{
        setNeutral();
        stage = 60;
      }

      break;

    case 60:
      moveForwardDistance(300);
      delay(500);
      turnRightAngle(100);
      delay(500);
      stage = 61;

      break;

    // ==================== STAGE 61-70 ====================

    case 61:
      while(!detectLight()){
        moveForward(130);
      }
      setNeutral();
      delay(1000);
      moveForwardDistance(300);
      delay(1000);
      setNeutral();
      stage = 62;

      break;

    case 62:
      parallelPark();
      setNeutral();
      delay(1000);
      stage = 63;

      break;

    case 63:
      pivotAlign();
      delay(500);
      stage = 64;

      break;

    case 64:
      while(!detectLight()){
        moveBackwards(150);
      }
      setNeutral();

      stage = 65;

      break;

    case 65:
      delay(500);
      pivotAlign();
      delay(500);
      setNeutral();

      stage = 66;

      break;

    case 66:
      while(!detectBottomLight()){
        moveForward(130);
      }
      setNeutral();
      stage = 67;

    case 67:
      delay(50);
      attachArmMotors();
      delay(250);
      stage = 68;
      break;

    case 68:
      raiseArm();
      delay(2500);
      stopArm();
      stage = 69;
      break;

    case 69:
      rotatePerpendicular();
      delay(3000);
      stopRotation();
      stage = 70;
      break;

    case 70:
      extendArm();
      delay(3750);
      stopArm();
      delay(500);
      openClaw();
      delay(3000);
      stopClaw();
      stage = 71;
      break;


    // ==================== STAGE 71-80 ====================

    case 71:
      while(!hitTopFront()){
        retractArm();
      }
      stopHorizontalArm();
      delay(500);
      closeClaw();
      delay(4500);
      stopClaw();
      stage = 72;
      break;

    case 72:
      rotateParallel();
      delay(3000);
      stopRotation();
      detachArmMotors();
      stage = 999;

    break;

    // ==================== LEGACY STAGES ====================

    case 150:
    if(!hitTopFront())
      retractArm();
    else{
    stopHorizontalArm();
    stage=14;
    }
      break;

    case 151:
     if(armPing()>250){
       extendArm();
       openClaw();
     }
     else{
     stopHorizontalArm();
     closeClaw();
     stage=13;
     }

      break;

    //Finding door from anywhere in the room

    case 152:
      // Stage status:
      // Start stage: Robot is in the middle of the room, unaligned
      pivotAlign();
      delay(500);
      stage = 22;

      break;

    case 153:
      moveForward(200);
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 23;
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 24;
      }

      break;

    case 154:
      smartMoveForwards();
      if (hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 24;
      }
      break;

    case 155:
    //At table
      setNeutral();
      pivotAlign();
      stage = 26;
      break;

    case 156:

      break;

    case 157:
      if(detectObjectRight())
        moveForward(150);
      else{
        turnRightAngle(90);
        setNeutral();
        stage=27;
      }
      break;

    case 158:
      smartMoveForwards();
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage=28;
      }
      break;

    case 159:
      //Past table
      smartMoveForwards();
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
      }
      //Find door light
      else if(detectLight())
      {
        moveForwardDistance(1000);
        turnRightAngle(90);
        moveBackDistance(500);
        stage = 29;
      }
      break;

    case 160:
      moveForward(500);
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
      }
      break;

    case 161:
      break;


  }

}


// --------------------------------------------------------------
// ------------------------- !FUNCTIONS -------------------------
// --------------------------------------------------------------



// -------------------- !SENSOR FUNCTIONS --------------------


float frontPing() {
  // Front ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, LOW);

  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_FRONT, HIGH, 10000);

  while (ping_time < 10){
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, LOW);
  ping_time = pulseIn(ULTRASONIC_OUT_PIN_FRONT, HIGH, 10000);
  delayMicroseconds(10);
  }

  Serial.println(ping_time);

  return ping_time;
}


float backPing(){
  // Back ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_BACK, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ULTRASONIC_IN_PIN_BACK, LOW);

  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_BACK, HIGH, 10000);

  while (ping_time < 10){
  digitalWrite(ULTRASONIC_IN_PIN_BACK, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_IN_PIN_BACK, LOW);
  ping_time = pulseIn(ULTRASONIC_OUT_PIN_BACK, HIGH, 10000);
  delayMicroseconds(10);
  }

  Serial.println(ping_time);

  return ping_time;
}

float armPing() {
  // Top ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_ARM, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_IN_PIN_ARM, LOW);

  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_ARM, HIGH, 10000);

  while (ping_time < 10){
    digitalWrite(ULTRASONIC_IN_PIN_ARM, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_IN_PIN_ARM, LOW);
    ping_time = pulseIn(ULTRASONIC_OUT_PIN_ARM, HIGH, 10000);
    delayMicroseconds(10);
  }

  Serial.println(ping_time);

  return ping_time;
}

boolean hitTable(){
  int bottom_lever = digitalRead(FRONT_BOTTOM_LEVER_SWITCH_PIN);

  if (bottom_lever == LOW){
    delay(300);
    int top_lever = digitalRead(FRONT_TOP_LEVER_SWITCH_PIN);
    if (top_lever == HIGH){
      Serial.println("Table");
      return true;
    }
    else {
      Serial.println("Nothing");
      return false;
    }
  }
  else{
    Serial.println("Nothing");
    return false;
  }
}

boolean hitWall(){
  int bottom_lever = digitalRead(FRONT_BOTTOM_LEVER_SWITCH_PIN);
  int top_lever = digitalRead(FRONT_TOP_LEVER_SWITCH_PIN);
  if ((top_lever == LOW)){
    Serial.println("Wall");
    return true;
  }
  else{
    Serial.println("Nothing");
    return false;
  }
}

boolean hitTopBack(){
  int topBackLever = digitalRead(TOP_BACK_LEVER_SWITCH_PIN);
  if (topBackLever == LOW){
    Serial.println("Back");
    return true;
  }
  else{
    Serial.println("Nothing");
    return false;
  }
}

boolean hitTopFront(){
  int topBackLever = digitalRead(TOP_FRONT_LEVER_SWITCH_PIN);
  if (topBackLever == LOW){
    Serial.println("Front");
    return true;
  }
  else{
    Serial.println("Nothing");
    return false;
  }
}

boolean hitArm(){
  int arm_lever = digitalRead(ARM_SWITCH_PIN);
  if (arm_lever == LOW){
    Serial.println("Hit Arm");
    return true;
  }
  else{
    Serial.println("Nothing");
    return false;
  }
}

boolean hitLowerArm(){
  int arm_lever = digitalRead(LOWER_ARM_SWITCH_PIN);
  if (arm_lever == LOW){
    Serial.println("Hit Lower Arm");
    return true;
  }
  else{
    Serial.println("Nothing");
    return false;
  }
}

// Ping the arm ultrasonic sensor -number_of_times- and returns the total value of the pings
float armPingNumberOfTimes(int number_of_times){
  float total_ping_value;
  float arm_ping;
  float times_counter;

  times_counter = number_of_times;

  while (times_counter != 0){
    arm_ping = armPing();
    total_ping_value += arm_ping;
    times_counter -= 1;
    delay(10);
    Serial.println("Pinged the arm sensor");
  }

  Serial.println("Done armPingNumberOfTimes");
  return total_ping_value;
}

void countLight(){
  //light_value = analogRead(RIGHT_LIGHT_SENSOR);
  next_light_value = analogRead(RIGHT_LIGHT_SENSOR);
  if(next_light_value < 50)
  {
    moveForwardDistance(1000);
    count++;
  }
}

// Returns true if the top light sensor encounters a light source, otherwise false
boolean detectLight(){
  int light_value;
  light_value = analogRead(RIGHT_LIGHT_SENSOR);
  Serial.print("Light value: ");
  Serial.println(light_value);
  if (light_value < 65){
    Serial.println("Light detected");
    return true;
  }
  else{
    Serial.println("No light detected");
    return false;
  }
}

// Returns true if the bottom light sensor encounters a light source, otherwise false
boolean detectBottomLight(){
  int light_value;
  light_value = analogRead(RIGHT_BOTTOM_LIGHT_SENSOR);
  Serial.print("Light value: ");
  Serial.println(light_value);
  if (light_value < 65){
    Serial.println("Light detected");
    return true;
  }
  else{
    Serial.println("No light detected");
    return false;
  }
}

void getEncoderPos(){
  Serial.print("Rot: ");
  Serial.println(encoder_RotMotor.getRawPosition());
  Serial.print("Encoders L: ");
  Serial.print(encoder_LeftMotor.getRawPosition());
  Serial.print(", R: ");
  Serial.print(encoder_RightMotor.getRawPosition());
}


// -------------------- !MOVEMENT FUNCTIONS --------------------

void turnLeftAngle(long angle){
  calcLeftTurn(2300, angle);
  while (!doneLeftTurn())
  {
    turnLeftOnSpot(200);
  }
  setNeutral();
}

void turnRightAngle(long angle){
  calcRightTurn(2300, angle);
  while (!doneRightTurn())
  {
    turnRightOnSpot(200);
  }
  setNeutral();
}

void moveForwardFixed(){
  Left_Motor_Speed = 1700;
  Right_Motor_Speed = 1700;
  implementMotorSpeed();
}

void moveBackwardsFixed(){
  Left_Motor_Speed = 1350;
  Right_Motor_Speed = 1350;
  implementMotorSpeed();
}

void moveForward(long speedFactor)
{
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor), 1500, 2100);
  implementMotorSpeed();
}

void moveBackwards(long speedFactor){
  Left_Motor_Speed = constrain((Left_Motor_Stop - speedFactor), 900, 1500);
  Right_Motor_Speed = constrain((Right_Motor_Stop - speedFactor), 900, 1500);
  implementMotorSpeed();
}

void moveBackDistance(long distance){
  leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
  leftEncoderStopTime -= distance;

  while (encoder_LeftMotor.getRawPosition() > leftEncoderStopTime)
  {
    Serial.println(leftEncoderStopTime);
    Serial.println(encoder_LeftMotor.getRawPosition());
    Left_Motor_Speed = constrain((Left_Motor_Stop - 200), 900, 1500);
    Right_Motor_Speed = constrain((Right_Motor_Stop - 200), 900, 1500);
    implementMotorSpeed();
  }
  setNeutral();
}

void moveForwardDistance(long distance){
  leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
  while ((leftEncoderStopTime + distance) > encoder_LeftMotor.getRawPosition())
  {
    Left_Motor_Speed = constrain((Left_Motor_Stop + 150), 1500, 2100);
    Right_Motor_Speed = constrain((Right_Motor_Stop + 150), 1500, 2100);
    implementMotorSpeed();
  }
  setNeutral();
}

// Turn functions

void veerRight(long speedFactor, long intensity){
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor + intensity), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor), 1500, 2100);
  implementMotorSpeed();
}

void veerLeft(long speedFactor, long intensity){
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor + intensity), 1500, 2100);
  implementMotorSpeed();
}

void turnLeftOnSpot(long speedFactor){
  Left_Motor_Speed = constrain((Left_Motor_Stop - speedFactor), 900, 1500);
  Right_Motor_Speed = constrain((Right_Motor_Stop + speedFactor), 1500, 2100);
  implementMotorSpeed();
}
void turnRightOnSpot(long speedFactor){
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = constrain((Right_Motor_Stop - speedFactor), 900, 1500);
  implementMotorSpeed();
}
void calcLeftTurn(long fullCircle, int angle){ // full circle should be around 2800
  rightEncoderStopTime = encoder_RightMotor.getRawPosition();
  rightEncoderStopTime += (fullCircle * angle) / 360;
}

void calcRightTurn(long fullCircle, int angle){
  leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
  leftEncoderStopTime += (fullCircle * angle) / 360;
}

boolean doneLeftTurn(){
  if (encoder_RightMotor.getRawPosition() > rightEncoderStopTime)
    return true;
  else
    return false;
}
boolean doneRightTurn(){
  if (encoder_LeftMotor.getRawPosition() > leftEncoderStopTime)
    return true;
  else
    return false;
}
boolean doneReverse(){
  if (encoder_LeftMotor.getRawPosition() < leftEncoderStopTime)
    return true;
  else
    return false;
}

void turnClockwise(long speedFactor) {
  Left_Motor_Speed = constrain((Left_Motor_Stop + speedFactor), 1500, 2100);
  Right_Motor_Speed = Left_Motor_Stop;
  implementMotorSpeed();
}

// Implementation movement functions

void implementMotorSpeed(){
  servo_LeftMotor.writeMicroseconds(constrain((Left_Motor_Speed + Left_Motor_Offset), 900, 2100));
  servo_RightMotor.writeMicroseconds(constrain((Right_Motor_Speed + Right_Motor_Offset), 900, 2100));
}

// Stops the robot
void setNeutral(){
  Left_Motor_Speed = 1500;
  Right_Motor_Speed = 1500;
  implementMotorSpeed();
}

// Engages the motor brakes
void brake(){
  Left_Motor_Speed = 200;
  Right_Motor_Speed = 200;
  implementMotorSpeed();
}

// Robot moves backwards
void backUp(){
  setNeutral();
  moveBackwardsFixed();
  delay(1250);
  setNeutral();
}

// ------------------- !SMART MOVEMENT FUNCTIONS -------------------

// Aligns the robot parallel to whatever's on the right
void pivotAlign(){
  float front_ping;
  float back_ping;

  front_ping = frontPing();
  back_ping = backPing();

  while ((abs(front_ping - back_ping)) > 65){
    if (front_ping > back_ping){
      Serial.println("Gotta pivot right");
      turnRightAngle(2);
    }
    else if (back_ping > front_ping){
      Serial.println("Gotta pivot left");
      turnLeftAngle(2);
    }
    delay(50);

    front_ping = frontPing();
    delay(15);
    back_ping = backPing();
  }
  Serial.println("Everything OK");
}

// Used to prevent the robot from crashing into the wall if it's angled too steeply
void reAlign(float ping_value){
  float front_ping = ping_value;
  if (front_ping < 300){
    setNeutral();
    turnLeftAngle(20);
    moveForward(200);
  }
  else{
    moveForward(200);
    delay(1000);
    setNeutral();
    turnLeftAngle(20);
  }
}


// Aligns the robot parallel to whatever's on the right approximately 15 centimeters away
void parallelPark(){
  float front_ping;
  front_ping = frontPing();

  while ((abs(front_ping - 1000) > 100)){
    if (front_ping > 1000){
      Serial.println("Too far, veering right");
      turnRightAngle(15);
      delay(500);
    }
    else if (front_ping < 1000){
      Serial.println("Too close, veering left");
      turnLeftAngle(15);
      delay(500);
    }

    moveForwardDistance(300);
    delay(500);
    pivotAlign();
    delay(500);

    front_ping = frontPing();
  }
}

// Moves the robot forwards, ensuring it's parallel to whatever's on the right
void smartMoveForwards(){
  // Keep between 900 and 650 for ping
  startWaiting();

  float front_ping;
  float back_ping;

  front_ping = frontPing();
  delay(10);
  back_ping = backPing();

  if (waitMilliSecond(150)){

    if ((back_ping - front_ping) > 400){
      reAlign(front_ping);
      Serial.println("Realigning");
    }
    else if (front_ping > 875){
      veerRight(100, 200);
      Serial.println("Too far, need to veer right");
    }
    else if (front_ping < 750){
      veerLeft(100, 200);
      Serial.println("Too close, need to veer left");
    }
    else {
      moveForward(200);
      Serial.println("Everything's perfect");
    }
  }
}

void slowSmartMoveForwards(){
  // Keep between 900 and 650 for ping
  startWaiting();

  float front_ping;
  float back_ping;

  front_ping = frontPing();
  delay(10);
  back_ping = backPing();

  if (waitMilliSecond(100)){

    if (front_ping > 1050){
      veerRight(50, 200);
      Serial.println("Too far, need to veer right");
    }
    else if (front_ping < 950){
      veerLeft(50, 200);
      Serial.println("Too close, need to veer left");
    }
    else {
      moveForward(130);
      Serial.println("Everything's perfect");
    }
  }
}

// -------------------- !ARM FUNCTIONS --------------------

void rotateAmount(long pos){
  if(encoder_RotMotor.getRawPosition() < (pos - 10))
    servo_RotMotor.writeMicroseconds(1700);

  else if(encoder_RotMotor.getRawPosition() > (pos + 10))
    servo_RotMotor.writeMicroseconds(1300);
  else
    servo_RotMotor.writeMicroseconds(1500);
}

// Pivots the arm until it's perpendicular to the robot
void rotatePerpendicular(){
  if(encoder_RotMotor.getRawPosition() < 640){
    while (encoder_RotMotor.getRawPosition() < 640){
      servo_RotMotor.writeMicroseconds(1700);
    }
  }
  else if(encoder_RotMotor.getRawPosition() > 660){
    while(encoder_RotMotor.getRawPosition() > 660){
      servo_RotMotor.writeMicroseconds(1300);
    }
  }
  servo_RotMotor.writeMicroseconds(1500);
}

// Pivots the arm until it's parallel to the robot
void rotateParallel(){
  if(encoder_RotMotor.getRawPosition() < -10){
    while(encoder_RotMotor.getRawPosition() < -10){
      servo_RotMotor.writeMicroseconds(1700);
    }
  }
  else if(encoder_RotMotor.getRawPosition() > 10){
    while(encoder_RotMotor.getRawPosition() > 10){
      servo_RotMotor.writeMicroseconds(1300);
    }
  }
  servo_RotMotor.writeMicroseconds(1500);
}

void stopRotation(){
  servo_RotMotor.writeMicroseconds(1500);
}

// Raise the arm
void raiseArm(){
  servo_VerticleMotor.writeMicroseconds(1900);
}

void stopArm(){
  servo_VerticleMotor.writeMicroseconds(1500);
}

// Lower the arm
void lowerArm(){
  servo_VerticleMotor.writeMicroseconds(1100);
}

// Extend the arm
void extendArm(){
  servo_ExtendMotor.writeMicroseconds(1250);
}

// Retract the arm
void retractArm(){
  servo_ExtendMotor.writeMicroseconds(1800);
}

void stopHorizontalArm(){
  servo_ExtendMotor.writeMicroseconds(1500);
}

void openClaw(){
  Serial.println("Opening the claw");
  servo_ClawMotor.writeMicroseconds(1650);
  delay(1500);
}

void closeClaw(){
  Serial.println("Closing the claw");
  servo_ClawMotor.writeMicroseconds(1400);
  delay(1500);
}

void stopClaw(){
  servo_ClawMotor.writeMicroseconds(1500);
}

void calcRotTurn(long fullCircle, int angle){
  rotEncoderStopTime = encoder_LeftMotor.getRawPosition();
  rotEncoderStopTime = (fullCircle * angle) / 360;
}

// -------------------- !TIME FUNCTIONS --------------------

// Call startWaiting first, and then waitMilliSecond
void startWaiting(){
  if (can_start_waiting) {
    time_previous = millis();
  }
}

// Returns true if -interval- amount of milliseconds has passed since startWaiting
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

// ------------------- !AUDIO FEEDBACK FUNCTIONS -------------------


// -------------------- !INTEGRATION FUNCTIONS --------------------

boolean findBottleDanny(){
  float arm_ping;

  // Calculate the average background noise
  float average_background_ping;
  float moveDistance;
  average_background_ping = (armPingNumberOfTimes(10) / 10);

  while(true){
    average_background_ping = (armPingNumberOfTimes(10) / 10);
    if (average_background_ping < 5000){
      Serial.println("Bottle detected");
      setNeutral();
      moveDistance=(average_background_ping/71.00)*0.1005; // 0.1305
      moveForwardDistance(25.00*moveDistance);
      return true;
    }
    else if (hitWall()){
      Serial.println("Hit the wall");
      setNeutral();
      return false;
    }
    else if (!detectObjectRight){
      Serial.println("Drove past the table");
      setNeutral();
      return false;
    }
    else{
      Serial.println("Moving forward");
      moveForward(100);
    }
  }
}

// Moves forward continuously and scans for a water bottle. Returns true if it detects the bottle, or false if it doesn't

boolean differenceDetected(float first_value, float second_value){
  if((abs(first_value - second_value)) > 600){
    return true;
  }
  else{
    false;
  }
}

void findBottleDifferenceMethod(){
  float first_ping;
  float second_ping;

  first_ping = (armPingNumberOfTimes(3)/3);

  while (first_ping > 770){
    Serial.println("Moving forward");
    first_ping = (armPingNumberOfTimes(3)/3);
    moveForward(110);
    delay(1000);
    second_ping = (armPingNumberOfTimes(3)/3);

    if (differenceDetected(first_ping, second_ping)){
      Serial.println("Difference detected, entering inner loop");
      setNeutral();
      while ((!differenceDetected(first_ping, second_ping)) && (first_ping > 770)){
        Serial.println("Extending");
        first_ping = (armPingNumberOfTimes(5) / 5);
        extendArm();
        delay(1000);
        second_ping = (armPingNumberOfTimes(5) / 5);
      }
      stopHorizontalArm();
      Serial.println("Stopping the arm");
    }
  }
  setNeutral();
  stopHorizontalArm();
}

void findBottle(){

  while(!bottleDetected(5000)){
    if (hitWall()){
      Serial.println("Hit the wall");
      setNeutral();
      return;
    }
    else if (!detectObjectRight){
      Serial.println("Drove past the table");
      setNeutral();
      return;
    }
    else{
      Serial.println("Moving forward");
      moveForwardDistance(300);
    }
  }
  Serial.println("Bottle detected, zooming in");
  setNeutral();
  return;
}

boolean bottleDetected(float ping_value){
  float arm_ping;
  arm_ping = (armPingNumberOfTimes(5) / 5);

  if (arm_ping < ping_value){
    return true;
  }
  else{
    return false;
  }
}

void zoomIntoBottle(){
  float arm_ping;

  int factor;
  factor = 1.5;
  
  int extend_factor;
  extend_factor = 2;

  while(!bottleDetected(770)){
    arm_ping = (armPingNumberOfTimes(10)/10);
    delay(1000);
    if(arm_ping > 5000){
      Serial.println("5000");
      Serial.println("5000");
      moveForwardDistance(500*factor);
      delay(500);
      extendArm();
      delay(5000*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 4500){
      Serial.println("4500");
      moveForwardDistance(450*factor);
      delay(500);
      extendArm();
      delay(4500*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 4000){
      Serial.println("4000");
      moveForwardDistance(400*factor);
      delay(500);
      extendArm();
      delay(4000*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 3500){
      Serial.println("3500");
      moveForwardDistance(350*factor);
      delay(500);
      extendArm();
      delay(3500*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 3000){
      Serial.println("3000");
      moveForwardDistance(300*factor);
      delay(500);
      extendArm();
      delay(3000*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 2500){
      Serial.println("2500");
      moveForwardDistance(250*factor);
      delay(500);
      extendArm();
      delay(2500*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 2000){
      Serial.println("2000");
      moveForwardDistance(200*factor);
      delay(500);
      extendArm();
      delay(2000*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 1500){
      Serial.println("1500");
      moveForwardDistance(150*factor);
      delay(500);
      extendArm();
      delay(1500*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 1000){
      Serial.println("1000");
      moveForwardDistance(100*factor);
      delay(500);
      extendArm();
      delay(1000*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 850){
      Serial.println("850");
      moveForwardDistance(85*factor);
      delay(500);
      extendArm();
      delay(850*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
    else if (arm_ping > 780){
      Serial.println("780");
      moveForwardDistance(78*factor);
      delay(500);
      extendArm();
      delay(780*extend_factor);
      stopHorizontalArm();
      delay(1000);
    }
  }
  Serial.println("Last moving forward...");
  moveForwardDistance(250);
  delay(1000);
}

void grabBottle(){
  openClaw();
  delay(4000);
  while(!bottleDetected(250)){
    extendArm();
  }
  stopHorizontalArm();
  delay(1000);
  lowerArmUntilHit();
  delay(1000);
  closeClaw();
  delay(4000);
}

void lowerArmUntilHit(){
  while(!hitArm()){
    lowerArm();
  }
  stopArm();
}

// Moves forward continuously and scans for a light source. Returns true if it detects a light source, or false if it doesn't
boolean detectLongSide(){
  while (detectObjectRight()){
    if (detectLight()){
      Serial.println("Long edge of the table detected, ending loop");
      setNeutral();
      return true;
    }
    if (hitWall()){
      Serial.println("Hit the wall, ending loop");
      setNeutral();
      return false;
    }
    Serial.println("Moving forward");
    moveForward(130);
    delay(25);
  }
  Serial.println("Long edge not detected");
  return false;
}

// Moves forward continuously and scans for a bottom light source. Returns true if it detects a light source, or false if it doesn't
boolean detectBottomLongSide(){
  while (detectObjectRight()){
    if (detectBottomLight()){
      Serial.println("Long edge of the table detected, ending loop");
      setNeutral();
      return true;
    }
    if (hitWall()){
      Serial.println("Hit the wall, ending loop");
      setNeutral();
      return false;
    }
    Serial.println("Moving forward");
    moveForward(130);
    delay(25);
  }
  Serial.println("Long edge not detected");
  return false;
}

// Returns true if there's an object on the right, otherwise it returns false
boolean detectObjectRight(){
  float back_ping;
  back_ping = backPing();

  if (back_ping < 2000){
    Serial.println("Object detected");
    return true;
  }
  else {
    Serial.println("No object detected");
    return false;
  }
}

// ------------------ Detach and attach functions ------------------

void detachArmMotors(){
  detachRotMotor();
  detachExtendMotor();
  detachVerticalMotor();
  detachClawMotor();
}

void detachRotMotor(){
  servo_RotMotor.detach();
}

void detachExtendMotor(){
  servo_ExtendMotor.detach();
}

void detachVerticalMotor(){
  servo_VerticleMotor.detach();
}

void detachClawMotor(){
  servo_ClawMotor.detach();
}

void attachArmMotors(){
  attachRotMotor();
  attachExtendMotor();
  attachVerticalMotor();
  attachClawMotor();
}

void attachRotMotor(){
  servo_RotMotor.attach(ROT_MOTOR_PIN);
}

void attachExtendMotor(){
  servo_ExtendMotor.attach(EXTEND_MOTOR_PIN);
}

void attachVerticalMotor(){
  servo_VerticleMotor.attach(VERTICLE_MOTOR_PIN);
}

void attachClawMotor(){
  servo_ClawMotor.attach(CLAW_MOTOR_PIN);
}

