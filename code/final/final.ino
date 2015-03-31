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
  Serial.begin(9600);

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
      /* To test:
      parallelPark
      findBottle
      */
      parallelPark();
      stage = 10;

    break;

    case 1:
      // Case status: COMPLETE by Daniel
      // Start case: Robot is in the middle of the room, unaligned
      pivotAlign();
      delay(500);
      stage = 2;

      break;
      // End case: robot is in the middle of the room, parallel to the wall or table

    case 2:
      // Case status: COMPLETE by Daniel
      // Start case: Robot is in the middle of the room, parallel to the wall or table
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
      // End case: Robot is parallel to the wall

    case 3:
      // Case status: COMPLETE by Daniel
      // Start case: Wall is in front of robot, robot is parallel to it
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
      // End case: Table is in front of robot, robot is parallel to it

    case 4:
      // Case status: IN PROGRESS by Daniel
      // Start case: Robot is parallel to table, need to move all the way to the back to the wall
      setNeutral();
      pivotAlign();
      stage = 5;

      break;
      // Robot is parallel to the table, at the far back, ready to scan for the light

    case 5:
      backUp();
      setNeutral();

    case 6:
      // Case status: IN PROGRESS by Daniel
      if(detectLongSide()){
        setNeutral();
        stage = 8;
      }
      else{
        setNeutral();
        stage = 7;
      }

      break;

    case 7:
      // Case status: IN PROGRESS by Daniel
      // Start case: Robot has escaped the short edge of the table
      moveForwardDistance(180);
      delay(500);
      turnRightAngle(92);
      delay(500);
      stage = 8;

      break;

    case 8:
      // Case status: IN PROGRESS by Daniel
      while(!detectLight()){
        moveForward(100);
      }
      setNeutral();
      stage = 9;

      break;

    case 9:
      parallelPark();
      setNeutral();
      stage = 10;

      break;

    case 10:
      while(!detectLight()){
        moveBackwards(100);
      }
      setNeutral();
      stage = 11;

      break;

    // ==================== STAGE 11-20 ====================

    case 11:

      break;

    case 12:

      break;

    case 13:
      break;

    case 14:
      break;

    case 15:
      break;

    case 16:
      break;

    case 17:
      break;

    case 18:
      break;

    case 19:
      break;

    case 20:
      break;

    // ==================== STAGE 21-30 ====================

    case 21:
      smartMoveForwards();
      countLight();

      if(count==1)
      {
        turnRightAngle(90);
        moveBackDistance(500);
        count=0;
        stage = 22;
      }
      break;

    case 22:
      moveForward(500);
     if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 2;
      }

      break;

    case 23:
      break;

    case 24:
      break;

    case 25:
      break;

    case 26:
      break;

    case 27:
      break;

    case 28:
      break;

    case 29:
      break;


  case 30:
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

  Serial.println(ping_time);

  return ping_time;
}


float backPing(){
  // Back ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_BACK, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ULTRASONIC_IN_PIN_BACK, LOW);

  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_BACK, HIGH, 10000);

  Serial.println(ping_time);

  return ping_time;
}

float armPing() {
  // Top ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_ARM, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_IN_PIN_ARM, LOW);

  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_ARM, HIGH, 10000);

  Serial.println(ping_time);

  return ping_time;
}

void getEncoderPos(){
  Serial.print("Rot: ");
	Serial.println(encoder_RotMotor.getRawPosition());
	Serial.print("Encoders L: ");
	Serial.print(encoder_LeftMotor.getRawPosition());
	Serial.print(", R: ");
	Serial.print(encoder_RightMotor.getRawPosition());
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
  if ((top_lever == LOW) && (bottom_lever == LOW)){
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

float armPingNumberOfTimes(int number_of_times){
  float total_ping_value;
  float arm_ping;
  float times_counter;
  times_counter = number_of_times;
  while (times_counter != 0){
    arm_ping = armPing();
    total_ping_value += arm_ping;
    times_counter -= 1;
    delay(100);
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
  if (light_value < 50){
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
  if (light_value < 50){
    Serial.println("Light detected");
    return true;
  }
  else{
    Serial.println("No light detected");
    return false;
  }
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
  Left_Motor_Speed = 1300;
  Right_Motor_Speed = 1300;
  implementMotorSpeed();
}

// Moves the robot forwards, ensuring it's parallel to whatever's on the right
void smartMoveForwards(){
  // Keep between 800 and 450 for ping
  startWaiting();

  float front_ping;
  float back_ping;

  front_ping = frontPing();
  delay(10);
  back_ping = backPing();

  if (waitMilliSecond(250)){
    while (front_ping < 5){
      front_ping = frontPing();
      delay(10);
    }
    while (back_ping < 5){
      back_ping = backPing();
      delay(10);
    }

    if ((back_ping - front_ping) > 400){
      reAlign(front_ping);
      Serial.println("Realigning");
    }
    else if (front_ping > 900){
      veerRight(100, 200);
      Serial.println("Too far, need to veer right");
    }
    else if (front_ping < 650){
//      setNeutral();
//      turnLeftAngle(10);
      veerLeft(100, 200);
      Serial.println("Too close, need to veer left");
    }
    else {
      moveForward(200);
      Serial.println("Everything's perfect");
    }
  }
}

void reAlign(float ping_value){
    float front_ping = ping_value;
    if (front_ping < 300){
      setNeutral();
      turnLeftAngle(25);
      moveForward(200);
    }
    else{
      moveForward(200);
      delay(1000);
      setNeutral();
      turnLeftAngle(20);
    }
}

void pivotAlign(){ //Aligns the robot parallel to whatever's on the right
  float front_ping;
  float back_ping;

  front_ping = frontPing();
  back_ping = backPing();

  while(front_ping < 5){
    front_ping = frontPing();
    delay(10);
  }
  while(back_ping < 5){
    back_ping = backPing();
    delay(10);
  }

  while ((abs(front_ping - back_ping)) > 100){
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
    back_ping = backPing();
    while(front_ping < 5){
      front_ping = frontPing();
      delay(10);
    }
    while(back_ping < 5){
      back_ping = backPing();
      delay(10);
    }
  }
  Serial.println("Everything OK");
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
    Left_Motor_Speed = constrain((Left_Motor_Stop + 200), 1500, 2100);
    Right_Motor_Speed = constrain((Right_Motor_Stop + 200), 1500, 2100);
    implementMotorSpeed();
  }
  setNeutral();
}

void backUp(){
  setNeutral();
  moveBackwardsFixed();
  delay(1500);
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

void setNeutral(){
  Left_Motor_Speed = 1500;
  Right_Motor_Speed = 1500;
  implementMotorSpeed();
}

void brake(){
  Left_Motor_Speed = 200;
  Right_Motor_Speed = 200;
  implementMotorSpeed();
}


// -------------------- TIME FUNCTIONS --------------------


// Call startWaiting first, and then waitMilliSecond
void startWaiting(){
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

// -------------------- !ARM FUNCTIONS --------------------

void rotateAmount(long pos){
  if(encoder_RotMotor.getRawPosition() < (pos - 10))
    servo_RotMotor.writeMicroseconds(1700);

  else if(encoder_RotMotor.getRawPosition() > (pos + 10))
    servo_RotMotor.writeMicroseconds(1300);
  else
    servo_RotMotor.writeMicroseconds(1500);
}

void rotatePerpendicular(){
  if(encoder_RotMotor.getRawPosition() < 740)
    servo_RotMotor.writeMicroseconds(1700);
  else if(encoder_RotMotor.getRawPosition() > 760)
    servo_RotMotor.writeMicroseconds(1300);
  else
    servo_RotMotor.writeMicroseconds(1500);
}

void rotateParallel(){
  if(encoder_RotMotor.getRawPosition() < -10)
    servo_RotMotor.writeMicroseconds(1700);
  else if(encoder_RotMotor.getRawPosition() > 10)
    servo_RotMotor.writeMicroseconds(1300);
  else
    servo_RotMotor.writeMicroseconds(1500);
}

void moveUp(){
  servo_VerticleMotor.writeMicroseconds(1900);
}

void moveDown(){
  servo_VerticleMotor.writeMicroseconds(1250);
}

void moveOut(){
  servo_ExtendMotor.writeMicroseconds(1250);
}

void moveIn(){
  servo_ExtendMotor.writeMicroseconds(1800);
}

void horizontalStop(){
  servo_ExtendMotor.writeMicroseconds(1500);
}

void openClaw(){
  servo_ClawMotor.writeMicroseconds(1650);
}

void closeClaw(){
  servo_ClawMotor.writeMicroseconds(1400);
}

void calcRotTurn(long fullCircle, int angle){
  rotEncoderStopTime = encoder_LeftMotor.getRawPosition();
  rotEncoderStopTime = (fullCircle * angle) / 360;
}

// ------------------- !AUDIO FEEDBACK FUNCTIONS -------------------


// -------------------- !COMBINED FUNCTIONS --------------------

// Moves forward continuously and scans for a water bottle. Returns true if it detects the bottle, or false if it doesn't
boolean findBottle(){
  float arm_ping;

  // Calculate the average background noise
  float average_background_ping;
  average_background_ping = (armPingNumberOfTimes(10) / 10);

  while(true){
    arm_ping = armPing();
    while (arm_ping < 5){ // Re-ping if a null value is returned
      arm_ping = armPing();
      delay(10);
    }

    if ((average_background_ping - arm_ping) > 500){
      Serial.println("Bottle detected");
      setNeutral();
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
      delay(250);
    }
  }
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
    moveForward(100);
    delay(25);
  }
  Serial.println("Long edge not detected");
  return false;
}

// Returns true if there's an object on the right, otherwise it returns false
boolean detectObjectRight(){
  float back_ping;
  back_ping = backPing();

  while (back_ping < 5){ // Re-ping if a null value is returned
    back_ping = backPing();
    delay(10);
  }

  if (back_ping < 2000){
    Serial.println("Object detected");
    return true;
  }
  else {
    Serial.println("No object detected");
    return false;
  }
}

// Aligns the robot parallel to whatever's on the right approximately 15 centimeters away
void parallelPark(){
  float front_ping;
  front_ping = frontPing();

  while (front_ping < 5){
    front_ping = frontPing();
    delay(10);
  }

  while ((abs(front_ping - 1000) > 75)){
    if (front_ping > 1000){
      Serial.println("Too far, veering right");
      turnRightAngle(10);
      delay(250);
    }
    else if (front_ping < 1000){
      Serial.println("Too close, veering left");
      turnLeftAngle(10);
      delay(250);
    }

    moveForwardDistance(200);
    pivotAlign();

    front_ping = frontPing();
    while (front_ping < 5){
      front_ping = frontPing();
      delay(10);
    }
  }
}
