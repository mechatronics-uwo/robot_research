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
Servo servo_TopMotor;
Servo servo_LeftMotor;

I2CEncoder encoder_RightMotor;

I2CEncoder encoder_TopMotor;
I2CEncoder encoder_LeftMotor;


// -------------------- WALL-FOLLOWING --------------------
const float speedConstant = 130;
const float leftConstant = 70;
const float rightConstant = 70;

// dimensions
const float sensorDistance = 2700; // distance between centre of sensors
const float limitDistance = 700; // maximum distance from the wall
const float backupDistance = 700; // distance to backup (encoder count)

// working variable
const float deltaTolerance = 70;
float frontReading = 0;
float backReading = 0;
float perpAdd = 0;



// -------------------- SENSORS --------------------

// Motor pins
const int LEFT_MOTOR_PIN = 3;
const int RIGHT_MOTOR_PIN = 2;

// Switch pins
const int FRONT_BOTTOM_LEVER_SWITCH_PIN = 4;
const int FRONT_TOP_LEVER_SWITCH_PIN = 5;

// Ultrasonic pins
const int ULTRASONIC_IN_PIN_FRONT = 52;
const int ULTRASONIC_OUT_PIN_FRONT = 53;

const int ULTRASONIC_IN_PIN_BACK = 50;
const int ULTRASONIC_OUT_PIN_BACK = 51;

const int ULTRASONIC_IN_PIN_ARM = 48;
const int ULTRASONIC_OUT_PIN_ARM = 49;

// Light sensor pins
const int right_light_sensor = A0;
const int right_bottom_light_sensor = A1;

// -------------------- VARIABLES --------------------
int light_value=0;
int next_light_value=0;

int count=0;//counts the lights

// Motor variables
unsigned int Left_Motor_Speed;
unsigned int Right_Motor_Speed;
unsigned int Right_Motor_Stop = 1500;
unsigned int Left_Motor_Stop = 1500;
long leftEncoderStopTime = 0;
long rightEncoderStopTime = 0;
boolean loopStarted = false;// start loop for turning
int Left_Motor_Offset = 0;
int Right_Motor_Offset = 30;

unsigned long time_previous = 0; // Used for time functions, do not change
unsigned long time_elapsed = 0; // Used for time functions, do not change
boolean can_start_waiting = false; // Used for time functions, do not change




// -------------------- STAGE COUNTER --------------------
// NOTE: Stage 0 is reserved for debugging

unsigned int stage = 0;

// ******************************************************************
// ************************* PROGRAM !SETUP *************************
// ******************************************************************
void setup() {

  Wire.begin();
  Serial.begin(9600);

  // Set-up motors

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  servo_LeftMotor.attach(LEFT_MOTOR_PIN);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  servo_RightMotor.attach(RIGHT_MOTOR_PIN);


  // Set-up buttons
  pinMode(FRONT_TOP_LEVER_SWITCH_PIN, INPUT_PULLUP);
  pinMode(FRONT_BOTTOM_LEVER_SWITCH_PIN, INPUT_PULLUP);

  digitalWrite(FRONT_TOP_LEVER_SWITCH_PIN, HIGH);
  digitalWrite(FRONT_BOTTOM_LEVER_SWITCH_PIN, HIGH);

  // Set-up ultrasonic
  pinMode(ULTRASONIC_OUT_PIN_FRONT, INPUT);
  pinMode(ULTRASONIC_IN_PIN_FRONT, OUTPUT);

  pinMode(ULTRASONIC_OUT_PIN_BACK, INPUT);
  pinMode(ULTRASONIC_IN_PIN_BACK, OUTPUT);

  pinMode(ULTRASONIC_OUT_PIN_ARM, INPUT);
  pinMode(ULTRASONIC_IN_PIN_ARM, OUTPUT);

  //Set up encoder
  encoder_TopMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false); // adjust for positive count when moving forward

  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true); // adjust for positive count when moving forward

  encoder_TopMotor.zero();
  encoder_LeftMotor.zero();
  encoder_RightMotor.zero();

}

// *****************************************************************
// ************************* PROGRAM !LOOP *************************
// *****************************************************************
void loop() {


  switch (stage) {

    // ==================== CASE 1-10 ====================

    case 0:
      // RESERVED FOR TESTING, PASTE CODE HERE AND SET STAGE = 0
      frontPing();
      delay(500);
      break;

    case 1:
      // Case status: DONE by Daniel
      // Start case: Robot is in the middle of the room
      moveForward(200);
      if(hitWall()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 2;
      }
      else if(hitTable()){
        setNeutral();
        moveBackDistance(300);
        turnLeftAngle(87);
        stage = 3;
      }
      break;
      // End case: Robot is parallel to the wall

    case 2:
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
        stage = 3;
      }
      break;
      // End case: Robot is parallel to table

    case 3:
      // Case status: IN PROGRESS by Daniel
      // Start case: Robot is parallel to table, need to determine if we're aligned to the short or the long edge
      setNeutral();
      break;
      // Robot is parallel to the long edge of the table

    case 4:
      // Case status: IN PROGRESS by Daniel
      // Start case: Robot is parallel to the long edge of the table


      break;
      // End case: Water bottle is directly in front of the arm

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      break;

    case 9:
      break;

    case 10:
      break;


    // ==================== CASE 11-20 ====================

    case 11:

      light_value = analogRead(right_light_sensor);
      moveForward(150);
      next_light_value = analogRead(right_light_sensor);

      if((next_light_value < light_value) && (next_light_value < 50))
      {
        setNeutral();
        light_value = analogRead(right_light_sensor);
        next_light_value = analogRead(right_light_sensor);
        boolean blinking=false;
        unsigned long begin_time=millis();

         do
         {

          light_value = analogRead(right_light_sensor);
          delay(100);
          next_light_value = analogRead(right_light_sensor);

          if((next_light_value - light_value) >= 20)
           blinking = true;
         }

         while(((millis() - begin_time) < 2000));

         if(blinking)
           backUp();
         else
           moveFowardDistance(300);
      }
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

    // ==================== CASE 21-30 ====================

    case 21:
      moveForward(200);
      countLight();
      
      if(count==1)
      {
        turnRightAngle(90);
        moveBackDistance(500);
        count=0;
        stage++;
      }
      break;

    case 22:
      moveForward(500);    
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



// -------------------- SENSOR FUNCTIONS --------------------

// Ping ultrasonic
// Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec

float frontPing() {
  //Front ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ULTRASONIC_IN_PIN_FRONT, LOW);
  // Serial.print("front: ");
  // Serial.println(ping_time1); //divide time by 58 to get
  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_FRONT, HIGH, 10000);

  Serial.print("cm: ");
  Serial.println(ping_time); //divide time by 58 to get

  return ping_time;
}

float backPing() {
  //Back ultrasonic
  digitalWrite(ULTRASONIC_IN_PIN_BACK, HIGH);
  delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ULTRASONIC_IN_PIN_BACK, LOW);

  // Use command pulseIn to listen to ultrasonic_Data pin to record the
  // time that it takes from when the Pin goes HIGH until it goes LOW
  // Serial.print("back: ");
  // Serial.println(ping_time2); //divide time by 58 to get distance in cm
  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_BACK, HIGH, 10000);


  Serial.print("cm: ");
  Serial.println(ping_time); //divide time by 58 to get distance in cm
  return ping_time;
}

float armPing(){
  digitalWrite(ULTRASONIC_IN_PIN_ARM, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_IN_PIN_ARM, LOW);
  float ping_time = pulseIn(ULTRASONIC_OUT_PIN_ARM, HIGH, 10000);

  Serial.print("CM: ");
  Serial.println(ping_time);

  return ping_time;
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

boolean hitWall() {
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

boolean detectBottle(){
  // Continually read the sensor on the arm
  // return true if found the bottle
}


// -------------------- MOVEMENT FUNCTIONS --------------------


// Smart movement functions

void countLight()
{
      light_value = analogRead(right_light_sensor);
      next_light_value = analogRead(right_light_sensor);
      if((next_light_value < light_value) && (next_light_value < 50))
      {
        moveFowardDistance(1000);
        count++;
      }
}

void turnLeftAngle(long angle)
{
    calcLeftTurn(2300, angle);
    while (!doneLeftTurn())
    {
      turnLeftOnSpot(200);
    }
    setNeutral();
}

void turnRightAngle(long angle)
{
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

void smartMoveForwards(){
  // Keep between 800 and 450 for ping
  startWaiting();

  float front_ping;
  float back_ping;

  if (waitMilliSecond(250)){
    front_ping = frontPing();
    delay(10);
    back_ping = backPing();

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
void moveFowardDistance(long distance)
{
  leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
  while ((leftEncoderStopTime + distance) > encoder_LeftMotor.getRawPosition())
  {
    Left_Motor_Speed = constrain((Left_Motor_Stop + 200), 1500, 2100);
    Right_Motor_Speed = constrain((Right_Motor_Stop + 200), 1500, 2100);
    implementMotorSpeed();
  }
  setNeutral();
}

void backUp() {
  setNeutral();
  moveBackwardsFixed();
  delay(1500);
  setNeutral();
}

// Turn functions

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
boolean doneReverse()
{
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

// -------------------- ARM FUNCTIONS --------------------
void raiseArm(){

}

void lowerArm(){

}

void pivotArmLeft(long encoder_count){

}

void pivotArmRight(long encoder_count){

}

void pivotArmPerpendicular(){
  pivotArmLeft(100); // Dummy value
}

void extendArm(){

}

void retractArm(){

}




// -------------------- WALL-FOLLOWING --------------------

void updateUltrasonics() // updates both ultrasonics, should only be used once per iteration
{
  frontReading = (float)frontPing();
  delay(10);
  backReading = (float)backPing();
  delay(10);
  
}

float perpMinimum() // returns minimum perpendicular distance to wall
{
  if (frontReading < backReading) // toward
    return (frontReading * sensorDistance) / (sqrt((float)sensorDistance * sensorDistance + (frontReading - backReading) * (frontReading - backReading)));
  else // away
    return (backReading * sensorDistance) / (sqrt((float)sensorDistance * sensorDistance + (frontReading - backReading) * (frontReading - backReading)));
}

float perpAverage()// returns average perpendicular distance to wall
{
  //perpAdd = (sensorDistance / 2) / (sqrt((float)sensorDistance * sensorDistance + (frontReading - backReading) * (frontReading - backReading)));
  //return perpMinimum() + perpAdd;
  return (frontReading + backReading)/2;
}

boolean square() // returns turn if square
{
  if (abs(frontReading - backReading) < deltaTolerance)
    return true;
  else
    return false;
}

void followWall() // corrects robot so it is parallel to the wall and returns true if in tolerance
{
  // follow
  if(perpAverage() > 1000 || perpAverage() < 800)
  {
    if (perpAverage() > 1000) // if straight enough, go straight
      veerRight(speedConstant, rightConstant);
    else if (perpAverage() < 800) // if straight enough, go straight
      veerLeft(speedConstant, rightConstant);
  }
  else
  {
    if (frontReading < backReading) // if going toward, turn left
      veerLeft(speedConstant, leftConstant);
    else if (frontReading > backReading)  // if going away, turn right
      veerRight(speedConstant, rightConstant);
    else
      moveForward(150);
  }
}
