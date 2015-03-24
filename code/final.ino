#include <Servo.h>
//Motors
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

//Variables
unsigned long Echo_Time;

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
  
servo_LeftMotor.writeMicroseconds(1800);
servo_RightMotor.writeMicroseconds(1900);
Ping();
  
}
void Ping()
{
//Ping Ultrasonic
//Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
digitalWrite(ultraSonic_In_address, HIGH);
delayMicroseconds(10); //The 10 microsecond pause where the pulse in "high"
digitalWrite(ultraSonic_In_address, LOW);

//use command pulseIn to listen to Ultrasonic_Data pin to record the
//time that it takes from when the Pin goes HIGH until it goes LOW
Echo_Time = pulseIn(ultraSonic_Out_address, HIGH, 10000);

// Print Sensor Readings
Serial.print("Time (microseconds): ");
Serial.print(Echo_Time, DEC);
Serial.print(", cm: ");
Serial.println(Echo_Time / 58); //divide time by 58 to get distance in cm

}

