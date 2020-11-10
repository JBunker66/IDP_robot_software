
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1); // the one is the port the motor is conected to
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);
int count;
void Streight(void){
    // function-body
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
    }
  void Left(void){
    // function-body
    LeftMotor->run(RELEASE);
    RightMotor->run(FORWARD);
    }
void setup() {
  count = 0;
  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();
  //To run the motor, call run(direction) where direction is FORWARD, BACKWARD or RELEASE
  
  LeftMotor->setSpeed(150);
  RightMotor->setSpeed(150);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(count = 0){
    Streight();
  }
  if(count = 4){
    Left();
    count = count - 7;
  }
  delay(1000); 
}
