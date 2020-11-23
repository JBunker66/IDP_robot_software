
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1); // the one is the port the motor is conected to
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);
int count;
void Streight(void){
    // function-body
    LeftMotor->setSpeed(135);
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
    }
  void Left(void){
    // function-body
    LeftMotor->setSpeed(70);
    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
    }
void setup() {
  count = 0;
  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();
  //To run the motor, call run(direction) where direction is FORWARD, BACKWARD or RELEASE
  
  LeftMotor->setSpeed(135);
  RightMotor->setSpeed(155);
}

void loop() {
  uint8_t i;
  // put your main code here, to run repeatedly:
  if(count == 0){
    Streight();
  }
  if(count == 8){
    Left();
    count = count - 16;
  }
  delay(500); 
  count += 1;
}
