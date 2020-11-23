#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Get motor in port 1 & 2
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

//variable for line following
  int vSpeedRight = 155;        // MAX 255
  int vSpeedLeft = 135;
  int turn_speed = 230;    // MAX 255 
  int turn_delay = 10;

//OPB line Sensor Connection
const int left_sensor_pin = A0;
const int right_sensor_pin = A1;

int left_sensor_state;
int right_sensor_state;

//if robot is set to turn
boolean isTurn = false;

void setup() {
  Serial.begin(9600);           
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  motorL->setSpeed(vSpeedLeft);
  motorR->setSpeed(vSpeedRight);
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
  // turn on motor
  motorL->run(RELEASE);
  motorR->run(RELEASE);
}

void loop() {
  lineFollowMain();
}


void lineFollowMain(){
  //read from OPB line sensor
  left_sensor_state = digitalRead(left_sensor_pin);
  right_sensor_state = digitalRead(right_sensor_pin);
 
  //check if right sensor over the line, need to turn right
  if(right_sensor_state == 1 && left_sensor_state == 0)
  {
    Serial.println("turning right");
    isTurn = true;
    motorL->setSpeed(turn_speed);
    delay(turn_delay);
    }

  //check if left sensor over the line, need to turn right
  if(right_sensor_state == 0 && left_sensor_state == 1)
  {
    Serial.println("turning left");
    isTurn = true;
    motorR->setSpeed(turn_speed);
    delay(turn_delay);
    }
    
  //check if motor move in straight line
  if(right_sensor_state == 0 && left_sensor_state == 0)
  {
    Serial.println("going forward");
  
    //if motor speed is changed for turn, then set it back to normal speed
    if (isTurn){
      motorR->setSpeed(vSpeedRight);
      motorL->setSpeed(vSpeedLeft);
    }
    isTurn = false;
    delay(turn_delay);
    }
}
