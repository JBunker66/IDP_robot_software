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
  int vSpeed = 110;        // MAX 255
  int turn_speed = 230;    // MAX 255 
  int turn_delay = 10;

//OPB line Sensor Connection
const int left_sensor_pin = A0;
const int right_sensor_pin = A1;

int left_sensor_state;
int right_sensor_state;

//if robot is set to turn
boolean isTurn = false;
//Same as linefollowing till here for continuity 
boolean HasBlock = true, BlockInFrount = true, EarlyBreak = false; // Need to code these functions but sensor dependent
int BigTurnDelay = 1000;

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);           
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  motorL->setSpeed(vSpeed);
  motorR->setSpeed(vSpeed);
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  // turn on motor
  motorL->run(RELEASE);
  motorR->run(RELEASE);
}
//Same as linefollowing till here for continuity

void loop() {
  // put your main code here, to run repeatedly:
  //  Checks for both having block and Obstructed By Block
  
  if(HasBlock && BlockInFrount){
    //set up a 90 degree left turn (might make seporate 90 turn functions if reused a lot)
    motorR->setSpeed(turn_speed);
    delay(BigTurnDelay);
    //sets up a streight section
    motorR->setSpeed(vSpeed);
    delay(2000);
    //Turns right and goes forward
    motorL->setSpeed(turn_speed);
    delay(BigTurnDelay);
    motorL->setSpeed(vSpeed);
    for (int i = 0; i <= 100;i=i+1){
      delay(30);
      if (analogRead(left_sensor_pin)>500){//posative read so crosses line in streight around block
        EarlyBreak = true;
        break;
      }
    }
    //Turns right again and returns to line location
    if (not EarlyBreak){
      motorL->setSpeed(turn_speed);
      delay(BigTurnDelay);
      motorL->setSpeed(vSpeed);
    }
   if (analogRead(left_sensor_pin)>500){//might need to add a reverse here
    motorL->run(BACKWARD);
    delay(BigTurnDelay);
    motorL->run(FORWARD);
   }
   }
//Hopefully this works. Will have to adjust the timings and speeds
    
}
