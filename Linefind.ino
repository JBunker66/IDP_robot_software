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

//robot is set to turn
boolean isTurn = false;
//robot has the red block
boolean hasRedBlock = false;
//robot has the blue block
boolean hasBlueBlock = false;
//robot on the left and use to check if robot has made the T turn after the tunnel on the right
boolean leftSide = false;
// Above same as lineFollowTargetDetection for continuity
// Unlickly block will stay in streight line for extended time
// if block does not hit line for a nuber of cycles this code is run
// Every streight adds one, every turn resets
int error_catcher = 0; 
// This will be distance detector 
int distance_detector = 10; // placeholder value before properly set up
int value1; // used as a temp value so the distance can be compared
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
  // Above same as lineFollowTargetDetection for continuity
  
}

void loop() {
  // put your main code here, to run repeatedly:
    uint8_t i;

  //read from OPB line sensor
  left_sensor_state = digitalRead(left_sensor_pin);
  right_sensor_state = digitalRead(right_sensor_pin);
  // Most of the code would go here

  if (error_catcher > 20){// Abitary nuber. Easy to change as needed, probably will need to be much larger
    while(error_catcher > 20){
      motorL->run(RELEASE);// Might not be needed depending on how fast it runs
      motorR->run(RELEASE);
      // Does a little reverse in case its stuck face first against a wall, gives room for turning circle
      motorL->setSpeed(vSpeed);
      motorR->setSpeed(vSpeed);
      motorL->run(BACKWARD);
      motorR->run(BACKWARD);
      delay(1000);
      motorL->run(RELEASE);
      motorR->run(RELEASE);
      // Rotates untill perpendicular to a wall
      while(true){ // Might want to add an infinite loop checker
        value1 = distance_detector;
        motorL->run(FORWARD);
        delay(500);// Abitary, want about a 20 degree rotation
        motorL->run(RELEASE);
        if(value1 - distance_detector<0){
          break;
        }
        }
      // Goes forward untill it hits a line
      motorL->run(FORWARD);
      motorR->run(FORWARD);
      for(int i=0; i<1000; i = i+1){
        left_sensor_state = digitalRead(left_sensor_pin);
        // When it hits a line break out of the loop
        if (left_sensor_state == 0){
          error_catcher = 0;
          break;
        }
        delay(100);
      }
      }
      // Turning codes goes here
    }
   }
