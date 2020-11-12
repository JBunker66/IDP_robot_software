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

void setup() {
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

void loop() {
  uint8_t i;

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
      motorR->setSpeed(vSpeed);
      motorL->setSpeed(vSpeed);
    }
    isTurn = false;
    delay(turn_delay);
    }
    
  //both sensor over line, no block is collected & robot on the leftSide, could possibly be used to detect T turn after the tunnel
  if(right_sensor_state == 1 && left_sensor_state == 1 && !hasRedBlock && !hasBlueBlock && leftSide)
  { 
    //could also use ultrasonic sensor for locating T turn

     delay(10);
     //set motor to turn 90 degrees to right
     //use OPB line sensor to stop turning right sensor first detects the line and goes off line again
     //or use magnet sensor to make motor turn for certain number of turns or ultrasonic distance sensor to check distance
     while(true){
        motorR->setSpeed(0);
        motorL->setSpeed(turn_speed);
        delay(turn_delay);
        
        right_sensor_state = digitalRead(right_sensor_pin);
        if(right_sensor_state == 1){
          while(true){
            motorR->setSpeed(0);
            motorL->setSpeed(turn_speed);
            delay(turn_delay);
            right_sensor_state = digitalRead(right_sensor_pin);
            if(right_sensor_state == 0){
              break;
            }
          }
          break;
        }
     }
    leftSide = false;
  }

  //Red delivery target detection
  if(right_sensor_state == 1 && left_sensor_state == 1 && hasRedBlock){
    motorR->setSpeed(0);
    motorL->setSpeed(0);
    
    //the block can be placed on the target
    
    //block is placed and the robot go around the block to move forward 
    blockAvoidance();
  }

  //Blue block collected and go to the left side
  if(hasBlueBlock && !leftSide){
    if(checkTunnel()){  //use ultrasonic sensor to estimate location of tunnel
      if(right_sensor_state == 1){ //arrive at centre of the tunnel
        //make the right turn

        while(true){
          //line follow to the left side of the tunnel
      
          //Y turn reached
          if(ultrasonicFront < 100){
            //make the slow right turn
            motorR->setSpeed(100);
            motorL->setSpeed(150);
            delay(20);
            rightSide = true;
            break;
          }
        }
        while(true){
          //line follow to the T turn on the left

           //T turn on the left
          if(right_sensor_state == 1 && left_sensor_state == 1)
          { 
             delay(10);
             //set motor to turn 90 degrees to right
             //use OPB line sensor to stop turning once left sensor is over the line
             //or use magnet sensor to make motor turn for certain number of turns or ultrasonic distance sensor to check distance
             while(true){
                motorR->setSpeed(0);
                motorL->setSpeed(turn_speed);
                delay(turn_delay);
                left_sensor_state = analogRead(left_sensor_pin);
                if(left_sensor_state < 500){
                  break;
                }
             }
          }
          break;       
        }        
      }    
    } 
  }

  
  //Blue delivery target detection
  if(right_sensor_state == 1 && left_sensor_state == 1 && hasBlueBlock && leftSide){
    motorR->setSpeed(0);
    motorL->setSpeed(0);
    
    //the block can be placed on the target
    
    //block is placed and the robot go around the block to move forward 
    blockAvoidance();
  }

  //T turn on the left after blue block placed
  if(right_sensor_state == 1 && leftSide && !hasBlueBlock){
    //make the right turn and follow the line to go back to the right side

    
  }



  

  
  
}
