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
  int vSpeedRight = 115;        // MAX 255
  int vSpeedLeft = 100;
  int turn_speed = 240;
  int turn_speed_offwheel = 30;// MAX 255 
  int turn_delay = 10;

//OPB line Sensor Connection
const int left_sensor_pin = A0;
const int right_sensor_pin = A1;

int left_sensor_state;
int right_sensor_state;
int left_sensor_state_prev;
int right_sensor_state_prev;
int sensor_count_left = 0;
int sensor_count_right = 0;

//robot is set to turn
boolean isTurn = false;
//robot has the red block
boolean hasRedBlock = false;
//robot has the blue block
boolean hasBlueBlock = false;
//robot on the left and use to check if robot has made the T turn after the tunnel on the right
boolean leftSide = true;

//could be used to determine whether the robot is lost, inAction++ if no if statement is matched in a loop;
int noAction = 0;

// Standard code above
int tunnel_delay = 0;
// Replace with real sensor
int side_sensor_pin = 3, side_sensor_history; // Arbitrary value
boolean Tunnel_near = false;
unsigned long delayStart = 0, delayPause; // the time the delay started
bool delayRunning = true; // true if still waiting for delay to finish
int line_detector_history, tunnel_counter = 0;


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
  uint8_t i;

  //basic line following function(finished)
  lineFollowMain();
  
    
  //both sensor over line, no block is collected & robot on the leftSide, could possibly be used to detect T turn after the tunnel
  if(checkHorizontalLine() && !hasRedBlock && !hasBlueBlock && leftSide)
  { 
    //could also use ultrasonic sensor for locating T turn (may not be possible)

     delay(10);
     //set motor to turn 90 degrees to right
     
    tTurn(0);
    leftSide = false;
  }

  //Blue block collected and go to the left side
  if(!leftSide){
      if (tunnel_delay = 10){// Dont want to check every loop so difference is clear, but short enough that corners dont trigger
      if((side_sensor_history - analogRead(side_sensor_pin)) < -450){ // Distance but might have to use voltage
        Tunnel_near = true;
        delayRunning = true;
        delayStart = millis(); // Start delay
      }
        side_sensor_history = analogRead(side_sensor_pin);
        tunnel_delay = 0;
      }else{
        tunnel_delay += 1;
     } 

    if (delayRunning && ((millis() - delayStart) >= 10000)) {
    delayRunning = false; // // prevent this code being run more then once
    Tunnel_near = false;
    }  
     
    if(Tunnel_near){  //use ultrasonic sensor to estimate location of tunnel
      if(right_sensor_state == 0){ //arrive at centre of the tunnel
        //make the right turn
        // Reverse code here if needed. Assuming not
        delayPause = millis();// Pause timer
        motorL->run(RELEASE);
        motorR->run(RELEASE);
        motorR->setSpeed(70);
        motorL->setSpeed(135);
        motorL->run(FORWARD);
        motorR->run(FORWARD);
        line_detector_history = digitalRead(1); // Actualy = digital read of line sensor
        for(int i = 0;  i < 40; i = i+1){
           // 8 second delay roughly 90 degrees so go for 10 for 100 degree
           delay(250); // 4 times a second should be ok.
           if(digitalRead(1) != line_detector_history){ // Checks to see if state changes and if it does updates the history
             tunnel_counter +=1;
             line_detector_history = digitalRead(1);
           }
           if(tunnel_counter == 3){
            Tunnel_near = true;
            delayRunning = true;
            motorL->run(RELEASE);
            motorR->run(RELEASE);
           //right turn finished, go to the left side
           side_sensor_history = analogRead(side_sensor_pin);
            while(true){
            //line follow to the left side of the tunnel
            lineFollowMain();
            //Y turn reached, use side sensor to check the wall of the tunnel 
            if(analogRead(side_sensor_pin) - side_sensor_history > 300){
              //make the slow right turn
              motorR->setSpeed(100);
              motorL->setSpeed(150);
              delay(20);
              leftSide = true;
              break;
            }else{
              side_sensor_history = analogRead(side_sensor_pin);
            }
           
          } 
       
          while(true){
            //line follow to the T turn on the left
            lineFollowMain();
             //T turn on the left
            if(checkHorizontalLine())
            { 
               delay(10);
               //set motor to turn 90 degrees to right
               //use OPB line sensor to stop turning once left sensor is over the line
               //or use magnet sensor to make motor turn for certain number of turns or ultrasonic distance sensor to check distance
               tTurn(0);
            }
            break;       
              }             
           }
        }
        if(tunnel_counter != 3){
            motorR->setSpeed(155);
            motorL->setSpeed(70);
            motorL->run(BACKWARD);
            motorR->run(BACKWARD);
            tunnel_counter = 0;
            for(int i = 0;  i < 45; i = i+1){
             // 8 second delay roughly 90 degrees so go for 10
             delay(250); // 4 times a second should be ok.
             if(right_sensor_state != line_detector_history){ // current digital read here
               tunnel_counter +=1;
               line_detector_history = digitalRead(left_sensor_pin); // - current digital read
             }
             if(tunnel_counter == 2){
              delayStart = delayPause - delayStart;
              break;
             }
          }
         }     
      }    
    } 
  }

}

// check horizontal line in front of robot, take into account robot may not move in straight line
boolean checkHorizontalLine(){
  return (left_sensor_state == 1 && right_sensor_state == 1) || (left_sensor_state_prev == 1 && right_sensor_state == 1) || (left_sensor_state == 1 && right_sensor_state_prev == 1);
}


//90 degree turn
void tTurn(int i){
   isTurn = true;   
  //right turn
  if(i == 0){
    while(true){
        motorR->setSpeed(70);      
        motorR->run(FORWARD);
        delay(turn_delay);
        
        right_sensor_state = digitalRead(right_sensor_pin);
        if(right_sensor_state == 1){
          while(true){
            motorR->setSpeed(70);      
            motorR->run(FORWARD);
            delay(turn_delay);
            right_sensor_state = digitalRead(right_sensor_pin);
            if(right_sensor_state == 0){
              break;
            }
          }
          break;
        }
     }
  }else{
    //left turn
    while(true){
        motorL->setSpeed(70);      
        motorL->run(BACKWARD);
        delay(turn_delay);
        
        left_sensor_state = digitalRead(left_sensor_pin);
        if(left_sensor_state == 1){
          while(true){
            motorL->setSpeed(70);      
            motorL->run(BACKWARD);
            delay(turn_delay);
            left_sensor_state = digitalRead(left_sensor_pin);
            if(left_sensor_state == 0){
              break;
            }
          }
          break;
        }
     }
  }
}

void lineFollowMain(){
  //read from OPB line sensor
  left_sensor_state = digitalRead(left_sensor_pin);
  right_sensor_state = digitalRead(right_sensor_pin);

  if(left_sensor_state == 1){
    left_sensor_state_prev = 1;
  }else{
    if(sensor_count_left == 15){
      left_sensor_state_prev = left_sensor_state;
      sensor_count_left = 0;
    }else{
      sensor_count_left++;
    }   
  }

  if(right_sensor_state == 15){
    right_sensor_state_prev = 1;
  }else{
    if(sensor_count_right == 10){
      right_sensor_state_prev = right_sensor_state;
      sensor_count_right = 0;
    }else{
      sensor_count_right++;
    }   
  }
  
 
  //check if right sensor over the line, need to turn right
  if(right_sensor_state == 1 && left_sensor_state == 0)
  {
    Serial.println("turning right");
    isTurn = true;
    motorL->setSpeed(turn_speed);
    motorR->setSpeed(turn_speed_offwheel);
    delay(turn_delay);
    }

  //check if left sensor over the line, need to turn right
  if(right_sensor_state == 0 && left_sensor_state == 1)
  {
    Serial.println("turning left");
    isTurn = true;
    motorR->setSpeed(turn_speed);
    motorL->setSpeed(turn_speed_offwheel);
    delay(turn_delay);
    }
    
  //check if motor move in straight line
  if((right_sensor_state == 0 && left_sensor_state == 0) || (checkHorizontalLine() && leftSide == false))
  {
    Serial.println("going forward");
    motorL->run(FORWARD);
    motorR->run(BACKWARD);
    //if motor speed is changed for turn, then set it back to normal speed
    if (isTurn){
      motorR->setSpeed(vSpeedRight);
      motorL->setSpeed(vSpeedLeft);
    }
    isTurn = false;
    delay(turn_delay);
    }
}
