#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Get motor in port 1 & 2
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
Adafruit_DCMotor *pincerL = AFMS.getMotor(3);
Adafruit_DCMotor *pincerR = AFMS.getMotor(4);
//variable for line following
  int vSpeedRight = 115;        // MAX 255
  int vSpeedLeft = 100;
  int turn_speed = 240;
  int turn_speed_offwheel = 30;// MAX 255 
  int turn_delay = 10;

// Variables for pincers
  int pincer_speed = 80;
  int pincer_delay = 1000;

//OPB line Sensor Connection
const int left_sensor_pin = A0;
const int right_sensor_pin = A1;
const int amber_pin = 12, red_pin = 10,blue_pin = 11;

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
boolean leftSide = false;

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

const int Front_sensor_pin = 8; // Set to real pin
int distanceBlock;

// Sensors for colour test
int sensorPin0 = A0;    // select the input pin for the potentiometer
int sensorValue0 = 0;  // variable to store the value coming from the sensor
int sensorPin1 = A1;    // select the input pin for the potentiometer
int sensorValue1 = 0;  // variable to store the value coming from the sensor
int sensorPin2 = A2;    // select the input pin for the potentiometer
int sensorValue2 = 0;  // variable to store the value coming from the sensor
int red_blue = 0;

// LED variables
const int RED = 1, BLUE = 2, AMBER = 3;
int current_LED = 12;
unsigned long two_hz_delay = 0;
boolean AmberON = false, On = true, BlueOn = false;
void setup() {
  Serial.begin(9600);           
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  motorL->setSpeed(vSpeedLeft);
  motorR->setSpeed(vSpeedRight);
  pincerL->setSpeed(pincer_speed);
  pincerR->setSpeed(pincer_speed);
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
  // turn on motor
  motorL->run(RELEASE);
  motorR->run(RELEASE);
  pinMode(7, OUTPUT); // For colour test
  pinMode(amber_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);
  pinMode(red_pin, OUTPUT);
}

void loop() {
  uint8_t i;

  //if a if statement is matched, noAction is set to zero
  noAction++;

  //avoid blocks
  //possibly need also include Linefind in the block avoidance 
  blockAovidance();

  //pick up block, need to check if block returned to the line after pick up
  blockPickUp();

  //the robot hit the wall, not sure which sensor is used to do this
  
  

  //basic line following function(finished)
  lineFollowMain();
  
    
  //both sensor over line, no block is collected & robot on the leftSide, could possibly be used to detect T turn after the tunnel
  if(checkHorizontalLine() && !hasRedBlock && !hasBlueBlock && leftSide)
  { 
    //could also use ultrasonic sensor for locating T turn (may not be possible)

     delay(10);
     //set motor to turn 90 degrees to right
     //use OPB line sensor to stop turning right sensor first detects the line and goes off line again
     //or use magnet sensor to make motor turn for certain number of turns or ultrasonic distance sensor to check distance
    tTurn(0);
    leftSide = false;
  }

  //Red delivery target detection
  if(checkHorizontalLine() && hasRedBlock){
    motorR->setSpeed(0);
    motorL->setSpeed(0);
    
    //the block can be placed on the target
    blockPlacing();
    
    //block is placed and the robot go around the block to move forward 
    blockAvoidance();
  }

  //Blue block collected and go to the left side
  if(hasBlueBlock && !leftSide){
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
            if(analogRead(side_sensor_pin) - side_sensor_history > 300)){
              //make the slow right turn
              motorR->setSpeed(100);
              motorL->setSpeed(150);
              delay(20);
              rightSide = true;
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

  
  //Blue delivery target detection
  if(checkHorizontalLine() && hasBlueBlock && leftSide){
    motorR->setSpeed(0);
    motorL->setSpeed(0);
    
    //the block can be placed on the target
    blockPlacing();
    
    //block is placed and the robot go around the block to move forward 
    blockAvoidance();
  }

  //T turn on the left after blue block placed
   //T turn on the left after blue block placed
  if(checkTturnLeft()){

     if(!hasBlueBlock && leftSide){
      if(true){  //use ultrasonic sensor to estimate location of tunnel
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
  }
}

// check horizontal line in front of robot, take into account robot may not move in straight line
boolean checkHorizontalLine(){
  return (left_sensor_state == 1 && right_sensor_state == 1) || (left_sensor_state_prev == 1 && right_sensor_state == 1) || (left_sensor_state == 1 && right_sensor_state_prev == 1);
}

//use ultrasonic sensor to check tunnel
boolean checkTunnel(){
  
}

//use ultrasonic sensor to check the T turn after blue block is placed on the left
//so that robot can return to right side
boolean checkTturnLeft(){

  
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
    if(sensor_count_left == 10){
      left_sensor_state_prev = left_sensor_state;
      sensor_count_left = 0;
    }else{
      sensor_count_left++;
    }   
  }

  if(right_sensor_state == 1){
    right_sensor_state_prev = 1;
  }else{
    if(sensor_count_right == 10){
      right_sensor_state_prev = right_sensor_state;
      sensor_count_right = 0;
    }else{
      senso_count_right++;
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
  if(right_sensor_state == 0 && left_sensor_state == 0)
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

void blockPlacing(){
    motorL->run(RELEASE); // Stops motors then picks up blocks
    motorR->run(RELEASE);
    pincerL->run(pincer_speed); // Picks up blocks
    pincerR->run(pincer_speed);
    delay(pincer_delay);

    //change LED
    
    //block placed, move backward a bit
    motorL->run(BACKWARD); // Sets motors going backward 
    motorR->run(FORWARD);
    delay(200);

    motorL->run(FORWARD); // Sets motors going forward
    motorR->run(BACKWARD);
}







void blockPickUp(){ // Assuming this is also used for ditection
  distanceBlock = analogRead(Front_sensor_pin);
  if(distanceBlock < 20 && not hasBlueBlock && not hasRedBlock){ // Unlikly distance block is correct
      motorL->run(RELEASE); // Stops motors then picks up blocks
      motorR->run(RELEASE);
      pincerL->run(pincer_speed); // Picks up blocks
      pincerR->run(pincer_speed);
      delay(pincer_delay);
      digitalWrite(7, HIGH); // turn blue LED on
      // Might need a delay
      sensorValue0 = analogRead(sensorPin0);
//  Serial.println(sensorValue0);
  
      sensorValue1 = analogRead(sensorPin1);
 // Serial.println(sensorValue1);
  
      sensorValue2 = analogRead(sensorPin2);
 // Serial.println(sensorValue2);
      if ((sensorValue1) > sensorValue2){
        LED_Change(RED,ON);
      }
      else{
        LED_Change(BLUE,ON);
      }

      Serial.print(red_blue);
      motorL->run(FORWARD); // Sets motors going forward again
      motorR->run(BACKWARD);
  }
}
// Function to change the LED's colour - quick and dirty improve over weekend
void LED_Change(int colour, boolean ONf){
 if (colour == RED){
  current_LED = red_pin;
 }
 if (colour == BLUE){
  current_LED = blue_pin;
 }
 if(ONf){
  digitalWrite(current_LED, HIGH);
 }
 else{
  digitalWrite(current_LED, LOW);
 } 
}

// Function to make the LED flash - if there is a long sub function will need to add this to that to
void LED_Flash(){
  
  if(millis() - two_hz_delay >= 250){
    if(AmberON){
      digitalWrite(amber_pin, LOW);
      Serial.println("OFF");
      AmberON = false;
    }
    else{
      digitalWrite(amber_pin, HIGH);
      Serial.println("ON");
      AmberON = true;
    } 
    two_hz_delay = millis();
  }
}
