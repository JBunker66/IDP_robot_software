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
  int vSpeedLeft = 140;
  int turn_speed = 230;
  int turn_speed_offwheel = 100;// MAX 255 
  int turn_delay = 10;

//OPB line Sensor Connection
const int left_sensor_pin = 0;
const int right_sensor_pin = 1;

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
  //motorL->run(RELEASE);
  //motorR->run(RELEASE);


// Standard code above
  side_sensor_history = analogRead(side_sensor_pin);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (tunnel_delay = 10){// Dont want to check every loop so difference is clear, but short enough that corners dont trigger
      if((side_sensor_history - analogRead(side_sensor_pin)) < -450){ // Distance but might have to use voltage
        Tunnel_near = true;
        delayRunning = true;
        delayStart = millis(); // Start delay
      }
      side_sensor_history = analogRead(side_sensor_pin);
      tunnel_delay = 0;
  }
  else{
    tunnel_delay += 1;
  }
 if (delayRunning && ((millis() - delayStart) >= 10000)) {
    delayRunning = false; // // prevent this code being run more then once
    Tunnel_near = false;
}  
  if (hasBlueBlock && Tunnel_near && right_sensor_state == 0){ // Right detector sensing white
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
        break;
       }
    }
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    // if not on tunnel line go back to original line
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
    // Now should be on a line
    motorR->setSpeed(155);
    motorL->setSpeed(135);
    motorL->run(FORWARD);
    motorR->run(BACKWARD);
    }
    // Can now go to normal line detection
  }
