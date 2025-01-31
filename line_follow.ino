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
const int left_sensor_pin = 0;
const int right_sensor_pin = 1;
const int amber_pin = 12, red_pin = 10,blue_pin = 11;

int left_sensor_state;
int right_sensor_state;
int left_sensor_state_prev;
int right_sensor_state_prev;
int sensor_count_left = 0;
int sensor_count_right = 0;

//if robot is set to turn
boolean isTurn = false;
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
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
  // turn on motor
  //motorL->run(RELEASE);
  //motorR->run(RELEASE);
  two_hz_delay = millis();
  pinMode(amber_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);
  pinMode(red_pin, OUTPUT);
}

void loop() {
  lineFollowMain();
  LED_Flash();
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
    if(ON){
      digitalWrite(current_LED, LOW);
      Serial.println("OFF");
      ON = false;
    }
    else{
      digitalWrite(current_LED, HIGH);
      Serial.println("ON");
      ON = true;
    } 
    two_hz_delay = millis();
  }
}
