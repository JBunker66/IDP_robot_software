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

// Standard code above
int tunnel_delay = 0;
// Replace with real sensor
int side_sensor = 36, side_sensor_history; // Arbitrary value
boolean Tunnel_near = false;
unsigned long delayStart = 0; // the time the delay started
bool delayRunning = true; // true if still waiting for delay to finish

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


// Standard code above
  side_sensor_history = side_sensor;
  delayStart = millis();   // start delay
}
void loop() {
  // put your main code here, to run repeatedly:
  if (tunnel_delay = 10){// Dont want to check every loop so difference is clear, but short enough that corners dont trigger
      if((side_sensor_history - side_sensor) < -450){ // Distance but might have to use voltage
        Tunnel_near = true;
        delayRunning = true;
        delayStart = 0;
      }
  }
  else{
    tunnel_delay += 1;
  }
 if (delayRunning && ((millis() - delayStart) >= 10000)) {
    delayRunning = false; // // prevent this code being run more then once
    Tunnel_near = false;
}  
  



















  

}
