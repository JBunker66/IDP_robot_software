int sensorPin0 = A0;    // select the input pin for the potentiometer
int sensorValue0 = 0;  // variable to store the value coming from the sensor
int sensorPin1 = A1;    // select the input pin for the potentiometer
int sensorValue1 = 0;  // variable to store the value coming from the sensor
//int sensorPin2 = A2;    // select the input pin for the potentiometer
//int sensorValue2 = 0;  // variable to store the value coming from the sensor
int red_blue = 0;

void setup() {
  Serial.begin(9600); //Start serial and set the correct Baud Rate
  pinMode(7, OUTPUT);
  }

void loop() {
  digitalWrite(7, HIGH); // turn blue LED on
  sensorValue0 = analogRead(sensorPin0);
  
  sensorValue1 = analogRead(sensorPin1);
 
  if ((sensorValue1 - sensorValue0) < 0){
  red_blue = 1;
  digitalWrite(11, HIGH);
  digitalWrite(10, LOW);
  }else{
  red_blue = 0;
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
  }

//  Serial.print(red_blue);
 
}
