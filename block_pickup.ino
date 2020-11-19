void blockPickUp(){
  //check block infront & robot has no block

  //pick up the block


  //return to line
  //first reverse a bit
  motorR->setSpeed(20);
  motorL->setSpeed(20);
  motorL->run(BACKWARD);
  motorR->run(BACKWARD);
  delay(10);
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  for(int i = 0; i<10; i++){
    //turn left a bit
    motorR->setSpeed(30);
    motorL->setSpeed(0);
    delay(5);
    //if line is detected, return to the main loop
    if(right_sensor_state == 1 || left_sensor_state == 1){
      return;
    }
  }

  for(int i = 0; i<20; i++){
    //turn right a bit
    motorR->setSpeed(0);
    motorL->setSpeed(30);
    delay(5);
    //if line is detected, return to the main loop
    if(right_sensor_state == 1 || left_sensor_state == 1){
      return;
    }
  }

 
}
