
void drive()  //  start driving
{
  int cal = 5;
  switch(out)
  {
    case 0: 
    stop();delay(820);   
    break;
    case 1: 
    back();
    break;
    case 2: 
    left();
    break;
    case 3: 
    right(); 
    break;
    case 4: 
    forward();  
    break;
    //case 5: digitalWrite(motA_1, LOW); digitalWrite(motA_2, HIGH);  digitalWrite(motB_1, LOW); digitalWrite(motB_2, HIGH);   break;   
  }
}


 void forward(){ 
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  Serial.println("Forward");
}

void back() {
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  Serial.println("Back");
}

void left() {
  rightBack.setSpeed(mtr_Spd+turn_Speed);                 //Set the motors to the motor speed
  rightFront.setSpeed(mtr_Spd+turn_Speed);
  leftFront.setSpeed(mtr_Spd+motorOffset+turn_Speed);
  leftBack.setSpeed(mtr_Spd+motorOffset+turn_Speed);
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  Serial.println("Left");
}

void right() {
  rightBack.setSpeed(mtr_Spd+turn_Speed);                 //Set the motors to the motor speed
  rightFront.setSpeed(mtr_Spd+turn_Speed);
  leftFront.setSpeed(mtr_Spd+motorOffset+turn_Speed);
  leftBack.setSpeed(mtr_Spd+motorOffset+turn_Speed);
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  Serial.println("Right");
}

void stop() {
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
  Serial.println("Stop!");
} 
