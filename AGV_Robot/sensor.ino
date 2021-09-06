void sensor()   // make a measurement
{  
  remeasure:  
  digitalWrite(trigPin, LOW); 
  
  delayMicroseconds(2); 
  
  digitalWrite(trigPin, HIGH);  
  
  delayMicroseconds(10); 
  
  digitalWrite(trigPin, LOW); 
  
  int duration1 = pulseIn(echoPin, HIGH);  
   // convert ToF to distance in cm
  double distance1 = duration1/58.3; 
  double result1 = distance1;
  if(distance1 < 250)
  {
    cells_in[input_A_toCell] = result1/1000;  Serial.print(" A= ");
    
    Serial.print(cells_in[input_A_toCell],5);
  }
  else
  {
    goto remeasure;   
  }   
}
