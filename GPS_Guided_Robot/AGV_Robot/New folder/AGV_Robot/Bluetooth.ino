//**************************************************************************************************************************************************
// This procedure reads the serial port - Serial1 - for bluetooth commands being sent from the Android device

void bluetooth()
{
 while (Serial1.available())                                    
 {  
  {  
      str = Serial1.readStringUntil('\n');                      
                                       
  } 
    
    blueToothVal = (str.toInt());                               
    Serial.print("BlueTooth Value ");
    Serial.println(blueToothVal);    

// **************************************************************************************************************************************************



 switch (blueToothVal) 
 {
   
     case 1:                                
        Serial1.println("Forward");
        Forward();
        break;

      case 2:                 
        Serial1.println("Reverse");
        Reverse();
        break;

      case 3:         
        Serial1.println("Left");
        LeftTurn();
        StopCar();
        break;
        
      case 4:                     
        Serial1.println("Right");
        RightTurn();
        StopCar();
        break;
        
      case 5:                                            
        Serial1.println("Stop Car ");
        StopCar();
        break; 

      case 6:                 
        setWaypoint();
        break;
      
      case 7:        
        goWaypoint();
        break;  
      
      case 8:        
        Serial1.println("Turn Around");
        turnAround();
        break;
      
      case 9:        
         Serial1.println("Compass Forward");
        setHeading();
        Compass_Forward();
        break;
      
      case 10:
        setHeading();
        break; 

      case 11:
         gpsInfo();
        break;
      
      case 12:  
        Serial1.println("Compass Turn Right");
        CompassTurnRight();
        break;
      
      case 13:  
        Serial1.println("Compass Turn Left");
        CompassTurnLeft();
        break;
        
      case 14:  
        Serial1.println("Calibrate Compass");
        calibrateCompass();
        break;

      case 15:  
        pingToggle();
        break;  

      case 16:
        clearWaypoints();
        break;  

      case 17:                    // finish with waypoints
        ac = 0;
        Serial1.print("Waypoints Complete");
        break;

      case 18:
     sensor(); load(); outvalue(); serial_print_out();  drive(); }
      break;

      case 19:                 
        setStartMap();
        break;
      
      case 20:        
        setFinishMap();
        break;         

       case 21:
         goMap();
       break; 
      

 } 

// **************************************************************************************************************************************************  
// Slider Value for Speed

if (blueToothVal)                                    
  {    
   //Serial.println(blueToothVal);
  if (blueToothVal >= 1000)
{
    Serial1.print("Speed set To:  ");
    Serial1.println(blueToothVal - 1000);
    turn_Speed = (blueToothVal - 1000); 
    Serial.println();
    Serial.print("Turn Speed ");
    Serial.println(turn_Speed);
 } 
  }  

/*

// **************************************************************************************************************************************************
// Detect for Mines - Sweep Not Used

 else if (blueToothVal== 15)                                   
  {    
    Serial1.println("Detecting");
    sweep();
  }

// **************************************************************************************************************************************************  
*/
 }                                                              // end of while loop Serial1 read
 
                                                               // if no data from Bluetooth 
   if (Serial1.available() < 0)                                 // if an error occurs, confirm that the arduino mega board is selected in the Tools Menu
    {
     Serial1.println("No Bluetooth Data ");          
    }
  
} // end of bluetooth procedure
