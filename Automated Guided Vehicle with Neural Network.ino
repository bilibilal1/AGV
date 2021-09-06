
//******************************************************************************************************
#include <AFMotor.h>
#include "Wire.h"
//#include "I2Cdev.h"
#include "HMC5883L.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


//******************************************************************************************************
// GPS Variables & Setup

int GPS_Course;
int Number_of_SATS;
TinyGPSPlus gps;

//******************************************************************************************************
// Setup Drive Motors

AF_DCMotor rightBack(2);
AF_DCMotor rightFront(3);
AF_DCMotor leftFront(4);
AF_DCMotor leftBack(1);

int turn_Speed = 175;
int mtr_Spd = 250;

//******************************************************************************************************
// Compass Setup

HMC5883L compass;
int16_t mx, my, mz;
int desired_heading;
int compass_heading;
int compass_dev = 5;
int Heading_A;
int Heading_B;
int pass = 0;

//******************************************************************************************************
// Servo Control

Servo myservo;
int pos = 0;

//******************************************************************************************************
// Ultrasound Sensor Collision Avoidance

boolean pingOn = false;

int trigPin = A8;
int echoPin = A9;
long duration, inches;
int Ping_distance;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 200;

//******************************************************************************************************
// Bluetooth Variables & Setup

String str;
int blueToothVal;
int bt_Pin = 34;

//*****************************************************************************************************
// GPS Locations

unsigned long Distance_To_Home;

int ac = 0;
int wpCount = 0;
double Home_LATarray[50];
double Home_LONarray[50];


int increment = 0;

//********************************************************************************************************
//Neural network initialisation
int angle = 0;

int rightDistance = 0, leftDistance = 0, middleDistance = 0;
int motorOffset = 20;

int modus = 1;  // 0 = training until double success then drive   1 = drive  2 = test motors with serial, input one of the outputs 0-5

const double InitWeights = 0.44;
double epsilon = 0.0076;  // learning rate
double mom = 0.024;
double success = 0.4;   //training loops until failure rate is lower than this
double bias = 1;

int patern = 25;    // number of test paterns in test_in and test_out
int count = 0;     // counter for print serial and view leds in training session
int count_to = 2000;    //cycles to count for serial out

// cells to input from sensor data, done in "sensor"
int input_A_toCell = 0;

int s, p = 0; //  global counters
int out = 0;  //  varable for calculated output

double Error, Err_sum;
double Accum;
double winrate = 0;

const int cell_raster_in = 2 ;    //  inputs

const int cell_raster_out  = 5 ;  //  outputs maybe +1 bias

const int cell_raster_hidden  = cell_raster_out;

double Delta_hidden[cell_raster_hidden];
double Delta_out[cell_raster_hidden];
double cells_in_hidden_weight[cell_raster_in + 1] [cell_raster_hidden]  = {   };
double cells_hidden_out_weight[cell_raster_hidden + 1][cell_raster_out] = {   };
//double cells_hidden_out_weight[cell_raster_out][cell_raster_hidden] = {   };
double cells_in_hidden_weight_change[cell_raster_in + 1] [cell_raster_hidden]  = {   };
double cells_hidden_out_weight_change[cell_raster_hidden + 1][cell_raster_out] = {   };
//double cells_hidden_out_weight_change[cell_raster_out][cell_raster_hidden] = {   };
double cells_in[cell_raster_in] = {   };
double cells_hidden[cell_raster_hidden] = {   };
double cells_out[cell_raster_out]   = {   };

// training data sensor in cm, test_in[patern]
double test_in[25][cell_raster_in] = {
  25, 14,  //1
  41, 33,  //2
  44, 44,  //3
  33, 41,  //4
  14, 25,  //5
  29, 22,  //6
  43, 33,  //7
  80, 90,  //8
  33, 43,  //9
  22, 29,  //10
  35, 26,  //11
  55, 35,  //12
  55, 55,  //13
  35, 55,  //14
  26, 35,  //15
  33, 25,  //16
  44, 32,  //17
  150, 150, //18
  32, 44,  //19
  25, 33,  //20
  38, 23,  //21
  50, 36,  //22
  90, 100, //23
  36, 50,  //24
  23, 38   //25
};

//  training data supposed output  << < | > >>   ,  test_out[patern]
double test_out[25][cell_raster_out] = {
  0, 0, 0, 0, 1, //1
  0, 1, 0, 0, 0, //2
  0, 1, 0, 0, 0, //3
  0, 0, 0, 1, 0, //4
  0, 0, 0, 0, 1, //5
  1, 0, 0, 0, 0, //6
  0, 1, 0, 0, 0, //7
  0, 0, 1, 0, 0, //8
  0, 0, 0, 1, 0, //9
  0, 0, 0, 0, 1, //10
  0, 0, 0, 1, 0, //11
  0, 1, 0, 0, 0, //12
  0, 1, 0, 0, 0, //13
  0, 0, 0, 1, 0, //14
  0, 0, 0, 0, 1, //15
  0, 0, 0, 1, 0, //16
  0, 1, 0, 0, 0, //17
  0, 0, 1, 0, 0, //18
  0, 0, 0, 1, 0, //19
  0, 0, 0, 0, 1, //20
  0, 0, 0, 1, 0, //21
  0, 1, 0, 0, 0, //22
  0, 0, 1, 0, 0, //23
  0, 0, 0, 1, 0, //24
  0, 0, 0, 0, 1 //25
};
int incomingByte = 0;

void setup()
{
  
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  myservo.attach(9);

  // Ultrasound Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Compass
  Wire.begin();
  compass.begin();                                                 // initialize the compass (HMC5883L)
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(67, -50);
  

 myservo.detach(); 
     Serial.println("Pause for Startup... ");
             
     for (int i=5; i >= 1; i--)                       // Count down for X seconds
      {         
        Serial1.print("Pause for Startup... "); 
        Serial1.print(i);
        delay(1000);                                   // Delay for X seconds
      }    
    
  Serial1.println("Searching for Satellites "); 
  Serial.println("Searching for Satellites "); 
      
  while (Number_of_SATS <= 4)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    getGPS();                                         // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       
    bluetooth();                                      // Check to see if there are any bluetooth commands being received     
  }    
  setWaypoint();                                      // set intial waypoint to current location
  wpCount = 0;                                        // zero waypoint counter
  ac = 0;                                             // zero array counter
  
  Serial1.print(Number_of_SATS);
  Serial1.print(" Satellites Acquired");   
  rst();
  seed(); 
  Serial1.print(" Neural network primed")
  
}

// Main Loop

void loop()
{

  bluetooth();                                                     // Run the Bluetooth procedure
  getGPS();                                                        // GPS location
  getCompass();                                                    // Compass Heading
  Ping();                                                          // Ultrasound
  getMap();                                                         //Map function



}

//**************************************************************************************************************************************************
// This procedure reads the serial port - Serial1 - for bluetooth commands being sent from the Android device

void bluetooth()
{
 while (Serial1.available())                                    // Read bluetooth commands over Serial1 // Warning: If an error with Serial1 occurs, make sure Arduino Mega 2560 is Selected
 {  
  {  
      str = Serial1.readStringUntil('\n');                      // str is the temporary variable for storing the last sring sent over bluetooth from Android device
      //Serial.print(str);                                      // for testing purposes
  } 
    
    blueToothVal = (str.toInt());                               //  convert the string 'str' into an integer and assign it to blueToothVal
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
      

 } // end of switch case

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


void Ping()
{ 
  currentMillis = millis();
  
 if ((currentMillis - previousMillis >= interval) && (pingOn == true)) 
 {
  previousMillis = currentMillis;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  
  int inches = (duration / 2) / 74;                       // convert the time into a distance
  Ping_distance == inches;  
 
    if ((inches < 12) && (blueToothVal != 5))
      {
        Serial1.print("Crash! ");
        Serial1.println(inches);
        Reverse();                                        // Quick reverse to Stop quickly
        delay(100);
        StopCar();   
        blueToothVal = 5;                                 // Set bluetooth value to "Stop"
      } 
  
  }       // end if statement
 
}         // end Ping()

void pingToggle()                                         // Turns Collision avoidance on/ off
 {
  if (pingOn == true) {
    pingOn = false;
    Serial1.print("Collision Avoidance OFF");
  }
    else if (pingOn == false) {
    pingOn = true;
    Serial1.print("Collision Avoidance ON");
  }
  
 }

 void calibrateCompass()                                           // Experimental Use Only to Calibrate Magnetometer/ Compass
{
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0; 

  for (int i=1000; i >= 1; i--) 
  {
  Vector mag = compass.readRaw();                                 // Read compass data
  
  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;
  
  offX = (maxX + minX)/2;                                         // Calculate offsets
  offY = (maxY + minY)/2;
  
  delay(10);
  //Serial.print("Compass X & Y offset: ");
  //Serial.print(offX);
  //Serial.print(" ");
  //Serial.print(offY);
  //Serial.print("\n");
  
  }                                                               // end of for loop
  
  StopCar();

  Serial1.print("Compass X & Y offset: ");
  Serial1.print(offX);
  Serial1.print(" ");
  Serial1.print(offY);
  Serial.print("\n");
  compass.setOffset(offX,offY);                                  // Set calibration offset
}
 
 // ************************************************************************************************************************************************* 

void getGPS()                                                 // Get Latest GPS coordinates
{
    while (Serial2.available() > 0)
    gps.encode(Serial2.read());
} 

// *************************************************************************************************************************************************
 
void setWaypoint()                                            // Set up to 5 GPS waypoints
{

//if ((wpCount >= 0) && (wpCount < 50))
if (wpCount >= 0)
  {
    Serial1.print("GPS Waypoint ");
    Serial1.print(wpCount + 1);
    Serial1.print(" Set ");
    getGPS();                                                 // get the latest GPS coordinates
    getCompass();                                             // update latest compass heading     
                                               
    Home_LATarray[ac] = gps.location.lat();                   // store waypoint in an array   
    Home_LONarray[ac] = gps.location.lng();                   // store waypoint in an array   
                                                              
    Serial.print("Waypoint #1: ");
    Serial.print(Home_LATarray[0],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0],6);
    Serial.print("Waypoint #2: ");
    Serial.print(Home_LATarray[1],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[1],6);
    Serial.print("Waypoint #3: ");
    Serial.print(Home_LATarray[2],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[2],6);
    Serial.print("Waypoint #4: ");
    Serial.print(Home_LATarray[3],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[3],6);
    Serial.print("Waypoint #5: ");
    Serial.print(Home_LATarray[4],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[4],6);

    wpCount++;                                                  // increment waypoint counter
    ac++;                                                       // increment array counter
        
  }         
  else {Serial1.print("Waypoints Full");}
}

// ************************************************************************************************************************************************* 

void clearWaypoints()
{
   memset(Home_LATarray, 0, sizeof(Home_LATarray));             // clear the array
   memset(Home_LONarray, 0, sizeof(Home_LONarray));             // clear the array
   wpCount = 0;                                                 // reset increment counter to 0
   ac = 0;
   
   Serial1.print("GPS Waypoints Cleared");                      // display waypoints cleared
  
}

 // *************************************************************************************************************************************************
 
void getCompass()                                               // get latest compass value
 {  

  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
 
  if(heading < 0)
     heading += 2 * M_PI;      
  compass_heading = (int)(heading * 180/M_PI);                   // assign compass calculation to variable (compass_heading) and convert to integer to remove decimal places                                                              

 }

 // *************************************************************************************************************************************************

void setHeading()
                                                                 // This procedure will set the current heading and the Heading(s) of the robot going away and returning using opposing degrees from 0 to 360;
                                                                 // for instance, if the car is leaving on a 0 degree path (North), it will return on a 180 degree path (South)
{
   for (int i=0; i <= 5; i++)                                    // Take several readings from the compass to insure accuracy
      { 
        getCompass();                                            // get the current compass heading
      }                                               
    
    desired_heading = compass_heading;                           // set the desired heading to equal the current compass heading
    Heading_A = compass_heading;                                 // Set Heading A to current compass 
    Heading_B = compass_heading + 180;                           // Set Heading B to current compass heading + 180  

      if (Heading_B >= 360)                                      // if the heading is greater than 360 then subtract 360 from it, heading must be between 0 and 360
         {
          Heading_B = Heading_B - 360;
         }
     
    Serial1.print("Compass Heading Set: "); 
    Serial1.print(compass_heading);   
    Serial1.print(" Degrees");

    Serial.print("desired heading");
    Serial.println(desired_heading);
    Serial.print("compass heading");
    Serial.println(compass_heading);

}

// *************************************************************************************************************************************************
 
void gpsInfo()                                                  // displays Satellite data to user
  {
        Number_of_SATS = (int)(gps.satellites.value());         //Query Tiny GPS for the number of Satellites Acquired 
        Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination    
        Serial1.print("Lat:");
        Serial1.print(gps.location.lat(),6);
        Serial1.print(" Lon:");
        Serial1.print(gps.location.lng(),6);
        Serial1.print(" ");
        Serial1.print(Number_of_SATS); 
        Serial1.print(" SATs ");
        Serial1.print(Distance_To_Home);
        Serial1.print("m"); 
Serial.print("Distance to Home ");
Serial.println(Distance_To_Home);
  
  }

  
void goWaypoint()
{   
 Serial1.println("Go to Waypoint");
//Serial.print("Home_Latarray ");
//Serial.print(Home_LATarray[ac],6);
//Serial.print(" ");
//Serial.println(Home_LONarray[ac],6);   

//Serial1.print("Distance to Home");   
//Serial1.print(Distance_To_Home);

//Serial1.print("ac= ");
//Serial1.print(ac);

 while (true)  
  {                                                                // Start of Go_Home procedure 
  bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
  if (blueToothVal == 5){break;}                                   // If a 'Stop' Bluetooth command is received then break from the Loop
  getCompass();                                                    // Update Compass heading                                          
  getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10
  
  if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
    Serial1.println(F("No GPS data: check wiring"));     
 
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),Home_LATarray[ac],Home_LONarray[ac]);                               //Query Tiny GPS for Course to Destination   
   
   /*
    if (Home_LATarray[ac] == 0) {
      Serial1.print("End of Waypoints");
      StopCar();      
      break;
      }      
   */ 
    if (Distance_To_Home == 0)                                   // If the Vehicle has reached it's Destination, then Stop
        {
        StopCar();                                               // Stop the robot after each waypoint is reached
        Serial1.println("You have arrived!");                    // Print to Bluetooth device - "You have arrived"          
        ac++;                                                    // increment counter for next waypoint
        break;                                                   // Break from Go_Home procedure and send control back to the Void Loop 
                                                                 // go to next waypoint
        
        }   
   
   
   if ( abs(GPS_Course - compass_heading) <= 15)                  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward                                                                  
                                                                  // otherwise find the shortest turn radius and turn left or right  
       {
         Forward();                                               // Go Forward
       } else 
         {                                                       
            int x = (GPS_Course - 360);                           // x = the GPS desired heading - 360
            int y = (compass_heading - (x));                      // y = the Compass heading - x
            int z = (y - 360);                                    // z = y - 360
            
            if ((z <= 180) && (z >= 0))                           // if z is less than 180 and not a negative value then turn left otherwise turn right
                  { SlowLeftTurn();  }
             else { SlowRightTurn(); }               
        } 
    

  }                                                              // End of While Loop

  
}                                                                // End of Go_Home procedure

void Startup()
{
    
}    
 

void Forward()
{
  Ping();
  Serial.println("Forward");
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);                           
  
  motor1.run(FORWARD);                                                         // go forward all wheels 
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

// **********************************************************************************************************************************************************************

void Forward_Meter()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);     
  
  motor1.run(FORWARD);                                                        // go forward all wheels for specified time
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
}

// **********************************************************************************************************************************************************************

void Reverse()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        // Reverse all wheels
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// **********************************************************************************************************************************************************************

void LeftTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        // Turn left
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(100);    
}

// **********************************************************************************************************************************************************************

void RightTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);   

  motor1.run(FORWARD);                                              
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(100);                                                                   //delay for 100ms more responsive turn per push on bluetooth  
}

// **********************************************************************************************************************************************************************


void SlowLeftTurn()
{
   
  motor1.setSpeed(turn_Speed);                                                
  motor2.setSpeed(turn_Speed);                      
  motor3.setSpeed(turn_Speed);                       
  motor4.setSpeed(turn_Speed);        
  
  motor1.run(BACKWARD);                         
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

// **********************************************************************************************************************************************************************

// Turning too fast will over-shoot the compass and GPS course


void SlowRightTurn()
{
   
  motor1.setSpeed(turn_Speed);                                                  
  motor2.setSpeed(turn_Speed);                      
  motor3.setSpeed(turn_Speed);                       
  motor4.setSpeed(turn_Speed);         

  motor1.run(FORWARD);                                                           
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

// **********************************************************************************************************************************************************************

void StopCar()
{
   
  
  motor1.run(RELEASE);                                                         
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
   
}


// Compass Drive Section
// This Portion of code steers the Vehicle based on the compass heading
// **********************************************************************************************************************************************************************

void CompassTurnRight()                                                          // This Function Turns the car 90 degrees to the right based on the current desired heading
{
  StopCar();    
  getCompass();                                                                  // get current compass heading      

  desired_heading = (desired_heading + 90);                                      // set desired_heading to plus 90 degrees 
  if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}       // if the desired heading is greater than 360 then subtract 360 from it

  while ( abs(desired_heading - compass_heading) >= compass_dev)                 // If the desired heading is more than Compass Deviation in degrees from the actual compass heading then
      {                                                                          // correct direction by turning left or right

    getCompass();                                                                // Update compass heading during While Loop
    bluetooth();                                                                 // if new bluetooth value received break from loop        
    if (blueToothVal == 5){break;}                                               // If a Stop Bluetooth command ('5') is received then break from the Loop
        
    if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}     // if the desired heading is greater than 360 then subtract 360 from it                                            
                                                                
    int x = (desired_heading - 359);                                             // x = the GPS desired heading - 360    
    int y = (compass_heading - (x));                                             // y = the Compass heading - x
    int z = (y - 360);                                                           // z = y - 360
            
        if ((z <= 180) && (z >= 0))                                              // if z is less than 180 and not a negative value then turn left 
            {                                                                    // otherwise turn right
              SlowLeftTurn();                            
            } 
            else
            {
              SlowRightTurn();        
            }  
        }    
    {
      StopCar();                                                                  // Stop the Car when desired heading and compass heading match
    }
 }    


// **********************************************************************************************************************************************************************

void CompassTurnLeft()                                                           // This procedure turns the car 90 degrees to the left based on the current desired heading
{
  StopCar();    
  getCompass();                                                                  // get current compass heading                                                                                  
  //desired_heading = (compass_heading - 90);                                    // set desired_heading to compass value minus 90 degrees

  desired_heading = (desired_heading - 90);                                      // set desired_heading to minus 90 degrees
  if (desired_heading <= 0) {desired_heading = (desired_heading + 360);}         // if the desired heading is greater than 360 then add 360 to it
  while ( abs(desired_heading - compass_heading) >= compass_dev)                 // If the desired heading is more than Compass Deviation in degrees from the actual compass heading then
      {                                                                          // correct direction by turning left or right
    getCompass();                                                                // Get compass heading again during While Loop
    bluetooth();                                                                 // if new bluetooth value received break from loop              
    if (blueToothVal == 5){break;}                                               // If a 'Stop' Bluetooth command is received then break from the Loop
    
    if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}     // if the desired heading is greater than 360 then subtract 360 from it                                            
                                                                
    int x = (desired_heading - 359);                                             // x = the desired heading - 360    
    int y = (compass_heading - (x));                                             // y = the Compass heading - x
    int z = (y - 360);                                                           // z = y - 360
        if (z <= 180)                                                            // if z is less than 180 and not a negative value then turn left         
       // if ((z <= 180) && (z >= 0))                                             
            {                                                                    // otherwise turn right
              SlowLeftTurn();                             
            } 
            else
            {
              SlowRightTurn();              
            }  
        }    
    {
      StopCar();                                                                 // Stop the Car when desired heading and compass heading match
    }
 }   

// **********************************************************************************************************************************************************************

void Compass_Forward()                                               
{            
  while (blueToothVal == 9)                                           // Go forward until Bluetooth 'Stop' command is sent

  //while (true)                                                        
   {  
    getCompass();                                                     // Update Compass Heading
    bluetooth();                                                      // Check to see if any Bluetooth commands have been sent
    if (blueToothVal == 5) {break;}                                   // If a Stop Bluetooth command ('5') is received then break from the Loop
    
    if ( abs(desired_heading - compass_heading) <= compass_dev )      // If the Desired Heading and the Compass Heading are within the compass deviation, X degrees of each other then Go Forward
                                                                      // otherwise find the shortest turn radius and turn left or right  
       {
         Forward(); 
         Ping();       
       } else 
         {    
            int x = (desired_heading - 359);                          // x = the GPS desired heading - 360
            int y = (compass_heading - (x));                          // y = the Compass heading - x
            int z = (y - 360);                                        // z = y - 360
                     
            if ((z <= 180) && (z >= 0))                               // if z is less than 180 and not a negative value then turn left 
            {                                                         // otherwise turn right
              SlowLeftTurn();
              Ping();           
            }
            else
            {
              SlowRightTurn();
              Ping(); 
            }              
        } 
 }                                                                   // End While Loop
}                                                                    // End Compass_Forward

// **********************************************************************************************************************************************************************

void turnAround()                                                   // This procedure turns the Car around 180 degrees, every time the "Turn Around" button is pressed
 {                                                                  // the car alternates whether the next turn will be to the left or right - this is determined by the 'pass' variable
                                                                    // Imagine you are cutting the grass, when you get to the end of the row - the first pass you are turning one way and on the next pass you turn the opposite   
    if (pass == 0) { CompassTurnRight(); }                          // If this is the first pass then turn right
    
    else { CompassTurnLeft(); }                                     // If this is the second pass then turn left
      
    //Forward_Meter();                                              // Go forward one meter (approximately)
    StopCar();                                                      // Stop the car
    
       
    if (pass == 0)                                                  // If this is the first pass then Turn Right
      {       
        CompassTurnRight();                                         // Turn right
        pass = 1 ;                                                  // Change the pass value to '1' so that the next turn will be to the left
      }
      
    else 
      {     
         
    if (desired_heading == Heading_A)                               // This section of code Alternates the desired heading 180 degrees
     {                                                              // for the Compass drive forward
      desired_heading = Heading_B;
     }
    else if (desired_heading == Heading_B)
     {
      desired_heading = Heading_A;
     }        
          
        CompassTurnLeft();                                          // If this is the second pass then Turn Left
        pass = 0;                                                   // Change the pass value to '0' so that the next turn will be to the right

      }
      
  Compass_Forward();                                                // Maintain the 'desired heading' and drive forward
}
   
    

  

void sweep()                          // Can be used to simulate Metal Detecting or to rotate Ping Sensor
{ 
  myservo.attach(9); 
  StopCar();
  Forward_Meter();
  StopCar();
    
  for(pos = 60; pos <= 120; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                   // in steps of 1 degree 
    myservo.write(pos);               // tell servo to go to position in variable 'pos' 
    delay(15);                        // waits 15ms for the servo to reach the position 
  } 
  for(pos = 120; pos>=60; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);               // tell servo to go to position in variable 'pos' 
    delay(15);                        // waits 15ms for the servo to reach the position 
  } 

    myservo.write(90);                // tell servo to go to position in variable 'pos' 
    delay(15);                        // waits 15ms for the servo to reach the position 

   myservo.detach();                  // Disengage Servo Motor
} 

void biasrandom()
{
  randomSeed(analogRead(3)); //srand(time(0)); 
  double upper = 900;
  double lower = 0;     
  double tmp = double(random(100))/100;
  bias = 2.0 * (tmp - 0.5 ) * InitWeights;   
}


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

void getMap() {
  // put your setup code here, to run once:

}

void learning()  // train the network
{  
  double sum = 0;

  for(int x = 0 ; x < cell_raster_hidden ; x++ ) 
  {    
    Accum = 0.0 ;
    for(int y = 0 ; y < cell_raster_out ; y++ )
    {
      Accum += cells_hidden_out_weight[x][y] * Delta_out[y] ;
    }
  Delta_hidden[x] = Accum * cells_hidden[x] * (1.0 - cells_hidden[x]) ;
  }
  
  for(int x = 0; x < cell_raster_hidden; x++)
  {    
    cells_in_hidden_weight_change[cell_raster_in][x] =  epsilon * Delta_hidden[x] + mom * cells_in_hidden_weight_change[cell_raster_in][x] ;
    cells_in_hidden_weight[cell_raster_in][x] += cells_in_hidden_weight_change[cell_raster_in][x];
    for(int y = 0; y < cell_raster_in; y++)
    {       
      cells_in_hidden_weight_change[y][x] =  epsilon * test_in[p][y] * Delta_hidden[x] + mom * cells_in_hidden_weight_change[y][x] ;
      cells_in_hidden_weight[y][x] +=  cells_in_hidden_weight_change[y][x];    
    }
  }
  
  for(int x = 0; x < cell_raster_out; x++)
  {
    cells_hidden_out_weight_change[cell_raster_hidden][x] = epsilon *  Delta_out[x] + mom * cells_hidden_out_weight_change[cell_raster_hidden][x] ;
    cells_hidden_out_weight[cell_raster_hidden][x] += cells_hidden_out_weight_change[cell_raster_hidden][x];
    for(int y = 0; y < cell_raster_hidden; y++)
    {     
      cells_hidden_out_weight_change[y][x] =  epsilon * cells_hidden[y] * Delta_out[x]  + mom * cells_hidden_out_weight_change[y][x] ;
      cells_hidden_out_weight[y][x] += cells_hidden_out_weight_change[y][x];
    }
  } 
}

void load()  // load data through the network
{
  //  biasrandom();  // test with a random bias
  double sum = 1.0; 
  int c = 0;
  Error = -4.32153;
  for(int x = 0; x < cell_raster_hidden; x++)  // input        
  {  
    sum = cells_in_hidden_weight[cell_raster_in][x];
    for(int y = 0; y < cell_raster_in; y++)
    {
      sum += cells_in[y] * cells_in_hidden_weight[y][x];         
    }   
    cells_hidden[x] = 1.0/(1.0 + exp(-sum));   //?
  }  
  for(int x = 0; x < cell_raster_out; x++)  // hidden  
  {
    sum = cells_hidden_out_weight[cell_raster_hidden][x];
    for(int y = 0; y < cell_raster_hidden; y++)
    {
      sum += cells_hidden[y] * cells_hidden_out_weight[y][x];          
    }   
    cells_out[x] =  1.0/(1.0 + exp(-sum));
    Delta_out[x] =  (test_out[p][x] - cells_out[x]) * cells_out[x] * (1.0 -cells_out[x]);  
    Error += 0.5 * (test_out[p][x] - cells_out[x]) * (test_out[p][x] - cells_out[x]);
  }
}

void rst()  
{
  for(int c = 0; c < cell_raster_in; c++)
  {  
    cells_in[c] = 0;  
  }
  for(int c = 0; c < cell_raster_hidden; c++)
  {   
    cells_hidden[c] = 0;    
  } 
  for(int c = 0; c < cell_raster_out; c++)
  {  
    cells_out[c] = 0;
  }   
}

void outvalue()  // get the output with the highest value
{
  double o = cells_out[0]; 
  out = 0; 
  for(int i = 0; i < cell_raster_out; i++)
  {    
    if(cells_out[i] >= o)
    {
      o = cells_out[i];
      out = i; 
    }  
  } 
}


void seed() // random seed the weights
{  
  double upper = 1000;
  double lower = 50;
  for(int x = 0; x < cell_raster_in; x++)
  {
  for (int sy = 0 ; sy < cell_raster_hidden; sy++)
    { randomSeed(analogRead(3)); //srand(time(0));       
      double tmp = double(random(100))/100;
      cells_in_hidden_weight[x][sy] = 2.0 * (tmp - 0.5 ) * InitWeights;     
      cells_in_hidden_weight_change [x][sy] = 0.0;
    }
  }
  
  for(int x = 0; x < cell_raster_hidden; x++)
  {
  for (int sy = 0 ; sy < cell_raster_out; sy++)
    { randomSeed(analogRead(A3));        
      double tmp = double(random(100))/100;
      cells_hidden_out_weight[x][sy] = 2.0 * (tmp - 0.5 ) * InitWeights;     
      cells_hidden_out_weight_change[x][sy] = 0.0;
    }
  }       
}

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

//  some serial prints
void line()
{
   Serial.print("----------------------------------------------------------------------------------err-"); Serial.print(Err_sum, 5); Serial.print(" -- winrate "); Serial.println(winrate, 5); 
}

void serial_print_in()
{
  Serial.print("in_wert -->  ");   
  for(int i = 0; i < cell_raster_in; i++)
  {          
    Serial.print(i);  Serial.print(" = "); Serial.print(cells_in[i],5); Serial.print(" - ");
  }
  Serial.println(); Serial.println(); 
}

void serial_print_hidden()
{
  Serial.print("hidden_wert --> "); 
  for(int i = 0; i < cell_raster_hidden +1; i++)
  {
    Serial.print(i); Serial.print(" = "); Serial.print(cells_hidden[i],5); Serial.print(" - ");
  }
  Serial.println(); Serial.println();  
} 

void serial_print_out()
{ 
  Serial.print("  output_wert -->  ");      
  for(int i = 0; i < cell_raster_out; i++)
  {
    Serial.print(i); Serial.print(" = "); Serial.print(cells_out[i],5); Serial.print("    -  ");
  }
  Serial.print(" out -> "); outvalue(); Serial.print(out);Serial.println();  
}

void serial_print_out_soll()
{ 
  Serial.print("  output soll -->  ");      
  for(int i = 0; i < cell_raster_out; i++)
  {    
    Serial.print(i); Serial.print(" = "); Serial.print(test_out[p][i]); Serial.print("       -  ");      
  }
  Serial.print(p+1); 
  Serial.println();  Serial.println(); 
}


void serial_print_hidden_weights()
{
   Serial.println("hidden_weights -->  ");   
   for(int i = 0; i < cell_raster_in; i++)
   { 
      for(int j = 0; j < cell_raster_hidden; j++)
      {
         Serial.print(i); Serial.print(":");     Serial.print(j);  Serial.print(" = "); Serial.print(cells_in_hidden_weight[i][j],5);  Serial.print(" - ");
      }
      Serial.println();
   }
Serial.println();   Serial.println();
}


void serial_print_out_weights()
{
   Serial.println("output_weights -->  ");   
   for(int i = 0; i < cell_raster_out; i++)
   { 
      for(int j = 0; j < cell_raster_hidden; j++)
      {
          Serial.print(i); Serial.print(":");     Serial.print(j);  Serial.print(" = "); Serial.print(cells_hidden_out_weight[j][i],5); Serial.print(" - ");
      }
     Serial.println();
   }
Serial.println();   Serial.println();
}

void serial_in()
{
  if (Serial.available()) {
    out = Serial.read();
    Serial.println(out);
  }
}

void training()
{  
 count++;
  for(p = 0; p < patern; p++)
  {    
    cells_in[input_A_toCell] = test_in[p][0]/1000;
   // serial_print_in();  
    load(); 
    learning();  
    if(count == count_to)
    {   
      serial_print_out();
      serial_print_out_soll();    
   // serial_print_hidden_weights(); 
   // serial_print_out_weights();      
    }
    if(count == patern+count_to) { count = 0; }   
    Err_sum += Error;    
  }
 if(count == 0)         { winrate = winrate - Err_sum; line(); winrate = Err_sum; }
 
 if(Err_sum <= success) { modus = 1; }
  Err_sum = 0.0; 
   
} 
  
    
  



   
 

  
