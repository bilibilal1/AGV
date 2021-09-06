
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
  
Startup();

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
