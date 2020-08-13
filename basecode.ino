// Base code.

#include <ECE3.h>
uint16_t sensorValues[8];

//AUTOCALIBRATION PARAMETERS
int button3 = 27;
int button4 = 8;
int button5 = 28;
int LED = 41;
int LED2 = 57;
int LED3 = 58;
int button3Status = 1;
int button4Status = 1;
int button5Status = 1;

//counting paramters
int blacki = 0; //if on all black surface, blacki turns to 2 and kickstarts the 180 process
int hasStopped=0; //if the car did the 180 already, stop car on next black strip
int starti = 0; //counts time starting at start
float prior_sensorError = 0; //previous reading of sensor error for Kp
                                
//COMMENT OUT THESE VALUES IF USING AUTOCALIBRATION
//const float minValues[8] = {639, 524, 570, 616, 547, 570, 570, 770.4};
//const float maxValues[8] = {1861, 1976, 1930, 1531, 1602.6, 1930, 1930, 1729.6};

//COMMENT OUT THE BELOW VALUES IF NOT USING AUTOCALIBRATION
float minValues[8] = {0,0,0,0,0,0,0,0};
float maxValues[8] = {0,0,0,0,0,0,0,0}; 

//Kp,Kd,SPEED values
float Kp = -0.034; //Kp at the start of the race
const float Kd = -0.11; //Kd throughout race
int baseSpeed = 75;

const int nSLPL =31; // nslp ==> awake & ready for PWM
const int DIR_L=29; //direction pin, wheel going forward or backwards
const int PWML=40; //this is the motor, apply a certain speed to it in analog Write
const int nSLPR=11;
const int DIR_R=30;
const int PWMR=39;

//set variable motor integers
int motorR; //represents right motor signal
int motorL; //left motor signal
int motorV;//motor variance added or subtracted to base




void setup() {
//left side set-up
  pinMode(nSLPL,OUTPUT);
  pinMode(DIR_L,OUTPUT);
  pinMode(PWML,OUTPUT);
  digitalWrite(DIR_L,LOW);
  digitalWrite(nSLPL,HIGH);

//right side set-up
  pinMode(nSLPR, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWMR, OUTPUT);
  digitalWrite(DIR_R, LOW);
  digitalWrite(nSLPR, HIGH);

//setup comment ability
  Serial.begin(9600);  // set the data rate in bits/second for serial data transmission
  delay(3000); //Wait 3 seconds before starting
  
//AUTOCALIBRATION SETUP
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(button5, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  LEDSon(); //turns on all LED
 

  ECE3_Init(); //initialize ECE.h
  AutoCalibration();
  delay(3000);
}

void loop() {
    //counter to determine if LEDS on or not
    int LIGHTCOUNTER = 0;

    //read in sensor values, do sensorfusion (either 8-4-2-1 or 15-14-12-8, I have functions for both) 
    //also create P and D values
    //for the first 2 reads (starti <2), make the prior error a multiple of current error - this allows for quick adjustment
    //create motor variable value
    ECE3_read_IR(sensorValues); 
    float sensorError = sensor_fusion8421(sensorValues);
    int P = Kp*sensorError;
    if(starti<2)
    {
      prior_sensorError = 1.2*sensorError;
    }
    int Ddiff = sensorError - prior_sensorError;
    int D = Kd*Ddiff;
    motorV = P+D; 

    //if the car is at the crossroads time possibilites, lower the motorV value by half
    //this helps stabilize the car at this point, since crossroads makes it figdety 
    if(((starti>=8)&&(starti<18))||(starti>120))
    {
      motorV = motorV/2;
      LEDSon();
      LIGHTCOUNTER = 1;
    }

    //PID math
    motorR = baseSpeed - motorV; 
    motorL = baseSpeed + motorV;
    analogWrite(PWML, motorL);
    analogWrite(PWMR, motorR);
    prior_sensorError = sensorError;

    //look to do 180 during the time intervals 2.5 sec to 5 sec and >6.75 seconds
    //also slow the car during this time for more accurate sensor readings
    if((starti>50&starti<110)||(starti>135))
    {
     countBlack(sensorValues); //look for black strip
     check180(); //if at black strip, check and do 180
     baseSpeed=63;
     LEDSon();
    }
    else
    {
      baseSpeed=75;
      if(LIGHTCOUNTER == 0)
      {
        LEDSoff();
      }
    }

    //reset the black line counter to 0
    //add 1 to the time counter
    blacki = 0;
    starti++;
    delay(50);
}



///////////////////
////LED FUNCTIONS//
///////////////////

void LEDSon()
{
  digitalWrite(LED, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
}

void LEDSoff()
{
  digitalWrite(LED, LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
}



///////////////////////////////////
////FUNCTIONS FOR 180 AND STOP/////
///////////////////////////////////

//see if the car is on black strip
void countBlack(uint16_t sensorValues[8])
{
  int blackAddCounter = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    blackAddCounter = blackAddCounter + sensorValues[i];
  }
  if(blackAddCounter >16000)
  {
    blacki=2;
  }
}

void do180()
{
  //this function stops the car when the black counter is greater than 1,
  //resets the black counter and starti time counter
  //turns the car 180
  //starts the car going forward
  //adds slight delay

  analogWrite(PWML,(.5*baseSpeed)); //slow down car first before stop to avoid dramatic stop
  analogWrite(PWMR,(.5*baseSpeed));
  delay(50);
  analogWrite(PWML, 0);
  analogWrite(PWMR, 0);
  delay(50);
  //shift the DIR_L or DIR_R to turn car
  digitalWrite(DIR_R, HIGH);
  if(hasStopped>0)
  {
    //the car has done 180 before, therefore done
    analogWrite(PWML,(.5*baseSpeed));
    analogWrite(PWMR,(.5*baseSpeed));
    delay(50);
    analogWrite(PWML,0);
    analogWrite(PWMR,0);
    digitalWrite(LED,LOW);
    delay(20000);
  }
  else{
  analogWrite(PWML,80);
  analogWrite(PWMR,80);
  delay(640);
  analogWrite(PWML,0);
  analogWrite(PWMR,0);
  delay(50);
  digitalWrite(DIR_R, LOW); 
  //speed up car to make it go straight for a 100ms
  analogWrite(PWML, (baseSpeed+2)); 
  analogWrite(PWMR, (baseSpeed+2));
  delay(100);
  blacki = 0; //reset black reading counter
  starti = 0; //reset position
  hasStopped++; //indicate that a 180 has been done
  
  }
}

void check180()
{
  if(blacki == 2)
    {
      do180();
    }
}



//////////////////////////////////////////////////////////
////SENSOR FUSION FUCNTIONS & RUNNING/SPEED FUNCTIONS ////
/////////////////////////////////////////////////////////

//sensor fusion using 8-4-2-1 weighing method
float sensor_fusion8421(uint16_t sensorValues[8])
{
  float floatValues[8];
  for (unsigned char i = 0; i < 8; i++)
  {
    floatValues[i] = sensorValues[i];
    floatValues[i] = floatValues[i] - minValues[i];
    if(floatValues[i] != 0)
    {
      floatValues[i] = (floatValues[i] * 1000)/maxValues[i];
    }
  }
  float sensorError = (floatValues[0]*-8) + (floatValues[1]*-4) + (floatValues[2]*-2) + (floatValues[3]*-1) + (floatValues[4]*1) + (floatValues[5]*2) + (floatValues[6]*4) + (floatValues[7]*8);
  sensorError = sensorError/4;
  return sensorError;
}


//sensor fusion using 15-14-12-8 weighing method
float sensor_fusion1514128(uint16_t sensorValues[8])
{
  float floatValues[8];
  for (unsigned char i = 0; i < 8; i++)
  {
    floatValues[i] = sensorValues[i];
    floatValues[i] = floatValues[i] - minValues[i];
    if(floatValues[i] != 0)
    {
      floatValues[i] = (floatValues[i] * 1000)/maxValues[i];
    }
  }
  float sensorError = (floatValues[0]*-15) + (floatValues[1]*-14) + (floatValues[2]*-12) + (floatValues[3]*-8) + (floatValues[4]*8) + (floatValues[5]*12) + (floatValues[6]*14) + (floatValues[7]*15);
  sensorError = sensorError/8;
  return sensorError;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////AUTOCALIBRATION CODE
////JAKE REILLY
////ECE3
////UID 004996061
////big autocalibration loop, auto calibrates MIN and MAX values when bumpers are pressed in order of 3-4-5.
////when bumper 3 is pressed, calibrate white space for MIN values 
////when bumper 4 is pressed, calibrate black space and do math for MAX values
////when bumper 5 is pressed, exit loop to start track
////both minValues and maxValues are already in global scope so I can manipulate them with these functions below
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//autocalibration function
//waits for buttons to be pressed
//calls CalibrateWhite() and CalibrateBlack()
void AutoCalibration()
{
  int LOOPER = 0;

  //check for bumper 3 to be pressed
  while(LOOPER == 0) 
  {
    button3Status = digitalRead(button3);
    //if button is pressed, Calibrate
    if(button3Status == 0) 
    {
      LEDSoff();
      CalibrateWhite(); 
      LOOPER = 1;
    }
    delay(50);
  }
  LEDSon();
  
  //check for bumper 4 to be pressed
  while(LOOPER == 1)
  {
    button4Status = digitalRead(button4);
    //if button is pressed, calibrate
    if(button4Status == 0)
    {
      LEDSoff();
      CalibrateBlack(); 
      LOOPER = 2;
    }
    delay(50);
  }

  //ODDCOUNTER blinks the lights ever 200 ms
  //check for bumper 5 to be pressed
  int ODDCOUNTER = 1;
  while(LOOPER == 2) 
  {
    button5Status = digitalRead(button5);
    //if bumper 5 is pressed, set LOOPER to 4
    //exit function
    if(button5Status == 0)
    {
      LEDSon();
      delay(2000);
      LOOPER = 4; 
    }
    if(ODDCOUNTER%4 == 0) 
    {
     LEDSoff();
    }
    else{
     LEDSon();
    }
    delay(50); 
    ODDCOUNTER++;
  }
 LEDSoff();
}


//read in whitespace 40 times
//calibrate minValues
void CalibrateWhite() 
{
  delay(1000);//delay one sec car is settled after pressing button
  for(unsigned char i = 0; i<40; i++) //read in the values and add them 40 times
  {
    ECE3_read_IR(sensorValues);
    for(unsigned char k = 0; k<8; k++)
    {
      minValues[k] = minValues[k] + sensorValues[k];
    }
    delay(50);
  }
  for(unsigned char i = 0; i<8; i++)
  {
    //divide by 40 to get average
    minValues[i] = (minValues[i])/(float)40; 
  }
  delay(1000); 
}


//read in blackspace 40 times
//subtract minValues to create maxValues array
void CalibrateBlack()
{
  delay(1000);//delay one sec car is settled after pressing button
  for(unsigned char i = 0; i<40; i++) //read in the values and add them 40 times
  {
    ECE3_read_IR(sensorValues);
    for(unsigned char k = 0; k<8; k++)
    {
      maxValues[k] = maxValues[k] + sensorValues[k];
      delay(10);//math catches up
    }
    delay(50);
  }
  for(unsigned char i = 0; i<8; i++)
  {
    maxValues[i] = (maxValues[i])/(float)40; //force a float value
    delay(10); //for math to catch up believe it ot not
    maxValues[i] = maxValues[i] - minValues[i];
  }
  delay(1000); //delay another second 
}

