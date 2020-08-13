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

int blacki = 0; //counts amount of times it hits the black edge
int hasStopped=0; //if the car did the 180 already, stop car on next black strip
int starti = 0; //counts time starting at start
float prior_sensorError = 0; //previous reading of sensor error for Kp
                                
//COMMENT OUT THESE VALUES IF USING AUTOCALIBRATION
//const float minValues[8] = {639, 524, 570, 616, 547, 570, 570, 770.4};
//const float maxValues[8] = {1861, 1976, 1930, 1531, 1602.6, 1930, 1930, 1729.6};

//COMMENT OUT THE BELOW VALUES IF NOT USING AUTOCALIBRATION
float minValues[8] = {0,0,0,0,0,0,0,0};
float maxValues[8] = {0,0,0,0,0,0,0,0}; 

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
  // put your main code here, to run repeatedly:

    int LIGHTCOUNTER = 0;
    ECE3_read_IR(sensorValues); //read in sensor values
    float sensorError = sensor_fusion8421(sensorValues);//create error value
    int P = Kp*sensorError;
    if(starti<2)
    {
      prior_sensorError = 1.2*sensorError;
    }
    int Ddiff = sensorError - prior_sensorError;
    int D = Kd*Ddiff;
    motorV = P+D; //int motor variance -> addition of Proportional and Derivitive Corrective signals
    if(((starti>=8)&&(starti<18))||(starti>120))//this function lowers the motorV value at the CrossRoads so the car doesn't freak out
    {
      motorV = motorV/2;
      LEDSon();
      LIGHTCOUNTER = 1;
    }
    motorR = baseSpeed - motorV; //necessary sign change to make right of track and left of track situations change the motor speed correctly
    motorL = baseSpeed + motorV;
    analogWrite(PWML, motorL);
    analogWrite(PWMR, motorR);
    prior_sensorError = sensorError;
    if((starti>50&starti<110)||(starti>135))
    {
     countBlack(sensorValues); //function that checks if on black strip
     check180(); //checks for end of track, if at end executes 180, or if back at start it stops the car
     baseSpeed=63;
     LEDSon();
    }
    else{
      baseSpeed=75;
      if(LIGHTCOUNTER == 0)
      {
        LEDSoff();
      }
    }
    blacki = 0;
    starti++;
    delay(50);
}


//LED FUNCTIONS
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




//FUNCTIONS FOR 180 AND STOP/////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    //car is complete on the track... celebrate
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
  digitalWrite(DIR_R, LOW); //change car back to normal direction
  analogWrite(PWML, (baseSpeed+2)); //speed up car to make it go straight for a few seconds
  analogWrite(PWMR, (baseSpeed+2));
  delay(100);
  blacki = 0; //reset black reading counter
  starti = 0; //reset position
  hasStopped++;
  
  }
}

void check180()
{
  if(blacki == 2)
    {
      do180();
    }
}

//SENSOR FUSION FUCNTIONS & RUNNING/SPEED FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////////////
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

float sensor_fusion1514128(uint16_t sensorValues[8])
{
  float floatValues[8];
  int counter= 0;
  if(sensorValues[0] > 900)
    {
      for (unsigned char k = 1; k<8; k++)
      {
        counter = counter + sensorValues[k];
      }
      if(counter <5000)
      {
        return -1700; //max value for left of track
      }
    }
  counter = 0;
  if(sensorValues[7] > 900)
    {
      for (unsigned char k = 1; k<8; k++)
      {
        counter = counter + sensorValues[k];
      }
      if(counter <5600)
      {
        return 1800; //max value for left of track
      }
    }
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


//AUTOCALIBRATION CODE
//JAKE REILLY
//ECE3
//UID 004996061
//big autocalibration loop, this basically reads in the bumper values.
//when bumper 3 is pressed, calibrate white space for MIN values 
//when bumper 4 is pressed, calibrate black space and do math for MAX values
//when bumper 5 is pressed, exit loop to start track
//both minValues and maxValues are already in global scope so I can manipulate them with these functions below

void AutoCalibration()
{
  int LOOPER = 0;
  while(LOOPER == 0) //check for first button press
  {
    button3Status = digitalRead(button3);
    if(button3Status == 0) //when button is pressed, it returns 0
    {
      LEDSoff();
      CalibrateWhite(); //find MIN values
      LOOPER = 1;
    }
    delay(50);
  }
  LEDSon();
  while(LOOPER == 1)
  {
    button4Status = digitalRead(button4);
    if(button4Status == 0)
    {
      LEDSoff();
      CalibrateBlack(); //find MAX values
      LOOPER = 2;
    }
    delay(50);
  }
  int ODDCOUNTER = 1;
  while(LOOPER == 2) 
  {
    button5Status = digitalRead(button5);
    if(button5Status == 0)
    {
      LEDSon();
      delay(2000);
      LOOPER = 4; //exit loop
    }
    if(ODDCOUNTER%4 == 0) //blink every 4 seconds
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


void CalibrateWhite() //read in values at WhiteSpace, 40 times, and then find the average of each, set these as MIN values
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
    minValues[i] = (minValues[i])/(float)40; //divide by 40 to get average
  }
  delay(1000); //delay another second 
}

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

