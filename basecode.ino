// Base code.

#include <ECE3.h>
uint16_t sensorValues[8];

int blacki = 0; //counts amount of times it hits the black edge
int hasStopped=0; //if the car did the 180 already, stop car on next black strip

int starti = 0; //counts start in order to initialize prior sensor error to current sensor error at start 
                //starti also generally counts the amount of time that has passed

float prior_sensorError = 0; //previous reading of sensor error for Kd
                                // 1000 provides slight variance but not as dramatic as 0

const float minValues[8] = {619, 504, 527, 572, 494.8, 545.4, 527, 683.4};
const float maxValues[8] = {1881,  1996,  1973,  1608,  1551.8, 1954.6, 1939.4, 1816.6};
float Kp = -0.034; //Kp at the start of the race
const float Kd = -0.11; //Kd throughout race
int baseSpeed = 65;

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

//debugging LED
const int LED = 41;


void setup() {
// put your setup code here, to run once:
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

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  ECE3_Init(); //initialize ECE.h
  Serial.begin(9600);  // set the data rate in bits/second for serial data transmission
  delay(3000); //Wait 3 seconds before starting

}

void loop() {
  // put your main code here, to run repeatedly:

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
    motorR = baseSpeed - motorV; //necessary sign change to make right of track and left of track situations change the motor speed correctly
    motorL = baseSpeed + motorV;
    analogWrite(PWML, motorL);
    analogWrite(PWMR, motorR);
    prior_sensorError = sensorError;
    starti++;
    if((starti>50&starti<100)||(starti>142))
    {
     countBlack(sensorValues); //function that checks if on black strip
     check180(); //checks for end of track, if at end executes 180, or if back at start it stops the car
     baseSpeed=55;
     digitalWrite(LED,HIGH);
    }
    else{
      digitalWrite(LED,LOW);
      baseSpeed=65;
    }
    blacki = 0;
    delay(50);
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
  if(blackAddCounter >19000)
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

  analogWrite(PWML,(.5*baseSpeed));
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
  analogWrite(PWML,70);
  analogWrite(PWMR,70);
  delay(650);
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


