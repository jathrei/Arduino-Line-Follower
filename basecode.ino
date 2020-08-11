// Base code.

#include <ECE3.h>
uint16_t sensorValues[8];

int blacki = 0; //counts amount of times it hits the black edge
int starti = 0; //counts start in order to initialize prior sensor error to current sensor error at start
float prior_sensorError; //previous reading of sensor error for Kd
                                // 1000 provides slight variance but not as dramatic as 0

const float minValues[8] = {684, 615, 684, 684, 593, 638, 570, 778};
const float maxValues[8] = {1319, 467.2, 917, 1083, 986, 1625, 1599, 1722};
float Kp = -0.002; //Kp at the start of the race
const float Kd = -.01;
const int baseSpeed = 60;

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

  ECE3_Init(); //initialize ECE.h
  Serial.begin(9600);  // set the data rate in bits/second for serial data transmission
  delay(3000); //Wait 3 seconds before starting

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);



}

void loop() {
  // put your main code here, to run repeatedly:

//read in sensor values
    ECE3_read_IR(sensorValues);
    float sensorError = sensor_fusion1514128(sensorValues);//create error value
    if(starti == 0)
    {
      prior_sensorError = sensorError;
    }
    if(starti == 20)
    {
      Kp = -.01;
    }
    //Serial.print(sensorError);
    //Serial.println();
    /*if(sensorError < -6000) //sensor errors of <-6000 indicate black line at end/beg of track
    {
      digitalWrite(LED,HIGH);
      analogWrite(PWML,0);
      analogWrite(PWMR,0);
      delay(5000);
      //do180();
      
    }*/
    motorV = int((Kp*sensorError)+ (Kd*(sensorError - prior_sensorError))); //int motor variance -> addition of Proportional and Derivitive Corrective signals
    //motorV = int(Kp*(sensorError-prior_sensorError));
    motorR = baseSpeed - motorV;
    motorL = baseSpeed + motorV;
    analogWrite(PWML, motorL);
    analogWrite(PWMR, motorR);
    prior_sensorError = sensorError;
    starti++;
    delay(50);

 // analogWrite(left_pwm_pin,leftSpd);

//

//  ECE3_read_IR(sensorValues);

 // digitalWrite(LED_RF, HIGH);
 // delay(250);
  //digitalWrite(LED_RF, LOW);
 // delay(250);

  }

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