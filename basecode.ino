// Base code.
  
// *  You will need to add more code to
// *  make the car do anything useful. 
// 

//#include <ECE3_LCD7.h>

//uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29; //direction pin, wheel going forward or backwards
const int left_pwm_pin=40; //this is the motor, apply a certain speed to it in analog Write

const int LED_RF = 41;

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
//  ECE3_Init(); //i guess this initalizes the ECE.h stuff

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  int leftSpd = 70;

//  ECE3_read_IR(sensorValues);

  analogWrite(left_pwm_pin,leftSpd);

// 
  
//  ECE3_read_IR(sensorValues);

  digitalWrite(LED_RF, HIGH);
  delay(250);
  digitalWrite(LED_RF, LOW);
  delay(250);
    
  }
