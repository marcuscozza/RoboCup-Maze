/**
* Author: Marcus 
* Date: 5/01/2020
* Aim: Functions for a range of different components
**/

#include <arduino.h>
#include <Wire.h> 
#include <SoftwareSerial.h>
#include "MeAuriga.h"
#include <millisDelay.h>
millisDelay Delay;

//defines variables for ultrasonic Sensor
//defines Port 6
int port6Trig=64;
int port6Echo=69;
int distanceTopRight;
long durationTopRight;
//defines Port 7
int port7Trig=63;
int port7Echo=68;
int distanceTopLeft;
long durationTopLeft;
//defines Port 8
int port8Trig=62;
int port8Echo=67;
int distanceBottomLeft;
long durationBottomLeft;
//defines Port 10
int port10Trig=60;
int port10Echo=65;
int distanceBottomRight;
long durationBottomRight;

//Delay for moveDirection Function
bool startMoveDelay = true;
bool moveDelayOn = true;

//Declare bool for alignment
bool frontAlign = false;
bool sideAlign = false;


int getUltrasonicBottomLeft(){ //Functions that returns the distance of Ultrasonics in CM for all 4 sides
    pinMode(port8Trig,OUTPUT);
    pinMode(port8Echo,INPUT);
    digitalWrite(port8Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port8Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port8Trig,LOW);
    durationBottomLeft=pulseIn(port8Echo,HIGH);
    distanceBottomLeft=durationBottomLeft*0.0343/2;
    return distanceBottomLeft;
}
int getUltrasonicTopLeft(){ 
    pinMode(port7Trig,OUTPUT);
    pinMode(port7Echo,INPUT);
    digitalWrite(port7Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port7Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port7Trig,LOW);
    durationTopLeft=pulseIn(port7Echo,HIGH);
    distanceTopLeft=durationTopLeft*0.0343/2;
    return distanceTopLeft;
}

int getUltrasonicTopRight(){
    pinMode(port6Trig,OUTPUT);
    pinMode(port6Echo,INPUT);
    digitalWrite(port6Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port6Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port6Trig,LOW);
    durationTopRight=pulseIn(port6Echo,HIGH);
    distanceTopRight=durationTopRight*0.0343/2;
    return distanceTopRight;
}


int getUltrasonicBottomRight(){
    pinMode(port10Trig,OUTPUT);
    pinMode(port10Echo,INPUT);
    digitalWrite(port10Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port10Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port10Trig,LOW);
    durationBottomRight=pulseIn(port10Echo,HIGH);
    distanceBottomRight=durationBottomRight*0.0343/2;
    return distanceBottomRight;
}


void displayText(int topText, int bottomText){ // function which has topText for the display and the bottomText.
  lcd.print("PID: ");
  lcd.print(topText); //display topText
  lcd.setCursor(0,1); //move down one row
  lcd.print("Angle: ");
  lcd.print(bottomText); //displaybottomText
  lcd.setCursor(0,0);
  lcd.home();
}

//PID Declare
int targetValue = 0; //the length away from the wall
int speedValue = 100; // speed of the motors

int threshold = 2; // min distance
int error = 0;
int lastError = 0;
int proportional = 0;
int integral = 0;
int derivative = 0;

int pidValue = 0;


int kP = 4; // PID variables
float kI = 0.01;
float kD = 0.3;



/**function for cqlculating PID value and changing motor speed
 *calculatePIDValue(int target)
 *target |  Value target
  */

int calculatePIDValue(int target){
  pidValue = 0;
   error = target - targetValue;
   if (abs(error) < threshold){
      integral = integral + error;
   }else{
      integral = 0;
      derivative = error - lastError;
      lastError = error;
      pidValue = error*kP + integral*kI + derivative*kD;
   }
   if (pidValue > 150){ // checks if the pidValue is too high/low and sets it to a upper/lower limit
      pidValue = 150;
   }
   else if (pidValue < -150){
      pidValue = -150;
    }
    return pidValue;
    
}




bool limitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(3) == HIGH or digitalRead(2) == HIGH){
    return true;
  }else{
    return false;
  }
}

void checkWall(){
  if(limitSwitch()){
    //go back
  }
  else if (frontUltraSensor.distanceCm() <= 7){  // // checks if front distance is <= 7 then stop motors
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        frontAlignment();
        
        if(getUltrasonicTopRight() >= 8){ // checks which way is clear and turns 
          turnAbsolute(90, true, 150);
        }else if (getUltrasonicTopLeft() >= 8){
          turnAbsolute(-90, true, 150);
        }
        else if (getUltrasonicTopRight() <= 7 and getUltrasonicTopLeft() <= 7){
         turnAbsolute(90, true, 150);
        }
         frontAlign = false;
   }
  
}




//getAbsolute function declare
int turnSpeed; // power at which to turn
int absolute; //getAbsolute() vlaue from Gyro
int target; //set target degress to int
bool resetGyro; // set resetGyro to bool

void turnAbsolute(int target, bool resetGyro, int turnSpeed){ 
  gyro.update(); //Set gyro active
  absolute = gyro.getAngleZ(); // set absolute to angle z
  while (abs(absolute - target) > 0){ //print checks if the target has been reached
    if (absolute > target){  // if absolute > target then turn left
      Encoder_1.setMotorPwm(-turnSpeed);
      Encoder_2.setMotorPwm(-turnSpeed);
    }
    if (absolute < target){ // if absolute > target then turn right
      Encoder_1.setMotorPwm(turnSpeed);
      Encoder_2.setMotorPwm(turnSpeed);
    }
    gyro.update();   //re reads angle z
    absolute = gyro.getAngleZ(); //set absolute to angle z
    }
  Encoder_1.setMotorPwm(0); // turn motors off
  Encoder_2.setMotorPwm(0);
  Encoder_1.updateSpeed(); //update speed
  Encoder_2.updateSpeed();
  if (resetGyro == true){ //checks if resetGyro is true
    gyro.begin(); //RESET GYRO so that after turning it will be on 0 degress
    gyro.update(); //activate gyro
    delay(100); 
  }

}

//limitSwitch function for left and right
bool leftLimitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(2) == HIGH){
    return true;
  }else{
    return false;
  }
}
bool rightLimitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(3) == HIGH){
    return true;
  }else{
    return false;
  }
}



   

// the amount of degress above ambient for the target temp
int targetTempAdd = 5;

float getLeftTemp(){ // function that returns the object temp in C
  mlx.AddrSet(leftTemp); 
  return mlx.readObjectTempC();
}
float getRightTemp(){
  mlx.AddrSet(rightTemp); 
  return mlx.readObjectTempC();
}
float getLeftTempAmbient(){ // function that returns the Ambient temp in C
  mlx.AddrSet(leftTemp); 
  return mlx.readAmbientTempC();
}
float getRightTempAmbient(){
  mlx.AddrSet(rightTemp); 
  return mlx.readAmbientTempC();
}

int dropRescueKit(){  // delcaring function for droping rescue kit
 servo.write(180);  // Pull back to drop rescue kit
 delay(1000);
 servo.write(0);    //push rescue kit
 delay(5000);
}

void showRGBLED(){ //function for turning ring of led on
for (uint8_t t = 0; t < LEDNUM; t++ ){
    led.setColorAt(t, 255, 0, 0);
  led.show();
  }
}
void turnOfRGBLED(){ //function for turning ring of led off
for (uint8_t t = 0; t < LEDNUM; t++ ){
    led.setColorAt(t, 0, 0, 0);
  led.show();
  }
}


void checkTemp(){
  int leftTempTarget = getLeftTempAmbient() + targetTempAdd;
  int rightTempTarget = getRightTempAmbient() + targetTempAdd;
  if(leftTempTarget <= getLeftTemp()){
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    turnAbsolute(90, true, 150);
   // showRGBLED();
    dropRescueKit();
    turnAbsolute(-90, true, 150);
   // turnOfRGBLED();
    //FIX AFTER!: command below will move the robot foward but needs a checkPosition Function added.
    delayOn();
    moveDirection(-100, 100, 2);
    //--------------------------
    
  }
  if(rightTempTarget <= getRightTemp()){
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    turnAbsolute(-90, true, 150);
   // showRGBLED();
    dropRescueKit();
    turnAbsolute(90, true, 150);
   // -turnOfRGBLED();
    //FIX AFTER!: command below will move the robot foward but needs a checkPosition Function added.
    delayOn();
    moveDirection(-100, 100, 2);
    //---------------------------
  }
  
}


/**function for going any direction
 *moveDirection(motor1Speed, motor2Speed, runTime) 
 *motor1Speed = int(the speed of the motor can be + or -)
 *motor2Speed = int(the speed of the motor can be + or -)
 *runTime = int(The amount of time in seoconds the motor will run for)
 */
void moveDirection(int motor1Speed, int motor2Speed, float runTime){
  while (moveDelayOn == true){
   if (startMoveDelay == true){
      Delay.start(runTime * 1000);
      startMoveDelay = false;
      Serial.println("delay start");
    }
    if(Delay.justFinished()){
       moveDelayOn = false;
       Encoder_1.setMotorPwm(0);
       Encoder_2.setMotorPwm(0);
       Serial.println("off");
    }else{
     Encoder_1.setMotorPwm(motor1Speed);
     Encoder_2.setMotorPwm(motor2Speed);
    }
  }
}


//Delay Functions to reset
void delayOn(){
  moveDelayOn = true;
  startMoveDelay = true;
}
void delayOff(){
  moveDelayOn = false;
  startMoveDelay = false;
}


//front alignmnet function
void frontAlignment(){
  if (frontAlign == false){
    if(frontUltraSensor.distanceCm() <= 6){
        while (!leftLimitSwitch() or !rightLimitSwitch()){
           Encoder_1.setMotorPwm(-100);
          Encoder_2.setMotorPwm(100);
          if(leftLimitSwitch()){
             Encoder_1.setMotorPwm(100);
          Encoder_2.setMotorPwm(100);
          }
          if(rightLimitSwitch()){
             Encoder_1.setMotorPwm(-100);
          Encoder_2.setMotorPwm(-100);
          }
        }
        if(rightLimitSwitch() and leftLimitSwitch()) {
         Encoder_1.setMotorPwm(0);
          Encoder_2.setMotorPwm(0);
          moveDirection(100, -100, 0.35);
          delay(100);
          gyro.update();
          delayOn();
          frontAlign = true;
        }
      }
  }
}
