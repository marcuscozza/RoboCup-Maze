/**
* Author: Marcus 
* Date: 4/04/2020
* Aim: Functions for a range of different components
**/

#include <arduino.h>
#include <Wire.h> 
#include <SoftwareSerial.h>
#include "MeAuriga.h"
#include <millisDelay.h>
millisDelay Delay;
millisDelay alignmentDelay;

/**
 * ########################################################################
 * #                       ULTRASONIC DELCARE                             #
 * ########################################################################
**/
//defines variables
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

/**
 * ########################################################################
 * #                       DELAY DEClARE                                  #
 * ########################################################################
**/
//Delay for moveDirection Function
bool startMoveDelay = true;
bool moveDelayOn = true;

bool startAlignDelay = true;
bool alignDelayOn = true;

/**
 * ########################################################################
 * #                       ULTRASONIC FUNCTIONS                           #
 * ########################################################################
**/
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


/**
 * ########################################################################
 * #                       DISPLAY FUNCTION                               #
 * ########################################################################
**/

void displayText(int topText, int bottomText){ // function which has topText for the display and the bottomText.
  lcd.print("PID: ");
  lcd.print(topText); //display topText
  lcd.setCursor(0,1); //move down one row
  lcd.print("Angle: ");
  lcd.print(bottomText); //displaybottomText
  lcd.setCursor(0,0);
  lcd.home();
}

/**
 * ########################################################################
 * #                       PID FUNCTION                                   #
 * ########################################################################
**/

//PID Declare
int targetValue = 0; //the length away from the wall
int speedValue = 100; // speed of the motors

int threshold = 1; // min target
int error = 0;
int lastError = 0;
int proportional = 0;
int integral = 0;
int derivative = 0;

int pidValue = 0;


int kP = 6; // PID variables
float kI = 0.01;
float kD = 0.4;



/**function for calculating PID value and changing motor speed
 *calculatePIDValue(int target)
 *target | ultrasonic Value
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

/**
 * ########################################################################
 * #                       CHECKWALL FUNCTION                             #
 * ########################################################################
**/
void checkWall(){
  if(limitSwitch()){
    Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        delay(200);
    delayOn();
    moveDirection(100, -100, 0.1);
    delayOnAlign();
    sideAlignment();
  }
  else if (topLeftLimitSwitch()){
    Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        delay(200);
     delayOn();
    moveDirection(-100, -100, 0.3);
     delayOnAlign();
    sideAlignment();
  }
  else if (topRightLimitSwitch()){
    Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        delay(200);
    delayOn();
    moveDirection(100, 100, 0.3);
     delayOnAlign();
    sideAlignment();
  }
  else if (frontUltraSensor.distanceCm() <= 7){  // // checks if front distance is <= 7 then stop motors
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        delay(1000);
        frontAlignment(); //align the front of the robot
        if(getUltrasonicTopRight() >= 8){ // checks which way is clear and turns 
          turnAbsolute(90, true, 100);
          delayOnAlign();
          backAlignment();
        }else if (getUltrasonicTopLeft() >= 8){
          turnAbsolute(-90, true, 100);
          delayOnAlign();
          backAlignment();
        }
        else if (getUltrasonicTopRight() <= 8 and getUltrasonicTopLeft() <= 8){
         turnAbsolute(90, true, 100);
         delay(100);
         turnAbsolute(90, true, 100);
         delayOnAlign();
    backAlignment();
        }
   }
   else if (getUltrasonicTopRight() >= 15 and getUltrasonicTopLeft() >= 15 and getUltrasonicBottomRight() >= 15 and getUltrasonicBottomLeft() >= 15){
    Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
      delayOn();
    moveDirection(-100, 100, 0.2);
    delay(200);
    turnAbsolute(-90, true, 100);
    delayOn();
    moveDirection(-100, 100, 0.9);
    delayOnAlign();
    sideAlignment();
   }
  
}


/**
 * ########################################################################
 * #                       TURNABSOLUTE FUNCTION                          #
 * ########################################################################
**/
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

/**
 * ########################################################################
 * #                       LIMITSWITCH FUNCTIONS                          #
 * ########################################################################
**/

//limitSwitch function for Top left and right
bool topLeftLimitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(2) == HIGH){
    return true;
  }else{
    return false;
  }
}
bool topRightLimitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(3) == HIGH){
    return true;
  }else{
    return false;
  }
}
//limitSwitch function for bottom left and right
bool bottomLeftLimitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(6) == HIGH){
    return true;
  }else{
    return false;
  }
}
bool bottomRightLimitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(7) == HIGH){
    return true;
  }else{
    return false;
  }
}
bool limitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(3) == HIGH or digitalRead(2) == HIGH){
    return true;
  }else{
    return false;
  }
}



/**
 * ########################################################################
 * #                       TEMPERATURE DECLARE                            #
 * ########################################################################
**/

// the amount of degress above ambient for the target temp
int targetTempAdd = 8;

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

/**
 * ########################################################################
 * #                       RESCUEKIT AND LED FUNCTION                     #
 * ########################################################################
**/

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


/**
 * ########################################################################
 * #                       CHECKTEMP FUNCTION                             #
 * ########################################################################
**/
void checkTemp(){
  int leftTempTarget = getLeftTempAmbient() + targetTempAdd;
  int rightTempTarget = getRightTempAmbient() + targetTempAdd;
  if(leftTempTarget <= getLeftTemp()){
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    turnAbsolute(90, true, 100);
    showRGBLED();
    dropRescueKit();
    turnAbsolute(-90, true, 100);
    turnOfRGBLED();
    delay(100);
    delayOnAlign();
    sideAlignment();
    delay(200);
    checkWallAfterLeft();
  }
  if(rightTempTarget <= getRightTemp()){
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    turnAbsolute(-90, true, 100);
    showRGBLED(); 
    dropRescueKit();
    turnAbsolute(90, true, 100);
    turnOfRGBLED();
    delay(100);
    delayOnAlign();
    sideAlignment();
    delay(200);
     checkWallAfterRight();
  }
  
}

//Check Wall 
void checkWallAfterLeft(){
   if (frontUltraSensor.distanceCm() <= 7){
      turnAbsolute(90, true, 100);
      delay(100);
    }
    else {
      delayOn();
    moveDirection(-100, 100, 0.5);
    }
}
void checkWallAfterRight(){
   if (frontUltraSensor.distanceCm() <= 7){
      turnAbsolute(-90, true, 100);
      delay(100);
    }
    else {
      delayOn();
    moveDirection(-100, 100, 0.5);
    }
}


/**
 * ########################################################################
 * #                       MOVEDIRECTION FUNCTION                             #
 * ########################################################################
**/

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
    }
    if(Delay.justFinished()){
       moveDelayOn = false;
       Encoder_1.setMotorPwm(0);
       Encoder_2.setMotorPwm(0);
    }else{
     Encoder_1.setMotorPwm(motor1Speed);
     Encoder_2.setMotorPwm(motor2Speed);
    }
  }
}


/**
 * ########################################################################
 * #                       DELAY FUNCTIONS                                #
 * ########################################################################
**/

//move Delay Functions to reset
void delayOn(){
  moveDelayOn = true;
  startMoveDelay = true;
}
void delayOff(){
  moveDelayOn = false;
  startMoveDelay = false;
}

//align delay function reset
void delayOnAlign(){
  alignDelayOn = true;
  startAlignDelay = true;
}

/**
 * ########################################################################
 * #                       ALIGNMENT FUNCTIONS                            #
 * ########################################################################
**/
//topRightLimitSwitch() and topLeftLimitSwitch()
//front alignmnet function
void frontAlignment(){
        if (value[0] == 0 or value[1] == 0){
          delayOn();
          moveDirection(-100, 100, 0.01);
        }
          else if(value[1] == 1){
          //   delayOn();
        //  moveDirection(100, 100, 0.05);
          }
          else if(value[0] == 1){
          //   delayOn();
          //  moveDirection(-100, -100, 0.05);
          }
        
       else if(value[0] == 1 and value[1] == 1) {
          Encoder_1.setMotorPwm(0);
          Encoder_2.setMotorPwm(0);
          delay(1000);
         // delayOn();
         // moveDirection(100, -100, 0.20);
          delay(100);
          gyro.update();
          //delayOnAlign();
          //frontAlignmentIR();
          delayOnAlign();
          sideAlignment();
        }

/*
 if (value[1] == 0 or value[0] == 0){
           Encoder_1.setMotorPwm(-50);
          Encoder_2.setMotorPwm(50);
          if (value[2] == 1){
             Encoder_1.setMotorPwm(70);
          Encoder_2.setMotorPwm(70);
          }
          if (value[1] == 1){
             Encoder_1.setMotorPwm(-70);
          Encoder_2.setMotorPwm(-70);
          }
   }
  else if(value[1] == 1 and value[2] == 1) {
       Encoder_1.setMotorPwm(0);
          Encoder_2.setMotorPwm(0);
          delayOn();
          moveDirection(-100, 100, 0.10);
          delay(1000);
          gyro.update();
          delayOnAlign();
          sideAlignment();
        
    }
   */

}





void leftAlignment(){
  /** fix after, need adjustment when stuck.
   *  if(getUltrasonicTopLeft() <= 4 and getUltrasonicBottomLeft() <= 4){
    delayOn();
    moveDirection(150, 0, 1);
    delayOn();
    moveDirection(0, 150, 0.5);
    delayOn();
    moveDirection(-100, -100, 1);
    
}
**/
 if (getUltrasonicTopLeft() > getUltrasonicBottomLeft()){
  //turn left
  delayOn();
  moveDirection(-100, -100, 0.02);
}if (getUltrasonicTopLeft() < getUltrasonicBottomLeft()){
  //turn right
  delayOn();
  moveDirection(100, 100, 0.02);
}
}
void rightAlignment(){
   /** fix after, need adjustment when stuck.
   if(getUltrasonicTopRight() <= 4 and getUltrasonicBottomRight() <= 4){
     delayOn();
    moveDirection(-150, 0, 1);
    delayOn();
    moveDirection(0, -150, 0.5);
     delayOn();
   moveDirection(100, 100, 1);
}
**/
if (getUltrasonicTopLeft() > getUltrasonicBottomLeft()){
  //turn left
 delayOn();
  moveDirection(100, 100, 0.02);
}if (getUltrasonicTopRight() < getUltrasonicBottomRight()){
  //turn right
 delayOn();
  moveDirection(-100, -100, 0.02);
}
}

void sideAlignment(){
  while (alignDelayOn == true){
   if (startAlignDelay == true){
      alignmentDelay.start(600);
      startAlignDelay = false;
    }
    if(alignmentDelay.justFinished()){
       alignDelayOn = false;
       Encoder_1.setMotorPwm(0);
       Encoder_2.setMotorPwm(0);
       delay(100);
       break;
    }else{
     if(getUltrasonicTopRight() <=6){
    rightAlignment();
  }
  if(getUltrasonicTopLeft() <=6){
    leftAlignment();
      }
    }
  }
}





void backAlignment(){
  while (alignDelayOn == true){
   if (startAlignDelay == true){
      alignmentDelay.start(600);
      startAlignDelay = false;
    }
    if(alignmentDelay.justFinished()){
       alignDelayOn = false;
       Encoder_1.setMotorPwm(0);
       Encoder_2.setMotorPwm(0);
       delay(100);
       break;
    }else{

        if (!bottomLeftLimitSwitch() or !bottomRightLimitSwitch()){
           Encoder_1.setMotorPwm(100);
          Encoder_2.setMotorPwm(-100);
          if(bottomLeftLimitSwitch()){
             Encoder_1.setMotorPwm(130);
          Encoder_2.setMotorPwm(130);
          }
          if(bottomRightLimitSwitch()){
             Encoder_1.setMotorPwm(-130);
          Encoder_2.setMotorPwm(-130);
          }
        }
        if(bottomRightLimitSwitch() and bottomLeftLimitSwitch()) {
          Encoder_1.setMotorPwm(0);
          Encoder_2.setMotorPwm(0);
          delayOn();
          moveDirection(-100, 100, 0.20);
          delay(100);
          gyro.update();
          //delayOnAlign();
          //frontAlignmentIR();
          delayOnAlign();
          sideAlignment();
        }
    }
  }
/*
        while (value[3] == 0 or value[4] == 0){
           Encoder_1.setMotorPwm(100);
          Encoder_2.setMotorPwm(-100);
          if(value[4] == 1){
             Encoder_1.setMotorPwm(130);
          Encoder_2.setMotorPwm(130);
          }
          if(value[3] == 1){
             Encoder_1.setMotorPwm(-130);
          Encoder_2.setMotorPwm(-130);
          }
        }
        if(value[3] == 1 and value[4] == 1) {
          Encoder_1.setMotorPwm(0);
          Encoder_2.setMotorPwm(0);
          //delayOn();
          //moveDirection(-100, 100, 0.20);
          delay(100);
          gyro.update();
          //delayOnAlign();
          //frontAlignmentIR();
          delayOnAlign();
          sideAlignment();
        }
        */
}
/**
 * ########################################################################
 * #                       I2C FUNCTION For Arduino Nano                  #
 * ########################################################################
**/


byte readI2C(int address){
  byte bval;
  long entry = millis();

  Wire.requestFrom(address, 1);

  while (Wire.available() == 0 && (millis() - entry) < 100) Serial.print("Waiting");

  if (millis() - entry < 100) bval = Wire.read();
  return bval;
}

void getValues(){
  float RGBRatio = 0.00;
  float RGBTotal = 0.00;

  while (readI2C(SLAVE_ADDR) < 255){
    Serial.print("Waiting");
  }
  for (bcount = 0; bcount < 7; bcount++){
    value[bcount] = readI2C(SLAVE_ADDR);
  }
  for (int i = 0; i < 7; i++){
    Serial.print(value[i]);
    Serial.print("\t");
  }
  Serial.println();
  for (int k = 4; k< 7; k++){
      RGBTotal += value[k];
    }
  delay(100);
  RGBRatio = (value[4] / RGBTotal);
  Serial.println(RGBTotal);
  Serial.println(RGBRatio);

  //FIX BLACK
  if (value[4] <= 5 and value[5] <= 5 and value[6] <=6){
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    delay(300);
    
    delayOn();
    moveDirection(100, -100, 0.9);
    delay(100);
    if(getUltrasonicTopRight() >= 8){ // checks which way is clear and turns 
          turnAbsolute(90, true, 100);
          delayOn();
    moveDirection(-100, 100, 0.9);
    delayOnAlign();
    sideAlignment();
        }else if (getUltrasonicTopLeft() >= 8){
          turnAbsolute(-90, true, 100);
          delayOn();
    moveDirection(-100, 100, 0.9);
    delayOnAlign();
    sideAlignment();
        }
        else if (getUltrasonicTopRight() <= 7 and getUltrasonicTopLeft() <= 7){
         turnAbsolute(90, true, 100);
         delayOnAlign();
          backAlignment();
          delayOnAlign();
          sideAlignment();
        }
        
  }
}


/**
 * ########################################################################
 * #                       RAMP FUNCTION                                  #
 * ########################################################################
**/


void Ramp(){
  int GyroValue = 10;
  gyro.update();
  int GyroX = abs(gyro.getAngleX());
  turnOfRGBLED();
  while (GyroX >= GyroValue){
    showRGBLED();
    int desiredLength = 5; //the length away from the wall
    int speedValue = 70; // speed of the motors
    
    int threshold = 4; // min distance
    int error1 = 0;
    int lastError1 = 0;
    int integral1 = 0;
    int derivative1 = 0;
    
    int pidValue1 = 0;
    
    int error2 = 0;
    int lastError2 = 0;
    int integral2 = 0;
    int derivative2 = 0;
    
    int pidValue2 = 0;
    
    int error3 = 0;
    int lastError3 = 0;
    int integral3 = 0;
    int derivative3 = 0;
    
    int pidValue3 = 0;
    
    int error4 = 0;
    int lastError4= 0;
    int integral4 = 0;
    int derivative4 = 0;
    
    int pidValue4 = 0;
    
    float kP = 6; // PID variables
    float kI = 0.1;
    float kD = 0.55;

    if (frontUltraSensor.distanceCm() <= 6){  // checks distance if less then (input), if so motors stop and program stop
        break;
   }
   int distanceTRight = getUltrasonicTopRight();
   error1 = distanceTRight - desiredLength;
   int distanceBRight = getUltrasonicBottomRight();
    error2 = distanceBRight - desiredLength;
   if (abs(error1) < threshold){
      integral1 = integral1 + error1;
   }else{
      integral1 = 0;
      derivative1 = error1 - lastError1;
      lastError1 = error1;
      pidValue1 = error1*kP + integral1*kI + derivative1*kD;
   if (pidValue1 > 100){ // checks if the pidValue is too high/low and sets it to a upper/lower limit
      pidValue1 = 60;
   }
   else if (pidValue1 < -100){
      pidValue1 = -60;
    }
    Encoder_1.setMotorPwm(-speedValue + pidValue1); //changes the speed of the motor
    Encoder_2.setMotorPwm(speedValue + pidValue1);
    Encoder_1.updateSpeed(); //update speed 
    Encoder_2.updateSpeed();
    }
   if (abs(error2) < threshold){
      integral2 = integral2 + error2;
   }else{
      integral2 = 0;
      derivative2 = error2 - lastError2;
      lastError2 = error2;
      pidValue2 = error2*kP + integral2*kI + derivative2*kD;
   if (pidValue2 > 100){ // checks if the pidValue is too high/low and sets it to a upper/lower limit
      pidValue2 = 60;
   }
   else if (pidValue2 < -100){
      pidValue2 = -60;
    }
    Encoder_1.setMotorPwm(-speedValue - pidValue2); //changes the speed of the motor
    Encoder_2.setMotorPwm(speedValue - pidValue2);
    Encoder_1.updateSpeed(); //update speed 
    Encoder_2.updateSpeed();
    }



    int distanceTLeft = getUltrasonicTopLeft();
   error3 = distanceTLeft - desiredLength;
   int distanceBLeft = getUltrasonicBottomLeft();
    error4 = distanceBLeft - desiredLength;
   if (abs(error3) < threshold){
      integral3 = integral3 + error3;
   }else{
      integral3 = 0;
      derivative3 = error3 - lastError3;
      lastError3 = error3;
      pidValue3 = error3*kP + integral3*kI + derivative3*kD;
   if (pidValue3 > 100){ // checks if the pidValue is too high/low and sets it to a upper/lower limit
      pidValue3 = 60;
   }
   else if (pidValue3 < -100){
      pidValue3 = -60;
    }
    Encoder_1.setMotorPwm(-speedValue + pidValue3); //changes the speed of the motor
    Encoder_2.setMotorPwm(speedValue + pidValue3);
    Encoder_1.updateSpeed(); //update speed 
    Encoder_2.updateSpeed();
    }
   if (abs(error4) < threshold){
      integral4 = integral4 + error4;
   }else{
      integral4 = 0;
      derivative4 = error4 - lastError4;
      lastError4 = error4;
      pidValue4 = error4*kP + integral4*kI + derivative4*kD;
   if (pidValue4 > 100){ // checks if the pidValue is too high/low and sets it to a upper/lower limit
      pidValue4 = 60;
   }
   else if (pidValue4 < -100){
      pidValue4 = -60;
    }
    Encoder_1.setMotorPwm(-speedValue - pidValue4); //changes the speed of the motor
    Encoder_2.setMotorPwm(speedValue - pidValue4);
    Encoder_1.updateSpeed(); //update speed 
    Encoder_2.updateSpeed();
    }
  }
  
}
