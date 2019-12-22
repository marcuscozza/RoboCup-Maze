/*
* Author: Marcus 
* Date: 22/12/2019
* Aim: Uses Gyro to drive straight and turn when meet at a wall.
* NEEDS getUltraonicValue TO READ VALUES
 */

//Librays
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeAuriga.h"
#include <LiquidCrystal_I2C.h>


//Functions

//Display Functions
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

//Limit Switch Function
bool limitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(3) == HIGH or digitalRead(2) == HIGH){
    return true;
  }else{
    return false;
  }
}


//CheckWall Function 
void checkWall(){
  if(limitSwitch()){
    //go back
  }
  else if (frontUltraSensor.distanceCm() <= 7){  // // checks if front distance is <= 7 then stop motors
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        if(getUltrasonicTopRight() >= 8){ // checks which way is clear and turns 
          turnAbsolute(90, true, 150);
        }else if (getUltrasonicTopLeft() >= 8){
          turnAbsolute(-90, true, 150);
        }
        else if (getUltrasonicTopRight() <= 7 and getUltrasonicTopLeft() <= 7){
         turnAbsolute(90, true, 150);
        }
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








//Encoder Motor Delcare
MeEncoderOnBoard Encoder_1(SLOT1); // Declare encoder motors
MeEncoderOnBoard Encoder_2(SLOT2);

//Ultrasonic Delcare
MeUltrasonicSensor frontUltraSensor(PORT_9);

//Display Declare
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//Gyro Delcare
MeGyro gyro (0,0x69); //declare onboard gyro

int motorSpeed = 75;

//Motor Declare
void isr_process_encoder1(void)
{
      if(digitalRead(Encoder_1.getPortB()) == 0){
            Encoder_1.pulsePosMinus();
      }else{
            Encoder_1.pulsePosPlus();
      }
}

void isr_process_encoder2(void)
{
      if(digitalRead(Encoder_2.getPortB()) == 0){
            Encoder_2.pulsePosMinus();
      }else{
            Encoder_2.pulsePosPlus();
      }
}

int calculatedPID;
int gyroValue;


void setup(){
    //PWM Motor Setup
    Serial.begin(9600);
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    //Set Pwm 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

    //Start 16x2 display
    lcd.begin(16,2); 
    lcd.clear(); 
    
    //Gyro Setup
    delay(500);
    gyro.begin();
    delay(500);
}
void loop(){
  gyro.update(); //Set gyro active
  gyroValue = gyro.getAngleZ(); // set absolute to angle z
  calculatedPID = calculatePIDValue(gyroValue);
  Encoder_1.setMotorPwm(-motorSpeed - calculatedPID); //changes the speed of the motor
  Encoder_2.setMotorPwm(motorSpeed - calculatedPID);
  Encoder_1.updateSpeed(); //update speed 
  Encoder_2.updateSpeed();
  checkWall();
  displayText(calculatedPID , gyroValue);;
}
