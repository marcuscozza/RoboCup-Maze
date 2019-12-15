/*
* Author: Marcus 
* Date: 15/12/2019
* Aim:  4 sensor PID wall follower
* NEEDS getUltraonicValue TO READ VALUES
*https://forum.makeblock.com/t/need-hint-to-get-mbot-ranger-to-move-through-libraries/8909/3
*http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_encoder_on_board.html
*https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html
*/
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>

//Encoder Motor Declare
MeEncoderOnBoard Encoder_1(SLOT1); // Declare encoder motors
MeEncoderOnBoard Encoder_2(SLOT2);

//Ultrasonic Declare
MeUltrasonicSensor frontUltraSensor(PORT_9);

//PID Declare
int desiredLength = 5; //the length away from the wall
int speedValue = 100; // speed of the motors

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

float kP = 5.5; // PID variables
float kI = 0.01;
float kD = 0.55;


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
}
void loop(){
  if (frontUltraSensor.distanceCm() <= 6){  // checks distance if less then (input), if so motors stop and program stop
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        
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
