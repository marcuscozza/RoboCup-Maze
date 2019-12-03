/*
* Author: Marcus 
* Date: 2/10/2019
* Aim: Moves foward untill robot is 7cm from a wall it will then turn 90 degress to the right and drive foward again.
*https://forum.makeblock.com/t/need-hint-to-get-mbot-ranger-to-move-through-libraries/8909/3
*/
//Librays
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeAuriga.h"

//Encoder Motor Delcare
MeEncoderOnBoard Encoder_1(SLOT1); // Declare encoder motors
MeEncoderOnBoard Encoder_2(SLOT2);

//Ultrasonic Delcare
MeUltrasonicSensor frontUltraSensor(PORT_9);

//Gyro Delcare
MeGyro gyro (0,0x69); //declare onboard gyro

//getAbsolute function declare
int turnSpeed; // power at which to turn
int absolute; //getAbsolute() vlaue from Gyro
int target; //set target degress to int
bool resetGyro; // set resetGyro to bool

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

    //Gyro Setup
    delay(500);
    gyro.begin();
    delay(500);
}
void loop(){
    Encoder_1.setMotorPwm(-100); //drive foward
    Encoder_2.setMotorPwm(100);
    if (frontUltraSensor.distanceCm() <= 7){  // checks distance if less then (input), if so motors stop and the turnAbsolute function is ran
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        turnAbsolute(90, true, 200);
        delay(1000);
    }
}


/**function for gyro turns
 *turnAbsolute(target, bool) 
 *target = int(amount of degress to turn in Z angle)
 *resetGyro = bool(True[Will reset angle z to 0 after turning] | False[will not reset angle z after turning])
**/
void turnAbsolute(int target, bool resetGyro, int turnSpeed){ 
  gyro.update(); //Set gyro active
  absolute = gyro.getAngleZ(); // set absolute to angle z
  Serial.println(absolute); //print angle z
  while (abs(absolute - target) > 0){ //print checks if the target has been reached
    Serial.println(absolute); //print angle z
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
