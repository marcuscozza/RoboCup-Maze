/*
* Author: Marcus 
* Date: 5/12/2019
* Aim: Moves any direction at a set amount of speed and time.
*https://forum.makeblock.com/t/need-hint-to-get-mbot-ranger-to-move-through-libraries/8909/3
*http://learn.makeblock.com/en/Makeblock-library-for-Arduino/class_me_encoder_on_board.html
*https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html
*/
//Librays
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <millisDelay.h>
millisDelay Delay;

bool startMoveDelay = true;
bool moveDelayOn = true;
 
//Encoder Motor Delcare
MeEncoderOnBoard Encoder_1(SLOT1); // Declare encoder motors
MeEncoderOnBoard Encoder_2(SLOT2);

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
  
    moveDirection(-100, 100, 3);
    delay(1000);
    delayOn();
    moveDirection(-100, 100, 3);
    delayOff();
    delay(1000); exit(0); //stop loop
}
/**function for going any direction
 *moveDirection(motor1Speed, motor2Speed, runTime) 
 *motor1Speed = int(the speed of the motor can be + or -)
 *motor2Speed = int(the speed of the motor can be + or -)
 *runTime = int(The amount of time in seoconds the motor will run for)
 */
void moveDirection(int motor1Speed, int motor2Speed, int runTime){
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
void delayOn(){
  moveDelayOn = true;
  startMoveDelay = true;
}
void delayOff(){
  moveDelayOn = false;
  startMoveDelay = false;
}
