/*
* Author: Marcus 
* Date: 29/03/2020
* Aim: Main Program which is using PID gyro to navigate. includes rescuekit and alignment
* NEEDS Functions in order to compile
 */

//Librays
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeAuriga.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#define leftTemp 0x5A //Defines the address of two temp sesnors
#define rightTemp 0x5B
Adafruit_MLX90614 mlx;

#define ALLLEDS 0
#define LEDNUM  12
MeRGBLed led( 0, LEDNUM );

//Servo Declare
Servo servo;


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

    //delcare limitswitch pins
    pinMode(3, INPUT); // sets pin 3 and 2 for port 2 to inputs
    pinMode(2, INPUT); 


    //delcare temp sensor
    mlx.begin(); 

    //LED delcare pin 44
    led.setpin( 44 );

    //Servo Delcare pin 4
    servo.attach(4);
    
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
  displayText(calculatedPID , gyroValue);
  checkWall();
  checkTemp();

     
}
