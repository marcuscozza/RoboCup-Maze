/**
* Author: Marcus 
* Date: 4/12/2019
* Aim: Read both temp sensors and drop rescue kit
* NEEDS getUltraonicValue TO READ VALUES
**/
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#define leftTemp 0x5A //Defines the address of two temp sesnors
#define rightTemp 0x5B
#include <MeAuriga.h>
Adafruit_MLX90614 mlx;
 
Servo servo;  

//LED
#define ALLLEDS 0
#define LEDNUM  12
MeRGBLed led( 0, LEDNUM );

// Declare encoder motors
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeUltrasonicSensor ultraSensor(PORT_9);
//Gyro Delcare
MeGyro gyro (0,0x69); //declare onboard gyro

//getAbsolute function declare
int turnSpeed; // power at which to turn
int absolute; //getAbsolute() vlaue from Gyro
int target; //set target degress to int
bool resetGyro; // set resetGyro to bool

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


void setup() {
  servo.attach(4);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    //Set Pwm 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    Encoder_1.setPulse(9);
      Encoder_2.setPulse(9);
      Encoder_1.setRatio(39.267);
      Encoder_2.setRatio(39.267);
      Encoder_1.setPosPid(1.8,0,1.2);
      Encoder_2.setPosPid(1.8,0,1.2);
      Encoder_1.setSpeedPid(0.18,0,0);
      Encoder_2.setSpeedPid(0.18,0,0);
    delay(500);
    gyro.begin();
    delay(500);
  Serial.begin(9600); 
  mlx.begin();
  led.setpin( 44 );
  
}

void loop() {
  checkTemp();
  if (ultraSensor.distanceCm() <= 7){  // checks distance if less then (input), if so motors stop and the turnAbsolute function is ran
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        turnAbsolute(90, true, 200);
        delay(500);
    }
     else if(getUltrasonicTopRight() >= 7 or getUltrasonicBottomRight() >= 7){
      Encoder_1.setMotorPwm(0); //turn  right
      Encoder_2.setMotorPwm(100);
    }else if(getUltrasonicTopRight() <= 3 or getUltrasonicBottomRight() <= 3){
      Encoder_1.setMotorPwm(-100); //turn  left
      Encoder_2.setMotorPwm(0);
    }else{
      Encoder_1.setMotorPwm(-100); //drive foward
      Encoder_2.setMotorPwm(100); 
    }
}

void checkTemp(){
  int leftTempTarget = getLeftTempAmbient() + targetTempAdd;
  int rightTempTarget = getRightTempAmbient() + targetTempAdd;
  if(leftTempTarget <= getLeftTemp()){
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    turnAbsolute(90, true, 200);
    showRGBLED();
    dropRescueKit();
    turnAbsolute(-90, true, 200);
    turnOfRGBLED();
  }
  if(rightTempTarget <= getRightTemp()){
     Encoder_1.setMotorPwm(0);
     Encoder_2.setMotorPwm(0);
    turnAbsolute(-90, true, 200);
    showRGBLED();
    dropRescueKit();
    turnAbsolute(90, true, 200);
    turnOfRGBLED();
  }
  
}

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
