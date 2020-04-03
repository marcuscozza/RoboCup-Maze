/* SLAVE
* Author: Marcus 
* Date: 3/04/2020
* Aim:  To send IR sensor and colour sensor valuesto the arduino Mega. 
* Value 1 = Top Right IR Sensor
* Value 2 = Top Left IR Sensor
* Value 3 = Bottom Right IR Sensor
* Value 4 = Bottom Left IR Sensor
* Value 5 = Red colour value
* Value 6 = Green colour value
* Value 7 = Blue colour value
* Connecting I2C devices: https://www.youtube.com/watch?v=yBgikWNoU9o
*/

#include <Wire.h>

// Assign I2C address
#define SLAVE_ADDR 9


// Declare pins for colour sensor
#define COLOUROUT A3
#define COLOURS2 A2
#define COLOURS3 A1

// Decalre pins for IR sensors
int topRightPin = 3;
int topLeftPin = 4;
int bottomRightPin = 5;
int bottomLeftPin = 6;

// bool Functions for IR sensors
bool getTopRightIRValue(){
  if (digitalRead(topRightPin) == HIGH){
  return false;
 }else{
  return true;
 }
}
bool getTopLeftIRValue(){
  if (digitalRead(topLeftPin) == HIGH){
  return false;
 }else{
  return true;
 }
}
bool getBottomRightIRValue(){
  if (digitalRead(bottomRightPin) == HIGH){
  return false;
 }else{
  return true;
 }
}
bool getBottomLeftIRValue(){
  if (digitalRead(bottomLeftPin) == HIGH){
  return false;
 }else{
  return true;
 }
}

// Colour sensor intensity function
int getintensity(){ //measure intensity with oversampling
 int a=0;
 int b=255;
 for(int i=0;i<10;i++){a=a+pulseIn(COLOUROUT,LOW);}
 if(a>9){b=2550/a;}
 return b;
}

// Functions for each colour
void colourred(){ //select red
 digitalWrite(COLOURS2,LOW);
 digitalWrite(COLOURS3,LOW);
}
void colourblue(){ //select blue
 digitalWrite(COLOURS2,LOW);
 digitalWrite(COLOURS3,HIGH);
}
void colourwhite(){ //select white
 digitalWrite(COLOURS2,HIGH);
 digitalWrite(COLOURS3,LOW);
}
void colourgreen(){ //select green
 digitalWrite(COLOURS2,HIGH);
 digitalWrite(COLOURS3,HIGH);
}

// Assign value to an array of 7
int value[7];

// Declare bcount as a counter
int bcount = 0;
void setup() {
  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  pinMode(topRightPin, INPUT); 
  pinMode(topLeftPin, INPUT);
  pinMode(bottomRightPin, INPUT);
  pinMode(bottomLeftPin, INPUT);

  pinMode(COLOUROUT,INPUT);
  pinMode(COLOURS2,OUTPUT);
  pinMode(COLOURS3,OUTPUT);
}

void requestEvent(){
  byte bval;

  switch (bcount){
    case 0:
      bval = 255;
      break;
    case 1:
      bval = value[0];
      break;
    case 2:
      bval = value[1];
      break;
    case 3:
      bval = value[2];
      break;
    case 4:
      bval = value[3];
      break;
    case 5:
      bval = value[4];
      break;
    case 6:
      bval = value[5];
      break;
    case 7:
      bval = value[6];
      break;
  }

  Wire.write(bval);

  bcount = bcount + 1;
  if (bcount > 7) bcount = 0;
}

void readValue(){
  if (getTopRightIRValue()){
    value[0] = 1;
  }else{
    value[0] = 0;
  }
  delay(10);
   if (getTopLeftIRValue()){
    value[1] = 1;
  }else{
    value[1] = 0;
  }
  delay(10);
   if (getBottomRightIRValue()){
    value[2] = 1;
  }else{
    value[2] = 0;
  }
  delay(10);
   if (getBottomLeftIRValue()){
    value[3] = 1;
  }else{
    value[3] = 0;
  }
  delay(10);
  if (getBottomLeftIRValue()){
    value[3] = 1;
  }else{
    value[3] = 0;
  }
  colourred();
  value[4] = getintensity();
  delay(10);
  colourgreen();
  value[5] = getintensity();
  delay(10);
  colourblue();
  value[6] = getintensity();
  delay(10);
}

void loop() {
  readValue();
  delay(500);
}
