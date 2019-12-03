/**
* Author: Marcus 
* Date: 3/12/2019
* Aim: Functions that will return all 5 values for ultrasonics
port | Trig | Echo
6    |  64  |  69
7    |  63  |  68
8    |  62  |  67
9    |  61  |  66
10   |  60  |  65  
**/

#include <arduino.h>
#include <Wire.h> 
#include <SoftwareSerial.h>
#include "MeAuriga.h"

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
//Define port 9 front ultrasonic
MeUltrasonicSensor frontUltraSensor(PORT_9);



int getUltrasonicBottomLeft(){ //Functions that returns the distance of Ultrasonics in CM for all 4 sides
    pinMode(port8Trig,OUTPUT);
    pinMode(port8Echo,INPUT);
    digitalWrite(port8Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port8Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port8Trig,LOW);
    durationBottomLeft=pulseIn(port8Echo,HIGH);
    distanceBottomLeft=durationBottomLeft*0.034/2;
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
    distanceTopLeft=durationTopLeft*0.034/2;
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
    distanceTopRight=durationTopRight*0.034/2;
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
    distanceBottomRight=durationBottomRight*0.034/2;
    return distanceBottomRight;
}
void setup() {
    Serial.begin(9600);
}

void loop() {
  Serial.print("TopLeft:");
  Serial.println(getUltrasonicTopLeft());
  Serial.print("TopRight:");
  Serial.println(getUltrasonicTopRight());
  Serial.print("BottomLeft:");
  Serial.println(getUltrasonicBottomLeft());
  Serial.print("BottomRight:");
  Serial.println(getUltrasonicBottomRight());
  Serial.print("Front: ");
  Serial.println(frontUltraSensor.distanceCm());
  delay (500);
}
