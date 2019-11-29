/**
* Author: Marcus 
* Date: 29/11/2019
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

//defines variables
//defines Port 7
int port7Trig=63;
int port7Echo=68;
int distanceTopLeft;
long durationTopLeft;
//defines Port 8
int port8Trig=62;
int port8Echo=67;
int distanceTopRight;
long durationTopRight;
//defines Port 9
int port9Trig=61;
int port9Echo=66;
int distanceBottomLeft;
long durationBottomLeft;
//defines Port 10
int port10Trig=60;
int port10Echo=65;
int distanceBottomRight;
long durationBottomRight;


int getUltrasonicTopLeft(){ //Functions that returns the distance of Ultrasonics in CM for all 4 sides
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
    pinMode(port8Trig,OUTPUT);
    pinMode(port8Echo,INPUT);
    digitalWrite(port8Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port8Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port8Trig,LOW);
    durationTopRight=pulseIn(port8Echo,HIGH);
    distanceTopRight=durationTopRight*0.034/2;
    return distanceTopRight;
}

int getUltrasonicBottomLeft(){
    pinMode(port9Trig,OUTPUT);
    pinMode(port9Echo,INPUT);
    digitalWrite(port9Trig,LOW);
    delayMicroseconds(2);
    digitalWrite(port9Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(port9Trig,LOW);
    durationBottomLeft=pulseIn(port9Echo,HIGH);
    distanceBottomLeft=durationBottomLeft*0.034/2;
    return distanceBottomLeft;
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
  delay (500);
}
