/**
* Author: Marcus 
* Date: 17/11/2019
* Aim: read distance from PORT_6 using RJ25 adapter

**/

#include <arduino.h>
#include <Wire.h> 
#include <SoftwareSerial.h>

//defines variables
long duration;
int distance;
int trig=64;
int echo=69;

void setup() {
    Serial.begin(9600);//Starts the serial communication
    pinMode(trig,OUTPUT);
    pinMode(echo,INPUT);
}

void loop() {
    digitalWrite(trig,LOW);
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    duration=pulseIn(echo,HIGH);
    distance=duration*0.034/2;
    Serial.print("Distance:");
    Serial.println(distance);
    delay(100);
}
