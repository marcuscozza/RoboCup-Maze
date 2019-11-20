/**
* Author: Marcus 
* Date: 19/11/2019
port | Trig | Echo
6    |  64  |  69
7    |  63  |  68
8    |  62  |  67
9    |  61  |  66
10   |  60  |  65  
**/

#include <arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SoftwareSerial.h>

//defines variables
long duration;
int distance;
int trig=60;
int echo=65;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
    Serial.begin(9600);//Starts the serial communication
    pinMode(trig,OUTPUT);
    pinMode(echo,INPUT);
    Serial.println("Adafruit MLX90614 test");  

  mlx.begin();  
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
    Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
  Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

  Serial.println();
  delay(500);
}
