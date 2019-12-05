/**
* Author: Marcus 
* Date: 29/11/2019
* Aim: Connect two Temp sensors
* https://os.mbed.com/users/ronixas/code/MLX90614-IR_Temp_Sensor-change-address/file/039f25b3de3e/main.cpp/
**/
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#define leftTemp 0x5A //Defines the address of two temp sesnors
#define rightTemp 0x5B
Adafruit_MLX90614 mlx;

float getLeftTemp(){ // function that returns the object temp in C
  mlx.AddrSet(leftTemp); 
  return mlx.readObjectTempC();
}
float getRightTemp(){
  mlx.AddrSet(rightTemp); 
  return mlx.readObjectTempC();
}
void setup() {
  Serial.begin(9600); 
  mlx.begin(); 
}

void loop() {
  Serial.print("Left:");
  Serial.println(getLeftTemp());
  delay(250);
   Serial.print("Right:");
  Serial.println(getRightTemp());
  delay(1000);
}
