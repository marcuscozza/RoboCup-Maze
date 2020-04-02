/*
* Author: Marcus 
* Date: 2/04/2020
* Aim:  To recieve variables from the arduino nano.
* 
 */

#include <Wire.h>

// Define Master I2c Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define ANSWERSIZE 5
void setup() {

  // Initialise I2C communications as master
  Wire.begin();

  Serial.begin(9600);
  Serial.println("I2C Master");
}

void loop() {
  delay(150);
  Serial.println("Write Data to slave");

  // Write a charatcer to the Slave
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  Serial.println("Receive Data");

  // Read response from Slave
  Wire.requestFrom(SLAVE_ADDR, ANSWERSIZE);

  // Add characters to string 
  String response = "";
  while (Wire.available()){
    char b = Wire.read();
    response += b;
  }
  if (response == "yyyyy"){
    Serial.println("Active");
  }
  
}
