/*
* Author: Marcus 
* Date: 2/04/2020
* Aim:  To send variables to the arduino Mega, it will send the value of the top right IR sensor 
*/

#include <Wire.h>

// Define Slave I2c Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define ANSWERSIZE 5

//Declare Ir sensor to A3
int pinNum = 3;

//Define Answer as a string
String answer;

void setup() {
  //make A3 an input
   pinMode(pinNum, INPUT);

  // Initialise I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // Function to run when data received from master
  Wire.onReceive(receiveEvent);

  
  Serial.begin(9600);
  Serial.println("I2C Slave");
}

void receiveEvent() {
  // Read while data is received
  while (0 < Wire.available()){
    byte x = Wire.read();
  }
  Serial.println("Receive Event");
}

void requestEvent() {
  // Setup byte variable in the correct size 
  byte response[ANSWERSIZE];

  // Format answer as an array
  for (byte i=0;i<ANSWERSIZE;i++){
    response[i] = (byte)answer.charAt(i);
  }

  // Send response back to Master
  Wire.write(response,sizeof(response));

  Serial.println("Request Event");
}

// Ir sensor bool function, if closer then 5cm it will return true.
bool irSensor(){
  if (digitalRead(pinNum) == HIGH){
  return false;
 }else{
  return true;
 }
}

//Check if the answer IR sensor is less then 5cm if so it will change the answer to yes
void loop() {
  if (irSensor()){
    answer = "yyyyy";
  }else{
    answer = "nnnnn";
  }
  delay(150);
 
}
