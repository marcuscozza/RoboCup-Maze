/**
* Author: Marcus 
* Date: 27/01/2020
* Aim: Testing short IR sensor.

**/

int pinNum = 3;

void setup() {
  Serial.begin(9600);
  pinMode(pinNum, INPUT); //Sets D3 to input, Is the output for IR Sensor
}

void loop() {
  if (irSensor()){
    Serial.println("Closer then <= 5cm");
  }else{
    Serial.println("Not close");
  }
  // Wait some time
  delay(50);
}

// function that returns true when pinNum is low
bool irSensor(){
  if (digitalRead(pinNum) == HIGH){
  return false;
 }else{
  return true;
 }
}
