/**
* Author: Marcus 
* Date: 20/11/2019
* Aim: Function for switch will return true/false if switch is high/low
* pin: Port 2
* 
**/

void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT); // sets pin 3 and 2 for port 2 to inputs
  pinMode(2, INPUT); 
}

void loop() {
  if (limitSwitch()){
    Serial.println("Switch is active");
  }else if (!limitSwitch()){
    Serial.println("Switch is not active");
  }else{
    Serial.println("Switch Error: Can't Read Status);
  }
  delay(500);
}

bool limitSwitch(){ // function that returns true if left and/or right are active, else its returns false
  if(digitalRead(3) == HIGH or digitalRead(2) == HIGH){
    return true;
  }else{
    return false;
  }
}
