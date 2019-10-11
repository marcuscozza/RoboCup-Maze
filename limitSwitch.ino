/**
* Author: Marcus 
* Date: 30/9/2019
* Aim: turn led on arduino board on if limit switch/s is activated
* pin: 5V > 2.2K R > NO > C > GND 

**/
void setup() {
  pinMode(7, INPUT); //Left Switch to D7
  pinMode(8, INPUT); //Right Switch D8
  pinMode(13, OUTPUT);//Onboard LED 
  digitalWrite(13,LOW); //set led a off
}

void loop() {
  if(digitalRead(7) == LOW and digitalRead(8) == LOW){ //Check if both are active
    digitalWrite(13,HIGH); //LED ON
  }else if (digitalRead(7) == LOW){ //Check if left is active
    digitalWrite(13,HIGH); //LED ON
  }else if (digitalRead(8) == LOW){ //check if right is active
    digitalWrite(13,HIGH); //LED ON
  }else{
    digitalWrite(13,LOW); //LED OFF when switch is open
  }
}
