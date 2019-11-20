#include <Servo.h>           //Servo library
 
Servo servo;        //initialize a servo object for the connected servo  

int dropRescueKit (){  // delcaring function for droping rescue kit
  servo.write(180);  // Pull back to drop rescue kit
 delay(1000);
 servo.write(0);    //push rescue kit
 delay(5000);
}


void setup() 
{ 
  servo.attach(5);      // attach the signal pin of servo to pin9 of arduino
  dropRescueKit ();     //calling function to drop rescue kit
} 
  
void loop() 
{
     
}
