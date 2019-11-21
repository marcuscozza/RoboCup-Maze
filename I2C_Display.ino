/**
* Author: Marcus 
* Date: 20/11/2019
* Aim: Display any text using I2C Display

**/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

void displayText(String topText, String bottomText){ // function which has topText for the display and the bottomText.
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
  lcd.begin(16,2); //Start 16x2 display
  lcd.clear(); 
  lcd.print(topText); //display topText
  lcd.setCursor(0,1); //move down one row
  lcd.print(bottomText); //displaybottomText
}

void setup(){
  char displayTextTop[] = "Hello";
  char displayTextBottom[] = "World";
  displayText(displayTextTop, displayTextBottom);
}
void loop(){
  
}
