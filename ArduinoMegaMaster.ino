/* MASTER
* Author: Marcus 
* Date: 3/04/2020
* Aim:  To recieve IR sensor and RGB values from the arduino nano.
* Value 1 = Top Right IR Sensor
* Value 2 = Top Left IR Sensor
* Value 3 = Bottom Right IR Sensor
* Value 4 = Bottom Left IR Sensor
* Value 5 = Red colour value
* Value 6 = Green colour value
* Value 7 = Blue colour value
* Connecting I2C devices: https://www.youtube.com/watch?v=yBgikWNoU9o
 */

#include <Wire.h>

#define SLAVE_ADDR 9

byte value[7];

int bcount;
void setup() {
  Wire.begin();
  Serial.begin(9600);
}

byte readI2C(int address){
  byte bval;
  long entry = millis();

  Wire.requestFrom(address, 1);

  while (Wire.available() == 0 && (millis() - entry) < 100) Serial.print("Waiting");

  if (millis() - entry < 100) bval = Wire.read();
  return bval;
}

void loop() {
  while (readI2C(SLAVE_ADDR) < 255){
    Serial.print("Waiting");
  }
  for (bcount = 0; bcount < 7; bcount++){
    value[bcount] = readI2C(SLAVE_ADDR);
  }
  for (int i = 0; i < 7; i++){
    Serial.print(value[i]);
    Serial.print("\t");
  }
  Serial.println();
  delay(200);

}
