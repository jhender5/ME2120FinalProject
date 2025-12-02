//Authored by Jasper Henderson
//Referenced: https://github.com/Arduino-IRremote/Arduino-IRremote/blob/master/examples/TinySender/TinySender.ino

#include <Arduino.h>
#include "TinyIRSender.hpp"

//Set up variables
int xPin = A7;
int yPin = A6;
int xVal = 511;
int yVal = 511;

//address to send to, and number of times to repeat command
  uint8_t address = 0x4;
  uint8_t repeats = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {};
  //Serial.println(IR_SEND_PIN);
}

void loop() {
  checkJoysticks();

  transmitCommand(xVal, yVal);  
}

//get Joystick Values (values are read as a number from 0 to 1023 with about 511 as the middle (no input))
void checkJoysticks() {
  xVal = analogRead(xPin);
  yVal = analogRead(yPin);
}

bool transmitCommand(int x, int y) {
  
  //Command must be made 16bit, x (forward) value is between 0 and 1023, y (turning) value between 1024 and 2047
  uint16_t xCommand = xVal;
  uint16_t yCommand = yVal + 0x400;

  //using the NEC protocol send the command
  sendNEC(IR_SEND_PIN, address, xCommand, repeats);
  delay(10);//Must delay to seperate commands
  sendNEC(IR_SEND_PIN, address, yCommand, repeats);
  delay(10);//Must delay to seperate commands
}
