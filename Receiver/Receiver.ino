//Referencing: https://github.com/Arduino-IRremote/Arduino-IRremote/blob/master/examples/TinyReceiver/TinyReceiver.ino

#include<Arduino.h>
#include <Servo.h>
#include "pinDefinitionsAndMore.h"

#define USE_ONKYO_PROTOCOL

#include "TinyIRReceiver.hpp"


//IRSignal Variables
int newCommand;
int speedCommand;
int turnCommand;


//could add more code to read encoder and double check speed

const int enMotor = 9;
const int motorF = 12;
const int motorR = 5; 

int speedCentered = 0;
int speedpwm = 0;

const int enServo = 3;
const int servoR = 6;
const int servoL = 8;
const int DEADBAND = 15;


int steerCentered = 512;
int steerpwm = map(abs(steerCentered), 0, 512, 0, 255);


Servo steer;

void setup() {
  Serial.begin(115200);

   // Enables the interrupt generation on change of IR input signal
    if (!initPCIInterruptForTinyReceiver()) {
        Serial.println(F("No interrupt available for pin " STR(IR_RECEIVE_PIN))); // optimized out by the compiler, if not required :-)
    }

  //Motor Control pins
  pinMode(motorF, OUTPUT);
  pinMode(motorR, OUTPUT);
  pinMode(enMotor, OUTPUT);
  
  pinMode(servoR, OUTPUT);
  pinMode(servoL, OUTPUT);
  
}

void loop() {

  //IR Signal
  if(TinyReceiverDecode()){
    
    newCommand = TinyIRReceiverData.Command;

    if (newCommand >= 1024){
      turnCommand = newCommand-1024;
    } else {
      speedCommand = newCommand;
    }
    
  }

  //Speed Control
  
  speedCentered = speedCommand - 512;
  speedpwm = map(abs(speedCentered), 0, 512, 0, 255);
  
  if (speedCentered > DEADBAND) { // forward
    digitalWrite(motorF, HIGH);
    digitalWrite(motorR, LOW);
    analogWrite(enMotor, speedpwm);
  } else if (speedCentered < -DEADBAND) { // reverse
    digitalWrite(motorF, LOW);
    digitalWrite(motorR, HIGH);
    analogWrite(enMotor, speedpwm);
  } else {
    analogWrite(enMotor, 0);
    digitalWrite(motorF, LOW);
    digitalWrite(motorR, LOW);
  }
    
  
  
  //Steering Control
  steerCentered = turnCommand - 512;
  steerpwm = map(abs(steerCentered), 0, 512, 0, 255);

  
  if (steerCentered > DEADBAND) { // forward
    digitalWrite(servoR, HIGH);
    digitalWrite(servoL, LOW);
    analogWrite(enServo, steerpwm);
    
  } else if (steerCentered < -DEADBAND) { // reverse
    digitalWrite(servoR, LOW);
    digitalWrite(servoL, HIGH);
    analogWrite(enServo, steerpwm);
  } else {
    analogWrite(enServo, 0);
    digitalWrite(servoL, LOW);
    digitalWrite(servoR, LOW);
  }
}
