/**
    \file CYBRos.ino
    Main file for the CYBR TRK hacky racer controller by Chris Stubbs.
*/

//input: -100% to 100% of steering range over serial
//todo: detect failure to control
//todo: detect motor vin
//todo: lockout if values are out of safe range

#include <PID_v1.h>    //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <Smoothed.h>  // from https://www.arduino.cc/reference/en/libraries/smoothed/ //works in ide 1.8.19 //doesn't work properly with signed ints, apply on raw adc only.
#include <SoftwareSerial.h>

#include "Settings.h"
#include "Globals.h"

SoftwareSerial HoverSerial(12, 4);  // RX, TX


Smoothed<int> SteeringWheelVal;
Smoothed<int> SteeringFeedbackVal;  //now FuseADC not anymore!!
Smoothed<int> AccelPedalVal;
Smoothed<int> BrakePedalVal;
Smoothed<int> ManualBrakeVal;
Smoothed<int> FuseADC;
Smoothed<int> ShuntADC;
bool manualBraking;
bool currentLimiting;
bool fusecurrentLimiting;

int pot;
int sp;
int pos = 0;  //mapped steeringwheelval avg
int posraw = 0;
int I_reading = 0;

int pedalval = 0;

long interval = 50;       // time constant for tick
unsigned long previousMillisA = 0;

void setup() {
  initGPIO();
  initAnalog();

  Serial.begin(115200);                  //hardware serial to pi
  HoverSerial.begin(HOVER_SERIAL_BAUD);  //software serial to hoverboard

  setupThrottleFuseControl();
}


void loop() {
  unsigned long currentMillisA = millis();  // store the current time
  processAnalog();
  checkFootAndHandBrakeHeld();  //power up hoverboards if brakes held. Note blocking while held.

  if (currentMillisA - previousMillisA >= interval) {  // Wait for next tick
    previousMillisA = currentMillisA;
    throttlecontrol();
    sendoldtelem();
  }
}
