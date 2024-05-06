/**
    \file CYBRos.ino
    Main file for the CYBR TRK hacky racer controller by Chris Stubbs.
    Compatible with hoverboard firmware: https://github.com/chrisstubbs93/hoverboard-firmware-hack-FOC
*/

//input: -100% to 100% of steering range over serial
//todo: detect failure to control
//todo: detect motor vin
//todo: lockout if values are out of safe range

#include <PID_v1.h>    //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <Smoothed.h>  // from https://www.arduino.cc/reference/en/libraries/smoothed/ //works in ide 1.8.19 //doesn't work properly with signed ints, apply on raw adc only.
#include <SoftwareSerial.h>
#include <ezBuzzer.h> // ezBuzzer library
#include <SPI.h>
#include <SD.h>
//https://github.com/SpacehuhnTech/SimpleCLI

#include "Settings.h"
#include "Globals.h"

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

  Serial.begin(115200); //Serial to USB / Interface
  Serial1.begin(HOVER_SERIAL_BAUD); //Serial to front HB
  Serial2.begin(HOVER_SERIAL_BAUD); //Serial to rear HB 1
  Serial3.begin(HOVER_SERIAL_BAUD); //Serial to rear HB 2

  Hoverboard[0].port = &Serial1;
  Hoverboard[1].port = &Serial2;
  Hoverboard[2].port = &Serial3;

  setupThrottleFuseControl();

  SDinit();

  //buzzerEvent();
}


void loop() {
  unsigned long currentMillisA = millis();  // store the current time
  processAnalog();
  processDigital();
  //checkFootAndHandBrakeHeld();  //power up hoverboards if brakes held. Note blocking while held.
  buzzerTick();
  Receive(*Hoverboard[0].port, Hoverboard[0].Feedback, Hoverboard[0].NewFeedback, Hoverboard[0].lastTimestamp);
  Receive(*Hoverboard[1].port, Hoverboard[1].Feedback, Hoverboard[1].NewFeedback, Hoverboard[1].lastTimestamp);
  Receive(*Hoverboard[2].port, Hoverboard[2].Feedback, Hoverboard[2].NewFeedback, Hoverboard[2].lastTimestamp);


  if (currentMillisA - previousMillisA >= interval) {  // Wait for next tick
    previousMillisA = currentMillisA;
    throttlecontrol();
    
    sendoldtelem();
    sendHovertelem(Hoverboard[0].Feedback, 0);
    sendHovertelem(Hoverboard[1].Feedback, 1);
    sendHovertelem(Hoverboard[2].Feedback, 2);

    isHoverboardConnected(0); //if disconnected, and ignition on, try to power on (relays not implemented yet).
    isHoverboardConnected(1);
    isHoverboardConnected(2);
  }

  // char looptime[128];
  // sprintf(looptime, "Loop time: %i", (millis()-currentMillisA));
  // sendInfo(looptime);
}
