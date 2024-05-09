/**
    \file CYBRos.ino
    Main file for the CYBR TRK hacky racer controller by Chris Stubbs.
    Compatible with hoverboard firmware: https://github.com/chrisstubbs93/hoverboard-firmware-hack-FOC
*/

//TODO implement startup relays

#include <PID_v1.h>    //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <Smoothed.h>  // from https://www.arduino.cc/reference/en/libraries/smoothed/ //works in ide 1.8.19 //doesn't work properly with signed ints, apply on raw adc only.
#include <SoftwareSerial.h>
#include <ezBuzzer.h>  // ezBuzzer library
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

long interval = 50;  // time constant for tick

unsigned long previousMillisA = 0;

void setup() {
  initGPIO();
  initAnalog();

  Serial.begin(115200);              //Serial to USB / Interface
  Serial1.begin(HOVER_SERIAL_BAUD);  //Serial to front HB
  Serial2.begin(HOVER_SERIAL_BAUD);  //Serial to rear HB 1
  Serial3.begin(HOVER_SERIAL_BAUD);  //Serial to rear HB 2

  Hoverboard[0].port = &Serial1;
  Hoverboard[1].port = &Serial2;
  Hoverboard[2].port = &Serial3;

#if defined(CONFIG_VOLTCRANEO) & defined(CONFIG_CYBRTRK)
#error MULTIPLE HACKY CONFIGS SELECTED
#elif defined(CONFIG_VOLTCRANEO)
  Hoverboard[0].enabled = true;
  Hoverboard[1].enabled = true;
  Hoverboard[2].enabled = true;
  sendInfo("Firmware CONFIG_VOLTCRANEO");
#elif defined(CONFIG_CYBRTRK)
  Hoverboard[0].enabled = true;
  Hoverboard[1].enabled = true;
  Hoverboard[2].enabled = false;
  sendInfo("Firmware CONFIG_CYBRTRK");
#else
#error HACKY CONFIG NOT SELECTED
#endif

  for (int i = 0; i <= 2; i++) {
    if (Hoverboard[i].enabled) {
      char HBinfo[128];
      sprintf(HBinfo, "Hoverboard %i enabled", i);
      sendInfo(HBinfo);
    }
  }

  setupThrottleFuseControl();
  SDinit();
}


void loop() {
  unsigned long currentMillisA = millis();  // store the current time
  processAnalog();
  processDigital();
  //checkFootAndHandBrakeHeld();  //power up hoverboards if brakes held. Note blocking while held.
  buzzerTick();
  for (int i = 0; i <= 2; i++) {
    if (Hoverboard[i].enabled) Receive(*Hoverboard[i].port, Hoverboard[i].Feedback, Hoverboard[i].NewFeedback, Hoverboard[i].lastTimestamp);
  }

   if(Hoverboard[0].enabled) isHoverboardConnected(0); //putting these in the loop makes the buzzer library shit the bed and I wish I knew why.
   if(Hoverboard[1].enabled) isHoverboardConnected(1);
   if(Hoverboard[2].enabled) isHoverboardConnected(2);

  if (currentMillisA - previousMillisA >= interval) {  // Wait for next tick
    previousMillisA = currentMillisA;
    throttlecontrol();

    //sendoldtelem();
    for (int z = 0; z <= 2; z++) {
      if (Hoverboard[z].enabled) {
        sendHovertelem(Hoverboard[z].Feedback, z);
      }
    }
  }

  // char looptime[128];
  // sprintf(looptime, "Loop time: %i", (millis()-currentMillisA));
  // sendInfo(looptime);
}
