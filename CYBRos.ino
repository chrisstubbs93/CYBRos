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
#include "cmdArduino.h" //https://freaklabs.org/cmdarduino/

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

long intervalA = 50;  // time constant for PID loop tick
long intervalB = 250;  // time constant for datalog loop tick
long intervalC = 500;  // time constant for watchdog loop tick


unsigned long previousMillisA = 0;
unsigned long previousMillisB = 0;
unsigned long previousMillisC = 0;

void setup() {
  initGPIO();
  initAnalog();

  //Serial.begin(115200);              //Serial to USB / Interface
  cmd.begin(115200);
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
  //TODO disable fuse monitoring??
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

  //init commands for cmdline - note MUST be CR terminated not LF / CRLF
  cmd.add("ls", cmdLS);
  cmd.add("q", cmdQ);
  cmd.add("t", cmdT);
  cmd.add("s", cmdS);
  cmd.add("br", cmdBR);
  cmd.add("del", cmdDEL);
  cmd.add("b", cmdB);
}


void loop() {
  unsigned long currentMillis = millis();  // store the current time

  //Tasks to run every loop:
  processAnalog();
  processDigital();

  //checkFootAndHandBrakeHeld();  //power up hoverboards if brakes held. Note blocking while held.
  for (int i = 0; i <= 2; i++) {
    if (Hoverboard[i].enabled) Receive(*Hoverboard[i].port, Hoverboard[i].Feedback, Hoverboard[i].NewFeedback, Hoverboard[i].lastTimestamp);
  }
  cmd.poll(); //read and evaluate command line


  //Tasks to run each PID tick:
  if (currentMillis - previousMillisA >= intervalA) {
    previousMillisA = currentMillis;
    throttlecontrol();
  }


  //Tasks to run each datalog tick:
  if (currentMillis - previousMillisB >= intervalB) {
    previousMillisB = currentMillis;
    if(!quietSerial && telemSerial)sendoldtelem();
    for (int z = 0; z <= 2; z++) {
      if (Hoverboard[z].enabled) {
        if(!quietSerial && telemSerial)sendHovertelem(Hoverboard[z].Feedback, z);
      }
    }
  }

  
  //Tasks to run each HB watchdog tick:
  if (currentMillis - previousMillisC >= intervalC) {
    previousMillisC = currentMillis;
    if(Hoverboard[0].enabled) isHoverboardConnected(0); //putting these in the loop makes the buzzer library shit the bed and I wish I knew why.
    if(Hoverboard[1].enabled) isHoverboardConnected(1);
    if(Hoverboard[2].enabled) isHoverboardConnected(2);
  }

  // char looptime[128];
  // sprintf(looptime, "Loop time: %i", (millis()-currentMillisA));
  // sendInfo(looptime);
}

void cmdLS(int argCnt, char **args)
{
  File printroot = SD.open("/LOGS");
  printDirectory(printroot);
}

void cmdQ(int argCnt, char **args)
{
  quietSerial = !quietSerial;
  Serial.print("quietSerial is ");
  Serial.println(quietSerial);
}

void cmdT(int argCnt, char **args)
{
  telemSerial = !telemSerial;
  Serial.print("telemSerial is ");
  Serial.println(telemSerial);
}

void cmdS(int argCnt, char **args)
{
  dumpLogFile(atoi(args[1]));
}

void cmdBR(int argCnt, char **args)
{
  Serial.flush();
  Serial.begin(atol(args[1]));
  while(Serial.available()) Serial.read();
}

void cmdDEL(int argCnt, char **args)
{
  deleteAll();
  Serial.println("Reset arduino to start datalogging again.");
}

void cmdB(int argCnt, char **args)
{
  disableBeep = !disableBeep;
  Serial.print("disableBeep is ");
  Serial.println(disableBeep);
}