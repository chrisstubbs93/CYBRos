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


Smoothed<int> SteeringFeedbackVal;
Smoothed<int> AccelPedalVal;
Smoothed<int> BrakePedalVal;
Smoothed<int> ManualBrakeVal;
Smoothed<int> FuseADC;
Smoothed<int> ShuntADC;
Smoothed<int> SteerCurr;
Smoothed<int> RCSteerPulse;
Smoothed<int> RCThrottlePulse;
Smoothed<int> RCAuxPulse;
bool manualBraking;
bool steercurrentLimiting;
bool fusecurrentLimiting;

int pot;

int pedalval = 0;

long intervalA = 25;  // time constant for PID loop tick //Pulses must be processed slower than they come in (tick > 20ms) //was 50!! May need to change fuse timing!!
long intervalB = 50;
long intervalC = 250;  // time constant for datalog loop tick
long intervalD = 250;  // time constant for watchdog loop tick

unsigned long previousMillisA, previousMillisB, previousMillisC, previousMillisD = 0;

void setup() {
  initGPIO();
  initAnalog();

  //Serial.begin(115200);              //Serial to USB / Interface
  cmd.begin(9600); //init at lower speed for  bt modem
  delay(200);
  Serial.print("AT+NAMECYBR");
  delay(200);
  Serial.print("AT+BAUD8"); //115200 baud serial bt modem
  delay(500);
  cmd.begin(115200); //switch to 115200
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
  setupSteeringControl();
  SDinit();

  //init commands for cmdline - note MUST be CR terminated not LF / CRLF
  cmd.add("?", cmdHELP); //help
  cmd.add("help", cmdHELP); //help
  cmd.add("ls", cmdLS); //List SD contents
  cmd.add("q", cmdQ); //toggle quietserial
  cmd.add("t", cmdT); //toggle telem over serial
  cmd.add("s", cmdS); //Send log file x
  cmd.add("br", cmdBR); //Switch baud rate to x
  cmd.add("del", cmdDEL); //Delete all files on SD
  cmd.add("b", cmdB); //toggle beep
  cmd.add("dread", cmdDREAD); //print IO status for diagnostics
  cmd.add("aread", cmdAREAD); //print IO status for diagnostics
}


void loop() {
  unsigned long currentMillis = millis();  // store the current time
  unsigned long Aloop,Bloop,Cloop,Dloop,Eloop; //for debugging
  bool Atick, Btick, Ctick, Dtick = 0;

  //Tasks to run every loop:
  processAnalog();
  processDigital();

  //checkFootAndHandBrakeHeld();  //power up hoverboards if brakes held. Note blocking while held.
  for (int i = 0; i <= 2; i++) {
    if (Hoverboard[i].enabled) Receive(*Hoverboard[i].port, Hoverboard[i].Feedback, Hoverboard[i].NewFeedback, Hoverboard[i].lastTimestamp);
  }
  cmd.poll(); //read and evaluate command line

  Aloop = millis()-currentMillis; //for debugging

  //Tasks to run each steering PID tick:
  if (currentMillis - previousMillisA >= intervalA) {
    Atick = 1;
    previousMillisA = currentMillis;
    throttlecontrol();
  }

  Bloop = millis()-currentMillis; //for debugging

    //Tasks to run each RC PID tick:
  if (currentMillis - previousMillisB >= intervalB) {
    Btick = 1;
    previousMillisB = currentMillis;
    SteeringControl();
    if(currentGear == GEAR_N) processRCPulse(); //Only process RC inputs in neutral to improve response in normal drive. //Pulses must be processed slower than they come in (tick > 20ms)
  }

   Cloop = millis()-currentMillis; //for debugging

  //Tasks to run each datalog tick:
  if (currentMillis - previousMillisC >= intervalC) {
    Ctick = 1;
    previousMillisC = currentMillis;
    if(!quietSerial && telemSerial){
      sendoldtelem();
      sendSteeringtelem();
      if(currentGear == GEAR_N) sendRCtelem();
    }
    for (int z = 0; z <= 2; z++) {
      if (Hoverboard[z].enabled) {
        if(!quietSerial && telemSerial)sendHovertelem(Hoverboard[z].Feedback, z);
      }
    }
  }

  Dloop = millis()-currentMillis; //for debugging


  //Tasks to run each HB watchdog tick:
  if (currentMillis - previousMillisD >= intervalD) {
    Dtick = 1;
    previousMillisD = currentMillis;
    if(Hoverboard[0].enabled) isHoverboardConnected(0); //putting these in the loop makes the buzzer library shit the bed and I wish I knew why.
    if(Hoverboard[1].enabled) isHoverboardConnected(1);
    if(Hoverboard[2].enabled) isHoverboardConnected(2);
  }


  if((millis()-currentMillis) > 100){
  //print the timestamp delta for debugging
    char looptime[128];
    sprintf(looptime, "TLoop time: %i", (millis()-currentMillis));
    sendInfo(looptime);

    sprintf(looptime, "Breakdown A: %i, B: %i, C: %i, D: %I", Aloop, Bloop, Cloop, Dloop);
    sendInfo(looptime);

    sprintf(looptime, "Ticks this cycle: A: %i, B: %i, C: %i", Atick, Ctick, Dtick);
    sendInfo(looptime);
  }
}

void cmdHELP(int argCnt, char **args)
{
    Serial.println("CybrOS Help");
    Serial.println("===========");
    Serial.println("List of commands:");
    Serial.println("?/help - This menu");
    Serial.println("ls - List SD card contents");
    Serial.println("q - Toggle quietserial (all messages)");
    Serial.println("t - Toggle telemetry over serial");
    Serial.println("s x - Send log file no. x");
    Serial.println("br x - Switch baud rate to x");
    Serial.println("del - Delete all files on SD card");
    Serial.println("b - Toggle beep");
    Serial.println("dread x - Print digital read for pin x");
    Serial.println("aread x - Print analogue read for pin x");
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

void cmdDREAD(int argCnt, char **args)
{
  Serial.print("Pin ");
  Serial.print(atoi(args[1]));
  Serial.print(" digitalRead: ");
  Serial.println(digitalRead(atoi(args[1])));
}

void cmdAREAD(int argCnt, char **args)
{
  Serial.print("Pin ");
  Serial.print(atoi(args[1]));
  Serial.print(" analogRead: ");
  Serial.println(analogRead(atoi(args[1])));
}