/**
    \file Globals.h
    Contains Globals.
*/

#ifndef GLOBALS_H
#define GLOBALS_H

double Input3Fuse, Output3Throttle;
int brkcmd;
int drvcmd;


// Global variables for hoverboard
int16_t currentDriveMode = TRQ_MODE;
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
byte telemBuffer[20];

typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  int16_t  brake;
  int16_t  driveMode;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  dcCurrent;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;

typedef struct{
  Stream *port;
  SerialFeedback Feedback;
  SerialFeedback NewFeedback;
  unsigned long lastTimestamp;
} Hboard;
Hboard Hoverboard[3];

#endif