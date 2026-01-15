/**
    \file Settings.h
    Contains user settings and presets that can be changed for build.
*/

#ifndef SETTINGS_H
#define SETTINGS_H

//======================Definitions for hardware NEW=======================
#define BuzzerPin 13 //clicks on serial connection, could swap for D45, OUTPUT

#define SteeringPot1Pin A5 //J8 - Steering wheel SP
#define SteeringPot2Pin A6 //J9 - Feedback

//J14 - Fuse Mon
#define FuseMonPin A7 //J14-3
#define ShuntMonPin A8 //J14-4

//J11 - F Mot
//Serial1 RX1 J11-2
//Serial1 TX1 J11-3
#define FHBPwrTellbackPin 33 //J11-4 INPUT
#define FHBPwrCommandPin 34 //J11-6 (5V TTL) / 5&6 (RLY) OUTPUT

//J12 - R Mot 1
//Serial2 RX2 J12-2
//Serial2 TX2 J12-3
#define RHB1PwrTellbackPin 35 //J12-4 INPUT
#define RHB1PwrCommandPin 36 //J12-6 (5V TTL) / 5&6 (RLY) OUTPUT

//J13 - R Mot 2
//Serial3 RX3 J13-2
//Serial3 TX3 J13-3
#define RHB2PwrTellbackPin 37 //J13-4 INPUT
#define RHB2PwrCommandPin 38 //J13-6 (5V TTL) / 5&6 (RLY) OUTPUT

#define BrakeHallPin A0 //J2-3 INPUT
#define LPedalPin A1 //J3-3 INPUT
#define RPedalPin A2 //J16-3 INPUT

#define FLightPin 4 //J16-2 OUTPUT
#define RLightPin 5 //J16-2 OUTPUT

//Connector J15 - Interface
//Serial0 RX0 J15-3
//Serial0 TX0 J15-4
//#define IFMISOPin 28 //J15-5 //just uses built in SPI, shared with SD card
//#define IFMOSIPin 29 //J15-6 //just uses built in SPI, shared with SD card
//#define IFSCKPin 30  //J15-7 //just uses built in SPI, shared with SD card
#define IFCSPin 53 //J15-8
#define IFSDAPin 20 //J15-9
#define IFSCLPin 21 //J15-10

//Connector J22 - SD Card
//#define SDMISOPin 28 //just uses built in SPI, shared with Interface
//#define SDMOSIPin 29 //just uses built in SPI, shared with Interface
//#define SDSCKPin 30 //just uses built in SPI, shared with Interface
#define SDCSPin 44

//Connector J6 - Gear Sw
#define DriveSwPin 28 //J6-3
#define RevSwPin 29 //J6-4
#define ParkSwPin 30  //J6-5 //NOW DIFF STEERING ENABLE
#define TrqSpdSwPin 31 //J6-6 HIGH for SPD, LOW (open) for TRQ
#define LightsSwPin 32 //J6-7

//Connector J10 - RC
#define RCCH1Pin 2 //J10-3 Steer
#define RCCH2Pin 3 //J10-4 Throttle
#define RCCH3Pin 46  //J10-5 Aux
#define RCCH4Pin 47 //J10-6
#define RCCH5Pin 48 //J10-7
#define RCCH6Pin 49 //J10-8

//Other interfaces (J7/17/18 H Bridge channels, J21 GenDIO) are not yet defined/implemented



//======================Definitions for hoverboard comms=======================
#define HOVER_SERIAL_BAUD   115200       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define SPD_MODE            2            // [-] SPEED mode
#define TRQ_MODE            3            // [-] TORQUE mode
#define HB_TIMEOUT          950          // in ms for warning 
#define HB_TIMEOUT_START    1450          // in ms to triger restart // battery startup takes 7.48s. Hotstart from USB takes 4.8s
#define HB_RESTART_WAIT     5000          // in ms before retrying restart //hb takes about 2.5 seconds to restart and clear comms

//======================Settings=======================
int PedalCentre = 550;//old
float revspd = 0.8; //reverse throttle map multiplier
float DiffSteerCoeff = 1; //Diff steer weighting multiplier

int maxthrottleTRQ = 1200; //Max throttle command (in TRQ mode)
int maxthrottleSPD = 1200; //Max throttle command (in SPD mode) //was 100

int FHBpowerSplit = maxthrottleTRQ*0.50; //% max power of front HB in TRQ mode DISABLED
int RHB1powerSplit = maxthrottleTRQ*1.00; //% max power of rear1 HB in TRQ mode DISABLED
int RHB2powerSplit = RHB1powerSplit; //% max power of rear2 HB in TRQ mode DISABLED

#define RCMaxSpeed 25
#define RCSteerL 895
#define RCSteerMid 1425
#define RCSteerR 1950
#define RCThrottleF 1650
#define RCThrottleMid 1435
#define RCThrottleR 895
#define RCAuxMid 1450 //> = on

#define Brakehallthreshold 250
int pedaldeadband = 50;
#define AccelPedalStart 200
#define AccelPedalEnd 875
#define BrakePedalStart 350 //was 250. Crane sometimes flashed brake lights.
#define BrakePedalEnd 875

//#define CONFIG_VOLTCRANEO //Uncomment only one config
#define CONFIG_CYBRTRK //Uncomment only one config

#if defined(CONFIG_VOLTCRANEO) & defined(CONFIG_CYBRTRK)
#error MULTIPLE HACKY CONFIGS SELECTED
#elif defined(CONFIG_VOLTCRANEO)
  #define SteerCentre 512
  #define SteerLeft  256
  #define SteerRight  512+256
  //#define EnableDiffSteering
#elif defined(CONFIG_CYBRTRK)
  #define SteerCentre 476 
  #define SteerLeft  329 //301 348
  #define SteerRight  580 //562 588
  #define EnableDiffSteering
#else
#error HACKY CONFIG NOT SELECTED
#endif





#endif