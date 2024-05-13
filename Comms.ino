/**
    \file Comms.ino
    Handles serial comms.

    REPL commands: (not implemented yet)
    q - quiet - mute all non cmd serial messages 
    t - telemetry - toggle local telemetry
    ls - list files - list datalog files on SD card
    s n - send file - send contents of file n.txt
    del - delete all - delete all data
    br n - set baud rate to n eg 115200, 1000000

*/
  char buf[128];

int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;
  // ignore the first $ sign,  no checksum in sentence
  for (i = 1; i < strlen(nmea_data); i ++) { // removed the - 3 because no cksum is present
    crc ^= nmea_data[i];
  }
  return crc;
}

void sendError(char *msg) {
  buzzerEvent();
  sprintf(buf, "$ERROR,");
  sprintf(buf, "%s%s", buf, msg);
  sprintf(buf, "%s*%02X", buf, nmea0183_checksum(buf));
  if(!quietSerial)Serial.println(buf);
  logSD(buf);
}

void sendInfo(char *msg) {
  sprintf(buf, "$INFO,");
  sprintf(buf, "%s%s", buf, msg);
  sprintf(buf, "%s*%02X", buf, nmea0183_checksum(buf));
  if(!quietSerial)Serial.println(buf);
  logSD(buf);
}


void sendoldtelem() {
  //structure: $STEER,INPUT,GEAR,MANUALBRAKE,PEDALAVG,FUSEADC,
  //THROTTLEIP,THROTTLEOP,SHUNTADC,CURRENTOP(not used),
  //CURRENTLIMITING,LOCKOUT,SENTSPEED,SENTBRAKE*AA

  // $STEER,0,D,0,-9,-63,0,0,0.67,0,0,0,0,0*2E out of date

  sprintf(buf, "$CON");

  //GEAR: D/N/R
  if (digitalRead(DriveSwPin)) {
    sprintf(buf, "%s,D", buf);
  } else if (digitalRead(RevSwPin)) {
    sprintf(buf, "%s,R", buf);
  } else {
    sprintf(buf, "%s,N", buf);
  }

  //manualbrake 0/1 (1 is braking)
  sprintf(buf, "%s,%d", buf, manualBraking);

  //Accel input 
  sprintf(buf, "%s,%d", buf, AccelPedalVal.get());

  //Brake input
  sprintf(buf, "%s,%d", buf, BrakePedalVal.get());

  //fuseADC in adc counts
  sprintf(buf, "%s,%d", buf, (SteeringFeedbackVal.get()*10)/2); //was pos

  //throttleip (feedback)
  sprintf(buf, "%s,%d", buf, int(Input3Fuse));

  //throttleop (loop output)
  sprintf(buf, "%s,%d", buf, int(Output3Throttle));

  //ShuntADC in adc counts
  sprintf(buf, "%s,%d", buf, (ShuntADC.get()*10)/6);

  //currentlimiting 0/1 1 is limiting (fuse current limiting)
  sprintf(buf, "%s,%d", buf, fusecurrentLimiting);

  //sentspeed (0-1000)
  sprintf(buf, "%s,%d", buf, drvcmd);

  //sentbrake (0-1000)
  sprintf(buf, "%s,%d", buf, brkcmd);

  //checksum
  sprintf(buf, "%s*%02X", buf, nmea0183_checksum(buf));

  if(!quietSerial)Serial.println(buf);
  logSD(buf);
}

void sendHovertelem(SerialFeedback &Feedback, int hbnum) {
  //    int16_t  cmd1;
  //    int16_t  cmd2;
  //    int16_t  speedR_meas;
  //    int16_t  speedL_meas;
  //    int16_t  batVoltage;
  //    int16_t  dcCurrent;
  //    int16_t  boardTemp;
  //    uint16_t cmdLed;

  // $STEER,0,D,0,-9,-63,0,0,0.67,0,0,0,0,0*2E out of date

  sprintf(buf, "$HB");

  //timestamp

  sprintf(buf, "%s,%d", buf, hbnum);

  sprintf(buf, "%s,%d", buf, Feedback.cmd1);
  sprintf(buf, "%s,%d", buf, Feedback.cmd2);
  sprintf(buf, "%s,%d", buf, Feedback.speedR_meas);
  sprintf(buf, "%s,%d", buf, Feedback.speedL_meas);
  sprintf(buf, "%s,%d", buf, Feedback.batVoltage);
  sprintf(buf, "%s,%d", buf, Feedback.dcCurrent);
  sprintf(buf, "%s,%d", buf, Feedback.boardTemp);
  sprintf(buf, "%s,%u", buf, Feedback.cmdLed);

  //checksum
  sprintf(buf, "%s*%02X", buf, nmea0183_checksum(buf));

  if(!quietSerial)Serial.println(buf);
  logSD(buf);
}



