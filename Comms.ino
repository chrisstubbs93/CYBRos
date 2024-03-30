/**
    \file Comms.ino
    Handles serial comms.

    REPL commands: (not implemented yet)
    q - quiet - mute all telemetry streaming
    v - verbose - unmute all telemetry streaming
    t - telemetry - toggle local telemetry
    g - gateway - toggle gateway of hoverboard telemetry
    l - list files - list datalog files on SD card
    sn - send file - send contents of file n.txt
    dy - delete all - delete all data, confirm with y, any other letter will cancel

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
  Serial.println(buf);
  logSD(buf);
}

void sendInfo(char *msg) {
  sprintf(buf, "$INFO,");
  sprintf(buf, "%s%s", buf, msg);
  sprintf(buf, "%s*%02X", buf, nmea0183_checksum(buf));
  Serial.println(buf);
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

  Serial.println(buf);
}



