/**
    \file Comms.ino
    Handles serial comms.
*/

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

void sendoldtelem() {
  //structure: $STEER,INPUT,GEAR,MANUALBRAKE,PEDALAVG,FUSEADC,
  //THROTTLEIP,THROTTLEOP,SHUNTADC,CURRENTOP(not used),
  //CURRENTLIMITING,LOCKOUT,SENTSPEED,SENTBRAKE*AA

  // $STEER,0,D,0,-9,-63,0,0,0.67,0,0,0,0,0*2E out of date

  char buf[64];
  sprintf(buf, "$STEER");

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

  //pedalavg (-550 (e-brake) - 0 - ~+500 (full accel))
  sprintf(buf, "%s,%d", buf, AccelPedalVal.get() - PedalCentre);

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

