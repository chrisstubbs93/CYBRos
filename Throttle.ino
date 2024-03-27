/**
    \file Throttle.ino
    Handles closed loop throttle control.
*/

// PID fuse voltage control limit
double Fuse_Pk = 0;  //I lim
double Fuse_Ik = 5;
double Fuse_Dk = 0;

double FuseIlim = 850; //  in very nonlinear units about 0.1A //Determined experimentally by measuring runaway point as fuse blows
PID FusePID(&Input3Fuse, &Output3Throttle, &FuseIlim, Fuse_Pk, Fuse_Ik , Fuse_Dk, DIRECT);


void setupThrottleFuseControl(){
  FusePID.SetMode(AUTOMATIC);              // PID constant current loop
  FusePID.SetOutputLimits(0, 1200);
  FusePID.SetSampleTime(20);
}


void throttlecontrol(){
//Handle braking
    if (analogRead(BrakeHallPin) > 250) {
      //Serial.println("Brake On");
      manualBraking = true;
      drvcmd = 0;
      brkcmd = 1000;
      Send(0, drvcmd, brkcmd, currentDriveMode);  //also regenbrake if manual braking //TDOD test this works?
    } else {                                      //brake is not on
      manualBraking = false;
      //TODO: and check if in local mode
      if (AccelPedalVal.get() - PedalCentre > pedaldeadband) {
        drvcmd = map(AccelPedalVal.get() - PedalCentre, pedaldeadband, (1023 - PedalCentre), 0, 1200);
        brkcmd = 0;
        //hoverboard firmware input range is -1000 to 1000 (or 1200 lol)
        if (digitalRead(DriveSwPin)) {
          //Send(0, drvcmd, brkcmd, currentDriveMode); //non PID mode disabled
          //only do PID throttle/fuse control in forward drive when not braking for safety
          Send(0, ThrottleFuseControl(drvcmd), brkcmd, currentDriveMode);
        } else if (digitalRead(RevSwPin)) {
          Send(0, -drvcmd * revspd, brkcmd, currentDriveMode);
        } else { //in neutral
          drvcmd = 0;
          brkcmd = 0;
          Send(0, drvcmd, brkcmd, currentDriveMode);
        }
      } else if (AccelPedalVal.get() - PedalCentre < (0 - pedaldeadband)) {                      //brake
        brkcmd = map(PedalCentre - AccelPedalVal.get(), pedaldeadband, (PedalCentre), 0, 1000);  //500 = full brake
        drvcmd = 0;
        Send(0, drvcmd, brkcmd, currentDriveMode);
      } else {
        drvcmd = 0;
        brkcmd = 0;
        Send(0, drvcmd, brkcmd, currentDriveMode);  //stop if no pedal input
      }
    }
}


int ThrottleFuseControl(int throttleSP) {
  //remember to bypass this for braking! -ve throttle
  // if (AccelPedalVal.get() - PedalCentre > pedaldeadband) {
  //   //Accelerate
  // } else if (AccelPedalVal.get() - PedalCentre < (0 - pedaldeadband)) {
  //   //brake
  // } else {
  //   drvcmd = 0;
  //   brkcmd = 0;
  //   Send(0, drvcmd, brkcmd, currentDriveMode); //stop if no pedal input
  // }

  Input3Fuse = (SteeringFeedbackVal.get()*10)/2; //SteeringFeedbackVal = FuseADC (scaled to almost 0.1 amps nonlinear)
  FusePID.Compute();
  // Serial.print("PID fed with ");
  // Serial.println(Input3Fuse);
  // Serial.print("PID OP ");
  // Serial.println(Output3Throttle);

  //TODO datalog loop inputs, outputs, modes. Do tuning

  //take minimum loop output
  if (abs(throttleSP) <= abs(Output3Throttle)) {
    //throttle mode
    fusecurrentLimiting = false;
    return throttleSP;
  }
  else {
    //i lim mode
    fusecurrentLimiting = true;
    return Output3Throttle;
  }
}