/**
    \file Throttle.ino
    Handles closed loop throttle control.
*/

// PID fuse voltage control limit
double Fuse_Pk = 0;  //I lim
double Fuse_Ik = 5;
double Fuse_Dk = 0;

//scrumpyu increased from 550 to 625
double FuseIlim = 700;  //  650 stable at scrumpy 25, was 700 for cybr truk in very nonlinear units about 0.1A //Determined experimentally by measuring runaway point as fuse blows as 850. Lowered to 700 at footfest
PID FusePID(&Input3Fuse, &Output3Throttle, &FuseIlim, Fuse_Pk, Fuse_Ik, Fuse_Dk, DIRECT);


void setupThrottleFuseControl() {
  FusePID.SetMode(AUTOMATIC);  // PID constant current loop
  FusePID.SetOutputLimits(0, 1200);
  FusePID.SetSampleTime(20);
}


void throttlecontrol() {
  //Handle braking
  if (analogRead(BrakeHallPin) > Brakehallthreshold) {  //if emergency brake (or parking brake are applied || digitalRead(ParkSwPin) REMOVED)
    //if trq mode
    manualBraking = true;
    drvcmd = 0;
    brkcmd = 1000;
    sendInfo("Manual Braking");
    Send(0, drvcmd, brkcmd, currentDriveMode);  //also ebrake if manual braking
  }
  else {                                      //brake is not on
    manualBraking = false;

    if (BrakePedalVal.get() > BrakePedalStart) {                                   //brake always takes priority
      brkcmd = map(BrakePedalVal.get(), BrakePedalStart, BrakePedalEnd, 0, 1000);  //500 = full brake
      drvcmd = 0;
      Send(0, drvcmd, brkcmd, currentDriveMode);
    }
    else {  //in drive or neutral
      drvcmd = constrain(map(AccelPedalVal.get(), AccelPedalStart, AccelPedalEnd, 0, maxthrottle), 0, 1200);
      brkcmd = 0;
      if (digitalRead(DriveSwPin)) {
        //calculate diff steering - only applied in drive
        strcmd = 0;
        #if defined(EnableDiffSteering)
          if (digitalRead(ParkSwPin)) {//reused as diff switch
            if (SteeringFeedbackVal.get() < SteerCentre) {                                     //assume left
              strcmd = map(SteeringFeedbackVal.get(), SteerLeft, SteerCentre, maxSteer, 0);    //assuming + steers left
            } else {                                                                           //assume right
              strcmd = map(SteeringFeedbackVal.get(), SteerCentre, SteerRight, 0, -maxSteer);  //assuming - steers right
            }
          }
        #endif
        strcmd = constrain(strcmd, -maxSteer, maxSteer);
        strcmd = strcmd * DiffSteerCoeff;
        //only do PID throttle/fuse control in forward drive when not braking for safety
        Send(strcmd, ThrottleFuseControl(drvcmd), brkcmd, currentDriveMode);  //TODO is this a good idea? Should we at least be feeding the PID loop at all times?
      }                                                                       //end of drive
      else if (digitalRead(RevSwPin)) {
        Send(0, -drvcmd * revspd, brkcmd, currentDriveMode);
      } else {                        //in neutral (also RC mode)
        if (RCAuxPulse.get() > RCAuxMid) {  //RC Aux Switch turned ON to activate remote mode
        RCModeActive = true;
          if (RCThrottlePulse.get() > RCThrottleMid) {  //fwd
            drvcmd = constrain(map(RCThrottlePulse.get(), RCThrottleMid, RCThrottleF, 0, RCMaxSpeed), 0, RCMaxSpeed);
          } else {  //rev
            drvcmd = constrain(map(RCThrottlePulse.get(), RCThrottleMid, RCThrottleR, 0, -RCMaxSpeed), -RCMaxSpeed * revspd, 0);
          }
          //strcmd = constrain(map(RCSteerPulse.get(), RCSteerR, RCSteerL, -RCMaxSpeed, RCMaxSpeed), -RCMaxSpeed, RCMaxSpeed);
          strcmd = 0;
          Send(strcmd, drvcmd, 0, SPD_MODE);
        }
        else {
          //in N with no RC input
          RCModeActive = false;
          Send(0, 0, 0, currentDriveMode);
        }
      }
    }
  }
}


int ThrottleFuseControl(int throttleSP) {

  //Quick disable:
  //return throttleSP;

  Input3Fuse = (FuseADC.get() * 10) / 2;
  if (Input3Fuse > Input3FuseMaxHold) { Input3FuseMaxHold = Input3Fuse; }
  FusePID.Compute();

  //TODO datalog loop inputs, outputs, modes. Do tuning
  //TODO switch to feed fusemon or digital current feedback in to control loop
  //TODO implement torque split

  throttleSpMonitor = throttleSP;

  //take minimum loop output
  if (abs(throttleSP) <= abs(Output3Throttle)) {
    //throttle mode
    fusecurrentLimiting = false;
    return throttleSP;
  } else {
    //i lim mode
    fusecurrentLimiting = true;
    buzzerEvent(1);
    return Output3Throttle;
  }
}