/**
    \file Throttle.ino
    Handles closed loop throttle control.
*/

// PID fuse voltage control limit
double Fuse_Pk = 0;  //I lim
double Fuse_Ik = 5;
double Fuse_Dk = 0;

double FuseIlim = 700; //  in very nonlinear units about 0.1A //Determined experimentally by measuring runaway point as fuse blows as 850. Lowered to 700 at footfest
PID FusePID(&Input3Fuse, &Output3Throttle, &FuseIlim, Fuse_Pk, Fuse_Ik , Fuse_Dk, DIRECT);


void setupThrottleFuseControl(){
  FusePID.SetMode(AUTOMATIC);              // PID constant current loop
  FusePID.SetOutputLimits(0, 1200);
  FusePID.SetSampleTime(20);
}


void throttlecontrol(){
//Handle braking
    if (analogRead(BrakeHallPin) > Brakehallthreshold) { //if emergency brake (or parking brake are applied || digitalRead(ParkSwPin) REMOVED)
      //if trq mode
      manualBraking = true;
      drvcmd = 0;
      brkcmd = 1000;
      sendInfo("Manual Braking");
      Send(0, drvcmd, brkcmd, currentDriveMode);  //also ebrake if manual braking
    } else {                                      //brake is not on
      manualBraking = false;
      if (AccelPedalVal.get() > AccelPedalStart) {
        drvcmd = constrain(map(AccelPedalVal.get(), AccelPedalStart, AccelPedalEnd, 0, maxthrottle),0,1200);
        brkcmd = 0;
        //hoverboard firmware input range is -1000 to 1000 (or 1200 lol)
        if (digitalRead(DriveSwPin)) {
          //Send(0, drvcmd, brkcmd, currentDriveMode); //non PID mode disabled
          //only do PID throttle/fuse control in forward drive when not braking for safety

          //calculate diff steering - only applied in drive
          #if defined(EnableDiffSteering)
            if (SteeringFeedbackVal.get() < SteerCentre){ //assume left
              strcmd = map(SteeringFeedbackVal.get(), SteerLeft, SteerCentre, maxSteer, 0);  //assuming + steers left
            } else { //assume right
              strcmd = map(SteeringFeedbackVal.get(), SteerCentre, SteerRight, 0, -maxSteer);  //assuming - steers right
            }
          #else
            strcmd = 0;
          #endif
          strcmd = constrain(strcmd,-maxSteer,maxSteer);
          strcmd = strcmd*DiffSteerCoeff;

          Send(strcmd, ThrottleFuseControl(drvcmd), brkcmd, currentDriveMode); //TODO is this a good idea? Should we at least be feeding the PID loop at all times?
        } else if (digitalRead(RevSwPin)) {
          Send(0, -drvcmd * revspd, brkcmd, currentDriveMode);
        } else { //in neutral
          drvcmd = 0;
          brkcmd = 0;
          Send(0, drvcmd, brkcmd, currentDriveMode);
        }
      } else if (BrakePedalVal.get() > BrakePedalStart) {                      //brake //should probably check this first
        brkcmd = map(BrakePedalVal.get(), BrakePedalStart, BrakePedalEnd, 0, 1000);  //500 = full brake
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

  //Quick disable:
  //return throttleSP;

  //#if defined(CONFIG_VOLTCRANEO)
    //bypass pid fuse control for crane because it's not fitted - BUT NOW IT IS!
  //  return throttleSP;
  //#endif

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

  Input3Fuse = (FuseADC.get()*10)/2; //SteeringFeedbackVal = FuseADC (NOT TRUE ANYMORE) (scaled to almost 0.1 amps nonlinear) //THIS WAS WRONG *when it was steeringfeedback*
  if(Input3Fuse > Input3FuseMaxHold) {Input3FuseMaxHold = Input3Fuse;}
  FusePID.Compute();

   //Serial.print("PID fed with ");
   //Serial.println(Input3Fuse);
   //Serial.print("PID OP ");
   //Serial.println(Output3Throttle);

  //TODO datalog loop inputs, outputs, modes. Do tuning
  //TODO switch to feed fusemon or digital current feedback in to control loop
  //TODO implement torque split

  throttleSpMonitor = throttleSP;

  //take minimum loop output
  if (abs(throttleSP) <= abs(Output3Throttle)) {
    //throttle mode
    fusecurrentLimiting = false;
    return throttleSP;
  }
  else {
    //i lim mode
    fusecurrentLimiting = true;
    buzzerEvent(1);
    return Output3Throttle;
  }
}