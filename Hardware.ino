/**
    \file Hardware.ino
    Handles initialisation of GPIO, scanning of buttons.
*/

//ezBuzzer buzzer(BuzzerPin); // create ezBuzzer object that attach to a pin;
unsigned long previousMillisBuzzer = 0;
unsigned long currentMillisBuzzer;
unsigned int BrakeFlasher;

/*! Initialise the GPIO pins. */
void initGPIO() {
  pinMode(BuzzerPin, OUTPUT);

  pinMode(SteeringPot1Pin, INPUT); //pot representing steering angle
  pinMode(SteeringPot2Pin, INPUT); //spare

  pinMode(FuseMonPin, INPUT);
  pinMode(ShuntMonPin, INPUT);

  pinMode(FHBPwrTellbackPin, INPUT);
  pinMode(FHBPwrCommandPin, OUTPUT);
  pinMode(RHB1PwrTellbackPin, INPUT);
  pinMode(RHB1PwrCommandPin, OUTPUT);
  pinMode(RHB2PwrTellbackPin, INPUT);
  pinMode(RHB2PwrCommandPin, OUTPUT);

  pinMode(BrakeHallPin, INPUT);
  pinMode(LPedalPin, INPUT);
  pinMode(RPedalPin, INPUT);

  pinMode(FLightPin, OUTPUT);
  pinMode(RLightPin, OUTPUT);

  //Pin modes for builtin SPI??
  //Pin modes for IFSDAPin IFSCLPin??
 pinMode(IFCSPin, OUTPUT);
 pinMode(SDCSPin, OUTPUT);

  pinMode(DriveSwPin, INPUT);
  pinMode(RevSwPin, INPUT);
  pinMode(ParkSwPin, INPUT); 
  pinMode(TrqSpdSwPin, INPUT);
  pinMode(LightsSwPin, INPUT);

  pinMode(RCCH1Pin, INPUT);
  pinMode(RCCH2Pin, INPUT);
  pinMode(RCCH3Pin, INPUT);
  pinMode(RCCH4Pin, INPUT);
  pinMode(RCCH5Pin, INPUT);
  pinMode(RCCH6Pin, INPUT);

  pinMode(SteerRISPin, OUTPUT);
  pinMode(SteerLPWMPin, OUTPUT);
  pinMode(SteerRPWMPin, OUTPUT);
  pinMode(SteerRENPin, OUTPUT);
}

void initAnalog() {
  SteeringFeedbackVal.begin(SMOOTHED_AVERAGE, 10);
  AccelPedalVal.begin(SMOOTHED_AVERAGE, 10);
  BrakePedalVal.begin(SMOOTHED_AVERAGE, 10);
  ManualBrakeVal.begin(SMOOTHED_AVERAGE, 10);
  FuseADC.begin(SMOOTHED_AVERAGE, 10);
  ShuntADC.begin(SMOOTHED_AVERAGE, 10);
  SteerCurr.begin(SMOOTHED_AVERAGE, 10);

  RCSteerPulse.begin(SMOOTHED_AVERAGE, 5);
  RCThrottlePulse.begin(SMOOTHED_AVERAGE, 5);
  RCAuxPulse.begin(SMOOTHED_AVERAGE, 5);

  //before doing anything, feed the analog smoothing with at least the smoothing factor n samples
  for (int i = 0; i < 10; i ++) {
    processAnalog();
    processRCPulse();
    delay(25);
  }
}

void processAnalog(){
  //read all analogue in to smoothing function
  SteeringFeedbackVal.add(analogRead(SteeringPot1Pin));
  AccelPedalVal.add(analogRead(RPedalPin));
  BrakePedalVal.add(analogRead(LPedalPin));
  ManualBrakeVal.add(analogRead(BrakeHallPin));
  FuseADC.add(analogRead(FuseMonPin));
  ShuntADC.add(analogRead(ShuntMonPin));
  SteerCurr.add(analogRead(SteerRISPin));
}

void processDigital(){
  if(digitalRead(TrqSpdSwPin)){
   currentDriveMode = SPD_MODE;
   maxthrottle = maxthrottleSPD;
  } else {
   currentDriveMode = TRQ_MODE;
   maxthrottle = maxthrottleTRQ;
  }

  //Front lights
  digitalWrite(FLightPin, digitalRead(LightsSwPin));

  //Brake lights
  //digitalWrite(RLightPin, (analogRead(BrakeHallPin) > Brakehallthreshold || BrakePedalVal.get() > BrakePedalStart));
  if((analogRead(BrakeHallPin) > Brakehallthreshold || BrakePedalVal.get() > BrakePedalStart)){
    //brakes on
    if(BrakeFlasher < 175){
      analogWrite(RLightPin, 255); 
    } else if (BrakeFlasher < 225) {
      analogWrite(RLightPin, 15);//flash brake
    } else {
      BrakeFlasher = 0;
    }
    BrakeFlasher++;
    
  } else {
    //brakes off, dim rear DRL
    analogWrite(RLightPin, 15); 
  }

  //Read gear
  if (digitalRead(DriveSwPin)) {
    currentGear = GEAR_D;
  }
  else if(digitalRead(RevSwPin)){
    currentGear = GEAR_R;
  }
  else{
    currentGear = GEAR_N;
  }

}

void processRCPulse(){
  // Read RC inputs (timeout prevents lockup if signal missing)
  RCSteerPulse.add(pulseIn(RCCH1Pin, HIGH, 25000));  // Steer
  RCThrottlePulse.add(pulseIn(RCCH2Pin, HIGH, 25000));  // Throttle
  RCAuxPulse.add(pulseIn(RCCH3Pin, HIGH, 25000));  // Aux
}

void checkFootAndHandBrakeHeld(){//Is this now old??
    //manual power up routine for hoverboards
  if (!digitalRead(DriveSwPin) && !digitalRead(RevSwPin)) {
    //in neutral
    if (AccelPedalVal.get() - PedalCentre < -300) {
      //holding foot brake
      if (ManualBrakeVal.get() > 250) {
        //holding hand brake
        //do something 
        while (ManualBrakeVal.get() > 250)
        {
          //wait until released
          ManualBrakeVal.add(analogRead(BrakeHallPin));
        }
      }
    }
  }
}

void buzzerEvent(int tonePattern){
  currentMillisBuzzer = millis();  //store time
  //Tasks to run each buzzer tick:
  if (currentMillisBuzzer - previousMillisBuzzer >= 1000) { //buzz at 1000ms interval
    previousMillisBuzzer = currentMillisBuzzer;
    int beepFreq, beepLen;

    switch (tonePattern) {
    case 1: //currentlimiting
      beepFreq = 850;
      beepLen = 1000;
      break;
    default: //error/undefined
      beepFreq = 600;
      beepLen = 250;
      break;
    }
    if (!disableBeep){tone(BuzzerPin, beepFreq, beepLen);}

  }
}