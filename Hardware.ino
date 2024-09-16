/**
    \file Hardware.ino
    Handles initialisation of GPIO, scanning of buttons.
*/

//ezBuzzer buzzer(BuzzerPin); // create ezBuzzer object that attach to a pin;
unsigned long previousMillisBuzzer = 0;
unsigned long currentMillisBuzzer;

/*! Initialise the GPIO pins. */
void initGPIO() {
  pinMode(BuzzerPin, OUTPUT);

  pinMode(SteeringPot1Pin, INPUT);
  pinMode(SteeringPot2Pin, INPUT);

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
}

void initAnalog() {
  SteeringWheelVal.begin(SMOOTHED_AVERAGE, 10);
  SteeringFeedbackVal.begin(SMOOTHED_AVERAGE, 10);
  AccelPedalVal.begin(SMOOTHED_AVERAGE, 10);
  BrakePedalVal.begin(SMOOTHED_AVERAGE, 10);
  ManualBrakeVal.begin(SMOOTHED_AVERAGE, 10);
  FuseADC.begin(SMOOTHED_AVERAGE, 10);
  ShuntADC.begin(SMOOTHED_AVERAGE, 10);

  //before doing anything, feed the analog smoothing with at least the smoothing factor n samples
  for (int i = 0; i < 10; i ++) {
    processAnalog();
  }
}

void processAnalog(){
  //read all analogue in to smoothing function
  SteeringWheelVal.add(analogRead(SteeringPot1Pin));
  SteeringFeedbackVal.add(analogRead(SteeringPot2Pin));
  AccelPedalVal.add(analogRead(RPedalPin));
  BrakePedalVal.add(analogRead(LPedalPin));
  ManualBrakeVal.add(analogRead(BrakeHallPin));
  FuseADC.add(analogRead(FuseMonPin));
  ShuntADC.add(analogRead(ShuntMonPin));
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
    analogWrite(RLightPin, 255); 
  } else {
    //brakes off, dim rear DRL
    analogWrite(RLightPin, 50); 
  }
  
  digitalWrite(RLightPin, (analogRead(BrakeHallPin) > Brakehallthreshold || BrakePedalVal.get() > BrakePedalStart));
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