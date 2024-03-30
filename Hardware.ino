/**
    \file Hardware.ino
    Handles initialisation of GPIO, scanning of buttons.
*/

ezBuzzer buzzer(BuzzerPin); // create ezBuzzer object that attach to a pin;

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

void checkFootAndHandBrakeHeld(){
    //manual power up routine for hoverboards
  if (!digitalRead(DriveSwPin) && !digitalRead(RevSwPin)) {
    //in neutral
    if (AccelPedalVal.get() - PedalCentre < -300) {
      //holding foot brake
      if (ManualBrakeVal.get() > 250) {
        //holding hand brake
        powerOnHB();
        while (ManualBrakeVal.get() > 250)
        {
          //wait until released
          ManualBrakeVal.add(analogRead(BrakeHallPin));
        }
      }
    }
  }
}

void buzzerTick(){
  buzzer.loop();
}

void buzzerEvent(){
  int melody[] = {NOTE_E5, NOTE_G5};
  int noteDurations[] = {8, 4};
  buzzer.playMelody(melody, noteDurations, sizeof(noteDurations) / sizeof(int)); // playing
}