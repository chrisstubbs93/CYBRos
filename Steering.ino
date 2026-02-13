/**
    \file Steering.ino
    Handles closed loop steering control (disabled).
*/

//input: -100% to 100% of steering range over serial
//t o d o: detect failure to control
//t o d o: detect motor vin
//t o d o: Steer_lockout if values are out of safe range



//from main:
/*
int pot;
int sp;
int pos = 0;  //mapped steeringwheelval avg
int posraw = 0;
*/


#define Steer_lockout_time 10 // time in constant current before tripping to Steer_lockout mode
#define poslimit 100 //150 is somewhat arbritrary number that represents 90 degrees each way
#define Steer_izerooffset 350        //adc counts at 0 amps
#define Steer_ical 30                //adc counts per 1amp
double Steer_ilim = 80;                 //in 0.1A

int Steer_CC_iterations = 0;  //number of iterations in constant current mode
double Steer_pwm;
bool Steer_lockout = false;

// PID for posn control
double Steer_Posn_Pk = 1;  //speed it gets there was 1
double Steer_Posn_Ik = 6; //was 6
double Steer_Posn_Dk = 0.05; //was 0.05
double Steer_Posn_Sp, Steer_Posn_Ip, Steer_Posn_Op;
PID Steer_Posn_PID(&Steer_Posn_Ip, &Steer_Posn_Op, &Steer_Posn_Sp, Steer_Posn_Pk, Steer_Posn_Ik , Steer_Posn_Dk, DIRECT);

// PID steering current limit
double Steer_Curr_Pk = 0;  //I lim
double Steer_Curr_Ik = 15;
double Steer_Curr_Dk = 0;
double Steer_Curr_Ip, Steer_Curr_Op;
PID Steer_Curr_PID(&Steer_Curr_Ip, &Steer_Curr_Op, &Steer_ilim, Steer_Curr_Pk, Steer_Curr_Ik , Steer_Curr_Dk, DIRECT);


void setupSteeringControl(){
  Steer_Posn_PID.SetMode(AUTOMATIC);              // PID posn control loop
  Steer_Posn_PID.SetOutputLimits(-255, 255);
  Steer_Posn_PID.SetSampleTime(intervalA);

  Steer_Curr_PID.SetMode(AUTOMATIC);              // PID constant current loop
  Steer_Curr_PID.SetOutputLimits(-255, 255);
  Steer_Curr_PID.SetSampleTime(intervalA);

}

void SteeringControl() { 

  if (RCModeActive){
    digitalWrite(SteerLENPin, HIGH); 
    digitalWrite(SteerRENPin, HIGH); 
  } else {
    digitalWrite(SteerLENPin, LOW); 
    digitalWrite(SteerRENPin, LOW); 
  }

  //Steer_Posn_Sp = constrain(map(Steer_Posn_Sp, -100, 100, -poslimit, poslimit), -poslimit, poslimit);
  if (RCSteerPulse.get() < RCSteerMid) {                                     //assume left
    Steer_Posn_Sp = map(RCSteerPulse.get(), RCSteerL, RCSteerMid, SteerLeft, SteerCentre);    //assuming + steers left
  } else {                                                                           //assume right
    Steer_Posn_Sp = map(RCSteerPulse.get(), RCSteerMid, RCSteerR, SteerCentre, SteerRight);  //assuming - steers right
  }
  Steer_Posn_Sp = constrain(Steer_Posn_Sp, SteerLeft, SteerRight);

  //pot = SteeringFeedbackVal.get();//analogRead(SteeringFeedbackPin) + SteerCentreOffset;
  Steer_Posn_Ip = SteeringFeedbackVal.get();
  //Serial.println(Steer_Posn_Ip);
  Steer_Posn_PID.Compute();

  Steer_Curr_Ip = (SteerCurr.get() - Steer_izerooffset) * 10 / Steer_ical; //current feedback in 0.1A //used to be disabled so check this
  Steer_Curr_PID.Compute();

  //take minimum loop output
  if (abs(Steer_Posn_Op) <= abs(Steer_Curr_Op)) {
    //position mode
    steercurrentLimiting = false;
    Steer_pwm = Steer_Posn_Op;
    //Serial.print(" Mode:Pos");
    Steer_CC_iterations = 0;
  }
  else {
    //i lim mode
    steercurrentLimiting = false;
    Steer_pwm = Steer_Curr_Op;
    if (Steer_Posn_Op < 0) {
      Steer_pwm = Steer_Curr_Op * -1; //current loop is unidirectional so flip it for constant negative current output (CC reverse)
    }
    //Serial.println(" Mode:CC");
    Steer_CC_iterations++;
    //Serial.println(Steer_CC_iterations);
  }

  if (Steer_CC_iterations > (1000 * Steer_lockout_time) / intervalA) {
    Steer_lockout = false; //latch into Steer_lockout mode
  }
  if (Steer_lockout) {
    Steer_pwm = 0;
    digitalWrite(SteerRENPin, LOW); //how do we handle turning this off?
    digitalWrite(SteerRENPin, LOW); //how do we handle turning this off?
  }

  if (Steer_pwm > 0) {
    analogWrite(SteerRPWMPin, Steer_pwm);
    analogWrite(SteerLPWMPin, 0);
  } else {
    analogWrite(SteerRPWMPin, 0);
    analogWrite(SteerLPWMPin, abs(Steer_pwm));
  }
}

//local pos = constrain(map(SteeringWheelVal.get(), 0, 1024, -100, 100), -100, 100);
//remote  pos = constrain(posraw, -100, 100);
////wiperServo(pos);              // tell servo to go to position in variable 'pos'


  //Serial.print(" pos_sp:");
  //Serial.print (Steer_Posn_Sp);
  //Serial.print(" pos_ip:");
  //Serial.print (Steer_Posn_Ip);
  //Serial.print(" pos_op:");
  //Serial.print(Steer_Posn_Op);
  //Serial.print(" I_raw:");
  //Serial.print(I_reading);
  //Serial.print(" I_ip:");
  //Serial.print(Steer_Curr_Ip);
  //Serial.print(" I_op:");
  //Serial.print(Steer_Curr_Op);
// print Steering control mode pos/ilim
//lockout status Steer_lockout

 char steerbuf[128];
 char floatStrBuf[20];
void sendSteeringtelem() {
  sprintf(steerbuf, "$STEER");

  sprintf(steerbuf, "%s,%d", steerbuf, RCModeActive);

  sprintf(steerbuf, "%s,%s", steerbuf, (dtostrf(Steer_Posn_Ip, 1, 0, floatStrBuf), floatStrBuf));
  sprintf(steerbuf, "%s,%s", steerbuf, (dtostrf(Steer_Posn_Op, 1, 0, floatStrBuf), floatStrBuf));
  sprintf(steerbuf, "%s,%s", steerbuf, (dtostrf(Steer_Posn_Sp, 1, 0, floatStrBuf), floatStrBuf));

  sprintf(steerbuf, "%s,%s", steerbuf, (dtostrf(Steer_Curr_Ip, 1, 0, floatStrBuf), floatStrBuf));
  sprintf(steerbuf, "%s,%s", steerbuf, (dtostrf(Steer_Curr_Op, 1, 0, floatStrBuf), floatStrBuf));
  sprintf(steerbuf, "%s,%s", steerbuf, (dtostrf(Steer_ilim,    1, 0, floatStrBuf), floatStrBuf));

  sprintf(steerbuf, "%s,%d", steerbuf, steercurrentLimiting);
  sprintf(steerbuf, "%s,%d", steerbuf, Steer_lockout);
  

  //checksum
  sprintf(steerbuf, "%s*%02X", steerbuf, nmea0183_checksum(steerbuf));

  //if(!quietSerial)Serial.println(buf);
  Serial.println(steerbuf);
  logSD(steerbuf);
}


