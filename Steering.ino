/**
    \file Steering.ino
    Handles closed loop steering control (disabled).
*/

//input: -100% to 100% of steering range over serial
//t o d o: detect failure to control
//t o d o: detect motor vin
//t o d o: lockout if values are out of safe range

/**
// PID for posn control
double Pk1 = 7;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID steering current limit
double Pk2 = 0;  //I lim
double Ik2 = 15;
double Dk2 = 0;
double Input2, Output2;
PID PID2(&Input2, &Output2, &ilim, Pk2, Ik2 , Dk2, DIRECT);

int lockout_time = 10; // time in constant current before tripping to lockout mode
bool lockout = false;
int poslimit = 100; //150 is somewhat arbritrary number that represents 90 degrees each way
double pwm;
float izerooffset = 350.0;        //adc counts at 0 amps
float ical = 30.0;                //adc counts per 1amp
double ilim = 80;                 //in 0.1A
unsigned long CC_iterations = 0;  //number of iterations in constant current mode


void setupSteeringControlPID(){
  PID1.SetMode(AUTOMATIC);              // PID posn control loop
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

  PID2.SetMode(AUTOMATIC);              // PID constant current loop
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(20);
  digitalWrite(MotorEnPin, HIGH);
}

void wiperServo(int sp) { //disabled
  Setpoint1 = constrain(map(sp, -100, 100, 0 - poslimit, poslimit), 0 - poslimit, poslimit);
  pot = SteeringFeedbackVal.get();//analogRead(SteeringFeedbackPin) + SteerCentreOffset;
  //Serial.print(" pos_sp:");
  //Serial.print (Setpoint1);
  //Serial.print(" pos_ip:");
  Input1 = map(pot, 0, 1023, -255, 255);
  //Serial.print (Input1);
  PID1.Compute();
  //Serial.print(" pos_op:");
  //Serial.print(Output1);
  I_reading = analogRead(CurrentSensePin);
  //Serial.print(" I_raw:");
  //Serial.print(I_reading);
  //Input2 = (I_reading - izerooffset) * 10 / ical; //current feedback in 0.1A //disabled to do fuse monitoring
  Input2 = I_reading;
  //Serial.print(" I_ip:");
  //Serial.print(Input2);
  PID2.Compute();
  //Serial.print(" I_op:");
  //Serial.print(Output2);

  //  if (digitalRead(DriveSwPin)) {
  //    Serial.print(" Gear:D ");
  //  } else if (digitalRead(RevSwPin)) {
  //    Serial.print(" Gear:R ");
  //  } else {
  //    Serial.print(" Gear:N ");
  //  }

  //take minimum loop output
  if (abs(Output1) <= abs(Output2)) {
    //position mode
    currentLimiting = false;
    pwm = Output1;
    //Serial.print(" Mode:Pos");
    CC_iterations = 0;
  }
  else {
    //i lim mode
    currentLimiting = false;
    pwm = Output2;
    if (Output1 < 0) {
      pwm = Output2 * -1; //current loop is unidirectional so flip it for constant negative current output (CC reverse)
    }
    //Serial.println(" Mode:CC");
    CC_iterations++;
    //Serial.println(CC_iterations);
  }

  if (CC_iterations > (1000 * lockout_time) / interval) {
    lockout = false; //latch into lockout mode
  }
  if (lockout) {
    pwm = 0;
    digitalWrite(EstopPin, HIGH);
    //Serial.println("LOCKOUT");
  }

  if (pwm > 0) {
    analogWrite(RPWMpin, pwm);
    analogWrite(LPWMpin, 0);
  } else {
    analogWrite(RPWMpin, 0);
    analogWrite(LPWMpin, abs(pwm));
  }
}

//local pos = constrain(map(SteeringWheelVal.get(), 0, 1024, -100, 100), -100, 100);
//remote  pos = constrain(posraw, -100, 100);
////wiperServo(pos);              // tell servo to go to position in variable 'pos'

*/