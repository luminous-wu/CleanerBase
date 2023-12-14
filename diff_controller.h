/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  double PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  double ITerm;                    //integrated term

  double output;                    // last motor setting

  double PrevMidInput;
  
}
SetPointInfo;

/* PID Parameters */
double Kp = 4.5;
double Ki = 0.01;
double Kd = 30;
double Ko = 50;

typedef struct {

  float targetVel;
  long Encoder;
  long PrevEnc;

  float actualVel;
  float error;
  float error_next;
  float error_last;
  float crossCouplGain;
  float Kp, Ki, Kd;
  float output;

}sCrossCoupling;


SetPointInfo leftPID{0.0, 0, 0, 0, 0, 0, 0};
SetPointInfo rightPID{0.0, 0, 0, 0, 0, 0, 0};

sCrossCoupling sLeftCrossCoupl{0.0, 0, 0, 0, 0, 0, 0, 0, 4.5, 0.01, 30, 50};
sCrossCoupling sRightCrossCoupl{0.0, 0, 0, 0, 0, 0, 0, 0, 4.5, 0.01, 30, 50};

// /* PID Parameters */
// float Kp = 4.5;
// float Ki = 0.01;
// float Kd = 30;
// float Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;
  //  leftPID.crossCouplGain = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
  //  rightPID.crossCouplGain = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  // 实时速度 = 当前编码器计数 - 上次调试结束时的编码器计数；(单位时间是 1/30 秒)
  input = p->Encoder - p->PrevEnc;
  // 误差 = 目标值 - 当前值
  Perror = p->TargetTicksPerFrame - input;
//  Serial.println(input);

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }
  // 分别调试左右轮
  /* Compute PID update for each motor */
   doPID(&rightPID);
   doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}

void resetCrossCoupl(){

  sLeftCrossCoupl.targetVel = 0.0;
  sLeftCrossCoupl.Encoder = readEncoder(LEFT);
  sLeftCrossCoupl.PrevEnc = sLeftCrossCoupl.Encoder;
  sLeftCrossCoupl.actualVel = 0;
  sLeftCrossCoupl.error = 0;
  sLeftCrossCoupl.error_next = 0;
  sLeftCrossCoupl.error_last = 0;
  sLeftCrossCoupl.crossCouplGain = 0;
  sLeftCrossCoupl.output = 0;

  sRightCrossCoupl.targetVel = 0.0;
  sRightCrossCoupl.Encoder = readEncoder(RIGHT);
  sRightCrossCoupl.PrevEnc = sRightCrossCoupl.Encoder;
  sRightCrossCoupl.actualVel = 0;
  sRightCrossCoupl.error = 0;
  sRightCrossCoupl.error_next = 0;
  sRightCrossCoupl.error_last = 0;
  sRightCrossCoupl.crossCouplGain = 0;
  sRightCrossCoupl.output = 0;
}

void doCrossCoupl(sCrossCoupling* sLeft, sCrossCoupling* sRight) {
  float crossCouplError;
  float Kp_CC;
  float Ki_CC;
  float Kd_CC;

  sLeft->crossCouplGain = 1;
  sRight->crossCouplGain = sLeft->targetVel / sRight->targetVel;

  sLeft->actualVel = sLeft->Encoder - sLeft->PrevEnc;
  sRight->actualVel = sRight->Encoder - sRight->PrevEnc;

  crossCouplError = sLeft->actualVel * sLeft->crossCouplGain - sRight->actualVel * sRight->crossCouplGain;

  crossCouplError *= Kp_CC;

  sLeft->error = sLeft->targetVel - crossCouplError - sLeft->actualVel;
  sRight->error = sRight->targetVel + crossCouplError - sRight->actualVel;

  float leftIncreVel = sLeft->Kp * (sLeft->error - sLeft->error_next) + sLeft->Ki * sLeft->error+ sLeft->Kd * (sLeft->error - 2 * sLeft->error_next + sLeft->error_last);
  float rightIncreVel =  sRight->Kp * (sRight->error - sRight->error_next) + sRight->Ki * sRight->error + sRight->Kd * (sRight->error - 2 * sRight->error_next + sRight->error_last);

  sLeft->error_last = sLeft->error_next;
  sLeft->error_next = sLeft->error;

  sRight->error_last = sRight->error_next;
  sRight->error_next = sRight->error;

  sLeft->actualVel += leftIncreVel;
  sRight->actualVel += rightIncreVel;

}
/*
void doCrossCoupl(SetPointInfo* sLeftCrossCoupl, SetPointInfo* sRightCrossCoupl) {
  double leftError, rightError;
  double leftMidInput, rightMidInput;
  double leftOutput, rightOutput;
  double leftActualVel, rightActualVel;
  double crossCouplError;
  double leftCrossCouplGain, rightCrossCouplGain;

  leftActualVel = sLeftCrossCoupl->Encoder - sLeftCrossCoupl->PrevEnc;
  rightActualVel = sRightCrossCoupl->Encoder - sRightCrossCoupl->PrevEnc;
  Serial.print("leftActualVel:");
  Serial.print(leftActualVel);
  Serial.print("  ");
  Serial.print("rightActualVel:");
  Serial.println(rightActualVel);

  leftCrossCouplGain = sRightCrossCoupl->TargetTicksPerFrame;
  rightCrossCouplGain = sLeftCrossCoupl->TargetTicksPerFrame;

  crossCouplError = leftCrossCouplGain * leftActualVel - rightCrossCouplGain * rightActualVel;
  
  leftMidInput = sLeftCrossCoupl->TargetTicksPerFrame - crossCouplError;
  rightMidInput = sRightCrossCoupl->TargetTicksPerFrame + crossCouplError;

  leftError = leftMidInput - leftActualVel;
  rightError = rightMidInput - rightActualVel;

  leftOutput = (sLeftCrossCoupl->Kp * leftError +sLeftCrossCoupl->ITerm + sLeftCrossCoupl->Kd * (leftMidInput - sLeftCrossCoupl->PrevMidInput)) / sLeftCrossCoupl->Ko;
  rightOutput = (sRightCrossCoupl->Kp * rightError +sRightCrossCoupl->ITerm + sRightCrossCoupl->Kd * (rightMidInput - sRightCrossCoupl->PrevMidInput)) / sRightCrossCoupl->Ko;

  sLeftCrossCoupl->PrevEnc = sLeftCrossCoupl->Encoder;
  sRightCrossCoupl->PrevEnc = sRightCrossCoupl->Encoder;

  leftOutput += sLeftCrossCoupl->output;
  rightOutput += sRightCrossCoupl->output;

  if(leftOutput >= MAX_PWM)
    leftOutput = MAX_PWM;
  else if(leftOutput <= -MAX_PWM)
    leftOutput = -MAX_PWM;
  else
    sLeftCrossCoupl->ITerm += sLeftCrossCoupl->Ki * leftError;
    
  if(rightOutput >= MAX_PWM)
    rightOutput = MAX_PWM;
  else if(rightOutput <= -MAX_PWM)
    rightOutput = -MAX_PWM;
  else
    sRightCrossCoupl->ITerm += sRightCrossCoupl->Ki * rightError;

  sLeftCrossCoupl->output = leftOutput;
  sRightCrossCoupl->output = rightOutput;

  sLeftCrossCoupl->PrevMidInput = leftMidInput;
  sRightCrossCoupl->PrevMidInput = rightMidInput;

}

void updateCrossCoupl() {
  sLeftCrossCoupl.Encoder = readEncoder(LEFT);
  sRightCrossCoupl.Encoder = readEncoder(RIGHT);

  if (!moving) {
    if (sLeftCrossCoupl.PrevMidInput != 0 || sRightCrossCoupl.PrevMidInput != 0) resetCrossCoupl();
    return;
  }

  doCrossCoupl(&sLeftCrossCoupl, &sRightCrossCoupl);
  setMotorSpeeds(sLeftCrossCoupl.output, sRightCrossCoupl.output);
}
*/
