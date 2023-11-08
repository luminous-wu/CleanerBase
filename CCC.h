#ifndef CCC_H   // CCC: Cross Coupling Control
#define CCC_H

// #define CCC_INC
#define CCC_POS

#define MAX_SPEED                   10000
#define deltaVelThreshold           30
extern int32_t i32LeftTempVel;
extern int32_t i32RightTempVel;

typedef struct {
    int targetVel;
    int actualVel;
    int actualVel_last;
    // long Encoder;
    // long Encoder_last;
    float error;
    float error_next;
    float error_last;
    float crossCouplGain;
    float iTerm;
    float Kp, Ki, Kd;
} sCrossCoupl;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float output;
    float error;
    float error_last;
    float iTerm;
}sCrossCouplPara;

unsigned char moving = 0;
sCrossCoupl sLeftCrossCoupl, sRightCrossCoupl;
sCrossCouplPara sMotorsCrossCouplPara{0.5, 0.02, -0.5};

void resetCrossCoupl() {
    sLeftCrossCoupl.targetVel = 0.0;
    sLeftCrossCoupl.actualVel = readFilterVel(LEFT);
    sLeftCrossCoupl.actualVel_last = sLeftCrossCoupl.actualVel;
    // sLeftCrossCoupl.Encoder = readEncoder(LEFT);
    // sLeftCrossCoupl.Encoder_last = sLeftCrossCoupl.Encoder;
    sLeftCrossCoupl.error = 0.0;
    sLeftCrossCoupl.error_next = 0.0;
    sLeftCrossCoupl.error_last = 0.0;
    sLeftCrossCoupl.crossCouplGain = 0.0;
    
    sRightCrossCoupl.targetVel = 0.0;
    sRightCrossCoupl.actualVel = readFilterVel(RIGHT);
    sRightCrossCoupl.actualVel_last = sRightCrossCoupl.actualVel;
    // sRightCrossCoupl.Encoder = readEncoder(RIGHT);
    // sRightCrossCoupl.Encoder_last = sRightCrossCoupl.Encoder;
    sRightCrossCoupl.error = 0.0;
    sRightCrossCoupl.error_next = 0.0;
    sRightCrossCoupl.error_last = 0.0;
    sRightCrossCoupl.crossCouplGain = 0.0;
#ifdef CCC_INC
    sLeftCrossCoupl.iTerm = 0.0;
    sLeftCrossCoupl.Kp = 1;   // 0.2
    sLeftCrossCoupl.Ki = 0.5; // 0.015
    sLeftCrossCoupl.Kd = 0;   // 0.2
    sRightCrossCoupl.iTerm = 0.0;
    sRightCrossCoupl.Kp = 1;   // 0.2
    sRightCrossCoupl.Ki = 0.5; // 0.015
    sRightCrossCoupl.Kd = 0;   // 0.2
#elif defined CCC_POS
    sLeftCrossCoupl.iTerm = 0.0;
    sLeftCrossCoupl.Kp = 12.0;   // 0.2
    sLeftCrossCoupl.Ki = 0.014; // 0.015
    sLeftCrossCoupl.Kd = 0.2;   // 0.2
    sRightCrossCoupl.iTerm = 0.0;
    sRightCrossCoupl.Kp = 12.0;   // 0.2
    sRightCrossCoupl.Ki = 0.013; // 0.015
    sRightCrossCoupl.Kd = 0.1;   // 0.2
#endif
}

#ifdef CCC_INC
void doCrossCoupl(sCrossCoupl* sLeft, sCrossCoupl* sRight) {

    sLeft->crossCouplGain = 1;
    sRight->crossCouplGain = sLeft->targetVel / sRight->targetVel;

    // sLeft->actualVel = sLeft->Encoder - sLeft->Encoder_last;
    // sRight->actualVel = sRight->Encoder - sRight->Encoder_last;
    sLeft->actualVel = readFilterVel(LEFT);
    sRight->actualVel = readFilterVel(RIGHT);

    sMotorsCrossCouplPara.error = sLeft->actualVel * sLeft->crossCouplGain - sRight->actualVel * sRight->crossCouplGain;

    sMotorsCrossCouplPara.iTerm += sMotorsCrossCouplPara.error;

    sMotorsCrossCouplPara.output = sMotorsCrossCouplPara.Kp * sMotorsCrossCouplPara.error + sMotorsCrossCouplPara.Ki * sMotorsCrossCouplPara.iTerm + sMotorsCrossCouplPara.Kd * (sMotorsCrossCouplPara.error - sMotorsCrossCouplPara.error_last);

    sMotorsCrossCouplPara.error_last = sMotorsCrossCouplPara.error;

    // sLeft->error = sLeft->targetVel - sMotorsCrossCouplPara.output - sLeft->actualVel;
    // sRight->error = sRight->targetVel - sMotorsCrossCouplPara.output - sRight->actualVel;

    sLeft->error = sLeft->targetVel - sLeft->actualVel;
    sRight->error = sRight->targetVel - sRight->actualVel;

    float leftIncreVel = sLeft->Kp * (sLeft->error - sLeft->error_next) + sLeft->Ki * sLeft->error+ sLeft->Kd * (sLeft->error - 2 * sLeft->error_next + sLeft->error_last);
    float rightIncreVel =  sRight->Kp * (sRight->error - sRight->error_next) + sRight->Ki * sRight->error + sRight->Kd * (sRight->error - 2 * sRight->error_next + sRight->error_last);

    // sLeft->Encoder_last = sLeft->Encoder;
    // sRight->Encoder_last = sRight->Encoder;

    sLeft->error_last = sLeft->error_next;
    sLeft->error_next = sLeft->error;
    sRight->error_last = sRight->error_next;
    sRight->error_next = sRight->error;

    Serial.println("read motor vel");
    Serial.print(sLeft->actualVel);
    Serial.print(" ");
    Serial.println(sRight->actualVel);

    Serial.println("error: targetvel - actualvel");
    Serial.print(sLeft->error);
    Serial.print(" ");
    Serial.println(sRight->error);

    sLeft->actualVel += leftIncreVel;
    sRight->actualVel += rightIncreVel;
    // Serial.println("do cross couple end");

}
#elif defined CCC_POS
void doCrossCoupl(sCrossCoupl* sLeft, sCrossCoupl* sRight) {

    // Serial.println("read motor lastvel");
    // Serial.print(sLeft->actualVel_last);
    // Serial.print(" ");
    // Serial.println(sRight->actualVel_last);

    sLeft->actualVel = readFilterVel(LEFT);
    sRight->actualVel = readFilterVel(RIGHT);
    i32LeftTempVel = sLeft->actualVel;
    i32RightTempVel = sRight->actualVel;
    // check if there is an error in reading the register value 
    while (abs(sLeft->actualVel - sLeft->actualVel_last) > deltaVelThreshold) {sLeft->actualVel = readFilterVel(LEFT);}
    while (abs(sRight->actualVel - sRight->actualVel_last) > deltaVelThreshold) {sRight->actualVel = readFilterVel(RIGHT);}

    sLeft->actualVel_last = sLeft->actualVel;
    sRight->actualVel_last = sRight->actualVel;

    sMotorsCrossCouplPara.error = sLeft->actualVel * sLeft->crossCouplGain - sRight->actualVel * sRight->crossCouplGain;

    sMotorsCrossCouplPara.iTerm += sMotorsCrossCouplPara.error;

    sMotorsCrossCouplPara.output = sMotorsCrossCouplPara.Kp * sMotorsCrossCouplPara.error + sMotorsCrossCouplPara.Ki * sMotorsCrossCouplPara.iTerm + sMotorsCrossCouplPara.Kd * (sMotorsCrossCouplPara.error - sMotorsCrossCouplPara.error_last);

    sMotorsCrossCouplPara.error_last = sMotorsCrossCouplPara.error;

    // sLeft->error = sLeft->targetVel - sMotorsCrossCouplPara.output - sLeft->actualVel;
    // sRight->error = sRight->targetVel - sMotorsCrossCouplPara.output - sRight->actualVel;

    sLeft->error = sLeft->targetVel - sLeft->actualVel;
    sRight->error = sRight->targetVel - sRight->actualVel;

    sLeft->iTerm += sLeft->error;
    sRight->iTerm += sRight->error;

    float leftIncreVel = sLeft->Kp * sLeft->error + sLeft->Ki * sLeft->iTerm+ sLeft->Kd * (sLeft->error - sLeft->error_last);
    float rightIncreVel =  sRight->Kp * sRight->error + sRight->Ki * sRight->iTerm + sRight->Kd * (sRight->error - sRight->error_last);

    sLeft->error_last = sLeft->error;
    sRight->error_last = sRight->error;

    // Serial.println("read motor vel");
    // Serial.print(sLeft->actualVel);
    // Serial.print(" ");
     Serial.println(sRight->actualVel);
//    Serial.println(sLeft->actualVel);

    // Serial.println("error: targetvel - actualvel");
    // Serial.print(sLeft->error);
    // Serial.print(" ");
    // Serial.println(sRight->error);

    sLeft->actualVel = leftIncreVel;
    sRight->actualVel = rightIncreVel;
    // Serial.println("do cross couple end");

}
#endif

void updateCrossCoupl() {
    // sLeftCrossCoupl.actualVel = readFilterVel(LEFT);
    // sRightCrossCoupl.actualVel = readFilterVel(RIGHT);

    if (!moving) {
        if (sLeftCrossCoupl.actualVel != 0 || sRightCrossCoupl.actualVel != 0) resetCrossCoupl();
        // Serial.println("isn't moving, update cross couple end");
        return;
    }
    doCrossCoupl(&sLeftCrossCoupl, &sRightCrossCoupl);
    setMotorSpeeds(sLeftCrossCoupl.actualVel, sRightCrossCoupl.actualVel);
    // Serial.println("write to motor vel:");
    // // delay(1000);
    // Serial.print(sLeftCrossCoupl.actualVel);
    // Serial.print(" ");
    // Serial.println(sRightCrossCoupl.actualVel);
    // setMotorSpeeds(sLeftCrossCoupl.targetVel, sRightCrossCoupl.targetVel);
    // Serial.println("after do cross couple, update cross couple end");
}

#endif // CCC_H
