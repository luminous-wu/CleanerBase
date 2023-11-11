#ifndef CCC_H   // CCC: Cross Coupling Control
#define CCC_H

#define CCC_INC
// #define CCC_POS

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
    sLeftCrossCoupl.Kp = 0.2;
    sLeftCrossCoupl.Ki = 0.5;
    sLeftCrossCoupl.Kd = 0.1;
    sRightCrossCoupl.iTerm = 0.0;
    sRightCrossCoupl.Kp = 0.2;
    sRightCrossCoupl.Ki = 0.5;
    sRightCrossCoupl.Kd = 0.1;
#elif defined CCC_POS
    sLeftCrossCoupl.iTerm = 0.0;
    sLeftCrossCoupl.Kp = 12.0;
    sLeftCrossCoupl.Ki = 0.014;
    sLeftCrossCoupl.Kd = 0.2;
    sRightCrossCoupl.iTerm = 0.0;
    sRightCrossCoupl.Kp = 12.0;
    sRightCrossCoupl.Ki = 0.013;
    sRightCrossCoupl.Kd = 0.1;
#endif
}

#ifdef CCC_INC
void doCrossCoupl(sCrossCoupl* sLeft, sCrossCoupl* sRight) {

    static int i = 0;
    sLeft->actualVel = readFilterVel(LEFT);
    sRight->actualVel = readFilterVel(RIGHT);
    i32LeftTempVel = sLeft->actualVel;
    i32RightTempVel = sRight->actualVel;
    // check if there is an error in reading the register value, when the deltaVelThreshold is too small, the program will go to a dead loop.
    while (abs(sLeft->actualVel - sLeft->actualVel_last) > deltaVelThreshold) {
        if (i == 3) {break;}
        ++i;
        sLeft->actualVel = readFilterVel(LEFT);
    }
    i = 0;
    while (abs(sRight->actualVel - sRight->actualVel_last) > deltaVelThreshold) {
        if (i == 3) {break;}
        ++i;
        sRight->actualVel = readFilterVel(RIGHT);
    }
    i = 0;

    sLeft->actualVel_last = sLeft->actualVel;
    sRight->actualVel_last = sRight->actualVel;

    sLeft->crossCouplGain = 1;
    sRight->crossCouplGain = sLeft->targetVel / sRight->targetVel;

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

    sLeft->error_last = sLeft->error_next;
    sLeft->error_next = sLeft->error;
    sRight->error_last = sRight->error_next;
    sRight->error_next = sRight->error;
//    Serial.println(sLeft->actualVel);
    Serial.println(sRight->actualVel);

    sLeft->actualVel += leftIncreVel;
    sRight->actualVel += rightIncreVel;
    // Serial.println("do cross couple end");

}
#elif defined CCC_POS
void doCrossCoupl(sCrossCoupl* sLeft, sCrossCoupl* sRight) {

    static int i = 0;
    sLeft->actualVel = readFilterVel(LEFT);
    sRight->actualVel = readFilterVel(RIGHT);
    i32LeftTempVel = sLeft->actualVel;
    i32RightTempVel = sRight->actualVel;
    // check if there is an error in reading the register value, when the deltaVelThreshold is too small, the program will go to a dead loop.
    while (abs(sLeft->actualVel - sLeft->actualVel_last) > deltaVelThreshold) {
        if (i == 3) {break;}
        ++i;
        sLeft->actualVel = readFilterVel(LEFT);
    }
    i = 0;
    while (abs(sRight->actualVel - sRight->actualVel_last) > deltaVelThreshold) {
        if (i == 3) {break;}
        ++i;
        sRight->actualVel = readFilterVel(RIGHT);
    }
    i = 0;

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

//     Serial.println(sRight->actualVel);
    Serial.println(sLeft->actualVel);

    sLeft->actualVel = leftIncreVel;
    sRight->actualVel = rightIncreVel;

}
#endif

void updateCrossCoupl() {
    // sLeftCrossCoupl.actualVel = readFilterVel(LEFT);
    // sRightCrossCoupl.actualVel = readFilterVel(RIGHT);

    if (!moving) {
        if (sLeftCrossCoupl.actualVel != 0 || sRightCrossCoupl.actualVel != 0) resetCrossCoupl();
        return;
    }
    doCrossCoupl(&sLeftCrossCoupl, &sRightCrossCoupl);
    setMotorSpeeds(sLeftCrossCoupl.actualVel, sRightCrossCoupl.actualVel);
}

#endif // CCC_H
