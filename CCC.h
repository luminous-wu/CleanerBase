#ifndef CCC_H   // CCC: Cross Coupling Control
#define CCC_H

#define MAX_SPEED 10000

typedef struct {
    float targetVel;
    float actualVel;
    long Encoder;
    long Encoder_last;
    float error;
    float error_next;
    float error_last;
    float crossCouplGain;
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
    sLeftCrossCoupl.actualVel = 0.0;
    sLeftCrossCoupl.Encoder = readEncoder(LEFT);
    sLeftCrossCoupl.Encoder_last = sLeftCrossCoupl.Encoder;
    sLeftCrossCoupl.error = 0.0;
    sLeftCrossCoupl.error_next = 0.0;
    sLeftCrossCoupl.error_last = 0.0;
    sLeftCrossCoupl.crossCouplGain = 0.0;
    sLeftCrossCoupl.Kp = 0.2;   // 0.2
    sLeftCrossCoupl.Ki = 0.015; // 0.015
    sLeftCrossCoupl.Kd = 0.2;   // 0.2

    sRightCrossCoupl.targetVel = 0.0;
    sRightCrossCoupl.actualVel = 0.0;
    sRightCrossCoupl.Encoder = readEncoder(RIGHT);
    sRightCrossCoupl.Encoder_last = sRightCrossCoupl.Encoder;
    sRightCrossCoupl.error = 0.0;
    sRightCrossCoupl.error_next = 0.0;
    sRightCrossCoupl.error_last = 0.0;
    sRightCrossCoupl.crossCouplGain = 0.0;
    sRightCrossCoupl.Kp = 0.2;   // 0.2
    sRightCrossCoupl.Ki = 0.015; // 0.015
    sRightCrossCoupl.Kd = 0.2;   // 0.2
}

void doCrossCoupl(sCrossCoupl* sLeft, sCrossCoupl* sRight) {

    sLeft->crossCouplGain = 1;
    sRight->crossCouplGain = sLeft->targetVel / sRight->targetVel;

    sLeft->actualVel = sLeft->Encoder - sLeft->Encoder_last;
    sRight->actualVel = sRight->Encoder - sRight->Encoder_last;

    sMotorsCrossCouplPara.error = sLeft->actualVel * sLeft->crossCouplGain - sRight->actualVel * sRight->crossCouplGain;

    sMotorsCrossCouplPara.iTerm += sMotorsCrossCouplPara.error;

    sMotorsCrossCouplPara.output = sMotorsCrossCouplPara.Kp * sMotorsCrossCouplPara.error + sMotorsCrossCouplPara.Ki * sMotorsCrossCouplPara.iTerm + sMotorsCrossCouplPara.Kd * (sMotorsCrossCouplPara.error - sMotorsCrossCouplPara.error_last);

    sMotorsCrossCouplPara.error_last = sMotorsCrossCouplPara.error;

    sLeft->error = sLeft->targetVel - sMotorsCrossCouplPara.output - sLeft->actualVel;
    sRight->error = sRight->targetVel - sMotorsCrossCouplPara.output - sRight->actualVel;

    float leftIncreVel = sLeft->Kp * (sLeft->error - sLeft->error_next) + sLeft->Ki * sLeft->error+ sLeft->Kd * (sLeft->error - 2 * sLeft->error_next + sLeft->error_last);
    float rightIncreVel =  sRight->Kp * (sRight->error - sRight->error_next) + sRight->Ki * sRight->error + sRight->Kd * (sRight->error - 2 * sRight->error_next + sRight->error_last);

    sLeft->Encoder_last = sLeft->Encoder;
    sRight->Encoder_last = sRight->Encoder;

    sLeft->error_last = sLeft->error_next;
    sLeft->error_next = sLeft->error;
    sRight->error_last = sRight->error_next;
    sRight->error_next = sRight->error;

    sLeft->actualVel += leftIncreVel;
    sRight->actualVel += rightIncreVel;

}

void updateCrossCoupl() {
    sLeftCrossCoupl.Encoder = readEncoder(LEFT);
    sRightCrossCoupl.Encoder = readEncoder(RIGHT);

    if (!moving) {
        if (sLeftCrossCoupl.actualVel != 0 || sRightCrossCoupl.actualVel != 0) resetCrossCoupl();
        return;
    }
    doCrossCoupl(&sLeftCrossCoupl, &sRightCrossCoupl);
    setMotorSpeeds(sLeftCrossCoupl.actualVel, sRightCrossCoupl.actualVel);
}

#endif // CCC_H