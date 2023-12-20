
#line 1 "E:\\OneDrive - mails.tsinghua.edu.cn\\-001.FreeCode\\cleaner\\arduino\\library_test\\servo_motor_driver.h"
#ifndef SERVO_MOTOR_DRIVER_H
#define SERVO_MOTOR_DRIVER_H


#include <ModbusMaster.h>
#include <SoftwareSerial.h>
// #include "CCC.h"
// extern sCrossCoupl sLeftCrossCoupl, sRightCrossCoupl;

#define BAUDRATE_MOTOR_DRIVER                           115200
#define BAUDRATE_ARDUINO_PC                             9600
#define leftMotorDriverAddress                          0x01
#define rightMotorDriverAddress                         0x02

#define rxPin_ttl2RS485                                 12
#define txPin_ttl2RS485                                 11

#define controlWord                                     0x6040
#define operationMode                                   0x6060
#define operateMode_PVM                                 0x0003
#define encoderAddress                                  0x2020

// example: uint32_t -1000 1's complement is 1111 1111 1111 1111 1111 1100 0001 0111
//                                    HEX is    0xff      0xff     0xfc      0x18
//                                            highByte1 lowByte1 highByte2 lowByte2

#define highByte1(w)  ((uint8_t) (((w) >> 24) & 0xFF))
#define lowByte1(w)  ((uint8_t) (((w) >> 16) & 0xFF))
#define highByte2(w)  ((uint8_t) (((w) >> 8) & 0xFF))
#define lowByte2(w)  ((uint8_t) ((w) & 0xFF))

#define FOREVER for(;;){}

ModbusMaster leftMotorDriverNode;
ModbusMaster rightMotorDriverNode;
SoftwareSerial MotorDriverSerial(rxPin_ttl2RS485, txPin_ttl2RS485);

uint8_t leftMotorVelocity_1000[13] = {0x01,0x10,0x60,0xff,0x00,0x02,0x04,0x00,0x00,0x03,0xe8,0x14,0x17};
uint8_t rightMotorVelocity_1000[13] = {0x02,0x10,0x60,0xff,0x00,0x02,0x04,0xff,0xff,0xfc,0x18,0x5a,0xc3};

uint8_t leftMotorVelocity_0[13] = {0x01,0x10,0x60,0xff,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x14,0xa9};
uint8_t rightMotorVelocity_0[13] = {0x02,0x10,0x60,0xff,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x1b,0xed};

uint8_t u8LeftMotorEncoder[8] = {0x01, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x01};
uint8_t u8RightMotorEncoder[8] = {0x02, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x32};

uint8_t u8LeftBufferSize = 0;
uint8_t u8RightBufferSize = 0;
uint8_t u8LeftReadBuffer[256];
uint8_t u8RightReadBuffer[256];
uint8_t u8ReadLeftEncoder[8] = {0x01, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x01};
uint8_t u8ReadRightEncoder[8] = {0x02, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x32};
uint8_t u8ReadLeftFilterVel[8] = {0x01, 0x03, 0x20, 0x28, 0x00, 0x02, 0x4F, 0xC3};
uint8_t u8ReadRightFilterVel[8] = {0x02, 0x03, 0x20, 0x28, 0x00, 0x02, 0x4F, 0xF0};

static const uint8_t ku8MaxBufferSize                = 64;
static const uint8_t ku8MBReadHoldingRegisters       = 0x03;
static const uint8_t ku8MBWriteSingleRegister        = 0x06;
static const uint8_t ku8MBWriteMultipleRegisters     = 0x10;
const uint32_t u32EncoderMaxCount                    = 10000;
const int32_t i32MaxSpeed                            = 2000;
const int8_t i8MaxReReadTimes                        = 3;
int32_t deltaVelThreshold                            = 30;      // RPM

uint8_t writeData[256];
uint8_t leftWriteData[256];
uint8_t rightWriteData[256];

uint8_t u8LeftMotorDriverReturnBuffer[256];
uint8_t u8RightMotorDriverReturnBuffer[256];

uint8_t _subIndex;
uint8_t u8LeftMotorDriverReturnBufferSize;
uint8_t u8RightMotorDriverReturnBufferSize;
uint8_t u8SpeedBufferSize;

uint32_t _u32LeftTransmitBuffer[ku8MaxBufferSize];
uint32_t _u32RightTransmitBuffer[ku8MaxBufferSize];
uint32_t u32LeftEncoderCount;
uint32_t u32RightEncoderCount;
int32_t i32LeftFilterVel;
int32_t i32RightFilterVel;
int32_t i32LeftTempVel;
int32_t i32RightTempVel;
int32_t i32LeftTempVel_last;
int32_t i32RightTempVel_last;

uint16_t _u16WriteAddress;
uint16_t _u16ReadAddress;
uint16_t _u16WriteQty;
uint16_t _u16ReadQty;

uint8_t u8LeftMotorBuffer[256];
uint8_t u8RightMotorBuffer[256];
uint8_t u8LeftMotorBufferSize;
uint8_t u8RightMotorBufferSize;


void initMotorController();
void printHEXcommand(uint8_t* commandArray, int maxNum);
void setBuffer();
void setMotorsSpeeds(uint32_t leftMotorSpeed, uint32_t rightMotorSpeed);
void setMotorSpeeds(uint32_t u32LeftWriteValue, uint32_t u32RightWriteValue);
void write2MotorsSingleRegister(uint16_t u16WriteAddress, uint32_t u32LeftRightWriteValue);
void write2MotorsSingleRegister(uint16_t u16WriteAddress, uint32_t u32LeftWriteValue, uint32_t u32RightWriteValue);
void write2MotorMultipleRegisters(uint16_t u16WriteAddress, uint16_t u16WriteQty, uint32_t u32LeftWriteValue, uint32_t u32RightWriteValue);
uint8_t readResponseData(ModbusMaster MotorDriverNode, uint8_t* u8MotorBuffer, uint8_t u8MotorBufferSize);
void readEncoders();
uint32_t readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void Modbus2Transaction(uint8_t u8MBFunction);
int32_t readVel(int i);
int32_t readFilterVel(int i);
void read2MotorsHoldingRegisters(uint16_t u16ReadAddress, uint16_t u16LeftRightReadSize);

void initMotorController() {
    
    Serial.begin(BAUDRATE_ARDUINO_PC);

    // instantiating a Soft Serial Port, TTL2RS485 module to servo motor driver
    
    MotorDriverSerial.begin(BAUDRATE_MOTOR_DRIVER);

    leftMotorDriverNode.begin(leftMotorDriverAddress, MotorDriverSerial);
    rightMotorDriverNode.begin(rightMotorDriverAddress, MotorDriverSerial);

    // set servo motor driver to PVM moode
    leftMotorDriverNode.writeSingleRegister(operationMode, operateMode_PVM, writeData);
    rightMotorDriverNode.writeSingleRegister(operationMode, operateMode_PVM, writeData);

    // Right and left motor enable
    leftMotorDriverNode.writeSingleRegister(controlWord, 0x0006, writeData);
    leftMotorDriverNode.writeSingleRegister(controlWord, 0x000f, writeData);
    rightMotorDriverNode.writeSingleRegister(controlWord, 0x0006, writeData);
    rightMotorDriverNode.writeSingleRegister(controlWord, 0x000f, writeData);
}

void initMotorsController() {
    Serial.begin(BAUDRATE_ARDUINO_PC);

    // instantiating a Soft Serial Port, TTL2RS485 module to servo motor driver
    MotorDriverSerial.begin(BAUDRATE_MOTOR_DRIVER);

    leftMotorDriverNode.begin(leftMotorDriverAddress, MotorDriverSerial);
    rightMotorDriverNode.begin(rightMotorDriverAddress, MotorDriverSerial);

    // set servo motor driver to PVM moode
    write2MotorsSingleRegister(operationMode, operateMode_PVM);
    // Right and left motor enable
    // write2MotorsSingleRegister(controlWord, 0x0006);
    // write2MotorsSingleRegister(controlWord, 0x000f);

}

void printHEXcommand(uint8_t* commandArray, int maxNum) {
    for (int i = 0; i < maxNum; ++i) {
        Serial.print("0x");
        Serial.print(commandArray[i], HEX);
        Serial.print(", ");
    }
    Serial.println("");
}

void setBuffer() {
    leftMotorDriverNode.setTransmitBuffer(0,highByte(1000));
    leftMotorDriverNode.setTransmitBuffer(1,lowByte(1000));
    rightMotorDriverNode.setTransmitBuffer(0,highByte(1000));
    rightMotorDriverNode.setTransmitBuffer(1,lowByte(1000));
}

void setMotorsSpeeds(uint32_t leftMotorSpeed, uint32_t rightMotorSpeed) {
    
    u8SpeedBufferSize = 13;
    uint8_t u8ResponseTimeOutFlag = 0;
    u8LeftMotorDriverReturnBufferSize = 0;
    u8RightMotorDriverReturnBufferSize = 0;
    uint32_t u32StartTime;

    rightMotorSpeed = -rightMotorSpeed;
    // leftMotorDriverNode.setTransmitBuffer()
    // Serial.println("==============Motor Buffer inialize===========");

    uint8_t u8LeftMotorSpeedBuffer[u8SpeedBufferSize] = {leftMotorDriverAddress, 0x10, 0x60, 0xff, 0x00, 0x02, 0x04,
        highByte1(leftMotorSpeed), lowByte1(leftMotorSpeed), highByte2(leftMotorSpeed), lowByte2(leftMotorSpeed), 0x00, 0x00};
    uint8_t u8RightMotorSpeedBuffer[u8SpeedBufferSize] = {rightMotorDriverAddress, 0x10, 0x60, 0xff, 0x00, 0x02, 0x04,
        highByte1(rightMotorSpeed), lowByte1(rightMotorSpeed), highByte2(rightMotorSpeed), lowByte2(rightMotorSpeed), 0x00, 0x00};

    // Serial.print("leftMotorSpeed: ");
    // Serial.println(leftMotorSpeed);
    // Serial.print("rightMotorSpeed: ");
    // Serial.println(rightMotorSpeed);
    // Serial.println("==============Append CRC===========");
    uint16_t u16CRC_leftMotor = 0xFFFF;
    uint16_t u16CRC_rightMotor = 0xFFFF;

    for (int i = 0; i < u8SpeedBufferSize - 2; i++) {
        u16CRC_leftMotor = crc16_update(u16CRC_leftMotor, u8LeftMotorSpeedBuffer[i]);
        u16CRC_rightMotor = crc16_update(u16CRC_rightMotor, u8RightMotorSpeedBuffer[i]);
    }
    u8LeftMotorSpeedBuffer[u8SpeedBufferSize-2] = lowByte(u16CRC_leftMotor);
    u8LeftMotorSpeedBuffer[u8SpeedBufferSize-1] = highByte(u16CRC_leftMotor);
    u8RightMotorSpeedBuffer[u8SpeedBufferSize-2] = lowByte(u16CRC_rightMotor);
    u8RightMotorSpeedBuffer[u8SpeedBufferSize-1] = highByte(u16CRC_rightMotor);
    
    // Serial.println("==============Verify Motor Speed Buffer===========");
    // printHEXcommand(u8LeftMotorSpeedBuffer, u8SpeedBufferSize);
    // printHEXcommand(u8RightMotorSpeedBuffer, u8SpeedBufferSize);

    // flush receive buffer before transmitting request
    while ((leftMotorDriverNode._serial->read() != -1) && (rightMotorDriverNode._serial->read() != -1));
    
    // transmit request
    if (leftMotorDriverNode._preTransmission) {leftMotorDriverNode._preTransmission();}
    if (rightMotorDriverNode._preTransmission) {rightMotorDriverNode._preTransmission();}

    for (auto i: u8LeftMotorSpeedBuffer) {leftMotorDriverNode._serial->write(i);}
    // for (auto i: u8LeftMotorSpeedBuffer) { Serial.print(i,HEX);Serial.print(",");}
    // Serial.println("");
    for (auto i: u8RightMotorSpeedBuffer) {rightMotorDriverNode._serial->write(i);}
    // for (auto i: u8RightMotorSpeedBuffer) { Serial.print(i,HEX);Serial.print(",");}
    // Serial.println("");

    leftMotorDriverNode._serial->flush();    // flush transmit buffer
    rightMotorDriverNode._serial->flush();
    
    if (leftMotorDriverNode._postTransmission) {leftMotorDriverNode._postTransmission();}
    if (rightMotorDriverNode._postTransmission) {rightMotorDriverNode._postTransmission();}

    // Serial.println("==============Receive Motor Speed Response Buffer from Registers===========");
    // u32StartTime = millis();
    // // Serial.print("_serial->available() value");
    // // Serial.println(leftMotorDriverNode._serial->available());
    // while (!u8ResponseTimeOutFlag) {
    //     if (leftMotorDriverNode._serial->available()) {
    //         u8LeftMotorDriverReturnBuffer[u8LeftMotorDriverReturnBufferSize++] = leftMotorDriverNode._serial->read();
    //     }

    //     if (rightMotorDriverNode._serial->available()) {
    //         u8RightMotorDriverReturnBuffer[u8RightMotorDriverReturnBufferSize++] = rightMotorDriverNode._serial->read();
    //     }
        
    //     if ((millis() - u32StartTime) > 4000) { // ///< Modbus timeout [milliseconds]
    //         u8ResponseTimeOutFlag = 1;
    //     }
    // }

    // Serial.println("==============Verify Motor Speed Response Buffer from Registers===========");
    // printHEXcommand(u8LeftMotorDriverReturnBuffer, u8LeftMotorDriverReturnBufferSize);
    // printHEXcommand(u8RightMotorDriverReturnBuffer, u8RightMotorDriverReturnBufferSize);
}

void setMotorSpeeds(uint32_t u32LeftWriteValue, uint32_t u32RightWriteValue) {
    leftMotorDriverNode.setSubIndex(0x00);
    rightMotorDriverNode.setSubIndex(0x00);
    u32LeftWriteValue = -u32LeftWriteValue;
    _u32LeftTransmitBuffer[0] = u32LeftWriteValue;
    _u32RightTransmitBuffer[0] = u32RightWriteValue;
    write2MotorMultipleRegisters(0x60ff, 4, u32LeftWriteValue, u32RightWriteValue);
}

void write2MotorsSingleRegister(uint16_t u16WriteAddress, uint32_t u32LeftRightWriteValue) {
    _u16WriteAddress = u16WriteAddress;
    _u16WriteQty = 0;
    _u32LeftTransmitBuffer[0] = u32LeftRightWriteValue;
    _u32RightTransmitBuffer[0] = u32LeftRightWriteValue;
    Modbus2Transaction(ku8MBWriteSingleRegister);
}

void write2MotorsSingleRegister(uint16_t u16WriteAddress, uint32_t u32LeftWriteValue, uint32_t u32RightWriteValue) {
    _u16WriteAddress = u16WriteAddress;
    _u16WriteQty = 0;
    _u32LeftTransmitBuffer[0] = u32LeftWriteValue;
    _u32RightTransmitBuffer[0] = u32RightWriteValue;
    Modbus2Transaction(ku8MBWriteSingleRegister);
}

// before using write2MotorMultipleRegisters(), you should using ModbusMaster::setSubIndex()
void write2MotorMultipleRegisters(uint16_t u16WriteAddress, uint16_t u16WriteQty, uint32_t u32LeftWriteValue, uint32_t u32RightWriteValue) {
    _u16WriteAddress = u16WriteAddress;
    _u16WriteQty = u16WriteQty;
    Modbus2Transaction(ku8MBWriteMultipleRegisters);
}

void Modbus2Transaction(uint8_t u8MBFunction) {
    u8LeftMotorBuffer[256] = {0};
    u8RightMotorBuffer[256] = {0};
    u8LeftMotorBufferSize = 0;
    u8RightMotorBufferSize = 0;
    uint8_t u8LeftSlave = leftMotorDriverAddress;
    uint8_t u8RightSlave = rightMotorDriverAddress;
    uint8_t i, u8Qty;
    uint8_t u8LeftResponseTimeOutFlag;
    uint8_t u8RightResponseTimeOutFlag;
    uint16_t u16CRC_leftMotor, u16CRC_rightMotor;

    u8LeftMotorBuffer[u8LeftMotorBufferSize++] = u8LeftSlave;
    u8LeftMotorBuffer[u8LeftMotorBufferSize++] = u8MBFunction;

    u8RightMotorBuffer[u8RightMotorBufferSize++] = u8RightSlave;
    u8RightMotorBuffer[u8RightMotorBufferSize++] = u8MBFunction;

    switch(u8MBFunction) {
        case ku8MBWriteSingleRegister:
        case ku8MBWriteMultipleRegisters:
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = highByte(_u16WriteAddress);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte(_u16WriteAddress);
            u8RightMotorBuffer[u8RightMotorBufferSize++] = highByte(_u16WriteAddress);
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte(_u16WriteAddress);
            break;
    }

    switch (u8MBFunction) {
        case ku8MBReadHoldingRegisters:
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = highByte(_u16ReadAddress);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte(_u16ReadAddress);
            u8RightMotorBuffer[u8RightMotorBufferSize++] = highByte(_u16ReadAddress);
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte(_u16ReadAddress);
            /* code */
            break;
    
    }

    switch(u8MBFunction) {
        case ku8MBWriteSingleRegister:
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = highByte(_u32LeftTransmitBuffer[0]);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte(_u32LeftTransmitBuffer[0]);

            u8RightMotorBuffer[u8RightMotorBufferSize++] = highByte(_u32RightTransmitBuffer[0]);  
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte(_u32RightTransmitBuffer[0]);
            break;
        case ku8MBWriteMultipleRegisters:
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = leftMotorDriverNode._u8SubIndex;
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = _u16WriteQty/2;
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte(_u16WriteQty);
            
            // Serial.print("_u32LeftTransmitBuffer[0]: ");
            // Serial.println(_u32LeftTransmitBuffer[0], HEX);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = highByte1(_u32LeftTransmitBuffer[0]);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte1(_u32LeftTransmitBuffer[0]);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = highByte2(_u32LeftTransmitBuffer[0]);
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte2(_u32LeftTransmitBuffer[0]);

            u8RightMotorBuffer[u8RightMotorBufferSize++] = rightMotorDriverNode._u8SubIndex;
            u8RightMotorBuffer[u8RightMotorBufferSize++] = _u16WriteQty/2;
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte(_u16WriteQty);

            // Serial.print("_u32RightTransmitBuffer[0]: ");
            // Serial.println(_u32RightTransmitBuffer[0], HEX);
            u8RightMotorBuffer[u8RightMotorBufferSize++] = highByte1(_u32RightTransmitBuffer[0]);  
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte1(_u32RightTransmitBuffer[0]);
            u8RightMotorBuffer[u8RightMotorBufferSize++] = highByte2(_u32RightTransmitBuffer[0]);  
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte2(_u32RightTransmitBuffer[0]);
            break;
        case ku8MBReadHoldingRegisters:
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = leftMotorDriverNode._u8SubIndex;
            u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte(_u16ReadQty);

            u8RightMotorBuffer[u8RightMotorBufferSize++] = rightMotorDriverNode._u8SubIndex;
            u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte(_u16ReadQty);
            break;
    }

    u16CRC_leftMotor = 0xFFFF;
    u16CRC_rightMotor = 0xFFFF;
    for (int i = 0; i < u8LeftMotorBufferSize; i++) {
        u16CRC_leftMotor = crc16_update(u16CRC_leftMotor, u8LeftMotorBuffer[i]);
        u16CRC_rightMotor = crc16_update(u16CRC_rightMotor, u8RightMotorBuffer[i]);
    }
    
    u8LeftMotorBuffer[u8LeftMotorBufferSize++] = lowByte(u16CRC_leftMotor);
    u8LeftMotorBuffer[u8LeftMotorBufferSize++] = highByte(u16CRC_leftMotor);
    u8RightMotorBuffer[u8RightMotorBufferSize++] = lowByte(u16CRC_rightMotor);
    u8RightMotorBuffer[u8RightMotorBufferSize++] = highByte(u16CRC_rightMotor);

    // printHEXcommand(u8LeftMotorBuffer, u8LeftMotorBufferSize);
    // printHEXcommand(u8RightMotorBuffer, u8RightMotorBufferSize);

    // flush receive buffer before transmitting request
    while ((leftMotorDriverNode._serial->read() != -1) && (rightMotorDriverNode._serial->read() != -1));
    
    // transmit request
    if (leftMotorDriverNode._preTransmission) {leftMotorDriverNode._preTransmission();}
    for (i = 0; i < u8LeftMotorBufferSize; ++i) {leftMotorDriverNode._serial->write(u8LeftMotorBuffer[i]);}
    leftMotorDriverNode._serial->flush();    // flush transmit buffer
    if (leftMotorDriverNode._postTransmission) {leftMotorDriverNode._postTransmission();}
    u8LeftResponseTimeOutFlag = readResponseData(leftMotorDriverNode, u8LeftMotorBuffer, u8LeftMotorBufferSize);

    if (rightMotorDriverNode._preTransmission) {rightMotorDriverNode._preTransmission();}
    for (i = 0; i < u8RightMotorBufferSize; ++i) {rightMotorDriverNode._serial->write(u8RightMotorBuffer[i]);}
    rightMotorDriverNode._serial->flush();
    if (rightMotorDriverNode._postTransmission) {rightMotorDriverNode._postTransmission();}
    u8RightResponseTimeOutFlag = readResponseData(rightMotorDriverNode, u8RightMotorBuffer, u8RightMotorBufferSize);
    
    // printHEXcommand(u8LeftMotorBuffer, u8LeftMotorBufferSize);
    // printHEXcommand(u8RightMotorBuffer, u8RightMotorBufferSize);

}

uint8_t readResponseData(ModbusMaster MotorDriverNode, uint8_t* u8MotorBuffer, uint8_t u8MotorBufferSize) {
    
    // uint8_t u8MotorBuffer[256];
    // uint8_t u8MotorBufferSize = 0;
    u8MotorBufferSize = 0;
    uint8_t u8ResponseTimeOutFlag = 0;
    uint8_t u8DataReading = 1;
    uint32_t u32StartTime = millis();
    while(u8DataReading && !u8ResponseTimeOutFlag) {
        while(MotorDriverNode._serial->available()) {
            u8MotorBuffer[u8MotorBufferSize++] = MotorDriverNode._serial->read();
            if(!(MotorDriverNode._serial->available())){u8DataReading = 0;}
        }

        if (millis() - u32StartTime > 2000) {u8ResponseTimeOutFlag = -1;}
    }
    // delay(8);
    // printHEXcommand(u8MotorBuffer, u8MotorBufferSize);
    return u8ResponseTimeOutFlag;
}

uint32_t readEncoder(int i){
    if (i == LEFT) {
        leftMotorDriverNode._serial->write(u8ReadLeftEncoder, sizeof(u8ReadLeftEncoder));
        leftMotorDriverNode._serial->flush();
        if (readResponseData(leftMotorDriverNode, u8LeftReadBuffer, u8LeftBufferSize) == -1) {Serial.println("read data time out");}
        if (u8LeftReadBuffer[1] != 0x03 && u8LeftReadBuffer[2] != 0x04) {readEncoder(LEFT);}
        u32LeftEncoderCount = ((uint32_t)u8LeftReadBuffer[3] << 24) | ((uint32_t)u8LeftReadBuffer[4] << 16) | ((uint32_t)u8LeftReadBuffer[5] << 8) | (uint32_t)u8LeftReadBuffer[6];
         if (u32LeftEncoderCount > u32EncoderMaxCount) {readEncoder(LEFT);}
        return u32LeftEncoderCount;
    }
    else {
        rightMotorDriverNode._serial->write(u8ReadRightEncoder, sizeof(u8ReadRightEncoder));
        rightMotorDriverNode._serial->flush();
        if (readResponseData(rightMotorDriverNode, u8RightReadBuffer, u8RightBufferSize) == -1) {Serial.println("read data time out");}
        if (u8RightReadBuffer[1] != 0x03 && u8RightReadBuffer[2] != 0x04){readEncoder(RIGHT);}
        u32RightEncoderCount = ((uint32_t)u8RightReadBuffer[3] << 24) | ((uint32_t)u8RightReadBuffer[4] << 16) | ((uint32_t)u8RightReadBuffer[5] << 8) | (uint32_t)u8RightReadBuffer[6];
         if (u32RightEncoderCount > u32EncoderMaxCount) {readEncoder(RIGHT);}
        return u32RightEncoderCount;
    }
}

int32_t readVel(int i) {
    static int iLeft_count = 0;
    static int iRight_count = 0;
    if (i == LEFT) {
        i32LeftTempVel = readFilterVel(LEFT);
        while (abs(i32LeftTempVel - i32LeftTempVel_last) > deltaVelThreshold) {
            if (iLeft_count == 3) {break;}
            ++iLeft_count;
            i32LeftTempVel = readFilterVel(LEFT);
        }
        i32LeftTempVel_last = i32LeftTempVel;
        iLeft_count = 0;
        return i32LeftTempVel;
    }
    else {
        i32RightTempVel = readFilterVel(RIGHT);
        while (abs(i32RightTempVel - i32RightTempVel_last) > deltaVelThreshold) {
            if (iRight_count == 3) {break;}
            ++iRight_count;
            i32RightTempVel = readFilterVel(RIGHT);
        }
        i32RightTempVel_last = i32RightTempVel;
        iRight_count = 0;
        return i32RightTempVel;
    }
}

int32_t readFilterVel(int i){
    static int8_t j = 0;
    if (i == LEFT) {
        leftMotorDriverNode._serial->write(u8ReadLeftFilterVel, sizeof(u8ReadLeftFilterVel));
        leftMotorDriverNode._serial->flush();
        if (readResponseData(leftMotorDriverNode, u8LeftReadBuffer, u8LeftBufferSize) == -1) {Serial.println("left node read data time out"); return -1;}
        if (u8LeftReadBuffer[1] != 0x03 && u8LeftReadBuffer[2] != 0x04) {
            if (j == i8MaxReReadTimes) {
                // Serial.println("left node the first time re-read data time out");
                j = 0;
                return i32LeftTempVel;
            }
            ++j;
            readFilterVel(LEFT);
        }
        j = 0;
        i32LeftFilterVel = ((uint32_t)u8LeftReadBuffer[3] << 24) | ((uint32_t)u8LeftReadBuffer[4] << 16) | ((uint32_t)u8LeftReadBuffer[5] << 8) | (uint32_t)u8LeftReadBuffer[6];
        static int8_t k = 0;
        if (i32LeftFilterVel > i32MaxSpeed || i32LeftFilterVel < -i32MaxSpeed) {
            if (k == i8MaxReReadTimes) {
                // Serial.println("left node the second time re-read data time out");
                k = 0;
                return i32LeftTempVel;
            }
            ++k;
            readFilterVel(LEFT);
        }
        k = 0;
        return -i32LeftFilterVel;
    }
    else {
        rightMotorDriverNode._serial->write(u8ReadRightFilterVel, sizeof(u8ReadRightFilterVel));
        rightMotorDriverNode._serial->flush();
        if (readResponseData(rightMotorDriverNode, u8RightReadBuffer, u8RightBufferSize) == -1) {Serial.println("right node read data time out"); return -1;}
        if (u8RightReadBuffer[1] != 0x03 && u8RightReadBuffer[2] != 0x04) {
            if (j == i8MaxReReadTimes) {
                // Serial.println("right node the first time re-read data time out");
                j = 0;
                return i32RightTempVel;
            }
            ++j;
            readFilterVel(RIGHT);
        }
        j = 0;
        i32RightFilterVel = ((uint32_t)u8RightReadBuffer[3] << 24) | ((uint32_t)u8RightReadBuffer[4] << 16) | ((uint32_t)u8RightReadBuffer[5] << 8) | (uint32_t)u8RightReadBuffer[6];
        static int8_t m = 0;
        if (i32RightFilterVel > i32MaxSpeed || i32RightFilterVel < -i32MaxSpeed) {
            if (m == i8MaxReReadTimes) {
                // Serial.println("right node the second time re-read data time out");
                m = 0;
                return i32RightTempVel;
            }
            ++m;
            readFilterVel(RIGHT);
        }
        m = 0;
        return i32RightFilterVel;
    }
}


void read2MotorsHoldingRegisters(uint16_t u16ReadAddress, uint16_t u16LeftRightReadSize) {
    _u16ReadAddress = u16ReadAddress;
    _u16ReadQty = u16LeftRightReadSize;
    Modbus2Transaction(ku8MBReadHoldingRegisters);
}

void resetEncoder(int i) {
    if (i == LEFT) {
        u32LeftEncoderCount = 0;
        return;
    } else { 
        u32RightEncoderCount = 0;
        return;
    }
}

void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
}

#endif // SERVO_MOTOR_DRIVER_H
