// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       library_test.ino
    Created:	2023/10/21 11:13:02
    Author:     DESKTOP-ISCEB5H\lumin
*/

// Define User Types below here or use a .h file
//
// #include <ModbusMaster.h>
// #include <SoftwareSerial.h>
#include "servo_motor_driver.h"


uint8_t leftMotorVelocity_1000[13] = {0x01,0x10,0x60,0xff,0x00,0x02,0x04,0x00,0x00,0x03,0xe8,0x14,0x17};
uint8_t rightMotorVelocity_1000[13] = {0x02,0x10,0x60,0xff,0x00,0x02,0x04,0xff,0xff,0xfc,0x18,0x5a,0xc3};

uint8_t leftMotorVelocity_0[13] = {0x01,0x10,0x60,0xff,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x14,0xa9};
uint8_t rightMotorVelocity_0[13] = {0x02,0x10,0x60,0xff,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x1b,0xed};
uint8_t u8ReadEncoder[8] = {0x01, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x01};
uint8_t u8LeftBufferSize = 0;
uint8_t u8RightBufferSize = 0;
uint8_t u8LeftReadBuffer[256];
uint8_t u8RightReadBuffer[256];
uint8_t u8ReadLeftEncoder[8] = {0x01, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x01};
uint8_t u8ReadRightEncoder[8] = {0x02, 0x03, 0x20, 0x20, 0x00, 0x02, 0xCE, 0x32};

void setup()
{
    // for(int i = 0; i < 3; ++i){}
    initMotorsController();

    // Serial.print("================init over============");
}

// Add the main program code into the continuous loop() function
void loop()
{
    static int i = 0;
    // read2MotorsHoldingRegisters(0x2020, 0x02);


    readEncoders();
    delay(5000);
    Serial.println("============orig encoder count==========");
    Serial.print("u32LeftEncoderCount: ");
    Serial.print(u32LeftEncoderCount);
    Serial.print("   ");
    Serial.print("u32RightEncoderCount: ");
    Serial.println(u32RightEncoderCount);
    // while(!(Serial.available())){}
    // if(Serial.read() == "OK"){
    //     readEncoders();
    //     delay(5000);
    //     Serial.println("============after rotate motor manually==========");
    //     Serial.print("u32LeftEncoderCount: ");
    //     Serial.print(u32LeftEncoderCount);
    //     Serial.print("   ");
    //     Serial.print("u32RightEncoderCount: ");
    //     Serial.println(u32RightEncoderCount);
    // }

    // setMotorsSpeeds(1000, 1000);
    // delay(5000);
    // setMotorsSpeeds(0, 0);
    // delay(5000);
    // static int i = 0;
    // leftMotorDriverNode._serial->write(u8ReadLeftEncoder, sizeof(u8ReadLeftEncoder));
    // leftMotorDriverNode._serial->flush();

    // while(!(leftMotorDriverNode._serial->available())) {}
    // while(leftMotorDriverNode._serial->available()) {
    //     u8LeftReadBuffer[u8LeftBufferSize++] = leftMotorDriverNode._serial->read();
    // }
    // rightMotorDriverNode._serial->write(u8ReadRightEncoder, sizeof(u8ReadRightEncoder));
    // rightMotorDriverNode._serial->flush();

    // while(!(rightMotorDriverNode._serial->available())) {}
    // if(rightMotorDriverNode._serial->available()) {
    //     u8RightReadBuffer[u8RightBufferSize++] = rightMotorDriverNode._serial->read();
    // }
    
    
    // printHEXcommand(u8LeftReadBuffer, u8LeftBufferSize);
    // // printHEXcommand(u8RightReadBuffer, u8RightBufferSize);
    // u8LeftBufferSize = 0;
    // u8RightBufferSize = 0;
    ++i;
    Serial.println("==============end================");
    // delay(5000);
    if(i==3){FOREVER;}
    
    // for(auto i: leftMotorVelocity_1000) {MotorDriverSerial.write(i);}
    // MotorDriverSerial.flush();
    // for(auto i: rightMotorVelocity_1000) {MotorDriverSerial.write(i);}
    // MotorDriverSerial.flush();
    // delay(5000);
    // for(auto i: leftMotorVelocity_0) {MotorDriverSerial.write(i);}
    // MotorDriverSerial.flush();
    // for(auto i: rightMotorVelocity_0) {MotorDriverSerial.write(i);}
    // MotorDriverSerial.flush();
    // delay(5000);


    // setMotorsSpeeds(2000, 2000);
    // delay(5000);
    // setMotorsSpeeds(0, 0);
    // delay(5000);
    // setMotorSpeeds(1000, 1000);
    // delay(5000);
    // setMotorSpeeds(0, 0);
    // delay(5000);

    // writeData[256] = {0};
    // leftMotorDriverNode._serial->write(u8ReadEncoder, sizeof(u8ReadEncoder));
    // leftMotorDriverNode._serial->flush();
    // while(!(leftMotorDriverNode._serial->available())){}
    // while(leftMotorDriverNode._serial->available()) {
    //     u8LeftReadBuffer[u8LeftBufferSize++] = leftMotorDriverNode._serial->read();
    // }
    // printHEXcommand(u8LeftReadBuffer, u8LeftBufferSize);
    // u8LeftBufferSize = 0;


    // for(auto i: leftMotorVelocity_1000) {leftMotorDriverNode._serial->write(i);}
    // leftMotorDriverNode._serial->flush();
    // for(auto i: rightMotorVelocity_1000) {rightMotorDriverNode._serial->write(i);}
    // rightMotorDriverNode._serial->flush();
    // delay(5000);

    // for(auto i: leftMotorVelocity_0) {leftMotorDriverNode._serial->write(i);}
    // leftMotorDriverNode._serial->flush();

    // for(auto i: rightMotorVelocity_0) {rightMotorDriverNode._serial->write(i);}
    // rightMotorDriverNode._serial->flush();
    // delay(5000);

    // MotorDriverSerial.write(u8ReadEncoder, sizeof(u8ReadEncoder));
    // MotorDriverSerial.flush();
    // while(!(MotorDriverSerial.available())){}
    // while(MotorDriverSerial.available()) {
    //     u8LeftReadBuffer[u8LeftBufferSize++] = MotorDriverSerial.read();
    // }
    // printHEXcommand(u8LeftReadBuffer, u8LeftBufferSize);
    // u8LeftBufferSize = 0;
    // Serial.println("===========reading encoder============");
    // leftMotorDriverNode.readHoldingRegisters(0x2020, 2, writeData);
    // printHEXcommand(writeData, 30);

    // Serial.println("========using writeMultipleRegisters set motors velocity========");
    
    
    // setMotorsSpeeds(2000, 2000);
    // delay(5000);
    // setMotorsSpeeds(0, 0);
    // delay(5000);
    // setMotorSpeeds(1000, 1000);
    // delay(5000);
    // setMotorSpeeds(0, 0);
    // delay(5000);

    // Serial.println("========OVER========");
    // for(;;){}


}
