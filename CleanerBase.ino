#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

// #include <ModbusMaster.h>
// #include <SoftwareSerial.h>
/* Motor driver function definitions */
#include "servo_motor_driver.h"

/* Cross Coupling Control parameters and functions */
#include "CCC.h"

/* Run the CCC loop at 30 times per second */
#define CCC_RATE           30     // Hz

/* Convert the rate into an interval */
const int CCC_INTERVAL = 1000 / CCC_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextCCC = CCC_INTERVAL;

/* Stop the robot if it hasn't received a movement command
in this number of milliseconds */
//  电机接收到速度指令后的运行时间
#define AUTO_STOP_INTERVAL 5000
long lastMotorCommand = AUTO_STOP_INTERVAL;



/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
    int i = 0;
    char *p = argv1;
    char *str;
    int CCC_args[8];
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    
    switch(cmd) {
    case GET_BAUDRATE:
        Serial.println(BAUDRATE);
        break;
    case ANALOG_READ:
        Serial.println(analogRead(arg1));
        break;
    case DIGITAL_READ:
        Serial.println(digitalRead(arg1));
        break;
    case ANALOG_WRITE:
        analogWrite(arg1, arg2);
        Serial.println("OK"); 
        break;
    case DIGITAL_WRITE:
        if (arg2 == 0) digitalWrite(arg1, LOW);
        else if (arg2 == 1) digitalWrite(arg1, HIGH);
        Serial.println("OK"); 
        break;
    case PIN_MODE:
        if (arg2 == 0) pinMode(arg1, INPUT);
        else if (arg2 == 1) pinMode(arg1, OUTPUT);
        Serial.println("OK");
        break;
    case PING:
        Serial.println(Ping(arg1));
        break;
    case READ_ENCODERS:
        Serial.print(readEncoder(LEFT));
        Serial.print(" ");
        Serial.println(readEncoder(RIGHT));
        break;
    case RESET_ENCODERS:
        resetEncoders();
        resetCrossCoupl();
        Serial.println("OK");
        break;
    case MOTOR_SPEEDS:
        /* Reset the auto stop timer */
        lastMotorCommand = millis();
        if (arg1 == 0 && arg2 == 0) {
            setMotorSpeeds(0, 0);
            resetCrossCoupl();
            moving = 0;
        }
        else moving = 1;
        // 设置CCC调试的目标
        sLeftCrossCoupl.targetVel = arg1;
        sRightCrossCoupl.targetVel = arg2;

        Serial.println("OK"); 
        break;
    case UPDATE_CCC:
        while ((str = strtok_r(p, ":", &p)) != '\0') {
            CCC_args[i] = atoi(str);
            i++;
        }
        sLeftCrossCoupl.Kp = CCC_args[0];
        sLeftCrossCoupl.Kd = CCC_args[1];
        sLeftCrossCoupl.Ki = CCC_args[2];

        sRightCrossCoupl.Kp = CCC_args[3];
        sRightCrossCoupl.Kd = CCC_args[4];
        sRightCrossCoupl.Ki = CCC_args[5];

        Serial.println("OK");
        break;
    default:
        Serial.println("Invalid Command");
        break;
    }
}

void setup() {
    initMotorsController();
    resetCrossCoupl();
}

void loop() {

    // Serial.println(Kp);
    while (Serial.available() > 0) {
    
        // Read the next character
        chr = Serial.read();

        // Terminate a command with a CR
        if (chr == 13) {
            if (arg == 1) argv1[index] = NULL;
            else if (arg == 2) argv2[index] = NULL;
            runCommand();
            resetCommand();
        }
        // Use spaces to delimit parts of the command
        else if (chr == ' ') {
            // Step through the arguments
            if (arg == 0) arg = 1;
            else if (arg == 1)  {
                argv1[index] = NULL;
                arg = 2;
                index = 0;
            }
            continue;
        }
        else {
            if (arg == 0) {
                // The first arg is the single-letter command
                cmd = chr;
            }
            else if (arg == 1) {
                // Subsequent arguments can be more than one character
                argv1[index] = chr;
                index++;
            }
            else if (arg == 2) {
                argv2[index] = chr;
                index++;
            }
        }
    }
  

    if (millis() > nextCCC) {
        updateCrossCoupl();
        nextCCC += CCC_INTERVAL;
    }
  
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
        setMotorSpeeds(0, 0);
        moving = 0;
    }
}
/*
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

*/