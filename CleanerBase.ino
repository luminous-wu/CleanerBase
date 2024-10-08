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
#define CCC_RATE           50     // Hz

/* Convert the rate into an interval */
const int CCC_INTERVAL = 1000 / CCC_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextCCC = CCC_INTERVAL;

/* Stop the robot if it hasn't received a movement command
in this number of milliseconds */
//  电机接收到速度指令后的运行时间
#define AUTO_STOP_INTERVAL 3000
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
char argv1[50];
char argv2[50];

// The arguments converted to integers
long arg1;
long arg2;
int i;
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
    float CCC_args[10];
    arg1 = atof(argv1);
    arg2 = atof(argv2);
    
    switch(cmd) {
    case GET_BAUDRATE:
        Serial.println(BAUDRATE_ARDUINO_PC);
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
    case READ_VEL:
        Serial.print(readVel(LEFT));
        Serial.print(" ");
        Serial.println(readVel(RIGHT));
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
            CCC_args[i] = atof(str);
            i++;
        }
        sLeftCrossCoupl.Kp = CCC_args[0];
        sLeftCrossCoupl.Ki = CCC_args[1];
        sLeftCrossCoupl.Kd = CCC_args[2];
        sRightCrossCoupl.Kp = CCC_args[3];
        sRightCrossCoupl.Ki = CCC_args[4];
        sRightCrossCoupl.Kd = CCC_args[5];
        sCCCPara.Kp = CCC_args[6];
        sCCCPara.Ki = CCC_args[7];
        sCCCPara.Kd = CCC_args[8];
        
        // Serial.print(sLeftCrossCoupl.Kp);
        // Serial.print(sLeftCrossCoupl.Ki);
        // Serial.print(sLeftCrossCoupl.Kd);
        // Serial.print(sRightCrossCoupl.Kp);
        // Serial.print(sRightCrossCoupl.Ki);
        // Serial.print(sRightCrossCoupl.Kd);
        // Serial.print(sCCCPara.Kp);
        // Serial.print(sCCCPara.Ki);
        // Serial.print(sCCCPara.Kd);
        Serial.println("OK");
        break;
    case READ_PID_PARAM:
        Serial.print(float(sLeftCrossCoupl.Kp));
        Serial.print(" ");
        Serial.print(float(sLeftCrossCoupl.Ki));
        Serial.print(" ");
        Serial.print(float(sLeftCrossCoupl.Kd));
        Serial.print(" ");
        Serial.print(float(sRightCrossCoupl.Kp));
        Serial.print(" ");
        Serial.print(float(sRightCrossCoupl.Ki));
        Serial.print(" ");
        Serial.print(float(sRightCrossCoupl.Kd));
        Serial.print(" ");
        Serial.print(float(sCCCPara.Kp));
        Serial.print(" ");
        Serial.print(float(sCCCPara.Ki));
        Serial.print(" ");
        Serial.println(float(sCCCPara.Kd));
        break;
    default:
        Serial.println("Invalid Command");
        break;
    }
}

void setup() {
    
    initMotorController();
    resetCrossCoupl();
    
}
//  /*
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
            // Serial.println("execute in "chr == 13" ");
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
            // Serial.println("execute in "chr == ' '" ");
            continue;
        }
        else {
            // Serial.println("execute in "chr == '  2'" ");
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
        // Serial.println("execute in millis() > nextCCC ");
    }
  
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
        setMotorSpeeds(0, 0);
        moving = 0;
        // Serial.println("execute in (millis() - lastMotorCommand) > AUTO_STOP_INTERVAL ");
    }
}
//  */
