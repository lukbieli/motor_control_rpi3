#include <stdio.h>
#include <math.h>
#include <string>

#include <pigpio.h>
#include <unistd.h>

#include "robot_move.hpp"
#include "adc.hpp"
#include "alphabot_cfg.h"

/* Single control */
/* g++ -Wall -pthread -o single_control ../single_control.cpp ../pid.cpp ../adc.cpp ../motor_ctrl.cpp ../robot_move.cpp ../rotary_encoder.cpp -lpigpio -lrt -lc -g */

/* run: sudo ./single_control -a 0.5 0.5 4 -c 24.0 55.7 0.9 -t 2 */
/* to plot: python plot.py plot.png*/

typedef enum{
    MODE_PID = 0,
    MODE_PWM
}MODE_e;

typedef struct
{
    MODE_e mode;
    bool tunningFlag;
    double speedLeft;
    double speedRight;
    int pwmLeft;
    int pwmRight;
    int time;
    double kp;
    double ki;
    double kd;
}InArgs;

void parseArgs(InArgs* outArgs, int argc, char* argv[])
{
    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        bool modeSel = false;

        if(arg == "-p") /* PWM mode with 2 int vals*/
        {
            if(modeSel)
            {
                printf("Cannot use -p and -a flags in same execution!");
            }
            else
            {
                modeSel = true;
                outArgs->mode = MODE_PWM;
                outArgs->pwmLeft = std::stoi(argv[i+1]);
                outArgs->pwmRight = std::stoi(argv[i+2]);
            }
            i += 2;
        }
        else if(arg == "-a") /* PiD mode with 2 double vals*/
        {
            if(modeSel)
            {
                printf("Cannot use -p and -a flags in same execution!");
            }
            else
            {
                modeSel = true;
                outArgs->mode = MODE_PID;
                outArgs->speedLeft = std::stod(argv[i+1]);
                outArgs->speedRight = std::stod(argv[i+2]);
            }
            i += 2;
        }
        else if(arg == "-t") /* time */
        {
            outArgs->time = std::stoi(argv[i+1]);
            i += 1;
        }
        else if(arg == "-c") /* PID tunning */
        {
            outArgs->tunningFlag = true;
            outArgs->kp = std::stod(argv[i+1]);
            outArgs->ki = std::stod(argv[i+2]);
            outArgs->kd = std::stod(argv[i+3]);
            i += 3;
        }
    }
}

int main(int argc, char* argv[]) {
    printf("You have entered %d arguments:\n", argc);
    
    for (int i = 0; i < argc; i++) {
        printf("%s\n", argv[i]);
    }

    InArgs args = 
    {
        .mode = MODE_PID,
        .tunningFlag = false,
        .speedLeft = 0.8,
        .speedRight = 0.8,
        .pwmLeft = 60,
        .pwmRight = 60,
        .time = 2,
        .kp = 24.0,
        .ki = 55.0,
        .kd = 1.0
    };
    parseArgs(&args,argc,argv);

    printf("AlphaBot Pi3 control start!\n");
    if (gpioInitialise() < 0){
        printf("Fail gpio\n");
        gpioTerminate();
        return 1;
    } 
    printf("Gpio OK\n");

    RobotMove Robot = RobotMove(0,1);
    ADCCtrl Adc = ADCCtrl(PIN_ADC_CS,PIN_ADC_ADDR,PIN_ADC_IOCLK,PIN_ADC_DOUT);

    printf("Battery volt: %.2f\n", Adc.readBatteryVoltage());
    // Robot.move(0.5,0.5);
    
    if(args.mode == MODE_PWM)
    {
        printf("Setting direct PWM. L: %d, R:%d for %d sec\n",args.pwmLeft,args.pwmLeft,args.time);
        Robot.moveDirect(args.pwmLeft,args.pwmLeft);
        sleep(args.time);   

    }
    else if(args.mode == MODE_PID)
    {
        if(args.tunningFlag)
        {
            printf("Sets PID tunnings: Kp: %.2f | Ki: %.2f | Kd: %.2f\n",args.kp, args.ki, args.kd);
            Robot.motorLeft.setPidTunnings(args.kp, args.ki, args.kd);
            Robot.motorRight.setPidTunnings(args.kp, args.ki, args.kd);
        }
        printf("Setting speed [m/s]. L: %.2f, R:%.2f for %d sec\n",args.speedLeft,args.speedRight,args.time);
        Robot.move(args.speedLeft, args.speedRight);
        sleep(args.time);   
    }

    // Robot.moveDirect(leftPwm,leftRight);
    // sleep(2);
    // Robot.move(0.8,0.8);
    // sleep(2);
    // Robot.move(1.0,1.0);
    // sleep(2);
    Robot.stop();

    if(args.mode == MODE_PID)
    {
        printf("Motor left\n");
        Robot.motorLeft.printHistory(10);
        Robot.motorLeft.saveHistoryToCSV("motorLeftPid.csv");
        printf("Motor right\n");
        Robot.motorRight.printHistory(10);
        Robot.motorRight.saveHistoryToCSV("motorRightPid.csv");
    }
    printf("ENC_L: %d | ENC_R: %d\n",Robot.motorLeft.enc_counter,Robot.motorRight.enc_counter);
    Robot.kill();
    gpioTerminate();
    return 0;
}