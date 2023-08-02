#include <stdio.h>
#include <math.h>

#include <pigpio.h>
#include <unistd.h>

#include "robot_move.hpp"
#include "adc.hpp"
#include "alphabot_cfg.h"
/* g++ -Wall -pthread -o foobar ../main.cpp ../pid.cpp ../acd.cpp ../motor_ctrl.cpp ../robot_move.cpp -lpigpio -lrt -lc */

// unsigned readADC(unsigned channel);

// TLC1543
// static const int GPIO_ADC_IOCLK = 25;
// static const int GPIO_ADC_ADDR = 24;
// static const int GPIO_ADC_DOUT = 23;
// static const int GPIO_ADC_CS = 5;
// static const int ADC_VIN = 10;

// bool running = false;
// int timerNumber = 0;
// long samplingInterval = 0;
// volatile int counter = 0;

// static const int GPIO_MOTORL = 6;
// static const int GPIO_MOTORR = 26;
// static const int GPIO_MOTOR_IN1 = 12; /* left motor */
// static const int GPIO_MOTOR_IN2 = 13; /* left motor */
// static const int GPIO_MOTOR_IN3 = 20; /* right motor */
// static const int GPIO_MOTOR_IN4 = 21; /* right motor */

// static const int GPIO_ENC_R = 8; /* CE0 - encoder right wheel */
// static const int GPIO_ENC_L = 7; /* CE1 - encoder left wheel */

// static const double wheel_diam = 65.0f; /* diameter in mm */
// static const int samplingRate = 150; /* encoder + pid sampling rate*/
// static const double samplingRateS = (double)samplingRate/1000.0; /* encoder + pid sampling rate*/

// static void pigpioTimerCallback(void* arg) {
//     // printf("%s %d\n",(char*)arg, counter++);
// }

// volatile int counter_L = 0;
// volatile int counter_R = 0;
// volatile bool calcFlag = false;

// PID pidLeft;
// PID pidRight;

// const int historySize = 100;

// double pidHistoryL[historySize] = {0.0};
// double pidInputL[historySize] = {0.0};
// double pidErrorL[historySize] = {0.0};
// double pidHistoryR[historySize] = {0.0};
// double pidInputR[historySize] = {0.0};
// double pidErrorR[historySize] = {0.0};
// int pidIdx = 0;

// static void pigpioTimerCallback_speed(void) {
//     // printf("%s %d\n",(char*)arg, counter++);
//     // float rps_l = ((float)counter_L/20.0f);
//     // float speed_l = (wheel_diam * 3.14 * 2 * rps_l)/1000; 
//     // counter_L = 0;

//     // float rps_r = ((float)counter_R/20.0f);
//     // float speed_r = (wheel_diam * 3.14 * 2 * rps_r)/1000; 
//     // counter_R = 0;

//     // printf("LEFT  | RPS: %.3f SPEED: %.3f [m/s]\n",rps_l,speed_l);
//     // printf("RIGHT | RPS: %.3f SPEED: %.3f [m/s]\n",rps_r,speed_r);

//     calcFlag = true;

// }

// typedef struct{
//     uint32_t min;
//     uint32_t max;
//     float avg;
//     int cnt;
//     uint32_t last;
//     uint32_t history[20];
//     uint8_t histIdx;
// }T_encoder_stats;

// static T_encoder_stats rightEncStats = {0xffffffff,0,0.0f,0,0,{0},0};
// static T_encoder_stats leftEncStats = {0xffffffff,0,0.0f,0,0,{0},0};


// void calcStats(T_encoder_stats* stats, uint32_t tick)
// {
//     stats->cnt++;
//     uint32_t diff = tick - stats->last;
//     stats->max = (diff > stats->max) ? diff : stats->max;
//     stats->min = (diff < stats->min) ? diff : stats->min;
//     if(stats->histIdx >= 20)
//     {
//         stats->histIdx = 0;
//     }
//     stats->history[stats->histIdx++] = diff;

//     stats->avg = 0;
//     for(int i = 0; i < 20; i++)
//     {
//         stats->avg += stats->history[i];
//     }
//     float div = (stats->cnt < 20) ? (float)stats->cnt : 20.0f;
//     stats->avg /= div;
// }

// void printStats(T_encoder_stats* stats)
// {
//     printf("CNT: %d, MIN: %zu, MAX: %zu, AVG: %f\n",stats->cnt, stats->min, stats->max, stats->avg);
//     printf("DELTA: %zu, ERR: %f",stats->max - stats->min, (float)(stats->max - stats->min)/stats->avg);
//     printf("Last 20: {");
//     for(int i = 0; i < 20; i++)
//     {
//         printf("%zu ,",stats->history[i]);
//     }
//     printf("}\n");

// }

// void pulseEnc(int gpio, int level, uint32_t tick)
// {
//     if(level == 0)
//     {
//         if(gpio == GPIO_ENC_L)
//         {
//             counter_L++;
//             calcStats(&leftEncStats,tick);
//         }
//         else if(gpio == GPIO_ENC_R)
//         {
//             counter_R++;
//             calcStats(&rightEncStats,tick);
//         }
//         else
//         {
//             /* do nothing */
//         }
//     }
// }

// double setpoints[5] = {0.5,0.7,1.0,1.5,2.0};
// double setpoints[1] = {0.55};

int main() {
    printf("AlphaBot Pi3 control start!\n");
    if (gpioInitialise() < 0){
        printf("Fail gpio\n");
        gpioTerminate();
        return 1;
    } 
    printf("Gpio OK\n");

    RobotMove Robot = RobotMove(0,1);
    ADCCtrl Adc = ADCCtrl(PIN_ADC_CS,PIN_ADC_ADDR,PIN_ADC_IOCLK,PIN_ADC_DOUT);

    printf("Battery volt: %.2f", Adc.readBatteryVoltage());
    Robot.move(0.5,0.5);
    sleep(1);
    Robot.stop();

    gpioTerminate();
    return 0;
}