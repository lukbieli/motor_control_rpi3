#ifndef C5F21679_1B27_4C83_AD6D_6B90A049401B
#define C5F21679_1B27_4C83_AD6D_6B90A049401B

#include <stdint.h>
#include <cmath>
#include "pid.hpp"
#include <string.h>
#include <iostream>
#include <fstream>
#include "alphabot_cfg.h"
#if defined(HW_N20D)
#include "rotary_encoder.hpp"
#endif

typedef enum MotorCtrl_Direction
{
    MotCtrl_FORWARD = 0,
    MotCtrl_BACKWARD = 1,
    MotCtrl_NULL
}MotorCtrl_Direction;

#define HISTORY_SIZE (200)

class MotorCtrl{
    public:
#if  defined(HW_N20D)
        MotorCtrl(const unsigned int PIN_PWM, const unsigned int PIN_IN1, const unsigned int PIN_IN2, const unsigned int PIN_ENC1, const unsigned int PIN_ENC2, const int timerNum);
#elif defined(HW_ORIGINAL)
        MotorCtrl(const unsigned int PIN_PWM, const unsigned int PIN_IN1, const unsigned int PIN_IN2, const unsigned int PIN_ENC1, const int timerNum);
#else
    #error "Must select HW configuration!"
#endif

        void setSpeed(double speed);
        void setDirection(MotorCtrl_Direction dir);
        void start(void); /* starts sampling */
        void stop(void); /* stops motor */
        void kill(void); /* kills entire sampling */
        void forcePwm(int pwm);

        void printHistory(int max);
        void saveHistoryToCSV(const std::string& filename);
        void setPidTunnings(double kp, double ki, double kd);
        
        PID pid;
        volatile int enc_counter;

    private:

        unsigned int GPIO_PWM;
        unsigned int GPIO_IN1 = 0;
        unsigned int GPIO_IN2 = 0;
        unsigned int GPIO_ENC1 = 0;
        unsigned int GPIO_ENC2 = 0;
#if  defined(HW_N20D)
        const double wheel_diam = 60.0; /* diameter in mm */
        const double encoder_ppr = 350.0; /* number of encoder pulses per full rotation */
        const double speedMin = 0; /* minimum speed in m/s */
        const double speedMax = 0.6; /* maximal speed in m/s */
        const int samplingRate = 20; /* encoder + pid sampling rate*/
        encoder enc;
        void encoderICbk(int way);
        static void encoderICbkExt(int way, void *user);
#elif defined(HW_ORIGINAL)
        const double wheel_diam = 65.0; /* diameter in mm */
        const double encoder_ppr = 20.0; /* number of encoder pulses per full rotation */
        const double speedMin = 0; /* minimum speed in m/s */
        const double speedMax = 1.5; /* maximal speed in m/s */
        const int samplingRate = 100; /* encoder + pid sampling rate*/
        void encoderCbk(int gpio, int level, uint32_t tick);
        static void encoderCbkExt(int gpio, int level, uint32_t tick, void *user);
#else
    #error "Must select HW configuration!"
#endif
        const double wheel_circ = wheel_diam * M_PI; /* wheel circuit */
        const double samplingRateS = (double)samplingRate/1000.0; /* encoder + pid sampling rate*/

        int timerId;
        MotorCtrl_Direction direction = MotCtrl_NULL;


        volatile double speed;
        volatile double setpoint;

        double historyOut[HISTORY_SIZE] = {0.0};
        double historyIn[HISTORY_SIZE] = {0.0};
        double historyErr[HISTORY_SIZE] = {0.0};
        int historyEnc[HISTORY_SIZE] = {0};
        double historySet[HISTORY_SIZE] = {0};
        int historyIdx = 0;
        

        void timerSample(void);
        static void timerSampleExt(void *user);
        int mode = 0;
        bool runFlag = false;
};


#endif /* C5F21679_1B27_4C83_AD6D_6B90A049401B */
