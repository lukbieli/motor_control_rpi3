#ifndef C5F21679_1B27_4C83_AD6D_6B90A049401B
#define C5F21679_1B27_4C83_AD6D_6B90A049401B

#include <stdint.h>
#include <cmath>
#include "pid.hpp"
#include <string.h>

typedef enum MotorCtrl_Direction
{
    MotCtrl_FORWARD = 0,
    MotCtrl_BACKWARD = 1
}MotorCtrl_Direction;

class MotorCtrl{
    public:
        void config(const unsigned int PIN_PWM, const unsigned int PIN_IN1, const unsigned int PIN_IN2, const unsigned int PIN_ENC, const int timerNum);
        void setSpeed(double speed);
        void setDirection(MotorCtrl_Direction dir);
        void start(void); /* starts sampling */
        void stop(void); /* stops motor */
        void kill(void); /* kills entire sampling */
        void forcePwm(int pwm);

        void printHistory(int max);
        void saveHistoryToFile(string name);

    private:

        unsigned int GPIO_PWM;
        unsigned int GPIO_IN1 = 0;
        unsigned int GPIO_IN2 = 0;
        unsigned int GPIO_ENC = 0;
        const double wheel_diam = 65.0; /* diameter in mm */
        const double wheel_circ = wheel_diam * M_PI; /* wheel circuit */
        const int samplingRate = 150; /* encoder + pid sampling rate*/
        const double samplingRateS = (double)samplingRate/1000.0; /* encoder + pid sampling rate*/
        const double speedMin = 0.5; /* minimum speed in m/s */
        const double speedMax = 1.5; /* maximal speed in m/s */

        PID pid;
        int timerId;
        MotorCtrl_Direction direction;
        volatile int enc_counter;
        volatile double speed;
        volatile double setpoint;

        double historyOut[100] = {0.0};
        double historyIn[100] = {0.0};
        double historyErr[100] = {0.0};
        int historyEnc[100] = {0};
        int historyIdx = 0;
        
        void encoderCbk(int gpio, int level, uint32_t tick);
        static void encoderCbkExt(int gpio, int level, uint32_t tick, void *user);
        void timerSample(void);
        static void timerSampleExt(void *user);
        int mode = 0;
        bool runFlag = false;
};


#endif /* C5F21679_1B27_4C83_AD6D_6B90A049401B */
