#include "motor_ctrl.hpp"
#include <stdio.h>
#include <math.h>

#include <pigpio.h>
#include <unistd.h>

void MotorCtrl::config(const unsigned int PIN_PWM, const unsigned int PIN_IN1, const unsigned int PIN_IN2, const unsigned int PIN_ENC, const int timerNum)
{
    this->GPIO_PWM = PIN_PWM;
    this->GPIO_IN1 = PIN_IN1;
    this->GPIO_IN2 = PIN_IN2;
    this->GPIO_ENC = PIN_ENC;

    /* prepare gpio */

    /* setup bridge control */
    gpioSetMode(this->GPIO_IN1,PI_OUTPUT);
    gpioSetMode(this->GPIO_IN2,PI_OUTPUT);
    gpioWrite(this->GPIO_IN1,0);
    gpioWrite(this->GPIO_IN2,0);

    /* set PWM freq */
    gpioSetPWMfrequency(this->GPIO_PWM,1000);
    
    /* setup encoders input */
    gpioSetMode(this->GPIO_ENC, PI_INPUT);
    gpioSetPullUpDown(this->GPIO_ENC, PI_PUD_UP);
    /* monitor encoder level changes */
    this->enc_counter = 0;
    gpioSetAlertFuncEx(this->GPIO_ENC, this->encoderCbkExt, (void*)this);

    /* PID setup*/
    this->pid.Reset();
    this->pid.SetTunings(24.0,55.0,1.0); /* 15.0,13.0,0.0 | 34.0,28.0,0.5 | 110.0 42.0 0.5*/
    this->pid.SetOutputLimits(0.0,255.0);
    this->pid.SetMode(AUTOMATIC);
    // this->pid.SetSetpoint(1.5);
    this->pid.SetSampleTime(this->samplingRate);

    /* enable sampling timer */
    this->timerId = timerNum;

    mode = 1; /* PID controller */

}

void MotorCtrl::setPidTunnings(double kp, double ki, double kd)
{
    this->pid.SetTunings(kp,ki,kd);
}

void MotorCtrl::setSpeed(double reqSpeed)
{
    if((reqSpeed >= this->speedMin) && (reqSpeed <= this->speedMax) && (reqSpeed != this->setpoint))
    {
        this->setpoint = reqSpeed;
        pid.SetSetpoint(this->setpoint);
        this->runFlag = true;
    }
}
void MotorCtrl::setDirection(MotorCtrl_Direction dir)
{
    if(dir != this->direction)
    {
        this->direction = dir;
        if(dir == MotCtrl_FORWARD)
        {
            gpioWrite(this->GPIO_IN1,1);
            gpioWrite(this->GPIO_IN2,0);
        }
        else
        {
            gpioWrite(this->GPIO_IN1,0);
            gpioWrite(this->GPIO_IN2,1);
        }
    }
    
}
void MotorCtrl::start(void)
{
    if(this->mode == 2)
    {
        this->mode = 1;
        this->stop();
        this->pid.Reset();
    }
    /* enable sampling timer */
    gpioSetTimerFuncEx(this->timerId,this->samplingRate,this->timerSampleExt,(void*)this);
    this->enc_counter = 0;
}
void MotorCtrl::stop(void)
{
    this->runFlag = false;
    this->direction = MotCtrl_NULL;
    gpioWrite(this->GPIO_IN1,0);
    gpioWrite(this->GPIO_IN2,0);
    gpioPWM(this->GPIO_PWM,0);
}
void MotorCtrl::kill(void)
{
    this->stop();
    gpioSetTimerFunc(this->timerId,this->samplingRate,NULL);
    this->pid.Reset();
}

void MotorCtrl::forcePwm(int pwm)
{
    if(this->mode == 1)
    {
        gpioSetTimerFunc(this->timerId,this->samplingRate,NULL);
        this->mode = 2;
    }
    gpioPWM(this->GPIO_PWM,pwm);
}

void MotorCtrl::encoderCbk(int gpio, int level, uint32_t tick)
{
    this->enc_counter++;
}

void MotorCtrl::encoderCbkExt(int gpio, int level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   MotorCtrl *mySelf = (MotorCtrl *) user;

   mySelf->encoderCbk(gpio, level, tick); /* Call the instance callback. */
}

void MotorCtrl::timerSample(void)
{
    // static int i = 0;
    // static bool once = true;
    if(this->runFlag)
    {
        if(this->enc_counter > 0)
        {
            double rps_l = ((double)this->enc_counter/20.0)/(samplingRateS);
            this->speed = (wheel_circ * rps_l)/1000.0; /* in m/s*/
        }
        else
        {
            this->speed = 0;
        }

        /* compute PID */
        double out = this->pid.Compute(this->speed);

        if(this->historyIdx < HISTORY_SIZE)
        {
            this->historyIn[this->historyIdx] = this->speed;
            this->historyOut[this->historyIdx] = out;
            this->historyErr[this->historyIdx] = this->setpoint - this->speed;
            this->historySet[this->historyIdx] = this->setpoint;
            this->historyEnc[this->historyIdx] = this->enc_counter;
            
            this->historyIdx++;
        }
        this->enc_counter = 0;

        gpioPWM(this->GPIO_PWM,out);

        // if (i++ == 20 && once) {
        //     this->pid.EnableAutoTune(10.0);
        //     once = false;
        // }
        // this->pid.AutoTuneStep();
    }
}

void MotorCtrl::timerSampleExt(void *user)
{
   /*
      Need a static callback to link with C.
   */

   MotorCtrl *mySelf = (MotorCtrl *) user;

   mySelf->timerSample(); /* Call the instance callback. */
}

void MotorCtrl::printHistory(int max){
    printf("History:\n");
    printf("OUT\t|\tENC\t|\tIN\n");

    for(int i = 0; i < max; i++)
    {
        printf("%.2f\t|\t%d\t|\t%.2f\n",this->historyOut[i],this->historyEnc[i],this->historyIn[i]);
    }
}

void MotorCtrl::saveHistoryToCSV(const std::string& filename) {
    std::ofstream outputFile(filename);

    if (outputFile.is_open()) {
        outputFile << "OUT,ENC,IN,SET" << std::endl;
        for (int i = 0; i < this->historyIdx; ++i) {
            // Separate elements with commas (CSV format)
            outputFile << this->historyOut[i] << "," << this->historyEnc[i] << "," << this->historyIn[i] << "," << this->historySet[i] << std::endl;
            
        }

        outputFile.close();
        std::cout << "Array saved to " << filename << " successfully." << std::endl;
    } else {
        std::cerr << "Error opening file: " << filename << std::endl;
    }
}