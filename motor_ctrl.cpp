#include "motor_ctrl.hpp"
#include <stdio.h>
#include <math.h>

#include <pigpio.h>
#include <unistd.h>

MotorCtrl::config(const unsigned int PIN_PWM, const unsigned int PIN_IN1, const unsigned int PIN_IN2, const unsigned int PIN_ENC, const int timerNum)
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
    gpioSetAlertFunc(this->GPIO_ENC, this->encoderCbk);

    /* PID setup*/

    this->pid.SetTunings(34.0,28.0,0.5); /* 15.0,13.0,0.0 */
    this->pid.SetMode(AUTOMATIC);
    this->pid.SetOutputLimits(50.0,255.0);
    // this->pid.SetSetpoint(1.5);
    this->pid.SetSampleTime(this->samplingRate);

    /* enable sampling timer */
    this->timerId = timerNum;

}

void MotorCtrl::setSpeed(double reqSpeed)
{
    if((reqSpeed >= this->speedMin) && (reqSpeed <= this>speedMax))
    {
        this->setpoint = reqSpeed;
        pid.SetSetpoint(this->setpoint);
    }
}
void MotorCtrl::setDirection(MotorCtrl_Direction dir)
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
void MotorCtrl::start(void)
{
    /* enable sampling timer */
    gpioSetTimerFunc(this->timerId,this->samplingRate,this->timerSample);
}
void MotorCtrl::stop(void)
{
    gpioWrite(this->GPIO_IN1,0);
    gpioWrite(this->GPIO_IN2,0);
    gpioPWM(this->GPIO_PWM,0);
}
void MotorCtrl::kill(void)
{
    this->stop();
    gpioSetTimerFunc(this->timerId,this->samplingRate,NULL);
}

void MotorCtrl::encoderCbk(int gpio, int level, uint32_t tick)
{
    enc_counter++;
}

void MotorCtrl::timerSample(void)
{
    double rps_l = ((double)enc_counter/20.0)/(samplingRateS);
    this->speed = (wheel_circ * rps_l)/1000.0; /* in m/s*/
    enc_counter = 0;

    /* compute PID */
    double out = pidLeft.Compute(this->speed);

    if(this->historyIdx > 100)
    {
        this->historyIdx = 0;
    }
    this->historyIn[this->historyIdx] = this->speed;
    this->historyOut[this->historyIdx] = out;
    this->historyErr[this->historyIdx] = this->setpoint;
    
    this->historyIdx++;

    gpioPWM(this->GPIO_PWM,out);
}