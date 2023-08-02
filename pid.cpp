#include "pid.hpp"

PID::PID(void)
{
    this->Input = 0;
    this->Output = 0;
    this->ITerm = 0;
    this->lastInput = 0;
}

double PID::Compute(double in)
{
    this->Input = in;
    if (!this->inAuto)
        return this->Output;

    /*Compute all the working error variables*/
    double error = this->Setpoint - this->Input;
    this->ITerm += (this->ki * error);
    if (this->ITerm > this->outMax)
        this->ITerm = this->outMax;
    else if (this->ITerm < this->outMin)
        this->ITerm = this->outMin;
    double dInput = (this->Input - this->lastInput);

    /*Compute PID this->Output*/
    this->Output = this->kp * error + this->ITerm - this->kd * dInput;
    if (this->Output > this->outMax)
        this->Output = this->outMax;
    else if (this->Output < this->outMin)
        this->Output = this->outMin;

    /*Remember some variables for next time*/
    this->lastInput = this->Input;

    return this->Output;
}

void PID::Reset()
{
    this->Input = 0;
    this->Output = 0;
    this->ITerm = 0;
    this->lastInput = 0;
}

void PID::SetSetpoint(double sp)
{
    this->Setpoint = sp;
}

void PID::SetTunings(double Kp, double Ki, double Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    // double SampleTimeInSec = ((double)this->SampleTime) / 1000;
    // this->kp = Kp;
    // this->ki = Ki * SampleTimeInSec;
    // this->kd = Kd / SampleTimeInSec;

    this->kp = Kp;
    this->ki = Ki;
    this->kd = Kd;

    if (controllerDirection == REVERSE)
    {
        this->kp = (0 - this->kp);
        this->ki = (0 - this->ki);
        this->kd = (0 - this->kd);
    }
}


void PID::GetTunings(double* Kp, double* Ki, double* Kd)
{
    *Kp = this->kp;
    *Ki = this->ki;
    *Kd = this->kd;
}

void PID::SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        // double ratio = (double)NewSampleTime / (double)this->SampleTime;
        // this->ki *= ratio;
        // this->kd /= ratio;
        this->SampleTime = (unsigned long)NewSampleTime;
    }
}

void PID::SetOutputLimits(double Min, double Max)
{
    if (Min > Max)
        return;
    this->outMin = Min;
    this->outMax = Max;

    if (this->Output > this->outMax)
        this->Output = this->outMax;
    else if (this->Output < this->outMin)
        this->Output = this->outMin;

    if (this->ITerm > this->outMax)
        this->ITerm = this->outMax;
    else if (this->ITerm < this->outMin)
        this->ITerm = this->outMin;
}

void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto == !inAuto)
    { /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void PID::Initialize()
{
    this->lastInput = this->Input;
    this->ITerm = this->Output;
    if (this->ITerm > this->outMax)
        this->ITerm = this->outMax;
    else if (this->ITerm < this->outMin)
        this->ITerm = this->outMin;
}

void PID::SetControllerDirection(int Direction)
{
    controllerDirection = Direction;
}

void PID::EnableAutoTune(double autoTuneStep) {
    autoTuneStep_ = autoTuneStep;
    autoTuneState_ = 1;
    inAuto = false; // Turn off the PID controller during auto-tuning
    autoTuneInitialOutput_ = Output;
    autoTuneSetpoint_ = Setpoint;

    // Store the start time for auto-tuning using iteration count
    autoTuneStartTime_ = SampleTime; // Initialize to first step duration
}

void PID::DisableAutoTune() {
    autoTuneState_ = 0;
    inAuto = true; // Re-enable the PID controller after auto-tuning
    SetTunings(autoTuneKp_, autoTuneKi_, autoTuneKd_); // Set the tuned PID gains
    Initialize(); // Initialize the PID controller's internal state
}

void PID::AutoTuneStep() {
    switch (autoTuneState_) {
        case 1: // First auto-tuning step
            autoTuneKp_ = 0.6 * autoTuneOutput_ / (autoTuneSetpoint_ - Input);
            autoTuneOutput_ = autoTuneInitialOutput_ + autoTuneStep_;
            autoTuneState_ = 2;
            break;
        case 2: // Second auto-tuning step
            autoTuneOutput_ = autoTuneInitialOutput_ - autoTuneStep_;
            autoTuneState_ = 3;
            break;
        case 3: // Third auto-tuning step
            // Calculate the elapsed time since the start of the third step
            autoTuneStartTime_ += SampleTime;

            // Check if the elapsed time is greater than the ultimate period (Tu)
            if (autoTuneStartTime_ >= autoTuneTu_) {
                autoTuneKu_ = autoTuneOutput_;
                autoTuneKc_ = 0.33 * autoTuneKu_;
                autoTuneTi_ = 0.5 * autoTuneTu_;
                autoTuneTd_ = 0.125 * autoTuneTu_;
                autoTuneKp_ = 0.6 * autoTuneKc_;
                autoTuneKi_ = autoTuneKp_ / autoTuneTi_;
                autoTuneKd_ = autoTuneKp_ * autoTuneTd_;
                DisableAutoTune(); // Auto-tuning complete
            }
            break;
        default:
            break;
    }
}