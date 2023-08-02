

#include "robot_move.hpp"
#include "alphabot_cfg.h"

// Constructor implementation
RobotMove::RobotMove(int timerId1, int timerId2) {
    // Add any necessary initialization code here
    this->motorLeft.config(PIN_MOTORL_EN,PIN_HBRIDGE_IN1,PIN_HBRIDGE_IN2,PIN_ENC_L,timerId1);
    this->motorRight.config(PIN_MOTORR_EN,PIN_HBRIDGE_IN4,PIN_HBRIDGE_IN3,PIN_ENC_R,timerId2);
}

// Method to move the robot implementation
void RobotMove::move(double left, double right) {
    // Add code to control the robot's left and right motors here
    if(left > 0.0)
    {
        this->motorLeft.setDirection(MotCtrl_FORWARD);
        this->motorLeft.setSpeed(left);
    }
    else if(left < 0.0)
    {
        this->motorLeft.setDirection(MotCtrl_BACKWARD);
        left = 0.0 - left;
        this->motorLeft.setSpeed(left);
    }
    else
    {
        this->motorLeft.stop();
    }
    if(right > 0.0)
    {
        this->motorRight.setDirection(MotCtrl_FORWARD);
        this->motorRight.setSpeed(right);
    }
    else if(right < 0.0)
    {
        this->motorRight.setDirection(MotCtrl_BACKWARD);
        right = 0.0 - right;
        this->motorRight.setSpeed(right);
    }
    else
    {
        this->motorRight.stop();
    }
    
}

// Method to stop the robot implementation
void RobotMove::stop() {
    // Add code to stop the robot's motors here
    this->motorLeft.stop();
    this->motorRight.stop();
}
