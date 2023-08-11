#ifndef EAA5739D_F4EB_463A_AD27_546F7C8E4214
#define EAA5739D_F4EB_463A_AD27_546F7C8E4214

#include "motor_ctrl.hpp"

class RobotMove {
    public:
        // Constructor
        RobotMove(int timer1, int timer2);

        // Public method to move the robot
        void move(double left, double right);
        void moveDirect(int left, int right);

        // Public method to stop the robot
        void stop();

        void kill();

        
        MotorCtrl motorLeft;
        MotorCtrl motorRight;
    // private:
        // MotorCtrl motorLeft;
        // MotorCtrl motorRight;
};

#endif /* EAA5739D_F4EB_463A_AD27_546F7C8E4214 */
