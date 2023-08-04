#include <stdio.h>
#include <math.h>
#include <string>

#include <pigpio.h>
#include <unistd.h>

#include "robot_move.hpp"
#include "adc.hpp"
#include "alphabot_cfg.h"
#include "xbox_controller.hpp"

/* Single control */
/* g++ -Wall -pthread -o remote_control ../remote_control.cpp ../pid.cpp ../adc.cpp ../motor_ctrl.cpp ../robot_move.cpp ../xbox_controller.cpp -I/usr/include/libevdev-1.0 -levdev -lpigpio -lrt -lc -g */

/* run: sudo ./remote_control */

volatile bool update = false;

void recalcSpeed(double* s_l, double* s_r, XB_Event* ev)
{
    static double last_speed = 0;
    static double last_x = 0;
    const double speedMax = 1.5;
    bool change = false;
    if (ev->type == XB_EV_GAS)
    {
        last_speed = ev->val * speedMax;
        change = true;
    }
    else if (ev->type == XB_EV_X_AXSIS)
    {
        change = true;
        last_x = ev->val;
    }

    if(change)
    {
        if(last_x < 0.01)
        {
            *s_l = last_speed * ev->val;
            *s_r = last_speed;
        }
        else if(last_x > 0.01)
        {
            *s_r = last_speed * -ev->val;
            *s_l = last_speed;
        }
        else
        {
            *s_l = last_speed;
            *s_r = last_speed;
        }
    }
}

void timer_callback(void)
{
    update = true;
}

int main(int argc, char* argv[]) {

    printf("AlphaBot Pi3 remote control start!\n");
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
    
    /* initialize box controller*/
    XboxController controller("/dev/input/event1");
    if (!controller.isValid())
    {
        std::cerr << "Failed to initialize the Xbox controller." << std::endl;
        return 1;
    }

    XB_Event xboxEvent;
    double speedLeft, speedRight = 0.0;

    gpioSetTimerFunc(2,150,timer_callback);
    
    // const int xboxInMax = 1.5;
    while (true)
    {
        if (controller.readEvent(xboxEvent))
        {
            // Handle the Xbox event here
            if ((xboxEvent.type == XB_EV_GAS) || (xboxEvent.type == XB_EV_X_AXSIS))
            {
                recalcSpeed(&speedLeft, &speedRight, &xboxEvent);
            }
            else if (xboxEvent.type == XB_EV_BURGER)
            {
                std::cout << "Burger event! " << std::endl;
                break;
            }
        }

        if(update)
        {
            update = false;
            std::cout << "L: " << speedLeft << " | R: " <<  speedRight << std::endl; 
            // Robot.move(speedLeft,speedRight);
        }
    }

    Robot.stop();

    Robot.kill();
    gpioTerminate();
    return 0;
}