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
    static double last_speed = 0.0;
    const double speedMax = 1.0;
    bool change = false;

    const double x_mid = 0.5;
    const double x_max = 1.0;
    const double x_dead = 0.2;
    static double last_x = x_mid;

    if (ev->type == XB_EV_GAS)
    {
        last_speed = ev->val * speedMax;
        // std::cout << "Speed: " << last_speed << std::endl; 
        change = true;
    }
    else if (ev->type == XB_EV_X_AXSIS)
    {
        change = true;
        last_x = ev->val;
    }
    else if (ev->type == XB_EV_BREAK)
    {
        last_speed = -ev->val * speedMax;
        // std::cout << "Speed: " << last_speed << std::endl; 
        change = true;
    }

    if(change)
    {
        if(last_x > x_mid + x_dead)
        {
            *s_r = (x_max-last_x)/(x_mid - x_dead) * last_speed;
            *s_l = last_speed;
        }
        else if(last_x < x_mid - x_dead)
        {
            *s_l = (last_x)/(x_mid - x_dead) * last_speed;
            *s_r = last_speed;
        }
        else
        {
            *s_l = last_speed;
            *s_r = last_speed;              
        }
        // std::cout << "CHANGE = L: " << *s_l  << " | R: " <<  *s_r << std::endl; 
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
    Robot.motorLeft.pid.SetTunings(80.0, 42.0, 0.5);
    Robot.motorRight.pid.SetTunings(110.0, 42.0, 0.5);
    ADCCtrl Adc = ADCCtrl(PIN_ADC_CS,PIN_ADC_ADDR,PIN_ADC_IOCLK,PIN_ADC_DOUT);

    printf("Battery volt: %.2f\n", Adc.readBatteryVoltage());
    // Robot.move(0.5,0.5);
    // sleep(1);
    // Robot.stop();

    // return 0;
    
    /* initialize box controller*/
    XboxController controller("/dev/input/event1");
    if (!controller.isValid())
    {
        std::cerr << "Failed to initialize the Xbox controller." << std::endl;
        return 1;
    }

    XB_Event xboxEvent;
    double speedLeft, speedRight = 0.0;

    gpioSetTimerFunc(2,200,timer_callback);
    bool change = false;
    // const int xboxInMax = 1.5;
    while (true)
    {
        if (controller.readEvent(xboxEvent))
        {
            // Handle the Xbox event here
            if ((xboxEvent.type == XB_EV_GAS) || (xboxEvent.type == XB_EV_X_AXSIS) || (xboxEvent.type == XB_EV_BREAK))
            {
                recalcSpeed(&speedLeft, &speedRight, &xboxEvent);
            }
            else if (xboxEvent.type == XB_EV_BURGER)
            {
                std::cout << "Burger event! " << std::endl;
                break;
            }
            change = true;
            // std::cout << "CHANGE" << std::endl;
        }
        // std::cout << "U: " << update << " | C: " <<  change << std::endl; 
        if(update && change)
        {
            update = false;
            change = false;
            std::cout << "L: " << speedLeft << " | R: " <<  speedRight << std::endl; 
            Robot.move(speedLeft,speedRight);
        }
    }

    Robot.stop();
    double kp,ki,kd = 0.0;
    Robot.motorLeft.pid.GetTunings(&kp,&ki,&kd);
    std::cout << "TUN_L: KP=" << kp << " | KI=" <<  ki << " | KD=" <<  kd << std::endl; 
    Robot.motorRight.pid.GetTunings(&kp,&ki,&kd);
    std::cout << "TUN_R: KP=" << kp << " | KI=" <<  ki << " | KD=" <<  kd << std::endl; 

    
    Robot.motorLeft.saveHistoryToCSV("motorLeftPid.csv");
    Robot.motorRight.saveHistoryToCSV("motorRightPid.csv");

    Robot.kill();
    gpioTerminate();
    return 0;
}