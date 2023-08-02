#include <stdio.h>
#include <math.h>

#include <pigpio.h>
#include <unistd.h>

#include "robot_move.hpp"
#include "adc.hpp"
#include "alphabot_cfg.h"
/* g++ -Wall -pthread -o foobar ../main.cpp ../pid.cpp ../acd.cpp ../motor_ctrl.cpp ../robot_move.cpp -lpigpio -lrt -lc -g */

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

    printf("Battery volt: %.2f\n", Adc.readBatteryVoltage());
    Robot.move(0.8,0.8);
    // Robot.moveDirect(80,80);
    sleep(1);
    Robot.stop();

    printf("Motor left\n");
    Robot.motorLeft.printHistory(10);
    printf("Motor right\n");
    Robot.motorRight.printHistory(10);

    Robot.kill();
    gpioTerminate();
    return 0;
}