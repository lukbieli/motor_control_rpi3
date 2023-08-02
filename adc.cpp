
#include <pigpio.h>
#include "adc.hpp"

ADCCtrl::ADCCtrl(const int pin_cs, const int pin_addr, const int pin_ioclk, const int pin_dout)
{
    this->PIN_CS = pin_cs;
    this->PIN_ADDR = pin_addr;
    this->PIN_IOCLK = pin_ioclk;
    this->PIN_DOUT = pin_dout;

    gpioSetMode(this->PIN_CS,PI_OUTPUT);
    gpioWrite(this->PIN_CS,1);
    gpioSetMode(this->PIN_ADDR,PI_OUTPUT);
    gpioSetMode(this->PIN_IOCLK,PI_OUTPUT);
    gpioSetMode(this->PIN_DOUT,PI_INPUT);

}

double ADCCtrl::readBatteryVoltage(void)
{
    unsigned int adcVal = readADC(this->ADC_VIN);

    // printf("ADC Raw: %d\n",adcVal);
    double adcVolt = (float)adcVal*5 / 1024;
    // printf("ADC Volt: %f\n",adcVolt);
    double batteryVolt = (adcVolt*2) + 0.353; /* 0.353V is diode voltage*/
    // printf("Battery Volt: %f\n",batteryVolt; /* 0.353V is diode voltage*/
    return batteryVolt;
}

unsigned ADCCtrl::readADC(unsigned channel) {
	// if (!running) return 0;
	unsigned us = 1;
    gpioWrite(this->PIN_CS,0);
	usleep(us);
	channel = channel << 4;
	for (unsigned i = 0; i < 4; i ++) {
		if(channel & 0x80)
			gpioWrite(this->PIN_ADDR,1);
		else 
			gpioWrite(this->PIN_ADDR,0);
		usleep(us);
		gpioWrite(this->PIN_IOCLK,1);
		usleep(us);
		gpioWrite(this->PIN_IOCLK,0);
		usleep(us);
		channel = channel << 1;
	}
	for (unsigned i = 0; i < 6;i ++) {
		usleep(us);
		gpioWrite(this->PIN_IOCLK,1);
		usleep(us);
		gpioWrite(this->PIN_IOCLK,0);
	}

    gpioWrite(this->PIN_CS,1);
	usleep(us);
	gpioWrite(this->PIN_CS,0);
	
        unsigned value = 0; 
	for (unsigned i = 0; i < 2; i ++) {
		gpioWrite(this->PIN_IOCLK,1);
		usleep(us);
		value <<= 1;
		usleep(us);
		if (gpioRead(this->PIN_DOUT))
			value |= 0x1;
		gpioWrite(this->PIN_IOCLK,0);
		usleep(us);
	} 

	for (unsigned i = 0; i < 8; i ++) {
		gpioWrite(this->PIN_IOCLK,1);
		usleep(us);
		value <<= 1;
		if (gpioRead(this->PIN_DOUT))
			value |= 0x1;
		gpioWrite(this->PIN_IOCLK,0);
		usleep(us);
	}
	usleep(us);
    gpioWrite(this->PIN_CS,1);

	return value; 
}