#include <stdio.h>
#include <math.h>

#include <pigpio.h>
#include <unistd.h>

#include "pid.hpp"
/* g++ -Wall -pthread -o foobar ../main.cpp ../pid.cpp -lpigpio -lrt -lc */

unsigned readADC(unsigned channel);

// TLC1543
static const int GPIO_ADC_IOCLK = 25;
static const int GPIO_ADC_ADDR = 24;
static const int GPIO_ADC_DOUT = 23;
static const int GPIO_ADC_CS = 5;
static const int ADC_VIN = 10;

bool running = false;
int timerNumber = 0;
long samplingInterval = 0;
volatile int counter = 0;

static const int GPIO_MOTORL = 6;
static const int GPIO_MOTORR = 26;
static const int GPIO_MOTOR_IN1 = 12; /* left motor */
static const int GPIO_MOTOR_IN2 = 13; /* left motor */
static const int GPIO_MOTOR_IN3 = 20; /* right motor */
static const int GPIO_MOTOR_IN4 = 21; /* right motor */

static const int GPIO_ENC_R = 8; /* CE0 - encoder right wheel */
static const int GPIO_ENC_L = 7; /* CE1 - encoder left wheel */

static const double wheel_diam = 65.0f; /* diameter in mm */
static const int samplingRate = 150; /* encoder + pid sampling rate*/
static const double samplingRateS = (double)samplingRate/1000.0; /* encoder + pid sampling rate*/

static void pigpioTimerCallback(void* arg) {
    // printf("%s %d\n",(char*)arg, counter++);
}

volatile int counter_L = 0;
volatile int counter_R = 0;
volatile bool calcFlag = false;

PID pidLeft;
PID pidRight;

const int historySize = 100;

double pidHistoryL[historySize] = {0.0};
double pidInputL[historySize] = {0.0};
double pidErrorL[historySize] = {0.0};
double pidHistoryR[historySize] = {0.0};
double pidInputR[historySize] = {0.0};
double pidErrorR[historySize] = {0.0};
int pidIdx = 0;

static void pigpioTimerCallback_speed(void) {
    // printf("%s %d\n",(char*)arg, counter++);
    // float rps_l = ((float)counter_L/20.0f);
    // float speed_l = (wheel_diam * 3.14 * 2 * rps_l)/1000; 
    // counter_L = 0;

    // float rps_r = ((float)counter_R/20.0f);
    // float speed_r = (wheel_diam * 3.14 * 2 * rps_r)/1000; 
    // counter_R = 0;

    // printf("LEFT  | RPS: %.3f SPEED: %.3f [m/s]\n",rps_l,speed_l);
    // printf("RIGHT | RPS: %.3f SPEED: %.3f [m/s]\n",rps_r,speed_r);

    calcFlag = true;

}

typedef struct{
    uint32_t min;
    uint32_t max;
    float avg;
    int cnt;
    uint32_t last;
    uint32_t history[20];
    uint8_t histIdx;
}T_encoder_stats;

static T_encoder_stats rightEncStats = {0xffffffff,0,0.0f,0,0,{0},0};
static T_encoder_stats leftEncStats = {0xffffffff,0,0.0f,0,0,{0},0};


void calcStats(T_encoder_stats* stats, uint32_t tick)
{
    stats->cnt++;
    uint32_t diff = tick - stats->last;
    stats->max = (diff > stats->max) ? diff : stats->max;
    stats->min = (diff < stats->min) ? diff : stats->min;
    if(stats->histIdx >= 20)
    {
        stats->histIdx = 0;
    }
    stats->history[stats->histIdx++] = diff;

    stats->avg = 0;
    for(int i = 0; i < 20; i++)
    {
        stats->avg += stats->history[i];
    }
    float div = (stats->cnt < 20) ? (float)stats->cnt : 20.0f;
    stats->avg /= div;
}

void printStats(T_encoder_stats* stats)
{
    printf("CNT: %d, MIN: %zu, MAX: %zu, AVG: %f\n",stats->cnt, stats->min, stats->max, stats->avg);
    printf("DELTA: %zu, ERR: %f",stats->max - stats->min, (float)(stats->max - stats->min)/stats->avg);
    printf("Last 20: {");
    for(int i = 0; i < 20; i++)
    {
        printf("%zu ,",stats->history[i]);
    }
    printf("}\n");

}

void pulseEnc(int gpio, int level, uint32_t tick)
{
    if(level == 0)
    {
        if(gpio == GPIO_ENC_L)
        {
            counter_L++;
            calcStats(&leftEncStats,tick);
        }
        else if(gpio == GPIO_ENC_R)
        {
            counter_R++;
            calcStats(&rightEncStats,tick);
        }
        else
        {
            /* do nothing */
        }
    }
}

// double setpoints[5] = {0.5,0.7,1.0,1.5,2.0};
double setpoints[1] = {0.55};

int main() {
    printf("Hello world\n");
    if (gpioInitialise() < 0){
        printf("Fail gpio\n");
        gpioTerminate();
        return 1;
    } 
    printf("Gpio OK\n");

    /* setup motor driver pins*/
    gpioSetMode(GPIO_MOTOR_IN1,PI_OUTPUT);
    gpioSetMode(GPIO_MOTOR_IN2,PI_OUTPUT);
    gpioSetMode(GPIO_MOTOR_IN3,PI_OUTPUT);
    gpioSetMode(GPIO_MOTOR_IN4,PI_OUTPUT);
    /* direction forward both */
    gpioWrite(GPIO_MOTOR_IN1,1);
    gpioWrite(GPIO_MOTOR_IN2,0);
    gpioWrite(GPIO_MOTOR_IN3,0);
    gpioWrite(GPIO_MOTOR_IN4,1);

    // gpioWrite(GPIO_MOTOR_IN1,0);
    // gpioWrite(GPIO_MOTOR_IN2,1);
    // gpioWrite(GPIO_MOTOR_IN3,1);
    // gpioWrite(GPIO_MOTOR_IN4,0);
    // int gpioPwm = GPIO_MOTORL;
    /* set PWM frequency*/
    gpioSetPWMfrequency(GPIO_MOTORL,1000);
    gpioSetPWMfrequency(GPIO_MOTORR,1000);
    // int rr = gpioGetPWMrealRange(gpioPwm);
    // if ( ( rr > 255) && (rr < 20000) ) gpioSetPWMrange(gpioPwm, rr);

    /* setup encoders input */
    gpioSetMode(GPIO_ENC_L, PI_INPUT);
    gpioSetMode(GPIO_ENC_R, PI_INPUT);

    /* pull up is needed as encoder common is grounded */

    gpioSetPullUpDown(GPIO_ENC_L, PI_PUD_UP);
    gpioSetPullUpDown(GPIO_ENC_R, PI_PUD_UP);

    /* monitor encoder level changes */

    gpioSetAlertFunc(GPIO_ENC_L, pulseEnc);
    gpioSetAlertFunc(GPIO_ENC_R, pulseEnc);

    /* setup PID */
    pidLeft.SetTunings(34.0,28.0,0.5); /* 15.0,13.0,0.0 */
    pidLeft.SetMode(AUTOMATIC);
    pidLeft.SetOutputLimits(50.0,255.0);
    pidLeft.SetSetpoint(1.5);
    pidLeft.SetSampleTime(samplingRate);
    
    pidRight.SetTunings(34.0,28.0,0.5);
    pidRight.SetMode(AUTOMATIC);
    pidRight.SetOutputLimits(50.0,255.0);
    pidRight.SetSetpoint(1.5);
    pidRight.SetSampleTime(samplingRate);
    double Kp,Ki,Kd;
    /* enable timer 1s*/
    gpioSetTimerFunc(1,samplingRate,pigpioTimerCallback_speed);

    for(int i=0; i < (int)(sizeof(setpoints)/sizeof(double)); i++)
    {
        printf("Setpoint: %.2f\n",setpoints[i]);
        pidLeft.SetSetpoint(setpoints[i]);
        pidRight.SetSetpoint(setpoints[i]);

        int timeoutCnt = 0;
        printf("Start motors...\n");
        while(1)
        {
            if(calcFlag)
            {
                calcFlag = false;
                timeoutCnt++;

                double rps_l = ((double)counter_L/20.0)/(samplingRateS);
                double speed_l = (wheel_diam * 3.14 * 2 * rps_l)/1000.0; 
                counter_L = 0;

                double rps_r = ((double)counter_R/20.0)/(samplingRateS);
                double speed_r = (wheel_diam * 3.14 * 2 * rps_r)/1000.0; 
                counter_R = 0;

                printf("LEFT  | RPS: %.3f SPEED: %.3f [m/s]\n",rps_l,speed_l);
                printf("RIGHT | RPS: %.3f SPEED: %.3f [m/s]\n",rps_r,speed_r);
                double out_l = pidLeft.Compute(speed_l);
                double out_r = pidRight.Compute(speed_r);

                printf("PID: L | %.3f, R | %.3f\n",out_l, out_r);
                gpioPWM(GPIO_MOTORL,out_l);
                gpioPWM(GPIO_MOTORR,out_r);

                if(pidIdx < historySize)
                {
                    pidHistoryL[pidIdx] = out_l;
                    pidInputL[pidIdx] = speed_l;
                    pidErrorL[pidIdx] = speed_l - setpoints[i];
                    pidHistoryR[pidIdx] = out_r;
                    pidErrorR[pidIdx] = speed_r - setpoints[i];
                    pidInputR[pidIdx] = speed_r;
                    pidIdx++;
                }
                // if(timeoutCnt == 6)
                // {
                //     pidLeft.GetTunings(&Kp, &Ki, &Kd);
                //     printf("Tunings L before: Kp: %.2f | Ki: %.2f | Kd: %.2f \n",Kp,Ki,Kd);
                //     pidRight.GetTunings(&Kp, &Ki, &Kd);
                //     printf("Tunings R before: Kp: %.2f | Ki: %.2f | Kd: %.2f \n",Kp,Ki,Kd);
                //     pidLeft.EnableAutoTune(10.0);  
                //     pidRight.EnableAutoTune(10.0);  
                // }
                // pidLeft.AutoTuneStep();
                // pidRight.AutoTuneStep();
            }
            if(timeoutCnt*samplingRateS > 4)
            {
                break;
            }
        }
    }
    // printf("Start motors...\n");
    // gpioPWM(GPIO_MOTORL,100);
    // gpioPWM(GPIO_MOTORR,100);
    // sleep(5);
    printf("Stop motors.\n");
    gpioPWM(GPIO_MOTORL,0);
    gpioPWM(GPIO_MOTORR,0);
    // gpioWrite(GPIO_MOTOR_IN1,0);
    // gpioWrite(GPIO_MOTOR_IN2,1);
    // sleep(1);
    gpioWrite(GPIO_MOTOR_IN1,0);
    gpioWrite(GPIO_MOTOR_IN2,0);
    gpioWrite(GPIO_MOTOR_IN3,0);
    gpioWrite(GPIO_MOTOR_IN4,0);

    printf("Encoder right stats:\n");
    printStats(&rightEncStats);
    printf("Encoder left stats:\n");
    printStats(&leftEncStats);
    /* cancel timer */
    gpioSetTimerFunc(1,1000,NULL);

    
    pidLeft.GetTunings(&Kp, &Ki, &Kd);
    printf("Tunings L after: Kp: %.2f | Ki: %.2f | Kd: %.2f \n",Kp,Ki,Kd);
    pidRight.GetTunings(&Kp, &Ki, &Kd);
    printf("Tunings R after: Kp: %.2f | Ki: %.2f | Kd: %.2f \n",Kp,Ki,Kd);

    // for(int i = 0; i < pidIdx; i++)
    // {
    //     printf
    // }
    gpioTerminate();
    return 0;
    // ADC
    gpioSetMode(GPIO_ADC_CS,PI_OUTPUT);
    gpioWrite(GPIO_ADC_CS,1);
    gpioSetMode(GPIO_ADC_ADDR,PI_OUTPUT);
    gpioSetMode(GPIO_ADC_IOCLK,PI_OUTPUT);
    gpioSetMode(GPIO_ADC_DOUT,PI_INPUT);

    samplingInterval = 100;
	timerNumber = 0;
    char str[] = "test ";
	int r = gpioSetTimerFuncEx(timerNumber,samplingInterval,pigpioTimerCallback,(void*)str);
	if (r < 0) {
		throw "Cannot start timer.";
	}
	running = true;

    // battery
    while(1)
    {
        unsigned adcVal = readADC(ADC_VIN);

        printf("ADC Raw: %d\n",adcVal);
        float adcVolt = (float)adcVal*5 / 1024;
        printf("ADC Volt: %f\n",adcVolt);
        printf("Battery Volt: %f\n",(adcVolt*2) + 0.353); /* 0.353V is diode voltage*/
        sleep(3);
    }
    return 0;
}

unsigned readADC(unsigned channel) {
	if (!running) return 0;
	unsigned us = 1;
        gpioWrite(GPIO_ADC_CS,0);
	usleep(us);
	channel = channel << 4;
	for (unsigned i = 0; i < 4; i ++) {
		if(channel & 0x80)
			gpioWrite(GPIO_ADC_ADDR,1);
		else 
			gpioWrite(GPIO_ADC_ADDR,0);
		usleep(us);
		gpioWrite(GPIO_ADC_IOCLK,1);
		usleep(us);
		gpioWrite(GPIO_ADC_IOCLK,0);
		usleep(us);
		channel = channel << 1;
	}
	for (unsigned i = 0; i < 6;i ++) {
		usleep(us);
		gpioWrite(GPIO_ADC_IOCLK,1);
		usleep(us);
		gpioWrite(GPIO_ADC_IOCLK,0);
	}

        gpioWrite(GPIO_ADC_CS,1);
	usleep(us);
	gpioWrite(GPIO_ADC_CS,0);
	
        unsigned value = 0; 
	for (unsigned i = 0; i < 2; i ++) {
		gpioWrite(GPIO_ADC_IOCLK,1);
		usleep(us);
		value <<= 1;
		usleep(us);
		if (gpioRead(GPIO_ADC_DOUT))
			value |= 0x1;
		gpioWrite(GPIO_ADC_IOCLK,0);
		usleep(us);
	} 

	for (unsigned i = 0; i < 8; i ++) {
		gpioWrite(GPIO_ADC_IOCLK,1);
		usleep(us);
		value <<= 1;
		if (gpioRead(GPIO_ADC_DOUT))
			value |= 0x1;
		gpioWrite(GPIO_ADC_IOCLK,0);
		usleep(us);
	}
	usleep(us);
        gpioWrite(GPIO_ADC_CS,1);

	return value; 
}
