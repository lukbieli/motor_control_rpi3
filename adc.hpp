#ifndef B590DFC9_6CB9_4E34_8B16_FFB8E65C9027
#define B590DFC9_6CB9_4E34_8B16_FFB8E65C9027

class ADCCtrl{
    public:
        ADCCtrl(const int pin_cs, const int pin_addr, const int pin_ioclk, const int pin_dout);

        double readBatteryVoltage(void);
        unsigned int checkProximityL(void);
        unsigned int checkProximityR(void);

    private:
        int PIN_CS;
        int PIN_ADDR;
        int PIN_IOCLK;
        int PIN_DOUT;

        const int ADC_VIN = 10;
        const int ADC_OBS_L = 8;
        const int ADC_OBS_R = 9;

        unsigned int readADC(unsigned int channel);
};

#endif /* B590DFC9_6CB9_4E34_8B16_FFB8E65C9027 */
