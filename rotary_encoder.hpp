#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP

#include <stdint.h>

typedef void (*encoderCB_t)(int,void*);

class encoder
{
    private:
        int mygpioA, mygpioB, levA, levB, lastGpio;

        encoderCB_t mycallback;
        void* userData;

        void _pulse(int gpio, int level, uint32_t tick);

        /* Need a static callback to link with C. */
        static void _pulseEx(int gpio, int level, uint32_t tick, void *user);


   public:

        encoder(int gpioA, int gpioB, encoderCB_t callback, void* user);
        /*
            This function establishes a rotary encoder on gpioA and gpioB.

            When the encoder is turned the callback function is called.
        */

        void re_cancel(void);
        /*
            This function releases the resources used by the decoder.
        */
};

#endif
