
#include <Wire.h>
#include "SparkFun_APDS9960.h"
#include <Arduino.h>
#include <mbed.h>
#include <nrf_nvic.h>
#include <nrf_soc.h>
// Pins

class ProximitySleep
{
    private:
        static SparkFun_APDS9960 apds;
        static int isr_flag;

    public:
        static void sleep(const uint8_t lowThreshold, const uint8_t highThreshold);

        static void interruptRoutine();
};