#pragma once
#define MAX_RANG (520)        // the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0) // ADC accuracy of Arduino UNO is 10bit
#include <Arduino.h>
#include <ArxContainer.h>
namespace Sensors
{
    arx::vector<PinStatus> digitalReadAll(arx::vector<pin_size_t> sensors);
    uint8_t encodeLineSensorReadings(arx::vector<PinStatus> readings);

    class Button
    {
    public:
        pin_size_t pin;
        explicit Button(pin_size_t pin);
        bool pressed();
    };
    class UltraSonicTheHedgehog
    {
        pin_size_t pin;
        float value = 0;
        float prev_value = 0;
        float crit_gradient = 7;
        float max_gradient = 45;
        float gradient();
    };
}