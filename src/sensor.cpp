#include <Arduino.h>
#include <ArxContainer.h>
#include "./sensor.h"
#include "sensor.h"

namespace Sensors
{
    arx::vector<PinStatus> digitalReadAll(arx::vector<pin_size_t> sensors)
    {
        arx::vector<PinStatus> readings;
        for (auto &s : sensors)
        {
            // cppcheck-suppress useStlAlgorithm
            readings.push_back(::digitalRead(s));
        }
        return readings;
    }

    /* Encode the readings in the form of 4 bits,
    where each bit represents a sensor,
    ordered from left to right.*/
    uint8_t encodeLineSensorReadings(arx::vector<PinStatus> readings)
    {
        int e = 0;
        for (const auto &r : readings)
        {
            e <<= 1;
            e += (r == HIGH);
        }
        return e;
    }

    Button::Button(pin_size_t pin) : pin(pin)
    {
        pinMode(pin, INPUT);
    }
    bool Button::pressed()
    {
        return digitalRead(pin) == HIGH;
    }

    float UltraSonicTheHedgehog::gradient()
    {
        value = analogRead(pin);
        float g = value - prev_value;
        prev_value = value;
        return g;
    }
}
