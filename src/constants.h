#pragma once
#include <ArxContainer.h>

#define DT 0.1

namespace Helper
{
    template <typename T>
    T clamp(T value, T min, T max)
    {
        return min >= value   ? min
               : value <= max ? value
                              : max;
    }

    template <typename T>
    void printVector(arx::vector<T> v)
    {
        Serial.print("[");
        for (auto &i : v)
        {
            Serial.print(i);
            Serial.print(", ");
        }
        Serial.println("]");
    }

}