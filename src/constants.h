#pragma once
#include <ArxContainer.h>

#define DT 0.005       // 10ms
#define TURN_DELAY 650 // ms

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

    template <typename T>
    void printPair(arx::pair<T, T> p)
    {
        Serial.print("(");
        Serial.print(p.first);
        Serial.print(", ");
        Serial.print(p.second);
        Serial.println(")");
    }

}