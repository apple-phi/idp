#include "./control.h"
namespace Control
{
    PID::PID(float kc, float Ti, float Td, FirstOrderFilter derivativeFilter) : kc(kc), Ti(Ti), Td(Td), derivativeFilter(derivativeFilter)
    {
        reset();
    }

    // https://controlguru.com/using-signal-filters-in-our-pid-loop/
    float PID::update(float error, float dt)
    {
        integral += error * dt;
        float derivative = derivativeFilter.filter(error - prevError) / dt;
        prevError = error;
        return kc * (error + integral / Ti + Td * derivative);
    }
    void PID::reset()
    {
        prevError = 0;
        integral = 0;
        derivativeFilter.reset();
    }

    FirstOrderFilter::FirstOrderFilter(const float alpha) : alpha(alpha)
    {
        reset();
    }
    float FirstOrderFilter::filter(float input)
    {
        output = alpha * input + (1 - alpha) * output;
        return output;
    }
    void FirstOrderFilter::reset()
    {
        input = output = 0;
    }
}