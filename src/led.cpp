#include <Arduino.h>
#include "./led.h"

LED::LED(pin_size_t pin_) : pin(pin_)
{
    pinMode(pin, OUTPUT);
    state = false;
}
void LED::on()
{
    digitalWrite(pin, HIGH);
    state = true;
}
void LED::off()
{
    digitalWrite(pin, LOW);
    state = false;
}
void LED::toggle()
{
    state = !state;
    digitalWrite(pin, state ? HIGH : LOW);
}