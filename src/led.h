#pragma once

class LED
{
public:
    pin_size_t pin;
    bool state = false;
    explicit LED(pin_size_t pin);
    void on();
    void off();
    void toggle();
};