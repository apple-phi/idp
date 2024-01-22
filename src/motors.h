#include <Adafruit_MotorShield.h>

namespace Motors
{
    struct MotorPair
    {
        Adafruit_DCMotor *left;
        Adafruit_DCMotor *right;
        MotorPair(Adafruit_DCMotor *left, Adafruit_DCMotor *right);
        MotorPair &setSpeedsAndRun(int16_t leftSpeed, int16_t rightSpeed);
        MotorPair &setSpeed(uint8_t speed);
        MotorPair &run(uint8_t direction);
        MotorPair &stop();
    };
}
