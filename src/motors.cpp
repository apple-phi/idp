#include "motors.h"

namespace Motors
{
    MotorPair::MotorPair(Adafruit_DCMotor *left, Adafruit_DCMotor *right)
    {
        this->left = left;
        this->right = right;
    }
    /* This also handles negative speeds, but not speeds out of the maximum...
    It handles motor running already */
    MotorPair &MotorPair::setSpeedsAndRun(int leftSpeed, int rightSpeed)
    {
        left->setSpeed(abs(leftSpeed));
        right->setSpeed(abs(rightSpeed));
        left->run(leftSpeed >= 0 ? BACKWARD : FORWARD);
        right->run(rightSpeed >= 0 ? BACKWARD : FORWARD);
        return *this;
    }
    MotorPair &MotorPair::setSpeed(uint8_t speed)
    {
        left->setSpeed(speed);
        right->setSpeed(speed);
        return *this;
    }
    MotorPair &MotorPair::run(uint8_t direction)
    {
        left->run(direction);
        right->run(direction);
        return *this;
    }
    MotorPair &MotorPair::stop()
    {
        run(RELEASE);
        return *this;
    }
}
