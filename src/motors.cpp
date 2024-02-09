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
        left->run(leftSpeed >= 0 ? Motors::forward : Motors::backward);
        right->run(rightSpeed >= 0 ? Motors::forward : Motors::backward);
        absLeftSpeed = abs(leftSpeed);
        absRightSpeed = abs(rightSpeed);
        return *this;
    }
    MotorPair &MotorPair::setSpeed(uint8_t speed)
    {
        setSpeedsAndRun(speed, speed);
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
        absLeftSpeed = absRightSpeed = 0;
        run(RELEASE);
        return *this;
    }
    bool MotorPair::isMoving()
    {
        return (absLeftSpeed > 0 || absRightSpeed > 0);
    }

    Servos::Servos()
    {
        arm.attach(9);
        claw.attach(8);
        delay(300);
    }
    Servos &Servos::setClaw(int angle)
    {
        angle += gearError;
        int initial = claw.read();
        bool isIncreasing = angle > initial;
        while (initial != angle)
        {
            if (isIncreasing)
            {
                initial++;
            }
            else
            {
                initial--;
            }
            claw.write(initial);
            delay(30);
        }
        return *this;
    }
    Servos &Servos::setArm(int angle)
    {
        int initial = arm.read();
        bool isIncreasing = angle > initial;
        while (initial != angle)
        {
            if (isIncreasing)
            {
                initial++;
            }
            else
            {
                initial--;
            }
            arm.write(initial);
            delay(30);
        }
        return *this;
    }
    Servos &Servos::reset()
    {
        halfOpenOrHalfCloseClaw();
        raiseArm();
        return *this;
    }
    Servos &Servos::openClaw()
    {
        setClaw(0 + gearError);
        return *this;
    }
    Servos &Servos::closeClaw()
    {
        setClaw(65 + gearError);
        return *this;
    }
    Servos &Servos::raiseArm()
    {
        setArm(105);
        return *this;
    }
    Servos &Servos::lowerArm()
    {
        setArm(5);
        return *this;
    }
    Servos &Servos::halfOpenOrHalfCloseClaw()
    {
        setClaw(30 + gearError);
        return *this;
    }
}
