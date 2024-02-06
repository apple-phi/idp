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
        armAngle = arm.read();
        clawAngle = claw.read();
    }
    Servos &Servos::setClaw(int angle)
    {
        bool isIncreasing = angle > clawAngle;
        while (clawAngle != angle)
        {
            if (isIncreasing)
            {
                clawAngle++;
            }
            else
            {
                clawAngle--;
            }
            claw.write(clawAngle);
            delay(30);
        }
        return *this;
    }
    Servos &Servos::setArm(int angle)
    {
        bool isIncreasing = angle > armAngle;
        while (armAngle != angle)
        {
            if (isIncreasing)
            {
                armAngle++;
            }
            else
            {
                armAngle--;
            }
            arm.write(armAngle);
            delay(30);
        }
        return *this;
    }
    Servos &Servos::reset()
    {
        setClaw(20);
        setArm(110);
        return *this;
    }
}
