#pragma once
#include <Adafruit_MotorShield.h>
#include <Servo.h>

namespace Motors
{
    struct MotorPair
    {
        Adafruit_DCMotor *left;
        Adafruit_DCMotor *right;
        MotorPair(Adafruit_DCMotor *left, Adafruit_DCMotor *right);
        MotorPair &setSpeedsAndRun(int leftSpeed, int rightSpeed);
        MotorPair &setSpeed(uint8_t speed);
        MotorPair &run(uint8_t direction);
        MotorPair &stop();
    };

    struct Servos
    {
        Servo arm;
        Servo claw;
        Servos();
        Servos &setClaw(int angle);
        Servos &setArm(int angle);
        Servos &reset();
    };
}
