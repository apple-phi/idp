#pragma once
#include <ArxContainer.h>
#include <Adafruit_MotorShield.h>
#include "./motors.h"
class Robot
{
public:
    int maxSpeed = 100;
    Motors::MotorPair motors;
    arx::vector<pin_size_t> line_sensors;
    Robot(Motors::MotorPair motors, arx::vector<pin_size_t> line_sensors);
    Robot &readSensors();
    Robot &assignAngleError();
    Robot &drive();
    Robot &steeringCorrection();
    Robot &junctionDecision(uint8_t sensorCode);

private:
    uint8_t encodedLineSensorReading = 0b0110;
    float angleError = 0; // in degrees, positive is too far right, negative is too far left
    enum
    {
        FOLLOW,
        TURN
    } drivingMode = FOLLOW;
};
