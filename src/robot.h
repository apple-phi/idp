#pragma once
#include <ArxContainer.h>
#include <Adafruit_MotorShield.h>
#include "./motors.h"
#include "./direction_matrix.h"
class Robot
{
public:
    int currentDirection = Direction::N;
    int latestNode = 1;
    int targetNode = 17;
    int maxSpeed = 255;
    Motors::MotorPair motors;
    arx::vector<pin_size_t> line_sensors;
    Robot(Motors::MotorPair motors, arx::vector<pin_size_t> line_sensors);
    Robot &readSensors();
    Robot &assignAngleError();
    Robot &drive();
    Robot &steeringCorrection();
    Robot &junctionDecision(uint8_t sensorCode);

private:
    float lastJunctionSeenAt = -1;
    uint8_t encodedLineSensorReading = 0b0110;
    float angleError = 0; // in degrees, positive is too far right, negative is too far left
    enum
    {
        FOLLOW,
        LEFT_TURN,
        RIGHT_TURN
    } drivingMode = FOLLOW;
};
