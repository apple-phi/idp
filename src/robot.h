#pragma once
#include <ArxContainer.h>
#include <Adafruit_MotorShield.h>
#include "./motors.h"
class Robot
{
public:
    int8_t currentDirection = 0;
    int8_t latestNode = 1;
    int8_t targetNode = 17; // must be 5 for real run
    int8_t targetDirection = 0;
    int8_t stage = 0;
    int maxSpeed = 200;
    Motors::MotorPair motors;
    arx::vector<pin_size_t> line_sensors;
    Robot(Motors::MotorPair motors, arx::vector<pin_size_t> line_sensors);
    Robot &readSensors();
    Robot &assignAngleError();
    Robot &drive();
    Robot &steeringCorrection();
    Robot &junctionDecision(uint8_t sensorCode);
    Robot &junctionTurn(uint8_t encodedLineSensorReadings, bool leftTurn);
    Robot &stageDecision();

private:
    uint8_t encodedLineSensorReading = 0b0110;
    float angleError = 0; // in degrees, positive is too far right, negative is too far left
    enum
    {
        FOLLOW,
        TURN,
        LTURN,
        RTURN
    } drivingMode = FOLLOW;
};
