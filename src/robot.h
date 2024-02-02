#pragma once
#include <ArxContainer.h>
#include <Adafruit_MotorShield.h>
#include "./motors.h"
#include "./direction_matrix.h"
#include "./control.h"

class Robot
{
public:
    int currentDirection = Direction::N;
    int latestNode = 6; // Want the robot to start at node 1
    int targetNode = 14;
    int maxSpeed = 255;

    Control::PID lineFollowPID = Control::PID(0.02, 1.1, 0);
    float angleError = 0; // in degrees, positive is too far right, negative is too far left

    Motors::MotorPair wheelMotors;
    arx::vector<pin_size_t> line_sensors;
    uint8_t encodedLineSensorReading = 0b0110;
    enum
    {
        FOLLOW,
        LEFT_TURN,
        RIGHT_TURN
    } drivingMode = FOLLOW;
    enum
    {
        FETCH,
        GRAB,
        DELIVER
    } deliveryTask = FETCH;
    Robot(Motors::MotorPair wheelMotors, arx::vector<pin_size_t> line_sensors);
    Robot &readSensors();
    Robot &drive();
    Robot &junctionDecision();

private:
    float lastJunctionSeenAt = -1;
    int blockNodes[4] = {5, 11, 14, 17};
    uint8_t blockNodeIndex = 0;
};
