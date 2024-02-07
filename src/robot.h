#pragma once
#include <ArxContainer.h>
#include <Adafruit_MotorShield.h>
#include "./motors.h"
#include "./direction_matrix.h"
#include "./control.h"

enum class Block_t
{
    NONE,
    SOLID,
    FOAM
};

class Robot
{
public:
    Block_t currentBlock = Block_t::NONE;
    uint8_t solidBlocksCollected = 0;
    uint8_t foamBlocksCollected = 0;

    int currentDirection = Direction::N;
    int targetDirection = Direction::N;
    int latestNode = 6; // Want the robot to start at node 1
    int targetNode = 5;
    int maxSpeed = 255;

    Control::PID lineFollowPID = Control::PID(0.025, 15, 0);
    float angleError = 0; // in degrees, positive is too far right, negative is too far left

    Motors::MotorPair wheelMotors;
    Motors::Servos servos;
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
        NAVIGATE,
        ENTER_BLOCK_ZONE,
        GRAB,
        EXIT_BLOCK_ZONE,
        ENTER_DROP_ZONE,
        DROP_OFF,
        EXIT_DROP_ZONE
    } deliveryTask = NAVIGATE;
    Robot(Motors::MotorPair wheelMotors, arx::vector<pin_size_t> line_sensors);
    Robot &readSensors();
    Robot &drive();
    Robot &junctionDecision();
    Robot &endTurn();

    void task_navigate();
    void task_enter_block_zone();
    void task_grab();
    void task_exit_block_zone();
    void task_enter_drop_zone();
    void task_drop_off();
    void task_exit_drop_zone();

private:
    float latestJunctionStartedAt = -1;
    float latestJunctionEndedAt = -1;
    int blockNodes[4] = {5, 11, 14, 17};
    uint8_t blockNodeIndex = 0;
};
