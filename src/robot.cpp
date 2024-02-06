#include "./robot.h"
#include "./sensor.h"
#include "./constants.h"
#include "./motors.h"
#include "./direction_matrix.h"
#include "./steering.h"

// // cppcheck-suppress passedByValue
Robot::Robot(Motors::MotorPair motors_, arx::vector<pin_size_t> line_sensors_) : wheelMotors(motors_), line_sensors(line_sensors_)
{
    wheelMotors
        .setSpeed(maxSpeed)
        .run(BACKWARD)
        .stop();
    for (auto &s : line_sensors)
    {
        pinMode(s, INPUT);
    }
}

Robot &Robot::readSensors()
{
    // Encode the readings in the form of 4 bits,
    // where each bit represents a sensor,
    // ordered from left to right.
    auto readings = Sensors::digitalReadAll(line_sensors);
    encodedLineSensorReading = Sensors::encodeLineSensorReadings(readings);
    // Serial.print("Line sensor readings: ");
    // Helper::printVector(readings);
    return *this;
}

Robot &Robot::drive()
{
    
    switch (deliveryTask)
    {
    case FETCH:
        if (latestNode == blockNodes[blockNodeIndex] && currentDirection == targetDirection)
        {
            deliveryTask = ENTER_ZONE;
            return *this;
        }
        
        
        if (drivingMode == FOLLOW)
        {
            Steering::assignAngleError(*this);
        }
        switch (drivingMode)
        {
        case FOLLOW:
            Steering::correctSteering(*this);
            // -1 gives the node right before due to node ordering
            switch (blockNodeIndex)
            {
            case 0:
            case 1:
                if (Direction::isNodeBeforeTarget(latestNode, currentDirection, targetNode))
                {
                    deliveryTask = ENTER_ZONE;
                }
                break;
            case 2:
            case 3:
                if (Direction::isNodeBeforeNodeBeforeTarget(latestNode, currentDirection, targetNode))
                {
                    deliveryTask = ENTER_ZONE;
                }
                break;
            }

            break;
        case LEFT_TURN:
            wheelMotors.setSpeedsAndRun(0, maxSpeed);
            if ((encodedLineSensorReading == 0b0010 || encodedLineSensorReading == 0b0011) && millis() - lastJunctionSeenAt > 500)
            {
                drivingMode = FOLLOW;
                lastJunctionSeenAt = millis();
                lineFollowPID.reset();
                currentDirection = targetDirection;
            }
            break;
        case RIGHT_TURN:
            wheelMotors.setSpeedsAndRun(maxSpeed, 0);
            if ((encodedLineSensorReading == 0b0100 || encodedLineSensorReading == 0b1100) && millis() - lastJunctionSeenAt > 500)
            {
                drivingMode = FOLLOW;
                lastJunctionSeenAt = millis();
                lineFollowPID.reset();
                currentDirection = targetDirection;
            }
            break;
        }
        break;
    case ENTER_ZONE:
        switch (blockNodeIndex)
        {
        // First two blocks
        // ENTER_ZONE starts from the turn into the zone
        case 0:
            servos.setClaw(0);
            servos.setArm(0);
            wheelMotors.stop();
            delay(50000);
        case 1:
            Steering::assignAngleError(*this);
            if (angleError > 10)
            {
                Steering::correctSteering(*this);
                break;
            }
            deliveryTask = GRAB;
            break;

        // Last two blocks
        // ENTER_ZONE starts from the node before the turn into the zone
        case 2:
            // TODO: You are at node 9 (or 15)
            // Go forwards a bit then turn
            // Then change deliveryTask to GRAB
            break;
        case 3:
            // TODO: You are at node 9 (or 15)
            // Go forwards a bit then turn
            break;
        }
        break;
    case GRAB:
        // TODO: locate block and pick it up
        // Maybe add a sub-state for grab_task being an enum of {SEARCH, APPROACH, GRAB} ?
        // Then change deliveryTask to EXIT_ZONE
        break;
    case EXIT_ZONE:
        // TODO: reverse out of the zone
        // Set the current node and direction appropriately so the robot knows where it is
        // Then change deliveryTask to DELIVER
        switch (blockNodeIndex)
        {
        case 0:
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        }
        break;
    case DELIVER:
        // TODO: change target node to the correct delivery zone
        // Then line follow and navigate to the delivery zone
        // Might need to abstract out the above FETCH case so that the code can be reused.
        // Might need an extra state for DROP_OFF
        // Then increase blockNodeIndex and set the new targetNode
        // Then change deliveryTask to FETCH
        break;
    }
    return *this;
}

Robot &Robot::junctionDecision()
{   
    if (millis() - lastJunctionSeenAt < 500)
    {
        drivingMode = FOLLOW;
        return *this;
    }
    lastJunctionSeenAt = millis();

    latestNode = Direction::nextNode(latestNode, currentDirection);
    targetDirection = Direction::nextDir(latestNode, targetNode);

    Serial.print("Reached: ");
    Serial.println(latestNode);

    // G: move forward a bit, probably be better to move until the sensors have left the junction

    if (targetDirection == currentDirection)
    {
        drivingMode = FOLLOW;
        return *this;
    }

    if (Direction::isRightTurn(currentDirection, targetDirection))
    {
        drivingMode = RIGHT_TURN;
        Serial.println("Right turn");
    }
    else if (Direction::isLeftTurn(currentDirection, targetDirection))
    {
        drivingMode = LEFT_TURN;
        Serial.println("180");
    }
    else
    {
        drivingMode = FOLLOW;
        targetDirection = currentDirection;
        Serial.println("Continue straight");
    }

    return *this;
}