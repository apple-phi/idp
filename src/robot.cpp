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
        if (drivingMode == FOLLOW)
        {
            Steering::assignAngleError(*this);
        }
        switch (drivingMode)
        {
        case FOLLOW:
            Steering::correctSteering(*this);
            // -1 gives the node right before due to node ordering
            if (latestNode == targetNode - 1)
            {
                deliveryTask = GRAB;
            }
            break;
        case LEFT_TURN:
            wheelMotors.setSpeedsAndRun(0, maxSpeed);
            if ((encodedLineSensorReading == 0b0100 || encodedLineSensorReading == 0b1100) && millis() - lastJunctionSeenAt > 500)
            {
                drivingMode = FOLLOW;
                lastJunctionSeenAt = millis();
                lineFollowPID.reset();
            }
            break;
        case RIGHT_TURN:
            wheelMotors.setSpeedsAndRun(maxSpeed, 0);
            if ((encodedLineSensorReading == 0b0010 || encodedLineSensorReading == 0b0011) && millis() - lastJunctionSeenAt > 500)
            {
                drivingMode = FOLLOW;
                lastJunctionSeenAt = millis();
                lineFollowPID.reset();
            }
            break;
        }
        break;
    case GRAB:
        // wheelMotors.setSpeedsAndRun(-255, 255);
        wheelMotors.stop();
        delay(1000);
        deliveryTask = FETCH; // TODO: this is a placeholder
        targetNode = (targetNode + 23) / 7;
        break;
    case DELIVER:
        break;
    }
    return *this;
}

Robot &Robot::junctionDecision()
{
    if (millis() - lastJunctionSeenAt < 700)
    {
        drivingMode = FOLLOW;
        return *this;
    }
    lastJunctionSeenAt = millis();

    latestNode = Direction::nav_matrix[latestNode][currentDirection];
    int targetDirection = Direction::dir_matrix[latestNode][targetNode];

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

    currentDirection = targetDirection;
    return *this;
}