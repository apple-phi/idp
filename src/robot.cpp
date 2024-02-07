#include "./robot.h"
#include "./sensor.h"
#include "./constants.h"
#include "./motors.h"
#include "./direction_matrix.h"
#include "./steering.h"
#include "./led.h"

#define MAX_RANG (520)        // the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0) // ADC accuracy of Arduino UNO is 10bit

// cppcheck-suppress passedByValue
Robot::Robot(Motors::MotorPair motors_, arx::vector<pin_size_t> line_sensors_) : wheelMotors(motors_), line_sensors(line_sensors_)
{
    wheelMotors
        .setSpeed(maxSpeed)
        .run(FORWARD)
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
    case NAVIGATE:
        // if (latestNode == blockNodes[blockNodeIndex] && currentDirection == targetDirection)
        // {
        //     deliveryTask = ENTER_ZONE;
        //     return *this;
        // }

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
                if (Direction::isNodeBeforeTarget(latestNode, currentDirection, targetNode) && abs(angleError) < 20)
                {
                    deliveryTask = ENTER_ZONE;
                }
                break;
            case 2:
            case 3:
                if (Direction::isNodeBeforeNodeBeforeTarget(latestNode, currentDirection, targetNode) && abs(angleError) < 20)
                {
                    deliveryTask = ENTER_ZONE;
                }
                break;
            }

            break;
        case LEFT_TURN:
            if (1 or Direction::isNodeBeforeTarget(latestNode, targetDirection, targetNode))
            {
                if (millis() - latestJunctionStartedAt > 600)
                {
                    wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed);
                    if ((encodedLineSensorReading & 0b0100) == 0b0100 && millis() - latestJunctionStartedAt > 1100)
                    {
                        Serial.println("Special turn left complete");
                        endTurn();
                    }
                }
                else
                {
                    wheelMotors.setSpeedsAndRun(maxSpeed, maxSpeed);
                }
            }
            else
            {
                wheelMotors.setSpeedsAndRun(50, maxSpeed);
                if ((encodedLineSensorReading & 0b0100) == 0b0100 && millis() - latestJunctionStartedAt > 1000)
                {
                    Serial.println("Standard turn left complete");
                    Serial.print("Next node: ");
                    Serial.println(Direction::nextNode(latestNode, targetDirection));
                    Serial.print("(");
                    Serial.print("(currentDirection, targetDirection): ");
                    Serial.print(currentDirection);
                    Serial.print(", ");
                    Serial.print(targetDirection);
                    Serial.println(")");
                    endTurn();
                }
            }
            break;
        case RIGHT_TURN:
            if (1 or Direction::isNodeBeforeTarget(latestNode, targetDirection, targetNode))
            {
                if (millis() - latestJunctionStartedAt > 600)
                {
                    wheelMotors.setSpeedsAndRun(maxSpeed, -maxSpeed);
                    if ((encodedLineSensorReading & 0b0010) == 0b0010 && millis() - latestJunctionStartedAt > 1100)
                    {
                        Serial.println("Special turn right complete");
                        endTurn();
                    }
                }
                else
                {
                    wheelMotors.setSpeedsAndRun(maxSpeed, maxSpeed);
                }
            }
            else
            {
                wheelMotors.setSpeedsAndRun(maxSpeed, 50);
                if ((encodedLineSensorReading & 0b0010) == 0b0010 && millis() - latestJunctionStartedAt > 500)
                {
                    Serial.println("Standard turn right complete");
                    Serial.print("Next node: ");
                    Serial.println(Direction::nextNode(latestNode, targetDirection));
                    Serial.print("(");
                    Serial.print("(currentDirection, targetDirection): ");
                    Serial.print(currentDirection);
                    Serial.print(", ");
                    Serial.print(targetDirection);
                    Serial.println(")");
                    endTurn();
                }
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
        case 1:
            wheelMotors.setSpeedsAndRun(-maxSpeed / 2, -maxSpeed / 2);
            delay(500);
            wheelMotors.stop();
            deliveryTask = GRAB;
            // readSensors();
            // if ((encodedLineSensorReading & 0b0110) == 0b0110)
            // {
            //     wheelMotors.stop();
            //     deliveryTask = GRAB;
            // }
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
        servos.setArm(0);
        // TODO: locate block and pick it up
        // Maybe add a sub-state for grab_task being an enum of {SEARCH, APPROACH, GRAB} ?
        // Then change deliveryTask to EXIT_ZONE

        int sensityPin = A0; // select the input pin
        if (blockNodeIndex == 2 || blockNodeIndex == 3)
        {
            double distances[2] = {0};
            float crit_gradient = 5, max_gradient = 15, min_distance = 20, max_distance = 35; // TODO: tune these values
            int delayTime = 100;                                                              // TODO: tune this value

            float speedCounter = 0;
            float gradient = 0;

            // BLOCK SWEEPING

            while (!((crit_gradient < gradient and gradient < max_gradient) and (distances[1] != 0) and (min_distance < distances[0] and distances[0] < max_distance)))
            {
                // Make the robot oscillate like a cosine wave.
                if (cos(speedCounter) > 0)
                {
                    wheelMotors.setSpeedsAndRun(maxSpeed / 3 + 10, -maxSpeed / 3);
                }
                else
                {
                    wheelMotors.setSpeedsAndRun(-maxSpeed / 3, maxSpeed / 3);
                }

                speedCounter += 0.5;

                // Measure gradient of last two readings
                distances[1] = distances[0];
                distances[0] = abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
                gradient = abs(-distances[0] + distances[1]);

                Serial.print(gradient, 0);
                Serial.print(", ");
                Serial.println(distances[0]);

                delay(delayTime); // delay helps for scaling constants and reading the values for testing purposes
            }
        }

        // BLOCK GRABBING
        float driveToBlockDistance = 5; // TODO: tune this value
        latestJunctionEndedAt = millis();
        maxSpeed = 180;
        while (abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION) > driveToBlockDistance)
        {
            Steering::assignAngleError(*this);
            Steering::correctSteering(*this);
            delay(1000 * DT);
        }
        wheelMotors.stop();
        maxSpeed = 255;
        servos.setClaw(90);

        // BLOCK IDENTIFICATION
        float max_solid = 10, min_solid = 6; // TODO: tune these values
        LED *identificationLED;

        int reading = analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION;
        Serial.print("Identification Reading: ");
        Serial.println(reading);
        if (min_solid <= reading && reading < max_solid)
        {
            identificationLED = new LED(4);
            // TODO: update a boolean variable to indicate that the block is solid
        }
        else
        {
            identificationLED = new LED(3);
            // TODO: update a boolean variable to indicate that the block is foam
        }
        identificationLED->on();
        delay(5500); // must light up for >5s
        identificationLED->off();

        servos.setArm(110); // Raise arm
        // deliveryTask = NAVIGATE;
        // drivingMode = LEFT_TURN;
        // blockNodeIndex++;
        // targetDirection = 2;
        // latestNode = targetNode;
        // targetNode = blockNodes[blockNodeIndex];

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
        // Might need to abstract out the above NAVIGATE case so that the code can be reused.
        // Might need an extra state for DROP_OFF
        // Then increase blockNodeIndex and set the new targetNode
        // Then change deliveryTask to NAVIGATE
        break;
    }
    return *this;
}

Robot &Robot::junctionDecision()
{
    if (millis() - latestJunctionEndedAt < 500)
    {
        drivingMode = FOLLOW;
        return *this;
    }
    latestJunctionStartedAt = millis();

    latestNode = Direction::nextNode(latestNode, currentDirection);
    targetDirection = Direction::nextDir(latestNode, targetNode);

    Serial.print("Reached: ");
    Serial.println(latestNode);

    // // G: move forward a bit, probably be better to move until the sensors have left the junction

    // if (targetDirection == currentDirection)
    // {
    //     drivingMode = FOLLOW;
    //     return *this;
    // }

    if (Direction::isRightTurn(currentDirection, targetDirection))
    {
        drivingMode = RIGHT_TURN;
        Serial.println("Right turn");
    }
    else if (Direction::isLeftTurn(currentDirection, targetDirection))
    {
        drivingMode = LEFT_TURN;
        Serial.println("Left turn");
    }
    else
    {
        drivingMode = FOLLOW;
        currentDirection = targetDirection;
        latestJunctionEndedAt = millis();
        Serial.println("Continue straight");
    }

    return *this;
}

Robot &Robot::endTurn()
{
    drivingMode = FOLLOW;
    latestJunctionEndedAt = millis();
    lineFollowPID.reset();
    currentDirection = targetDirection;
    return *this;
}