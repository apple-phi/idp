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
    switchButton = new Sensors::Button(7);
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
    const auto readings = Sensors::digitalReadAll(line_sensors);
    encodedLineSensorReading = Sensors::encodeLineSensorReadings(readings);
    return *this;
}

void Robot::task_navigate()
{
    switch (drivingMode)
    {
    case FOLLOW:
        Steering::assignAngleError(*this);
        Steering::correctSteering(*this);
        // -1 gives the node right before due to node ordering
        if (abs(angleError) < 20)
        {
            if ((
                    currentBlock == Block_t::NONE &&
                    Direction::isNodeBeforeTarget(latestNode, currentDirection, targetNode)))
            {
                deliveryTask = ENTER_BLOCK_ZONE;
                latestJunctionStartedAt = millis();
            }
            else if ((millis() - latestJunctionEndedAt > 300) &&
                     currentBlock != Block_t::NONE &&
                     Direction::isNodeBeforeTarget(latestNode, currentDirection, targetNode))
            {
                deliveryTask = ENTER_DROP_ZONE;
                latestJunctionStartedAt = millis();
            }
            if (blockNodeIndex >= 2 && millis() - latestJunctionEndedAt > 800 && currentBlock == Block_t::NONE && Direction::isNodeBeforeNodeBeforeTarget(latestNode, currentDirection, targetNode))
            {
                junctionDecision();
                latestJunctionStartedAt = millis();
            }
        }

        // if (blockNodeIndex <= 1 &&
        //     Direction::isNodeBeforeTarget(latestNode, currentDirection, targetNode) &&
        //     )
        // {
        //     if (currentBlock == Block_t::NONE)
        //     {
        //         deliveryTask = ENTER_BLOCK_ZONE;
        //     }
        //     else if (millis() - latestJunctionEndedAt > 800)
        //     {
        //         deliveryTask = ENTER_DROP_ZONE;
        //     }
        //     latestJunctionStartedAt = millis();
        // }
        // else
        // if (Direction::isNodeBeforeNodeBeforeTarget(latestNode, currentDirection, targetNode) && abs(angleError) < 20)
        // {
        //     if (currentBlock == Block_t::NONE)
        //     {
        //         deliveryTask = ENTER_BLOCK_ZONE;
        //     }
        //     else
        //     {
        //         deliveryTask = DROP_OFF;
        //     }
        //     latestJunctionStartedAt = millis();
        // }
        // break;

        break;
    case LEFT_TURN:
        if (millis() - latestJunctionStartedAt > TURN_DELAY)
        {
            wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed);
            if ((encodedLineSensorReading & 0b0100) == 0b0100 && millis() - latestJunctionStartedAt > 500 + TURN_DELAY)
            {
                Serial.println("Special turn left complete");
                endTurn();
            }
        }
        else
        {
            wheelMotors.setSpeedsAndRun(maxSpeed, maxSpeed);
        }
        break;
    case RIGHT_TURN:
        if (millis() - latestJunctionStartedAt > TURN_DELAY)
        {
            wheelMotors.setSpeedsAndRun(maxSpeed, -maxSpeed);
            if ((encodedLineSensorReading & 0b0010) == 0b0010 && millis() - latestJunctionStartedAt > 500 + TURN_DELAY)
            {
                Serial.println("Special turn right complete");
                endTurn();
            }
        }
        else
        {
            wheelMotors.setSpeedsAndRun(maxSpeed, maxSpeed);
        }
        break;
    }
}
void Robot::task_enter_block_zone()
{
    switch (blockNodeIndex)
    {
    // First two blocks
    // ENTER_BLOCK_ZONE starts from the turn into the zone
    case 0:
    case 1:
        wheelMotors.setSpeedsAndRun(-maxSpeed / 2, -maxSpeed / 2);
        delay(500);
        wheelMotors.stop();
        deliveryTask = GRAB;
        break;

    // Last two blocks
    // ENTER_BLOCK_ZONE starts from the node before the turn into the zone
    case 2:
        // TODO: You are at node 9 (or 15)
        // Go forwards a bit then turn
        // Then change deliveryTask to GRAB

        latestJunctionStartedAt = millis();
        while(millis() - latestJunctionStartedAt < 2200)
        {
            readSensors();
            Steering::assignAngleError(*this);
            Steering::correctSteering(*this);
            delay(1000 * DT);
        }
        latestJunctionStartedAt = millis();
        wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed);
        while(millis() - latestJunctionStartedAt < 550)
        {
            delay(1000 * DT);
        }
        wheelMotors.stop();
        
        deliveryTask = GRAB;
        latestNode = 13;
        currentDirection = Direction::W;
        targetDirection = Direction::S;

        break;
    case 3:
        // TODO: You are at node 9 (or 15)
        // Go forwards a bit then turn
        break;
    }
}
void Robot::task_grab()
{
    servos.setArm(0);
    // TODO: locate block and pick it up
    // Maybe add a sub-state for grab_task being an enum of {SEARCH, APPROACH, GRAB} ?
    // Then change deliveryTask to EXIT_BLOCK_ZONE

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

            speedCounter += 0.7;

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
    float driveToBlockDistance = 3.8; // TODO: tune this value
    maxSpeed = 120;
    latestJunctionEndedAt = millis();
    while (abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION) > driveToBlockDistance && millis() - latestJunctionEndedAt < 1000)
    {
        readSensors();
        Steering::assignAngleError(*this);
        Steering::correctSteering(*this);
        delay(1000 * DT);
    }
    wheelMotors.stop();
    maxSpeed = 255;
    servos.setClaw(85);
    delay(200);

    // BLOCK IDENTIFICATION
    float max_solid = 10, min_solid = 8; // TODO: tune these values
    LED *identificationLED;

    int reading = analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION;
    Serial.print("Identification Reading: ");
    Serial.println(reading);
    if (!switchButton->pressed()) //(min_solid <= reading && reading < max_solid && solidBlocksCollected < 2)
    {
        identificationLED = new LED(4);
        currentBlock = Block_t::SOLID;
        solidBlocksCollected++;
        targetNode = 2;
    }
    else
    {
        identificationLED = new LED(3);
        currentBlock = Block_t::FOAM;
        foamBlocksCollected++;
        targetNode = 0;
    }
    identificationLED->on();
    delay(5500); // must light up for >5s
    identificationLED->off();
    servos.setArm(110); // Raise arm

    targetDirection = Direction::nextDir(latestNode, targetNode);
    latestJunctionStartedAt = millis();
    deliveryTask = EXIT_BLOCK_ZONE;
}
void Robot::task_exit_block_zone()
{
    if (latestJunctionStartedAt + 1000 > millis())
    {
        if (Direction::isLeftTurn(currentDirection, targetDirection))
        {
            wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed / 4);
        }
        else
        {
            wheelMotors.setSpeedsAndRun(maxSpeed / 4, -maxSpeed);
        }
    }
    else
    {
        Serial.println("Exited block zone");
        latestJunctionEndedAt = millis();
        currentDirection = targetDirection;
        deliveryTask = NAVIGATE;
    }
}
void Robot::task_enter_drop_zone()
{
    readSensors();
    Steering::assignAngleError(*this);
    Steering::correctSteering(*this);
    if (millis() - latestJunctionEndedAt > 400 && millis() - latestJunctionStartedAt > 400)
    {
        wheelMotors.stop();
        deliveryTask = DROP_OFF;
    }
}
void Robot::task_drop_off()
{
    Serial.println("Dropping off");
    servos.setArm(0);
    servos.setClaw(10);
    servos.setArm(110);
    blockNodeIndex++;
    deliveryTask = EXIT_DROP_ZONE;
    latestJunctionStartedAt = millis();
}
void Robot::task_exit_drop_zone()
{
    if (blockNodeIndex < 4)
    {
        // NOTE: New version
        if (latestJunctionStartedAt + 1600 > millis())
        {
            if (currentBlock == Block_t::SOLID)
            {
                latestNode = 6;
                targetDirection = Direction::E;
            }
            else
            {
                latestNode = 4;
                targetDirection = Direction::W;
            }
            if (Direction::isLeftTurn(currentDirection, targetDirection))
            {
                wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed / 3);
            }
            else
            {
                wheelMotors.setSpeedsAndRun(-maxSpeed / 3, -maxSpeed);
            }
        }
        else if (latestJunctionStartedAt + 2000 > millis())
        {
            wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed);
        }
        else
        {
            currentDirection = targetDirection;
            Serial.println("Exited drop zone");
            latestJunctionEndedAt = millis();
            deliveryTask = NAVIGATE;
            currentBlock = Block_t::NONE;
            targetNode = blockNodes[blockNodeIndex];
        }

        // // NOTE: Old version
        // latestNode = targetNode;
        // targetNode = blockNodes[blockNodeIndex];
        // targetDirection = Direction::N;
        // latestJunctionStartedAt = millis() - TURN_DELAY;
        // if (currentBlock == Block_t::SOLID)
        // {
        //     drivingMode = LEFT_TURN;
        // }
        // else
        // {
        //     drivingMode = RIGHT_TURN;
        // }
        // deliveryTask = NAVIGATE;
    }
}

Robot &Robot::drive()
{
    switch (deliveryTask)
    {
    case NAVIGATE:
        task_navigate();
        break;
    case ENTER_BLOCK_ZONE:
        task_enter_block_zone();
        break;
    case GRAB:
        task_grab();
        break;
    case EXIT_BLOCK_ZONE:
        task_exit_block_zone();
        break;
    case ENTER_DROP_ZONE:
        task_enter_drop_zone();
        break;
    case DROP_OFF:
        task_drop_off();
        break;
    case EXIT_DROP_ZONE:
        task_exit_drop_zone();
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

    int prevNode = latestNode;
    latestNode = Direction::nextNode(latestNode, currentDirection);
    targetDirection = Direction::nextDir(latestNode, targetNode);
    Serial.print("Reached: ");
    Serial.println(latestNode);

    if (Direction::isRightTurn(currentDirection, targetDirection))
    {
        drivingMode = RIGHT_TURN;
        targetDirection = Direction::rightOf(currentDirection);
        Serial.println("Right turn");
    }
    else if (currentDirection == targetDirection)
    {
        drivingMode = FOLLOW;
        latestJunctionEndedAt = millis() + 600;
    }
    else if (Direction::isLeftTurn(currentDirection, targetDirection))
    {
        drivingMode = LEFT_TURN;
        targetDirection = Direction::leftOf(currentDirection);
        Serial.println("Left turn");
    }
    else if (Direction::is180(currentDirection, targetDirection))
    {
        if (Direction::nextNode(latestNode, Direction::leftOf(currentDirection)) >= 0)
        {
            drivingMode = LEFT_TURN;
            targetDirection = Direction::leftOf(currentDirection);
            Serial.println("180 requested, doing left turn instead");
        }
        else if (Direction::nextNode(latestNode, Direction::rightOf(currentDirection)) >= 0)
        {
            drivingMode = RIGHT_TURN;
            targetDirection = Direction::rightOf(currentDirection);
            Serial.println("180 requested, doing right turn instead");
        }
        else
        {
            drivingMode = FOLLOW;
            Serial.println("180 requested, going straight instead");
            latestJunctionEndedAt = millis() + 600;
        }
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