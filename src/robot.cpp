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
        .run(Motors::forward)
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

        break;
    case LEFT_TURN:
        if (millis() - latestJunctionStartedAt > TURN_DELAY)
        {
            wheelMotors.setSpeedsAndRun(-180, maxSpeed);
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
            wheelMotors.setSpeedsAndRun(maxSpeed, -180);
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
    int blindMovingTimeBlock2 = 2700;
    int blindMovingTimeBlock3 = 3000;
    int blindTurningTime = 1000;

    switch (blockNodeIndex)
    {
    case 0:
        break;
    case 1:
        break;
    case 3:
        latestJunctionStartedAt = millis();
        while (millis() - latestJunctionStartedAt < blindMovingTimeBlock2)
        {
            readSensors();
            Steering::assignAngleError(*this);
            Steering::correctSteering(*this);
            delay(1000 * DT);
        }
        latestJunctionStartedAt = millis();
        wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed);
        while (millis() - latestJunctionStartedAt < blindTurningTime)
        {
            delay(1000 * DT);
        }
        wheelMotors.stop();

        deliveryTask = GRAB;
        latestNode = 13;
        currentDirection = Direction::W;
        targetDirection = Direction::S;

        break;
    case 2:

        latestJunctionStartedAt = millis();
        while (millis() - latestJunctionStartedAt < blindMovingTimeBlock3)
        {
            readSensors();
            Steering::assignAngleError(*this);
            Steering::correctSteering(*this);
            delay(1000 * DT);
        }
        latestJunctionStartedAt = millis();
        wheelMotors.setSpeedsAndRun(maxSpeed, -maxSpeed);
        while (millis() - latestJunctionStartedAt < blindTurningTime)
        {
            delay(1000 * DT);
        }
        wheelMotors.stop();

        deliveryTask = GRAB;
        latestNode = 16;
        currentDirection = Direction::S;
        targetDirection = Direction::W;
        break;
    }
    wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed);
    delay(600);
    wheelMotors.stop();
    deliveryTask = GRAB;
}
void Robot::task_grab()
{

    switch (blockNodeIndex)
    {
    case 0:
        servos.openClaw();
        break;
    case 1:
        servos.openClaw();
        break;
    }

    servos.lowerArm();

    int sensityPin = A0; // select the input pin
    if (blockNodeIndex == 2 || blockNodeIndex == 3)
    {
        servos.fullyOpenClaw();

        double distances[2] = {0};
        float crit_gradient = 3, max_gradient = crit_gradient + 4, min_distance = 10, max_distance = 30;
        int delayTime = 100;

        float speedCounter = 0;
        float gradient = 0;

        // BLOCK SWEEPING

        do
        {
            delay(delayTime);

            // Make the robot oscillate like a cosine wave.
            if (cos(speedCounter) > 0)
            {
                wheelMotors.setSpeedsAndRun(maxSpeed / 2, -maxSpeed / 2);
            }
            else
            {
                wheelMotors.setSpeedsAndRun(-maxSpeed / 2, maxSpeed / 2);
            }

            speedCounter += 0.5;

            // Measure gradient of last two readings
            distances[1] = distances[0];
            distances[0] = abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
            gradient = -distances[0] + distances[1];

            Serial.print(gradient, 0);
            Serial.print(", ");
            Serial.println(distances[0]);
        } while (!((crit_gradient < gradient && gradient < max_gradient) && (distances[1] != 0) && (min_distance < distances[0] && distances[0] < max_distance)));
        Serial.println("BLOCK");
        driveToBlockDistance = 0.5;
    }

    // BLOCK GRABBING
    wheelMotors.stop();
    servos.openClaw();
    delay(500);
    latestJunctionEndedAt = millis();
    float ultraSensorReading;
    do
    {
        delay(1000 * DT);
        readSensors();
        Steering::assignAngleError(*this);
        Steering::correctSteering(*this);
        ultraSensorReading = abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
    } while (ultraSensorReading > driveToBlockDistance && ultraSensorReading != 0 && millis() - latestJunctionEndedAt < 1000);
    wheelMotors.stop();
    servos.halfOpenOrHalfCloseClaw();

    // Shake the claw to wiggle the block in case it is stuck
    for (int i = 0; i < 2; i++)
    {
        wheelMotors.setSpeedsAndRun(maxSpeed / 3, -maxSpeed / 3);
        delay(500);
        wheelMotors.setSpeedsAndRun(-maxSpeed / 3, maxSpeed / 3);
        delay(500);
    }

    latestJunctionEndedAt = millis();
    do
    {
        delay(1000 * DT);
        readSensors();
        Steering::assignAngleError(*this);
        Steering::correctSteering(*this);
    } while (millis() - latestJunctionEndedAt < 200);

    wheelMotors.stop();

    servos.closeClaw();
    delay(200);

    // BLOCK IDENTIFICATION
    LED *identificationLED;
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
    delay(5100); // must light up for >5s
    identificationLED->off();
    servos.raiseArm(); // Raise arm

    targetDirection = Direction::nextDir(latestNode, targetNode);
    latestJunctionStartedAt = millis();
    deliveryTask = EXIT_BLOCK_ZONE;
}
void Robot::task_exit_block_zone()
{
    if (latestJunctionStartedAt + 1800 > millis())
    {
        if (Direction::isLeftTurn(currentDirection, targetDirection))
        {
            wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed / 3);
        }
        else
        {
            wheelMotors.setSpeedsAndRun(maxSpeed / 3, -maxSpeed);
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
    // TODO: CHECK THIS WORKS
    if (millis() - latestJunctionEndedAt > 500 && millis() - latestJunctionStartedAt > 500)
    {
        wheelMotors.stop();
        deliveryTask = DROP_OFF;
    }
}
void Robot::task_drop_off()
{
    Serial.println("Dropping off");
    servos.halfRaiseOrHalfLowerArm();
    servos.halfOpenOrHalfCloseClaw();
    servos.raiseArm();
    blockNodeIndex++;
    deliveryTask = EXIT_DROP_ZONE;
    latestJunctionStartedAt = millis();
}
void Robot::task_exit_drop_zone()
{
    // if (blockNodeIndex < 4)
    {
        // NOTE: New version
        if (latestJunctionStartedAt + 2000 > millis())
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
                wheelMotors.setSpeedsAndRun(-maxSpeed, 0);
            }
            else
            {
                wheelMotors.setSpeedsAndRun(0, -maxSpeed);
            }
        }
        else if (latestJunctionStartedAt + 2000 + 1500 > millis())
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

    // Fix to skip ghost node 13
    if (latestNode == 13)
    {
        if (currentDirection == Direction::N)
            latestNode = 15;
        else
            latestNode = 9;
    }

    targetDirection = Direction::nextDir(latestNode, targetNode);
    Serial.print("Reached: ");
    Serial.println(latestNode);

    if (currentDirection == targetDirection)
    {
        drivingMode = FOLLOW;
        latestJunctionEndedAt = millis() + 1500;
    }
    else if (Direction::isRightTurn(currentDirection, targetDirection))
    {
        drivingMode = RIGHT_TURN;
        targetDirection = Direction::rightOf(currentDirection);
        Serial.println("Right turn");
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