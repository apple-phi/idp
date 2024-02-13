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
    switchButton = new Sensors::Button(2);
    wheelMotors
        .setSpeed(maxSpeed)
        .run(Motors::forward)
        .stop();
    for (auto &s : line_sensors)
    {
        pinMode(s, INPUT);
    }
    moveLED = new LED(5);
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
            if ((!goHome &&
                 currentBlock == Block_t::NONE &&
                 Direction::isNodeBeforeTarget(latestNode, currentDirection, targetNode)))
            {
                deliveryTask = ENTER_BLOCK_ZONE;
                latestJunctionStartedAt = millis();
            }
            else if (!goHome && (millis() - latestJunctionEndedAt > 300) &&
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
            if (goHome && latestNode == 1)
            {
                wheelMotors.setSpeed(maxSpeed);
                delayAndBlinkIfMoving(2000);
                wheelMotors.stop();
                asm volatile("jmp 0");
                while (1)
                    ;
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
    int blindMovingTimeBlock2 = 2650;
    int blindMovingTimeBlock3 = 3150;
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
            delayAndBlinkIfMoving(1000 * DT); // delay(1000 * DT);
        }

        wheelMotors.setSpeedsAndRun(-maxSpeed, maxSpeed);
        delayAndBlinkIfMoving(blindTurningTime);
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
            delayAndBlinkIfMoving(1000 * DT); // delay(1000 * DT);
        }

        wheelMotors.setSpeedsAndRun(maxSpeed, -maxSpeed);
        delayAndBlinkIfMoving(blindTurningTime);
        wheelMotors.stop();

        deliveryTask = GRAB;
        latestNode = 16;
        currentDirection = Direction::S;
        targetDirection = Direction::W;
        break;
    }
    if (blockNodeIndex != 3)
    {
        wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed);
        delayAndBlinkIfMoving(600);
        wheelMotors.stop();
    }

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

    int sensityPin = A0; // select the input pin
    if (blockNodeIndex == 2 || blockNodeIndex == 3)
    {
        servos.fullyOpenClaw();
        servos.fullyLowerArm();

        double distances[2] = {0};
        float crit_gradient = 3, max_gradient = crit_gradient + 100, min_distance = 0, max_distance = 30;
        int delayTime = 50;
        float counterIncrement = 0.3;
        int turnSpeed = 175;

        float speedCounter = 0;
        float gradient = 0;

        // BLOCK SWEEPING
        wheelMotors.setSpeedsAndRun(turnSpeed, -turnSpeed);
        delayAndBlinkIfMoving((3.14159 / counterIncrement - 1) * delayTime);
        wheelMotors.stop();

        do
        {
            delayAndBlinkIfMoving(delayTime); // delay(delayTime);

            // Make the robot oscillate like a cosine wave.
            if (sin(speedCounter) > 0)
            {
                wheelMotors.setSpeedsAndRun(-turnSpeed, turnSpeed);
            }
            else
            {
                wheelMotors.setSpeedsAndRun(turnSpeed, -turnSpeed);
            }

            speedCounter += counterIncrement;

            // Measure gradient of last two readings
            distances[1] = distances[0];
            distances[0] = abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
            gradient = -distances[0] + distances[1];

            Serial.print(gradient, 0);
            Serial.print(", ");
            Serial.println(distances[0]);
        }
        // while (1);
        while (!((crit_gradient < gradient && gradient < max_gradient) && (distances[1] != 0) && (min_distance < distances[0] && distances[0] < max_distance)));
        Serial.println("BLOCK");
        // driveToBlockDistance = 0.5;
    }

    // BLOCK GRABBING
    wheelMotors.stop();
    servos.lowerArm();
    servos.openClaw();
    delayAndBlinkIfMoving(100);
    latestJunctionEndedAt = millis();
    double distances[2] = {0};
    distances[0] = driveToBlockDistance;
    do
    {
        delayAndBlinkIfMoving(1000 * DT); // delay(1000 * DT);
        readSensors();
        Steering::assignAngleError(*this);
        Steering::correctSteering(*this);

        distances[1] = distances[0];
        distances[0] = abs(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
    } while (!((distances[0] < driveToBlockDistance && distances[1] < driveToBlockDistance) || millis() - latestJunctionEndedAt > 1000));
    wheelMotors.stop();
    servos.halfOpenOrHalfCloseClaw();

    // Shake the claw to wiggle the block in case it is stuck
    wheelMotors.setSpeedsAndRun(maxSpeed / 3, -maxSpeed / 3);
    delayAndBlinkIfMoving(200);
    wheelMotors.setSpeedsAndRun(-maxSpeed / 3, maxSpeed / 3);
    delayAndBlinkIfMoving(400);
    wheelMotors.setSpeedsAndRun(maxSpeed / 3, -maxSpeed / 3);
    delayAndBlinkIfMoving(200);
    wheelMotors.stop();

    latestJunctionEndedAt = millis();
    do
    {
        delayAndBlinkIfMoving(1000 * DT); // delay(1000 * DT);
        readSensors();
        Steering::assignAngleError(*this);
        Steering::correctSteering(*this);
    } while (millis() - latestJunctionEndedAt < 150);

    wheelMotors.stop();

    servos.closeClaw();
    delayAndBlinkIfMoving(100);
    wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed);
    delayAndBlinkIfMoving(100);
    wheelMotors.stop();

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

    switch (blockNodeIndex)
    {
    case 1:
        wheelMotors.setSpeedsAndRun(maxSpeed, maxSpeed);
        delayAndBlinkIfMoving(400);
        wheelMotors.stop();
        break;
    case 2:
        wheelMotors.setSpeedsAndRun(maxSpeed, maxSpeed);
        delayAndBlinkIfMoving(400);
        wheelMotors.stop();
        break;
    case 3:
        wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed);
        delayAndBlinkIfMoving(550);
        wheelMotors.stop();
        break;
    }

    latestJunctionStartedAt = millis();
    deliveryTask = EXIT_BLOCK_ZONE;
}
void Robot::task_exit_block_zone()
{
    if (latestJunctionStartedAt + 2400 > millis())
    {
        if (Direction::isLeftTurn(currentDirection, targetDirection))
        {
            wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed / 4);
        }
        else
        {
            wheelMotors.setSpeedsAndRun(-maxSpeed / 4, -maxSpeed);
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
    if (blockNodeIndex < 4)
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
            deliveryTask = NAVIGATE;
            currentDirection = targetDirection;
            Serial.println("Exited drop zone");
            latestJunctionEndedAt = millis();
            currentBlock = Block_t::NONE;
            targetNode = blockNodes[blockNodeIndex];
        }
    }
    else
    {
        deliveryTask = NAVIGATE;
        targetNode = 1;
        wheelMotors.setSpeedsAndRun(-maxSpeed, -maxSpeed);
        delayAndBlinkIfMoving(2700);
        currentDirection = targetDirection = Direction::S;
        if (currentBlock == Block_t::SOLID)
        {
            latestNode = 12;
        }
        else
        {
            latestNode = 8;
        }
        currentBlock = Block_t::NONE;
        goHome = true;
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

void Robot::delayAndBlinkIfMoving(int delayTime)
{
    for (int i = 0; i < delayTime; i += DT * 1000)
    {
        if (wheelMotors.isMoving())
        {
            if (millis() % 500 < 250)
            {
                moveLED->on();
            }
            else
            {
                moveLED->off();
            }
        }
        else
        {
            moveLED->off();
        }
        delay(DT * 1000);
    }
}