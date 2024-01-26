#include <ArxContainer.h>
#include "./robot.h"
#include "./sensor.h"
#include "./constants.h"
#include "./motors.h"
#include "./direction_matrix.h"

// cppcheck-suppress passedByValue
Robot::Robot(Motors::MotorPair motors_, arx::vector<pin_size_t> line_sensors_) : motors(motors_), line_sensors(line_sensors_)
{
    motors
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
    // Serial.print("Line sensor readings: ");
    // Helper::printVector(readings);
    return *this;
}

Robot &Robot::assignAngleError()
{
    // Set an associated angle error for each reading.
    // This is the angle that the robot will read to correct its steering.
    // Positive is too far right, negative is too far left.
    // Currently these are arbitrary values.
    // TODO: Calibrate these values

    // The encoded line sensor reading is a 4-bit number,
    // where each bit represents a sensor,
    // ordered from left to right.
    switch (encodedLineSensorReading)
    {
    case 0b0000:
        drivingMode = FOLLOW;
        angleError = 0;
        break;

    // G: Weird sensor, just keep doing what it's doing
    case 0b1001:
        break;

    // Very far left of line
    case 0b0001:
        drivingMode = FOLLOW;
        angleError = -70;
        break;

    // Very far right of line
    case 0b1000:
        drivingMode = FOLLOW;
        angleError = 70;
        break;

    // Slightly too far left of line
    case 0b0010:
        drivingMode = FOLLOW;
        angleError = -15;
        break;

    // Slightly too far right of line
    case 0b0100:
        drivingMode = FOLLOW;
        angleError = 15;
        break;

    // Just before a junction,
    // offset to the right
    // and angled left,
    // into the junction.
    // But if we keep going,
    // we'll be fine!
    case 0b0101:
        drivingMode = FOLLOW;
        angleError = -20; // G: Consider revisiting this scenario
        break;

    // Just before a junction,
    // offset to the left
    // and angled right,
    // into the junction.
    // But if we keep going,
    // we'll be fine!
    case 0b1010:
        drivingMode = FOLLOW;
        angleError = +20; // G: Consider revisiting this scenario
        break;

    // Junction reached
    case 0b0110:
        drivingMode = FOLLOW;
        angleError = 0;
        break;

    // G: Junction reached
    case 0b0011:
        drivingMode = TURN;
        angleError = 0;
        break;

    // G: Weird sensor, just keep doing what it's doing
    case 0b1011:
        break;

    // G: Junction reached
    case 0b1100:
        drivingMode = TURN;
        angleError = 0;
        break;

    // G: Weird sensor, just keep doing what it's doing
    case 0b1101:
        break;

    // G: Junction reached
    case 0b0111:
        drivingMode = TURN;
        angleError = 0;
        break;

    // G: Junction reached
    case 0b1110:
        drivingMode = TURN;
        angleError = 0;
        break;

    // G: Junction reached
    case 0b1111:
        angleError = 0;
        drivingMode = TURN;
        break;

    default:
        Serial.println("Invalid sensor code:");
        Serial.println(encodedLineSensorReading);
        break;
    }
    return *this;
}

Robot &Robot::drive()
{
    // G: If the robot is about to pick up a cube, stop and turn around
    if (latestNode == targetNode)
    {
        // stageDecision();
    }

    switch (drivingMode)
    {
    case FOLLOW:
        assignAngleError();
        steeringCorrection();
        break;
    case TURN:
        junctionDecision(encodedLineSensorReading);
        break;
    case LTURN:
        junctionTurn(encodedLineSensorReading, true);
        break;
    case RTURN:
        junctionTurn(encodedLineSensorReading, false);
        break;
    default:
        motors.stop();
        break;
    }
    return *this;
}

/*
Implement a single step of the controller.
https://controlguru.com/pid-with-controller-output-co-filter/
TODO: tune constants
TODO: add more filters? https://controlguru.com/using-signal-filters-in-our-pid-loop/
TODO: add feed forward control into the open loop
*/
Robot &Robot::steeringCorrection()
{
    // TODO: tune constants
    const float Kc = 0.02;
    const float Ti = 0.005;
    const float Td = 0.003;
    const float alpha = 0.1; // Note: T_f = alpha * T_d
    const float e = angleError;

    static float prevError = 0;
    static double errorIntegral = 0;
    static float prevCo = 0;

    const float pidCo = Kc * e + Kc * Td * ((e - prevError) / DT); //+ (Kc / Ti) * errorIntegral
    const float filteredCo = pidCo;                                // prevCo + DT / (alpha * Td) * (pidCo - prevCo);
    prevError = e;
    prevCo = filteredCo;
    errorIntegral += e * DT;

    // Now apply the correction to the motors
    // +ve error means too far right, so turn left
    // and $ error \approx K_{c}\times e $ so
    const float rightSpeedMinusLeftSpeed = Helper::clamp<float>(filteredCo * maxSpeed, -maxSpeed, +maxSpeed); // TODO: tune this
    if (rightSpeedMinusLeftSpeed < 0)
    {
        motors.setSpeedsAndRun(maxSpeed, maxSpeed + rightSpeedMinusLeftSpeed);
    }
    else if (rightSpeedMinusLeftSpeed > 0)
    {
        motors.setSpeedsAndRun(maxSpeed - rightSpeedMinusLeftSpeed, maxSpeed);
    }
    else
    {
        motors.setSpeedsAndRun(maxSpeed, maxSpeed);
    }
    return *this;
}

Robot &Robot::junctionDecision(uint8_t encodedLineSensorReadings)
{
    // TODO: decide which way to turn,
    // based on the direction_matrix (see `direction_matrix.cpp`)
    // Make sure to check that `encodedLineSensorReadings` gives the right code for the expected junction!
    Serial.println("--- Junction reached");

    Serial.println("Target node: ");
    Serial.println(targetNode);
    Serial.println("Latest node: ");
    Serial.println(latestNode);
    Serial.println("Current direction: ");
    Serial.println(currentDirection);

    // Display input into navigation map
    // Serial.print("Navigation map input: ");
    // Helper::printPair<int>({latestNode, currentDirection});
    // Serial.print("Navigation map output: ");
    // Serial.println(Direction::navigation_map[{latestNode, currentDirection}]);

    // G: Update position and navigation, decide next direction
    // latestNode = Direction::navigation_map[{latestNode, currentDirection}];
    latestNode = Direction::nav_matrix[latestNode][currentDirection];
    targetDirection = Direction::dir_matrix[latestNode][targetNode];

    Serial.print("New Target node: ");
    Serial.println(targetNode);
    Serial.print("New Target direction: ");
    Serial.println(targetDirection);
    Serial.print("New Latest node: ");
    Serial.println(latestNode);

    Serial.println("Junction; targetNode; currentDirection; targetDirection;");
    Serial.println(latestNode);
    Serial.println(targetNode);
    Serial.println(currentDirection);
    Serial.println(targetDirection);

    // G: mark the stop, not necessary
    motors.stop();
    delay(1000);

    // G: move forward a bit
    motors.setSpeedsAndRun(150, 150);
    delay(100);

    // G: If already facing the right direction, keep going and break
    if (targetDirection == currentDirection)
    {
        delay(300);
        return *this;
    }

    // G: Right turn.
    // G: Only time robot will turn 180 degrees is after picking up a block, no need to code it here.
    if (Direction::isRightTurn(currentDirection, targetDirection))
    {
        motors.setSpeedsAndRun(maxSpeed, 0);
        delay(500);
    }
    else // G: left turn
    {
        motors.setSpeedsAndRun(0, maxSpeed);
        delay(500);
    }
    return *this;
}

Robot &Robot::junctionTurn(uint8_t encodedLineSensorReadings, bool leftTurn)
{
    if ((leftTurn && encodedLineSensorReadings == 0b0100) || (!leftTurn && encodedLineSensorReadings == 0b0010))
    {
        motors.stop();
        drivingMode = FOLLOW;
        currentDirection = targetDirection;
    }
    return *this;
}

// Robot &Robot::stageDecision()
// {
//     // G: Stage 0,  go to node 5;  Stage 1,  pick up cube; Stage 2,  go to node 0 or 2; Stage 3,  drop off cube;

//     // G: Stage 4,  go to node 11; Stage 5,  pick up cube; Stage 6,  go to node 0 or 2; Stage 7,  drop off cube;

//     // G: Stage 8,  go to node 14; Stage 9,  pick up cube; Stage 10, go to node 0 or 2; Stage 11, drop off cube;

//     // G: Stage 12, go to node 17; Stage 13, pick up cube; Stage 14, go to node 0 or 2; Stage 15, drop off cube;
//     stage++;
//     switch (stage)
//     {
//     case 1:
//         break;

//     case 2:
//         break;

//     case 3:
//         break;

//     case 4:
//         break;

//     case 5:
//         break;

//     case 6:
//         break;

//     case 7:
//         break;

//     case 8:
//         break;

//     case 9:
//         break;

//     case 10:
//         break;

//     case 11:
//         break;

//     case 12:
//         break;

//     case 13:
//         break;

//     case 14:
//         break;

//     case 15:
//         break;
//     }
//     return *this;
// }