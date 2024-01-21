#include "./robot.h"
#include "./sensor.h"

Robot::Robot(arx::vector<pin_size_t> motors_, arx::vector<pin_size_t> line_sensors_) : motors(motors_), line_sensors(line_sensors_)
{
    for (auto &m : motors)
    {
        pinMode(m, OUTPUT);
    }
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
    return *this;
}

Robot &Robot::assignAngleError()
{
    // Set an associated angle error for each reading.
    // This is the angle that the robot will read to correct its steering.
    // Positive is too far right, negative is too far left.
    // Currently these are arbitrary values.
    // TODO: Calibrate these values and/or the PD controller

    // The encoded line sensor reading is a 4-bit number,
    // where each bit represents a sensor,
    // ordered from left to right.
    switch (encodedLineSensorReading)
    {
    // Alas, we're lost!
    case 0b0000:
    case 0b1001:
        drivingMode = TURN;
        break;

    // Very far left of line
    case 0b0001:
        drivingMode = FOLLOW;
        angleError = -60;
        break;

    // Very far right of line
    case 0b1000:
        drivingMode = FOLLOW;
        angleError = 60;
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
        angleError = -30;
        break;

    // Just before a junction,
    // offset to the left
    // and angled right,
    // into the junction.
    // But if we keep going,
    // we'll be fine!
    case 0b1010:
        drivingMode = FOLLOW;
        angleError = +30;
        break;

    // Junction reached
    case 0b0110:
    case 0b0011:
    case 0b1011:
    case 0b1100:
    case 0b1101:
    case 0b0111:
    case 0b1110:
    case 0b1111:
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
    if (drivingMode == FOLLOW)
    {
        steeringCorrection();
    }
    else if (drivingMode == TURN)
    {
        junctionDecision(encodedLineSensorReading);
    }
    return *this;
}

Robot &Robot::steeringCorrection()
{
    // TODO: Implement PD controller based on angleError
    // Would be helpful to do some maths
    // Based on the robot and line dimensions
    return *this;
}

Robot &Robot::junctionDecision(uint8_t encodedLineSensorReadings)
{
    // TODO: decide which way to turn,
    // based on the direction_matrix (see `direction_matrix.cpp`)
    // Make sure to check that `encodedLineSensorReadings` gives the right code for the expected junction!
    return *this;
}