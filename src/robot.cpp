#include "./robot.h"
#include "./sensor.h"
#include "./constants.h"
#include "./motors.h"

// cppcheck-suppress passedByValue
Robot::Robot(Motors::MotorPair motors_, arx::vector<pin_size_t> line_sensors_) : motors(motors_), line_sensors(line_sensors_)
{
    motors
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
    Serial.print("Line sensor readings: ");
    Helper::printVector(readings);
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

/*
Implement a single step of the controller.
https://controlguru.com/pid-with-controller-output-co-filter/
*/
Robot &Robot::steeringCorrection()
{
    // TODO: tune constants
    const float Kc = 0.02;
    const float Ti = 0.5;
    const float Td = 0.125;
    const float alpha = 0.2; // Note: T_f = alpha * T_d
    const float e = angleError;

    static float prevError = 0;
    static double errorIntegral = 0;
    static float prevCo = 0;

    const float pidCo = Kc * e + (Kc / Ti) * errorIntegral + Kc * Td * ((e - prevError) / DT);
    const float filteredCo = prevCo + DT / (alpha * Td) * (pidCo - prevCo);
    prevError = e;
    prevCo = pidCo;
    errorIntegral += e * DT;

    // Now apply the correction to the motors
    // +ve error means too far right, so turn left
    // and $ error \approx K_{c}\times e $ so
    const float rightSpeedMinusLeftSpeed = Helper::clamp<float>(filteredCo * maxSpeed, -maxSpeed, +maxSpeed); // TODO: tune this
    if (rightSpeedMinusLeftSpeed > 0)
    {
        motors.setSpeedsAndRun(maxSpeed - rightSpeedMinusLeftSpeed, maxSpeed);
    }
    else
    {
        motors.setSpeedsAndRun(maxSpeed, maxSpeed + rightSpeedMinusLeftSpeed);
    }
    return *this;
}

Robot &Robot::junctionDecision(uint8_t encodedLineSensorReadings)
{
    // TODO: decide which way to turn,
    // based on the direction_matrix (see `direction_matrix.cpp`)
    // Make sure to check that `encodedLineSensorReadings` gives the right code for the expected junction!
    motors.stop();
    return *this;
}