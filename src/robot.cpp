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
    case 0b0000:
    
    // G: Weird sensor, just keep doing what it's doing
    case 0b1001:

    // Very far left of line
    case 0b0001:
        drivingMode = FOLLOW;
        angleError = -30;
        break;

    // Very far right of line
    case 0b1000:
        drivingMode = FOLLOW;
        angleError = 30;
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
        drivingMode = TURN;
        angleError = 0;
        break;
    
    // G: Junction reached
    case 0b0011:
        drivingMode = TURN;
        angleError = 0;
        break;

    // G: Weird sensor, just keep doing what it's doing
    case 0b1011:

    // G: Junction reached
    case 0b1100:
        drivingMode = TURN;
        angleError = 0;
        break;

    // G: Weird sensor, just keep doing what it's doing
    case 0b1101:

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
TODO: tune constants
TODO: add more filters? https://controlguru.com/using-signal-filters-in-our-pid-loop/
TODO: add feed forward control into the open loop
*/
Robot &Robot::steeringCorrection()
{
    // TODO: tune constants
    const float Kc = 0.01;
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
    
    // G: Update position and navigation, decide next direction

    // G: move forward a bit, probably be better to move until the sensors have left the junction
    motors.setSpeedsAndRun(255, 255);
    delay(500);
    motors.stop();
    
    // N = 1,
    // E = 2,
    // S = -1,
    // W = -2,
    
    // G: replace these variables with the actual direciton we want to go in
    int initial = 1; // This needs to be the current direction
    int final = -2; // This needs to be the direction we want to go in
    int sensor = 0x0000; // This needs to be the sensor reading

    // G: Left turn.
    // G: Only time robot will turn 180 degrees is after picking up a block, no need to code it here.
    if ((initial == 1 && final == 2) || (initial == 2 && final == -1) || (initial == -1 && final == -2) || (initial == -2 && final == 1))
    {
        motors.setSpeedsAndRun(-50, 50);
        while (sensor != 0x0100){
            // need update sensor
            delay(10);
        }
    }
    else // G: Right turn
    {
        motors.setSpeedsAndRun(50, -50);
        while (sensor != 0x0010){
            // need update sensor
            delay(10);
        }
    }

    // G: We need to leave the junction to avoid this code being run again
    motors.setSpeedsAndRun(255, 255);
    delay(500);
    motors.stop();
    return *this;
}