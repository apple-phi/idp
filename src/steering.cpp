#include "./robot.h"
#include "steering.h"
#include "./constants.h"
namespace Steering
{
    void assignAngleError(Robot &r)
    {
        // Set an associated angle error for each reading.
        // This is the angle that the robot will read to correct its steering.
        // Positive is too far right, negative is too far left.
        // Currently these are arbitrary values.

        // The encoded line sensor reading is a 4-bit number,
        // where each bit represents a sensor,
        // ordered from left to right.
        switch (r.encodedLineSensorReading)
        {
        case 0b0000:
        case 0b0110:
            r.drivingMode = r.FOLLOW;
            r.angleError = 0;
            break;

        // G: Weird sensor, just keep doing what it's doing
        case 0b1001:
        case 0b1011:
        case 0b1101:
            break;

        case 0b0001: // Very far left of line
            r.drivingMode = r.FOLLOW;
            r.angleError = -70;
            break;

        case 0b1000: // Very far right of line
            r.drivingMode = r.FOLLOW;
            r.angleError = 70;
            break;

        case 0b0010: // Slightly too far left of line
            r.drivingMode = r.FOLLOW;
            r.angleError = -15;
            break;

        case 0b0100: // Slightly too far right of line
            r.drivingMode = r.FOLLOW;
            r.angleError = 15;
            break;

        // Just before a junction,
        // offset to the right
        // and angled left,
        // into the junction.
        // But if we keep going,
        // we'll be fine!
        case 0b0101:
            r.drivingMode = r.FOLLOW;
            r.angleError = -20; // G: Consider revisiting this scenario
            break;

        // Just before a junction,
        // offset to the left
        // and angled right,
        // into the junction.
        // But if we keep going,
        // we'll be fine!
        case 0b1010:
            r.drivingMode = r.FOLLOW;
            r.angleError = +20; // G: Consider revisiting this scenario
            break;

        // G: Junction reached
        case 0b0011:
        case 0b1100:
        case 0b0111:
        case 0b1110:
        case 0b1111:
            r.angleError = 0;
            r.junctionDecision();
            break;

        default:
            Serial.println("Invalid sensor code:");
            Serial.println(r.encodedLineSensorReading);
            break;
        }
    }
    /*
    Implement a single step of the controller.
    https://controlguru.com/pid-with-controller-output-co-filter/
    TODO: tune constants
    TODO: add more filters? https://controlguru.com/using-signal-filters-in-our-pid-loop/
    TODO: add feed forward control into the open loop
    */
    void correctSteering(Robot &r)
    {
        float pidCo = r.lineFollowPID.update(r.angleError, DT);

        // Now apply the correction to the motors
        // +ve error means too far right, so turn left
        // and $ error \approx K_{c}\times e $ so
        const float rightSpeedMinusLeftSpeed = r.maxSpeed * Helper::clamp<float>(pidCo, -1.0, +1.0);
        if (rightSpeedMinusLeftSpeed < 0)
        {
            r.wheelMotors.setSpeedsAndRun(r.maxSpeed, r.maxSpeed + rightSpeedMinusLeftSpeed);
        }
        else if (rightSpeedMinusLeftSpeed > 0)
        {
            r.wheelMotors.setSpeedsAndRun(r.maxSpeed - rightSpeedMinusLeftSpeed, r.maxSpeed);
        }
        else
        {
            r.wheelMotors.setSpeedsAndRun(r.maxSpeed, r.maxSpeed);
        }
    }
}
