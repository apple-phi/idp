#include <ArxContainer.h>
class Robot
{
public:
    arx::vector<pin_size_t> motors;
    arx::vector<pin_size_t> line_sensors;
    Robot(arx::vector<pin_size_t> motors, arx::vector<pin_size_t> line_sensors);
    Robot &readSensors();
    Robot &assignAngleError();
    Robot &drive();
    Robot &steeringCorrection();
    Robot &junctionDecision(uint8_t sensorCode);

private:
    uint8_t encodedLineSensorReading = 0b0110;
    float angleError = 0; // in degrees, positive is too far right, negative is too far left
    enum
    {
        FOLLOW,
        TURN
    } drivingMode = FOLLOW;
};
