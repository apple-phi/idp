#include <ArxContainer.h>
#include <Arduino.h>
#include "./src/robot.h"

Robot *robot;

void setup()
{
    Serial.begin(9600);
    Serial.println("Hello World");
    robot = new Robot({}, {2, 3, 4, 5});
}

void loop()
{
    robot->readSensors();
    robot->assignAngleError();
    robot->drive();
}
