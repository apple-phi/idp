#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <ArxContainer.h>
#include "./src/robot.h"
#include "./src/motors.h"
#include "./src/constants.h"
#include "./src/direction_matrix.h"

Robot *robot;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left = AFMS.getMotor(1);
Adafruit_DCMotor *right = AFMS.getMotor(2);

void setup()
{
    Serial.begin(9600);
    if (!AFMS.begin())
    {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1)
            ;
    }
    Serial.println("Motor Shield found.");

    Serial.println("Waiting for start button...");
    pinMode(1, INPUT); // Start button
    while (digitalRead(1) == LOW)
    {
        delay(1000 * DT);
    }
    Serial.println("Start button pressed.");
    robot = new Robot({left, right}, {13, 12, 11, 10});

    // Go straight for 1 second
    // TODO: toggle LED
    robot->motors.run(FORWARD).setSpeed(255);
    for (int i = 0; i < 1000 / DT; i++)
    {
        delay(1000 * DT);
    }
}

void loop()
{
    // TODO: toggle LED
    robot->readSensors();
    robot->drive();
    delay(1000 * DT);
}
