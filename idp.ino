#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <ArxContainer.h>
#include "./src/robot.h"
#include "./src/motors.h"
#include "./src/constants.h"

Robot *robot;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *left = AFMS.getMotor(1);
Adafruit_DCMotor *right = AFMS.getMotor(2);

void setup()
{
    Serial.begin(9600);
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");
    if (!AFMS.begin())
    {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1)
            ;
    }
    Serial.println("Motor Shield found.");

    robot = new Robot({left, right}, {13, 12, 11, 10});
}

void loop()
{
    robot->readSensors();
    robot->assignAngleError();
    robot->drive();
    delay(1000 * DT);
}
