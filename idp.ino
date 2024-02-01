#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <ArxContainer.h>
#include "./src/robot.h"
#include "./src/motors.h"
#include "./src/constants.h"
#include "./src/sensor.h"
#include "./src/direction_matrix.h"

Robot *robot;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left = AFMS.getMotor(1);
Adafruit_DCMotor *right = AFMS.getMotor(2);
Servo arm;
Servo claw;
Sensors::Button *startButton;

void beginMotors()
{
    if (!AFMS.begin())
    {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1)
            ;
    }
    Serial.println("Motor Shield found.");
}

void handleStartButton()
{
    Serial.println("Waiting for start button...");
    pinMode(2, INPUT); // Start button
    while (digitalRead(2) == LOW)
    {
        delay(1000 * DT);
    }
    Serial.println("Start button pressed.");
    delay(500);
}

void (*reset)(void) = 0;

void checkRobotReset()
{
    if (digitalRead(2) == HIGH)
    {
        // delete robot;
        reset();
        // setup();
    }
}

void setup()
{
    Serial.begin(9600);
    beginMotors();
    // arm.attach(9);
    // claw.attach(8);
    // arm.write(-20);
    // claw.write(10);
    // delay(500);
    // Serial.println("Arm and claw servos attached.");
    robot = new Robot({left, right}, {13, 12, 11, 10});
    Direction::nav_matrix[robot->latestNode][robot->currentDirection] = 1;
    delay(500);
    handleStartButton();
}

void loop()
{
    // TODO: toggle LED
    checkRobotReset();
    robot->readSensors();
    robot->drive();
    delay(1000 * DT);
}
