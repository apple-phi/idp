#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "./src/robot.h"
#include "./src/motors.h"
#include "./src/constants.h"
#include "./src/sensor.h"
#include "./src/led.h"

Robot *robot;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left = AFMS.getMotor(4);
Adafruit_DCMotor *right = AFMS.getMotor(3);
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
    startButton = new Sensors::Button(7);
    robot->servos.reset();
    while (!startButton->pressed())
    {
        delay(10);
    }
    delay(500);
}

void reset()
{
    delete robot;
    robot = new Robot({left, right}, {13, 12, 11, 10});
    asm volatile("jmp 0");
}

void setup()
{
    Serial.begin(9600);
    beginMotors();
    

    robot = new Robot({left, right}, {13, 12, 11, 10});

    Direction::nav_matrix[robot->latestNode][robot->currentDirection] = 1;

    delay(500);
    handleStartButton();
}

void loop()
{
    if (startButton->pressed())
    {
        reset();
    }
    (*robot)
        .readSensors()
        .drive()
        .delayAndBlinkIfMoving(1000 * DT);
}
