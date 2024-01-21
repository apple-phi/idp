#pragma once
#include <Arduino.h>
#include <ArxContainer.h>
namespace Sensors
{
    arx::vector<PinStatus> digitalReadAll(arx::vector<pin_size_t> sensors);
    uint8_t encodeLineSensorReadings(arx::vector<PinStatus> readings);
}