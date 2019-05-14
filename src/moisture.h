#pragma once

#include "sensor.h"

class Moisture_Sensor < Sensor
public:
  Moisture_Sensor(uint8_tpin, uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) {};
end
