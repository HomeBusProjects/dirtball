#pragma once

#include <Arduino.h>

#include "sensor.h"

class Moisture_Sensor : public Sensor {
public:
  Moisture_Sensor(uint8_t pin, uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) , _pin(pin) {};

  void begin() { };
  void begin(uint16_t dry, uint16_t wet) { _dry = dry; _wet = wet; };
  void handle() {};

  void calibrate(uint16_t dry, uint16_t wet) { _dry = dry; _wet = wet; };

  uint8_t read(); // returns % moist, 0 - 100
  uint16_t read_raw() { return analogRead(_pin); };
private:
  uint8_t _pin;

  uint16_t _dry = 0;
  uint16_t  _wet = 0;
};
