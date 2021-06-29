#pragma once

#include "sensor.h"

#include <Adafruit_AS726x.h>

class AS726x_Sensor : public Sensor {
 public:
  AS726x_Sensor(uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) {};

  void begin();
  void handle();


 private:
  Adafruit_AS726x _as726x;

  float _violet;
  float _blue;
  float _green;
  float _yellow;
  float _orange;
  float _red;

  uint8_t _temperature;
};
