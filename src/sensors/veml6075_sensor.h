#pragma once

#include "sensor.h"

#include <Adafruit_VEML6075.h>

class VEML6075_Sensor : public Sensor {
 public:
  VEML6075_Sensor(uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) {};

  void begin();
  void handle();

  float uva() { _mark_read(); return _uva;};
  float uvb() { _mark_read(); return _uvb;};
  float uvindex() { _mark_read(); return _uvindex;};

 private:
  //  Adafruit_VEML6075_Unified _tsl2561 = Adafruit_VEML6075_Unified(0x39, VEML6075_PACKAGE_T_FN_CL);
  Adafruit_VEML6075 _veml6075;

  float _uva = 0;
  float _uvb = 0;
  float _uvindex = 0;
};
