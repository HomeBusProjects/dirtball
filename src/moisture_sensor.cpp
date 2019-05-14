#include "moisture_sensor.h"

uint8_t Moisture_Sensor::read() {
  uint16_t raw =  analogRead(_pin); 
  int cal;

  if(_wet < _dry)
    cal = 100 - map(raw,  _wet, _dry, 0, 100);
  else
    cal = map(raw, _dry, _wet, 0, 100);

  if(cal < 0)
    cal = 0;

  if(cal > 100)
    cal = 100;

  return (uint8_t)cal;
}
