#include "hw.h"
#include "pmsa003_sensor.h"

void PMSA003_Sensor::begin() {
  if(_pms.begin_I2C())
    _present = true;
  else
    _present = false;
}

#define PMS_READ_DELAY 1000

void PMSA003_Sensor::handle() {
  if(millis() - _last_read_request < PMS_READ_DELAY)
    return;

  _pms.read(&_data);

  _last_read_request = millis();
}
