#include "as726x_sensor.h"

void AS726x_Sensor::begin() {
  if (!_as726x.begin()) {
    _status = SENSOR_NOT_FOUND;
    Serial.println("AS726X not found");
    return;
  }
  
  _status = SENSOR_OKAY;

}

void AS726x_Sensor::handle() {
  uint16_t sensorValues[AS726x_NUM_CHANNELS];

  _temperature = _as726x.readTemperature();

  _as726x.startMeasurement();
  while(!_as726x.dataReady()) {
    delay(5);
  }

  _violet = sensorValues[AS726x_VIOLET];
  _blue = sensorValues[AS726x_BLUE];
  _green = sensorValues[AS726x_GREEN];
  _yellow = sensorValues[AS726x_YELLOW];
  _orange = sensorValues[AS726x_ORANGE];
  _red = sensorValues[AS726x_RED];
}
