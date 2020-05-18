#include "veml6075_sensor.h"

void VEML6075_Sensor::begin() {
  if (!_veml6075.begin()) {
    _status = SENSOR_NOT_FOUND;
    Serial.println("VEML6075 not found");
    return;
  }
  
  _status = SENSOR_OKAY;

  _veml6075.setIntegrationTime(VEML6075_100MS);
  _veml6075.setHighDynamic(true);
  _veml6075.setForcedMode(false);
  _veml6075.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
		   2.95, 1.74,  // UVB_C and UVB_D coefficients
		   0.001461, 0.002591); // UVA and UVB responses
}

void VEML6075_Sensor::handle() {
  _uva = _veml6075.readUVA();
  _uvb = _veml6075.readUVB();
  _uvindex = _veml6075.readUVI();
}
