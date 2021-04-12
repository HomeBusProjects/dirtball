#pragma once

#define DDC_AIR_SENSOR         "org.homebus.experimental.air-sensor"
#define DDC_LIGHT_SENSOR       "org.homebus.experimental.light-sensor"
#define DDC_AIR_QUALITY_SENSOR "org.homebus.experimental.air-quality-sensor"
#define DDC_AQI_SENSOR         "org.homebus.experimental.aqi-pm25"
#define DDC_UV_SENSOR          "org.homebus.experimental.uv-light-sensor"
#define DDC_SOIL_SENSOR        "org.homebus.experimental.soil-sensor"
#define DDC_SYSTEM             "org.homebus.experimental.system"
#define DDC_DIAGNOSTIC         "org.homebus.experimental.diagnostic"

void dirtball_setup();
void dirtball_loop();
void dirtball_stream();
