#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/wifi.h>
#include <multiball/ota_updates.h>
#include <multiball/homebus.h>

#include "dirtball.h"

#include "sensors/bme280_sensor.h"
#include "sensors/bme680_sensor.h"
#include "sensors/tsl2561_sensor.h"
#include "sensors/tsl2591_sensor.h"
#include "sensors/veml6075_sensor.h"
#include "sensors/moisture_sensor.h"
#include "sensors/uptime.h"

#define MQTT_UPDATE_FREQUENCY 0

BME280_Sensor bme280(MQTT_UPDATE_FREQUENCY, 0, 0, false);
BME680_Sensor bme680(MQTT_UPDATE_FREQUENCY, 0, 0, false);

TSL2561_Sensor tsl2561(MQTT_UPDATE_FREQUENCY, 0, 0, false);
TSL2591_Sensor tsl2591(MQTT_UPDATE_FREQUENCY, 0, 0, false);

VEML6075_Sensor veml6075(MQTT_UPDATE_FREQUENCY, 0, 0, false);

Moisture_Sensor moisture_cap(MOISTURE_CAPACITIVE_PIN, MQTT_UPDATE_FREQUENCY, 0, 0, false);
// Moisture_Sensor moisture_res(MOISTURE_RESISTIVE_PIN, MQTT_UPDATE_FREQUENCY, 0, 0, false);

Uptime uptime;

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(DALLAS_IO_PIN);
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1 = { 0x28, 0xBD, 0xC4, 0x79, 0x97, 0x12, 0x03, 0x98 };

#ifdef DIRTBALL_CTRLH_ONE
DeviceAddress sensor1 = { 0x28, 0xE8, 0xCD, 0x79, 0x97, 0x13, 0x3, 0x6A };
#endif

void dirtball_setup() {
#ifdef HAS_BME280
  bme280.begin();
#ifdef VERBOSE
  Serial.println("[bme280]");
#endif
#endif

#ifdef HAS_BME680
  bme680.begin();
#ifdef VERBOSE
  Serial.println("[bme680]");
#endif
#endif

#ifdef HAS_TSL2561
  tsl2561.begin();
#ifdef VERBOSE
  Serial.println("[tsl2561]");
#endif
#endif

#ifdef HAS_TSL2591
  tsl2591.begin();
#ifdef VERBOSE
  Serial.println("[tsl2591]");
#endif
#endif

  veml6075.begin();
#ifdef VERBOSE
  Serial.println("[veml6075]");
#endif
  sensors.begin();
#ifdef VERBOSE
  Serial.println("[dallas]");
#endif

  moisture_cap.begin(MOISTURE_CAP_CAL_DRY, MOISTURE_CAP_CAL_WET);
#ifdef VERBOSE
  Serial.println("[capacitive moisture]");
#endif

#if 0
  moisture_res.begin(MOISTURE_RES_CAL_DRY, MOISTURE_RES_CAL_WET);
#ifdef VERBOSE
  Serial.println("[resistive moisture]");
#endif
#endif

#ifdef MQTT_HOST
  Serial.println("[mqtt credentials]");
  homebus_stuff(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_UUID);
#ifdef MQTT_OVERRIDE_TOPIC_PREFIX
  homebus_mqtt_override_prefix(MQTT_OVERRIDE_TOPIC_PREFIX);
#endif
#endif

#ifdef HOMEBUS_NO_ENVELOPE
  homebus_use_envelope(false);
#endif

  homebus_set_provisioner(HOMEBUS_SERVER, HOMEBUS_AUTHENTICATION_TOKEN);

  static const char *ro_ddcs[] = {
		      DDC_AIR_SENSOR,
		      DDC_AIR_QUALITY_SENSOR,
		      DDC_AQI_SENSOR,
		      DDC_SOIL_SENSOR,
		      DDC_LIGHT_SENSOR,
		      DDC_UV_SENSOR,
		      DDC_SYSTEM,
		      DDC_DIAGNOSTIC,
		      NULL
  };
  static const char *wo_ddcs[] = { NULL };
  static char mac_address[3*6];

  strncpy(mac_address, App.mac_address().c_str(), 3*6);

  // this is... wrong - needs to be sorted for proper Homebus use
  homebus_configure("Homebus",
		    "Dirtball garden monitor",
		    mac_address,
		    "",
		    ro_ddcs,
		    wo_ddcs);

  homebus_setup();
  Serial.println("[homebus]");


}

static boolean dirtball_air_update(char* buf, size_t buf_len) {
  float temperature = 0, humidity = 0, pressure = 0;

#ifdef HAS_BME280
  bme280.handle();
  temperature = bme280.temperature();
  humidity = bme280.humidity();
  pressure = bme280.pressure();
#endif
  
#ifdef HAS_BME680
  bme680.handle();
  temperature = bme680.temperature();
  humidity = bme680.humidity();
  pressure = bme680.pressure();
#endif

#ifdef TEMPERATURE_ADJUSTMENT
  temperature += TEMPERATURE_ADJUSTMENT;
#else

  snprintf(buf,
	   buf_len,
	   "{ \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f }",
	   temperature, humidity, pressure);

  Serial.println(buf);
#endif

  return true;
}

static boolean dirtball_air_quality_update(char* buf, size_t buf_len) {
#if 0
  uint16_t pm1 = pms5003.density_1_0();
  uint16_t pm25 = pms5003.density_2_5();
  uint16_t pm10 = pms5003.density_10_0();

  if(pm1 > 10000 && uptime.uptime() < 60*1000)
    pm1 = 0;

  if(pm25 > 10000 && uptime.uptime() < 60*1000)
    pm25 = 0;

  if(pm10 > 10000 && uptime.uptime() < 60*1000)
    pm10 = 0;

  snprintf(buf,
	   buf_len,
	   "{ \"tvoc\": %0.2f, \"pm1\": %d, \"pm25\": %d, \"pm10\": %d }",
	   bme680.gas_resistance(), pm1, pm25, pm10);

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
#endif

  return false;
}

static boolean dirtball_light_update(char* buf, size_t buf_len) {
  snprintf(buf,
	   buf_len,
	   "{ \"lux\": %d, \"full_light\": %d, \"ir\": %d, \"visible\": %d }",
	   tsl2591.lux(), tsl2591.full(), tsl2591.ir(), tsl2591.visible());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean dirtball_uv_light_update(char* buf, size_t buf_len) {
  snprintf(buf,
	   buf_len,
	   "{ \"uva\": %.1f, \"uvb\": %.1f, \"uvindex\": %.1f }",
	   veml6075.uva(), veml6075.uvb(), veml6075.uvindex());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean dirtball_soil_update(char* buf, size_t buf_len) {
  sensors.requestTemperatures();
  float dirt_temp = sensors.getTempC(sensor1);

  snprintf(buf,
	   buf_len,
	   "{ \"temperature\": %.1f, \"moisture\": %d, \"--moisture-raw\": %d }",
	   dirt_temp, moisture_cap.read(), moisture_cap.read_raw());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}


/*
 * we do this once at startup, and not again unless our IP address changes
 */
static boolean dirtball_system_update(char* buf, size_t buf_len) {
  static IPAddress oldIP = IPAddress(0, 0, 0, 0);
  static String mac_address = WiFi.macAddress();
  IPAddress local = WiFi.localIP();

  if(oldIP == local)
    return false;

  snprintf(buf,
	   buf_len,
	   "{ \"name\": \"%s\", \"platform\": \"%s\", \"build\": \"%s\", \"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" }",
	   App.hostname().c_str(), "dirtball", App.build_info().c_str(), local[0], local[1], local[2], local[3], mac_address.c_str()
	   );

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean dirtball_diagnostic_update(char* buf, size_t buf_len) {
  snprintf(buf,
	   buf_len,
	   "{ \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": %d, \"wifi_failures\": %d }",
	   ESP.getFreeHeap(), uptime.uptime()/1000, WiFi.RSSI(), App.boot_count(), App.wifi_failures());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}


void dirtball_loop() {
  static unsigned long next_loop = 0;

  if(next_loop > millis())
    return;

  next_loop = millis() + UPDATE_DELAY;

  bme680.handle();
  tsl2591.handle();
  veml6075.handle();
  moisture_cap.handle();

#define BUFFER_LENGTH 600
  char buffer[BUFFER_LENGTH];

  if(dirtball_air_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_AIR_SENSOR, buffer);

  if(dirtball_air_quality_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_AIR_QUALITY_SENSOR, buffer);

  if(dirtball_light_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_LIGHT_SENSOR, buffer);

  if(dirtball_uv_light_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_UV_SENSOR, buffer);

  if(dirtball_soil_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_SOIL_SENSOR, buffer);

  if(dirtball_system_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_SYSTEM, buffer);

  if(dirtball_diagnostic_update(buffer, BUFFER_LENGTH))
    homebus_publish_to(DDC_DIAGNOSTIC, buffer);
}

void homebus_mqtt_callback(char const*, char const*);

void homebus_mqtt_callback(char const*, char const*) {
}

/* 
 * this callback is used to stream sensor data for diagnostics
 */
#ifdef USE_DIAGNOSTICS
void dirtball_stream() {
  static uint8_t count = 0;

  if(count == 0)
    Serial.println("TEMP PRES HUMD TVOC   IR VISB FULL  LUX");

  if(++count == 10)
    count = 0;

  bme680.handle();
  tsl2561.handle();

  Serial.printf( "%03.1f %4.0f %4.0f %4.0f %4d %4d %4d %4d\n",
		 bme680.temperature(),
		 bme680.pressure(),
		 bme680.humidity(),
		 bme680.gas_resistance(),
		 tsl2561.ir(),
		 tsl2561.visible(),
		 tsl2561.full(),
		 tsl2561.lux());

  if(0) {
  Serial.println("[system]");
  Serial.printf("  Uptime %.2f seconds\n", uptime.uptime() / 1000.0);
  Serial.printf("  Free heap %u bytes\n", ESP.getFreeHeap());
  Serial.printf("  Wifi RSSI %d\n", WiFi.RSSI());
  }
}
#endif

