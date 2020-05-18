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

#ifdef DIRTBALL_HIPSTER_ONE
DeviceAddress sensor1 = { 0x28, 0xBD, 0xC4, 0x79, 0x97, 0x12, 0x03, 0x98 };
#endif

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
	   "{ \"id\": \"%s\", \"org.homebus.experimental.air-sensor\": { \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f } }",
	   homebus_uuid(),
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
	   "{ \"id\": \"%s\", \"org.homebus.experimental.air-quality-sensor\": {  \"tvoc\": %0.2f, \"pm1\": %d, \"pm25\": %d, \"pm10\": %d } }",
	   homebus_uuid(),
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
	   "{ \"id\": \"%s\", \"org.homebus.experimental.light-sensor\": {  \"lux\": %d, \"full_light\": %d, \"ir\": %d, \"visible\": %d } }",
	   homebus_uuid(),
	   tsl2591.lux(), tsl2591.full(), tsl2591.ir(), tsl2591.visible());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean dirtball_uv_light_update(char* buf, size_t buf_len) {
  snprintf(buf,
	   buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.uv-light-sensor\": {  \"uva\": %.1f, \"uvb\": %.1f, \"uvindex\": %.1f  } }",
	   homebus_uuid(),
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
	   "{ \"id\": \"%s\", \"org.homebus.experimental.soil-sensor\": { \"temperature\": %.1f, \"moisture\": %d, \"--moisture-raw\": %d } }",
	   homebus_uuid(),
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
	   "{ \"id\": \"%s\", \"org.homebus.experimental.dirtball-system\": { \"name\": \"%s\", \"build\": \"%s\", \"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" } }",
	   homebus_uuid(),
	   App.hostname().c_str(), App.build_info().c_str(), local[0], local[1], local[2], local[3], mac_address.c_str()
	   );

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean dirtball_diagnostic_update(char* buf, size_t buf_len) {
  snprintf(buf, buf_len, "{ \"id\": \"%s\", \"org.homebus.experimental.dirtball-diagnostic\": { \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": %d, \"wifi_failures\": %d } }",
	   homebus_uuid(),
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
    homebus_publish_to("org.homebus.experimental.air-sensor", buffer);

  if(dirtball_air_quality_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.air-quality-sensor", buffer);

  if(dirtball_light_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.light-sensor", buffer);

  if(dirtball_uv_light_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.uv-light-sensor", buffer);

  if(dirtball_soil_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.soil-sensor", buffer);

  if(dirtball_system_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.dirtball-system", buffer);

  if(dirtball_diagnostic_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.dirtball-diagnostic", buffer);
}

/* 
 * this callback is used to stream sensor data for diagnostics
 */
#ifdef USE_DIAGNOSTICS
void dirtball_stream() {
  static uint8_t count = 0;

  if(count == 0)
    Serial.println("TEMP PRES HUMD TVOC   IR VISB FULL  LUX  1.0  2.5 10.0  SMAX  SMIN  SAVG  SCNT  PIR");

  if(++count == 10)
    count = 0;

  bme680.handle();
  tsl2561.handle();
  pms5003.handle();
  sound_level.handle();

  Serial.printf( "%03.1f %4.0f %4.0f %4.0f %4d %4d %4d %4d %4d %4d %4d %5d %5d %5d %5d    %c\n",
		 bme680.temperature(),
		 bme680.pressure(),
		 bme680.humidity(),
		 bme680.gas_resistance(),
		 tsl2561.ir(),
		 tsl2561.visible(),
		 tsl2561.full(),
		 tsl2561.lux(),
		 pms5003.density_1_0(),
		 pms5003.density_2_5(),
		 pms5003.density_10_0(),
		 sound_level.sound_max(),
		 sound_level.sound_min(),
		 sound_level.sound_level(),
		 sound_level.sample_count(),
		 pir.presence() ? '1' : '0');

  if(0) {
  Serial.println("[system]");
  Serial.printf("  Uptime %.2f seconds\n", uptime.uptime() / 1000.0);
  Serial.printf("  Free heap %u bytes\n", ESP.getFreeHeap());
  Serial.printf("  Wifi RSSI %d\n", WiFi.RSSI());
  }
}
#endif
