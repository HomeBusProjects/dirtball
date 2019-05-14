#include <Arduino.h>

#include <Esp.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <Ticker.h>

#include "config.h"
#include "hw.h"

WiFiMulti wifiMulti;

#define MQTT_UPDATE_FREQUENCY 0

#include "bme280_sensor.h"
BME280_Sensor bme280(MQTT_UPDATE_FREQUENCY, 0, 0, false);

#include "tsl2561_sensor.h"

TSL2561_Sensor tsl2561(MQTT_UPDATE_FREQUENCY, 0, 0, false);


#include "moisture_sensor.h"

Moisture_Sensor moisture_cap(MOISTURE_CAPACITIVE_PIN, MQTT_UPDATE_FREQUENCY, 0, 0, false);
Moisture_Sensor moisture_res(MOISTURE_RESISTIVE_PIN, MQTT_UPDATE_FREQUENCY, 0, 0, false);

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(DALLAS_IO_PIN);
DallasTemperature sensors(&oneWire);
// DeviceAddress sensor1 = { 0x28, 0x3F, 0x0B, 0x77, 0x91, 0x06, 0x02, 0x1C };
// DeviceAddress sensor1 = { 0x28, 0x4B, 0x8, 0x46, 0x92, 0x09, 0x02, 0x2D };
// DeviceAddress sensor1 = { 0x28, 0x3F, 0xB, 0x77, 0x91, 0x6, 0x2, 0x0 };
// DeviceAddress sensor1 = { 0x28, 0xE6, 0xB9, 0x46, 0x92, 0x10, 0x2, 0xE7 };
DeviceAddress sensor1 = { 0x28, 0xE8, 0xCD, 0x79, 0x97, 0x13, 0x3, 0x6A };

#include "uptime.h"
Uptime uptime;

static char hostname[sizeof(DIRTBALL_HOSTNAME) + 9];

#include <PubSubClient.h>
static WiFiClient wifi_mqtt_client;

static PubSubClient mqtt_client(MQTT_SERVER, MQTT_PORT, wifi_mqtt_client);


RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int wifi_failures = 0;

#define uS_TO_S_FACTOR 1000000
#define SLEEP_SECONDS 60 * 1

void sensors_on() {
  pinMode(ENABLE_MOISTURE_CAPACITIVE, OUTPUT);
  pinMode(ENABLE_MOISTURE_RESISTIVE, OUTPUT);
  pinMode(ENABLE_DALLAS, OUTPUT);

  digitalWrite(ENABLE_MOISTURE_CAPACITIVE, HIGH);
  digitalWrite(ENABLE_MOISTURE_RESISTIVE, HIGH);
  digitalWrite(ENABLE_DALLAS, HIGH);
}

void sensors_off() {
  digitalWrite(ENABLE_MOISTURE_CAPACITIVE, LOW);
  digitalWrite(ENABLE_MOISTURE_RESISTIVE, LOW);
  digitalWrite(ENABLE_DALLAS, LOW);
}

void deep_sleep() {
  sensors_off();

  esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void setup() {
  byte mac_address[6];

  //  delay(1000);

  Serial.begin(115200);
#ifdef VERBOSE
  Serial.println("Hello World!");
  Serial.println(bootCount);
#endif

  bootCount++;
  sensors_on();

  wifiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
  wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);
  wifiMulti.addAP(WIFI_SSID3, WIFI_PASSWORD3);

  WiFi.macAddress(mac_address);
  snprintf(hostname, sizeof(hostname), "%s-%02x%02x%02x", DIRTBALL_HOSTNAME, (int)mac_address[3], (int)mac_address[4], (int)mac_address[5]);
#ifdef VERBOSE
  Serial.printf("Hostname is %s\n", hostname);
#endif

  WiFi.setHostname(hostname);
  wifiMulti.run();

#ifdef VERBOSE
  Serial.println("[wifi]");
#endif

  bme280.begin();
#ifdef VERBOSE
  Serial.println("[bme280]");
#endif

  tsl2561.begin();
#ifdef VERBOSE
  Serial.println("[tsl2561]");
#endif

  sensors.begin();
#ifdef VERBOSE
  Serial.println("[dallas]");
#endif

  moisture_cap.begin(MOISTURE_CAP_CAL_DRY, MOISTURE_CAP_CAL_WET);
#ifdef VERBOSE
  Serial.println("[capacitive moisture]");
#endif

  moisture_res.begin(MOISTURE_RES_CAL_DRY, MOISTURE_RES_CAL_WET);
#ifdef VERBOSE
  Serial.println("[resistive moisture]");
#endif

  bme280.handle();

  int temperature = 0, humidity = 0, pressure = 0;
  int dirt_temp = 0;
  uint8_t cap_moist = 0, res_moist = 0;
  uint16_t cap_moist_raw = 0, res_moist_raw = 0;
  uint32_t lux = 0, ir = 0, full = 0;

  mqtt_client.loop();

  bme280.handle();
  temperature = bme280.temperature();
  humidity = bme280.humidity();
  pressure = bme280.pressure();

  tsl2561.handle();
  lux = tsl2561.lux();
  ir = tsl2561.ir();
  full = tsl2561.full();

  // dry, cap_moist is 3263, 387, 3250, 3313
  // wet, cap_moist is 1854, 1898, 1878, 1849, 1846
  cap_moist = moisture_cap.read();
  cap_moist_raw = moisture_cap.read_raw();

  // dry, res_moist is 0
  // wet, res_moist is 2819, 2911, 2958, 3143, 3025, 2901
  res_moist = moisture_res.read();
  res_moist_raw = moisture_res.read_raw();

  mqtt_client.loop();

  sensors.requestTemperatures();
  dirt_temp = sensors.getTempC(sensor1);

#ifdef VERBOSE
    Serial.printf("Dallas %d\n", dirt_temp);

    Serial.printf("Temperature %d\n", temperature);
    Serial.printf("Pressure %d\n", pressure);
    Serial.printf("Humidity %d\n", humidity);

    Serial.printf("Cap %d\n", cap_moist);
    Serial.printf("Res %d\n", res_moist);
#endif

  #define LG_BUFFER_LEN 600
  char buffer[LG_BUFFER_LEN];
  #define SM_BUFFER_LEN 200
  char sm_buffer[SM_BUFFER_LEN];

  snprintf(buffer, LG_BUFFER_LEN, "{ \"id\": \"%s\", ", MQTT_UUID);

  snprintf(sm_buffer, SM_BUFFER_LEN, "\"system\": {\"hostname\": \"%s\", \"freeheap\": %d, \"reboots\": %d, \"wifi_failures\": %d }, ", hostname, ESP.getFreeHeap(), bootCount, wifi_failures);
  strncat(buffer, sm_buffer, LG_BUFFER_LEN);
  
  snprintf(sm_buffer, SM_BUFFER_LEN, "\"environment\": { \"temperature\": %d, \"humidity\": %d, \"pressure\": %d }, ", temperature, humidity, pressure);
  strncat(buffer, sm_buffer, LG_BUFFER_LEN);

  snprintf(sm_buffer, SM_BUFFER_LEN, "\"light\": { \"lux\": %d, \"ir\": %d, \"full\": %d }, ", lux, ir, full);
  strncat(buffer, sm_buffer, LG_BUFFER_LEN);

  snprintf(sm_buffer, 200, "\"soil\": { \"moisture_cap\": %d, \"moisture_cap_raw\": %d, \"moisture_res\": %d, \"moisture_res_raw\": %d, \"temperature\": %d }", cap_moist, cap_moist_raw, res_moist, res_moist_raw, dirt_temp);
  strncat(buffer, sm_buffer, LG_BUFFER_LEN);

  strncat(buffer, "}", LG_BUFFER_LEN);

#ifdef VERBOSE
  Serial.println(buffer);
#endif

  unsigned long wifi_timeout = millis() + 1000;
  while(wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(50);

    if(millis() > wifi_timeout) {
      wifi_failures++;
      deep_sleep();
    }

  }

  mqtt_client.connect(MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);

  wifi_timeout = millis() + 1000;
  while(!mqtt_client.connected()) {
    Serial.print('+');
    delay(50);

    if(millis() > wifi_timeout) {
      wifi_failures++;

      deep_sleep();
    }
  }

#ifdef VERBOSE
  Serial.println("[MQTT]");
#endif

  mqtt_client.loop();

  mqtt_client.publish("/dirtball", buffer, true);

  mqtt_client.loop();

  delay(250);

  mqtt_client.loop();

  deep_sleep();
}


void loop() {
}
