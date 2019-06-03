# Dirtball - Sensors for your garden

Dirtball is a [Furball](https://github.com/HomeBusProjects/furball)-variant which helps monitor gardens. It includes environmental sensors, a light sensor, and soil moisture and temperature sensors.

Dirtball is intended to be battery powered, so it spends most of its time in deep sleep and wakes up periodically to report in. You can fine tune the sleep time in config.h. The more often it wakes up, the  shorter its battery life.

## Hardware

- [optional] BME280 or BME680 - air temperature, humidity, pressure; BME680 adds VOCs for air quality (code currently only supports BME280)
- reistive soil moisture sensor
- capacitive soil moisture sensor
- Dallas 1 wire waterproof temperature sensor
- TSL2561 light sensor or TSL2591 high dynamic range light sensor (code currently only supports TSL2561)
- VEML6075 UVA/UVB/UV Index sensor

## Soil Moisture Sensors

Dirtball supports resistive and capacitive soil moisture sensors. Both are analog devices. They need power, ground and have an analog signal return. Both need to be calibrated.

Resistive sensors will read 0 when perfectly dry and a higher number when wet.

Capactive sensors read higher when dry and lower when wet.

To calibrate, take several readings on the sensors when perfectly dry. Take several readings when dipped in water.

## Power

The biggest thing we can do to save power is deep sleep. Dirtball will spend most of its time in deep sleep. When it wakes from deep sleep it's like it was powered on, except that it the RTC static RAM will still be valid and some data can be stored there.

Currently the software:
- spends most of its time in deep sleep
- powers on the moisture sensors and soil temperature sensor only when taking readings
- only posts to a single MQTT broker

Left to do:

- Static IP support - saves needing DHCP
- IP address for MQTT server - saves needing a DNS query
- turn off BME280
- turn off TSL2561/TSL2591
- turn off VEML6075
- look at reducing the time connected to wifi
- identify an ESP32 breakout board with a more efficient voltage regulator and serial chip
- move to a complete custom surface mount design to eliminate redundancies in the breakout boards and give us control of the voltage regulator, LiPo charging circuit and serial circuit

## Case

The case is currently left as a challenge for the reader.

Given that it will be used in wet areas like garden beds, Dirtball would like to be in a waterproof case. However, if you're using a BMEx80 sensor in it, the sensor will also need airflow. And the case will need to allow unfiltred light to reach the light sensor, if you're using that.


## Capacitive soil moisture sensor

https://wiki.dfrobot.com/Capacitive_Soil_Moisture_Sensor_SKU_SEN0193#target_0

## License

Dirtball's software is licensed under the [MIT license](https://romkey.mit-license.org). Hardware is open source.
