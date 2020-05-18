#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/wifi.h>
#include <multiball/ota_updates.h>
#include <multiball/homebus.h>

#include "dirtball.h"

MultiballApp App;

void setup() {
  const wifi_credential_t wifi_credentials[] = {
    { WIFI_SSID1, WIFI_PASSWORD1 },
    { WIFI_SSID2, WIFI_PASSWORD2 },
    { WIFI_SSID3, WIFI_PASSWORD3 }
  };

  delay(500);

  App.wifi_credentials(3, wifi_credentials);
  App.begin();
  Serial.println("[app]");

  homebus_configure("dirtball", "Back Yard", "Homebus", "v4");
  homebus_setup();

  //  indicator_begin();
  //  Serial.println("[indicator]");

  dirtball_setup();
  Serial.println("[dirtball]");

  delay(500);
}

void loop() {
  App.handle();

  dirtball_loop();
}
