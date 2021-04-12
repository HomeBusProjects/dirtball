#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/wifi.h>
#include <multiball/ota_updates.h>
#include <multiball/homebus.h>

#ifdef USE_DIAGNOSTICS
#include <diagnostics.h>
#endif

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

  //  indicator_begin();
  //  Serial.println("[indicator]");

  dirtball_setup();
  Serial.println("[dirtball]");

#ifdef USE_DIAGNOSTICS
  diagnostics_setup(APP_NAME);
  Serial.println("[diagnostics]");
#endif

  delay(500);
}

void loop() {
  App.handle();

  dirtball_loop();

#ifdef USE_DIAGNOSTICS
  diagnostics_loop(dirtball_stream);
#endif
}
