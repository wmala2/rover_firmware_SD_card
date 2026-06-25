#pragma once
#ifndef WIFI_SSID
  // #define WIFI_SSID "BadBunny"
  #define WIFI_SSID "BaleNet"
#endif
#ifndef WIFI_PASS
  // #define WIFI_PASS "DonnieDarko"
  #define WIFI_PASS "F1ockOfTurtle$"
#endif

#ifndef USE_STATIC_IP
  #define USE_STATIC_IP true
#endif

#include <WiFi.h>
extern IPAddress WS_local_IP;
extern IPAddress WS_gateway;
extern IPAddress WS_subnet;
extern IPAddress WS_dns1;
extern IPAddress WS_dns2;


