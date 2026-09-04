#pragma once

/* Copy this file to secrets.h and replace every placeholder locally.
 * secrets.h is ignored by Git. */

#define WIFI_SSID       "your-wifi-name"
#define WIFI_PASSWORD   "your-wifi-password"
#define MQTT_HOST       "192.168.1.10"
#define MQTT_PORT       1883
#define MQTT_USERNAME   ""
#define MQTT_PASSWORD   ""
#define MQTT_CLIENT_ID  "balancebot-bridge"

#define MQTT_COMMAND_TOPIC    "balancebot/command"
#define MQTT_TELEMETRY_TOPIC  "balancebot/telemetry"
