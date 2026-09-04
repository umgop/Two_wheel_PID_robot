/*
 * ESP32 MQTT bridge for the STM32 balance controller.
 *
 * ESP32 GPIO17 (TX2) -> STM32 PA3 (USART2_RX)
 * ESP32 GPIO16 (RX2) <- STM32 PA2 (USART2_TX)
 * ESP32 GND          -> STM32 GND
 *
 * Both boards use 3.3 V serial logic.  Install PubSubClient from the Arduino
 * Library Manager, copy secrets.example.h to secrets.h, then fill it in.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

constexpr int STM32_UART_RX_PIN = 16;
constexpr int STM32_UART_TX_PIN = 17;
constexpr long STM32_UART_BAUD = 115200;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
HardwareSerial robotSerial(2);

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void onMqttMessage(char *topic, uint8_t *payload, unsigned int length) {
  (void)topic;

  String command;
  command.reserve(length + 1U);
  for (unsigned int i = 0; i < length; ++i) {
    command += (char)payload[i];
  }
  command.trim();

  /* STM32 accepts: ENABLE, DISABLE, SETPOINT <degrees>, TURN <step-hz>. */
  if (command.length() > 0U) {
    robotSerial.println(command);
  }
}

void connectMqtt() {
  while (!mqtt.connected()) {
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      mqtt.subscribe(MQTT_COMMAND_TOPIC);
    } else {
      delay(1000);
    }
  }
}

void forwardRobotTelemetry() {
  static String line;

  while (robotSerial.available()) {
    const char character = (char)robotSerial.read();
    if (character == '\n') {
      line.trim();
      if (line.length() > 0U && mqtt.connected()) {
        mqtt.publish(MQTT_TELEMETRY_TOPIC, line.c_str());
      }
      line = "";
    } else if (character != '\r' && line.length() < 127U) {
      line += character;
    }
  }
}

void setup() {
  robotSerial.begin(STM32_UART_BAUD, SERIAL_8N1,
                    STM32_UART_RX_PIN, STM32_UART_TX_PIN);
  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }
  if (!mqtt.connected()) {
    connectMqtt();
  }

  mqtt.loop();
  forwardRobotTelemetry();
}
