// Copyright (c) Microsoft Corporation. All rights reserved.
// SPDX-License-Identifier: MIT

// Wifi
#define IOT_CONFIG_WIFI_SSID              "Rasmus iPhone"
#define IOT_CONFIG_WIFI_PASSWORD          "12345678"

//// Removed any DHT any Espressif ESP32 Azure IoT Kit sensor code ////

// Removed code including X.509 certificates for device authentication since it not being used

// Azure IoT Central
#define DPS_ID_SCOPE                      "0ne00C56874"
#define IOT_CONFIG_DEVICE_ID              "2iaa9zx3v42"
#define IOT_CONFIG_DEVICE_KEY             "pBgacnYR7FPXggIiFiQWcEx+M0gUk1NaYGLl60UoU2I="

// User-agent (url-encoded) provided by the MQTT client to Azure IoT Services.
// When developing for your own Arduino-based platform,
// please update the suffix with the format '(ard;<platform>)' as an url-encoded string.
#define AZURE_SDK_CLIENT_USER_AGENT       "c%2F" AZ_SDK_VERSION_STRING "(ard%3Besp32)"

// Publish 1 message every 2 seconds.
#define TELEMETRY_FREQUENCY_IN_SECONDS    2

// For how long the MQTT password (SAS token) is valid, in minutes.
// After that, the sample automatically generates a new password and re-connects.
#define MQTT_PASSWORD_LIFETIME_IN_MINUTES 60
