/*
  Rui Santos
  Complete project details at Complete project details at https://RandomNerdTutorials.com/esp32-http-get-post-arduino/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "WifiCredentials.h"
#include <RF24Network.h>

// NRF intialisation and variables
RF24 radio(17,16); // (CE, CSN)
RF24Network network(radio);
int channel = 0x76;
const uint16_t this_node = 00;
const uint16_t other_node = 01;

#define PIN_BATT 39

enum WiFiState {
  DISCONNECTED,
  DISCONNECTING,
  CONNECTING,
  CONNECTED
};

WiFiState wifiState = DISCONNECTED;
unsigned long wifiConnectStart;

enum DeviceMode {RADIO = 0, A2DP = 1, CHG = 2, NONE = 3};

typedef enum DeviceMode t_DeviceMode;

//Domain name with URL path or IP address with path
const char* ServerName = "https://internetradioapi.azurewebsites.net/radio/play";
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
unsigned long lastPostTime = 2000; // When the last post was made
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;


// Initialsing variables for extracting and processing JSON data
String serverReadings;
String oldUrlString;
String oldIsPlayingString;
String oldVolumeString;
String urlString;
String isPlayingString;
String volumeString;


const char* RadioURL;
const char* RadioIsPlaying;
const char* RadioVolume;

// Protoype functions
String httpGETRequest(const char* serverName);
void httpPostRequest(const char* serverName, String payload);
void connectToWiFi(void);
String nrfCheck();
String postDataString(bool isPlaying, DeviceMode deviceMode, int8_t volume, uint8_t batterylevel);

#define MAX_URL_LENGTH 141 // Define the maximum length of the URL

struct nrfTransmitPayload_t {  // Structure of our payload
  char url[MAX_URL_LENGTH];
  bool isPlaying;
  int8_t volume;
  uint8_t paramBitMask = 0;
};

struct nrfReceivePayload_t
{
  bool isPlaying;
  DeviceMode deviceMode;
  int8_t volume;
  uint8_t batterylevel;
};


void setup() {
  Serial.begin(115200);
  
  connectToWiFi();

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
  
  // Setting up nRF radio
  radio.begin();
  radio.setChannel(channel);
  network.begin(this_node);
}

void loop() {
  network.update();
  String postWebData = nrfCheck();
  // Check WiFi connection status and reconnect if necessary // MÅSKE RYK DET UD AF IF MILLIS()
  connectToWiFi();
  if (postWebData != "") {
    if (millis() - lastPostTime >= 5000) { // 5 seconds have passed since the last post
      // Send data to web server every 5 seconds
      if (wifiState == CONNECTED) {
        httpPostRequest(ServerName, postWebData);
        lastPostTime = millis();
        delay(1000); // Delay of 1 second to allow the server to process the data 
      } else {
          Serial.println("WiFi Disconnected");
      }
    }
  }
  //Send an HTTP GET request every 5 seconds
  if ((millis() - lastTime) > timerDelay) {
    
    //Check WiFi connection status
    if (wifiState == CONNECTED) {
              
      // Getting data from web server
      serverReadings = httpGETRequest(ServerName);
      JSONVar serverReadingsJSON = JSON.parse(serverReadings);
  
      // Checking the format of the JSONVAR
      if (JSON.typeof(serverReadingsJSON) == "undefined" || JSON.stringify(serverReadingsJSON) == "{}") {
        Serial.println("Parsing input failed or received empty JSON object!");
        return;
      }

      Serial.print("JSON object = ");
      Serial.println(serverReadingsJSON);
      Serial.println();

      RadioURL = serverReadingsJSON[0]["url"];
      RadioIsPlaying = serverReadingsJSON[0]["isPlaying"];
      RadioVolume = serverReadingsJSON[0]["volume"];

      Serial.println("Extracted data from JSON object");
      Serial.println(RadioURL);
      Serial.println(RadioIsPlaying);
      Serial.println(RadioVolume);

      urlString = String(RadioURL);
      isPlayingString =  String(RadioIsPlaying);
      volumeString = String(RadioVolume);

      Serial.print("RadioURL: ");
      Serial.println(urlString);
      Serial.print("RadioIsPlaying: ");
      Serial.println(isPlayingString);
      Serial.print("RadioVolume: ");
      Serial.println(volumeString);

      if (urlString != oldUrlString || isPlayingString != oldIsPlayingString || volumeString != oldVolumeString) {
        uint8_t paramBitMask = 0; // Create a bitmask to keep track of which parameters have changed
        if (urlString != oldUrlString) {
            paramBitMask |= 1 << 0; // Set bit 0
        }
        if (isPlayingString != oldIsPlayingString) {
            paramBitMask |= 1 << 1; // Set bit 1
        }
        if (volumeString != oldVolumeString) {
            paramBitMask |= 1 << 2; // Set bit 2
        }
        Serial.println("New info detected");
        oldUrlString = urlString;
        oldIsPlayingString = isPlayingString;
        oldVolumeString = volumeString;
        Serial.print("RadioURL: ");
        Serial.println(oldUrlString);
        Serial.print("RadioIsPlaying: ");
        Serial.println(oldIsPlayingString);
        Serial.print("RadioVolume: ");
        Serial.println(oldVolumeString);
        Serial.print("ParamBitMask: ");
        Serial.println(paramBitMask, BIN);

        char oldUrlChar[oldUrlString.length() + 1];
        oldUrlString.toCharArray(oldUrlChar, oldUrlString.length() + 1);
        oldUrlChar[oldUrlString.length()] = '\0';

        // Convert oldIsPlayingString to boolean
        bool oldIsPlayingBool = (oldIsPlayingString == "true");

        // Convert oldVolumeString to int
        int oldVolumeInt = oldVolumeString.toInt();

        nrfTransmitPayload_t payload;
        strncpy(payload.url, oldUrlChar, MAX_URL_LENGTH); // copy oldUrlChar to payload.url
        payload.isPlaying = oldIsPlayingBool;
        payload.volume = oldVolumeInt;
        payload.paramBitMask = paramBitMask; // Add the bitmask to the payload
        RF24NetworkHeader header(other_node);
        bool ok = network.write(header, &payload, sizeof(payload));
        if (ok) {
          Serial.println("Message sent");
        } else {
          Serial.println("Message failed");
        }
      } 
      Serial.println("Nothing new detected");
    } else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

String httpGETRequest(const char* serverName) {
  // Rasmus has deleted the Wifi client from the example code
  HTTPClient httpGet;
    
  // Establishing connection the the server
  httpGet.begin(serverName); 
  
  // Send HTTP GET request
  int httpResponseCode = httpGet.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = httpGet.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  httpGet.end();
  delay(100); // Delay of 100ms

  return payload;
}

void httpPostRequest(const char* serverName, String payload){
    HTTPClient httpPost;

    // Your Domain name with URL path or IP address with path
    httpPost.begin(serverName);

    httpPost.addHeader("Content-Type", "application/json");

    Serial.print("Payload: ");
    Serial.println(payload);

    int httpResponseCode = httpPost.POST(payload);

    Serial.print("HTTP Post response code: ");
    Serial.println(httpResponseCode);

    // If the response code is less than zero, it's an error code
    if (httpResponseCode < 0) {
        Serial.print("HTTP Post error: ");
        Serial.println(httpPost.errorToString(httpResponseCode));
    }

    // Free resources
    httpPost.end();
    delay(100); // Delay of 100ms
}

void connectToWiFi(void){
  switch (wifiState) {
    case DISCONNECTED:
      WiFi.begin(WifiCredentials::SSID, WifiCredentials::PASSWORD);
      Serial.println("Connecting to WiFi");
      wifiConnectStart = millis();
      wifiState = CONNECTING;
      break;

    case CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.print("Connected to WiFi network with IP Address: ");
        Serial.println(WiFi.localIP());
        wifiState = CONNECTED;
        delay(500);
      } else if (millis() - wifiConnectStart > 5000) { // Try to connect for 5 seconds
        Serial.println("Failed to connect to WiFi");
        wifiState = DISCONNECTED;
      }
      break;

    case CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        wifiState = DISCONNECTING;
      }
      break;

    case DISCONNECTING:
      Serial.println("WiFi Disconnected. Attempting to Reconnect...");
      wifiState = DISCONNECTED;
      break;

    default:
      break;
  }
}

String postDataString(bool isPlaying, DeviceMode deviceMode, int8_t volume, uint8_t batterylevel) {
  String data = "{\"deviceisPlaying\":\"";
  data += isPlaying ? "true" : "false";
  data += "\",\"deviceDeviceMode\":\"";
  data += deviceMode;
  data += "\",\"deviceVolume\":\"";
  data += volume;
  data += "\",\"deviceBatteryLevel\":\"";
  data += batterylevel;
  data += "\"}";
  return data;
}

String nrfCheck() {
  String postWebServerData = "";
  while (network.available()) {
    RF24NetworkHeader header;
    nrfReceivePayload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print(F("Received packet: isPlaying="));
    Serial.println(payload.isPlaying);
    Serial.print(F(", deviceMode="));
    Serial.println(payload.deviceMode);
    Serial.print(F(", batterylevel="));
    Serial.println(payload.batterylevel);
    Serial.print(F(", volume="));
    Serial.println(payload.volume);
    postWebServerData = postDataString(payload.isPlaying, payload.deviceMode, payload.volume, payload.batterylevel);
  }
  return postWebServerData;
}