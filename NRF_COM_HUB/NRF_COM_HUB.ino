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

// NRF intialisation and variables
RF24 radio(17,16); // (CE, CSN)
int channel = 0x76;


#define PIN_BATT 39

//Domain name with URL path or IP address with path
const char* getServerName = "https://internetradioapi.azurewebsites.net/radio/play";
const char* postServerName = "https://internetradioapi.azurewebsites.net/radio/battery";
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
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
int httpPost(String payload);
void connectToWiFi(void);
String processBatteryLevel(void);


void setup() {
  Serial.begin(115200);
  
  connectToWiFi();

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
  
  // Setting up nRF radio
  radio.begin();
  radio.setChannel(channel);
  radio.setDataRate(RF24_1MBPS);  // set to 1 Mb/s
  radio.setPALevel(RF24_PA_MAX); // Set high
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  radio.openReadingPipe(1,0xF0F0F0F0E1LL);
}

void loop() {

  if (radio.available()) {

  }
  
  //Send an HTTP GET request every 5 seconds
  if ((millis() - lastTime) > timerDelay) {
    

    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
              
      // Getting data from web server
      serverReadings = httpGETRequest(getServerName);
      JSONVar serverReadingsJSON = JSON.parse(serverReadings);
  
      // Checking the format of the JSONVAR
      if (JSON.typeof(serverReadingsJSON) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }

      Serial.print("JSON object = ");
      Serial.println(serverReadingsJSON);
      Serial.println();

      RadioURL = serverReadingsJSON[0]["url"];
      RadioIsPlaying = serverReadingsJSON[0]["isPlaying"];
      RadioVolume = serverReadingsJSON[0]["volume"];

      urlString = String(RadioURL);
      isPlayingString =  String(RadioIsPlaying);
      volumeString = String(RadioVolume);
      //Add markers to the strings for comparison
      urlString = "UL=" + urlString + "=UL";
      isPlayingString = "PS=" + isPlayingString + "=PS";
      volumeString = "VO=" + volumeString + "=VO";

      if (urlString != oldUrlString) {
        Serial.println("New URL detected");
        oldUrlString = urlString;
        Serial.print("RadioURL: ");
        Serial.println(oldUrlString);
        // Calculate the number of parts
        int numParts = (oldUrlString.length() / 32) + 1;

        // Split the string into parts and send each part
        for (int i = 0; i < numParts; i++) {
            // Get the current part
            String part = oldUrlString.substring(i * 32, (i + 1) * 32);

            // Convert the part to a character array
            char partCharArray[part.length() + 1];
            part.toCharArray(partCharArray, sizeof(partCharArray));

            // Send the part
            radio.stopListening();
            radio.write(&partCharArray, sizeof(partCharArray));
            radio.startListening();
        }

      } 
      if (isPlayingString != oldIsPlayingString) {
        Serial.println("New isPlaying detected");
        oldIsPlayingString = isPlayingString;
        Serial.print("RadioIsPlaying: ");
        Serial.println(oldIsPlayingString);

        // Send the isPlayingString
        radio.stopListening();
        radio.write(&oldIsPlayingString, sizeof(oldIsPlayingString));
        radio.startListening();

      } 
      if (volumeString != oldVolumeString) {
        Serial.println("New volume detected");
        oldVolumeString = volumeString;
        Serial.print("RadioVolume: ");
        Serial.println(oldVolumeString);

        // Send the volumeString
        radio.stopListening();
        radio.write(&oldVolumeString, sizeof(oldVolumeString));
        radio.startListening();
      } 
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

  return payload;
}

int httpPostRequest(const char* serverName, String payload){
    HTTPClient httpPost;

    // Your Domain name with URL path or IP address with path
    httpPost.begin(serverName);

    httpPost.addHeader("Content-Type", "application/json");

    int httpResponseCode = httpPost.POST(payload);

    Serial.print("HTTP Post response code: ");
    Serial.println(httpResponseCode);

    // Free resources
    httpPost.end();

    return httpResponseCode;

}

void connectToWiFi(void){

  WiFi.begin(WifiCredentials::SSID, WifiCredentials::PASSWORD);
  Serial.println("Connecting to WiFi");

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}


String processBatteryLevel(void){

  String levelString = "{\"level\":\"XX\"}";        // initialising generic level string
  float voltage = analogRead(PIN_BATT)*3.3/4096*6;  // Reading an calculating voltage level
  int8_t percent = (voltage/12.4)*100;              // Converting to percent
  levelString.replace("XX", (String) percent);      // Adding the percent to the levelString

  return levelString;
}

