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

// NRF intialisation and variables
RF24 radio(4,5); // (CE, CSN)
String rxString;
String txString;
int channel = 0x76;
long test;
//long pipe = 0xF0F0F0F0E1LL;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;



// HTTP variables
int postResponseCode;


// Initialsing variables for extracting and processing JSON data

String url;
String name;
String isPlaying;
String rxVolume; // should be changed int16_t
String deviceMode;

bool flag = true;


// Protoype functions
//String httpGETRequest(const char* serverName);
//int httpPost(String payload);
//void connectToWiFi(void);
//String processBatteryLevel(void);

void setup() {
  Serial.begin(115200);

  // Setting up nRF radio
  radio.begin();
  radio.setChannel(channel);
  radio.setDataRate(RF24_1MBPS);  // set to 1 Mb/s
  radio.setPALevel(RF24_PA_HIGH); // Set high
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  radio.openReadingPipe(1,0xF0F0F0F0E1LL);
  radio.startListening();
}

void loop() {

  if(radio.available()){
    char rxBuffer[100] = "";
    radio.read(&rxBuffer,sizeof(rxBuffer));
    Serial.print("Recieved nRF string is: ");
    Serial.println(rxBuffer);

    rxString = String(rxBuffer);
    
    if(rxString.indexOf("url") >= 0){
        if(rxString != url){
            url = rxString;
        }
    } else if(rxString.indexOf("isPlaying") >= 0){
        if(rxString != url){
            isPlaying = rxString;
        }
    } else if(rxString.indexOf("volume") >= 0){
        if(rxString != rxVolume){
            rxVolume = rxString; // need to be adjustet for interger
        }
    } else if(rxString.indexOf("deviceMode") >= 0){
        if(rxString != deviceMode){
            deviceMode = rxString;
        }
    }
  }

  //test values
  char txBuffer[32] = "";
  char intBuffer[2] = ""; 
  bool playstate = true;
  int8_t deviceMode = 1;
  int8_t batteryData = 75;
  int8_t volume = 39;

  memset(txBuffer, 0, sizeof(txBuffer));

  // Add strings to txBuffer
  strcat(txBuffer, "ps: ");   //Playstate
  if (playstate){
    strcat(txBuffer, "1");
  } else {
    strcat(txBuffer, "0");
  }
  strcat(txBuffer, ",");

  itoa(deviceMode, intBuffer, 10);
  strcat(txBuffer, "dM: ");   // Device mode
  strcat(txBuffer, intBuffer);
  strcat(txBuffer, ",");

  itoa(batteryData, intBuffer, 10);
  strcat(txBuffer, "bD: ");     // Battery Data
  strcat(txBuffer, intBuffer);
  strcat(txBuffer, ",");

  itoa(volume, intBuffer, 10);
  strcat(txBuffer, "vol: "); //
  strcat(txBuffer, intBuffer);

  // Print the contents of txBuffer to Serial monitor
  Serial.println(txBuffer);

  delay(1000); // Wait for a second
  if(flag){

  }

}
