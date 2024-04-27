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
# define PIN_BATT 39

const char* ssid = "Rasmus iPhone";
const char* password = "12345678";

//Domain name with URL path or IP address with path
const char* ServerName = "https://internetradioapi.azurewebsites.net/radio/play";
const char* postServerName = "https://internetradioapi.azurewebsites.net/radio/battery";
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
String musicReadings;
String oldUrlString;
String oldNameString;
String urlString;
String nameString;
String islayingString;
String volumeString;
const char* url;
const char* name;
const char* isplaying;
const char* volume;

// Initialsing variables for battery
String batteryData;


// Protoype functions
String httpGETRequest(const char* serverName);
int httpPost(String payload);
void connectToWiFi(void);
String processBatteryLevel(void);


void setup() {
  Serial.begin(115200);
  
  connectToWiFi();

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
  
}

void loop() {
  //Send an HTTP GET request every 5 seconds
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
              
      // Getting data from web server
      musicReadings = httpGETRequest(ServerName);
      JSONVar data = JSON.parse(musicReadings);
  
      // Checcking the format of the JSONVAR
      if (JSON.typeof(data) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }

      Serial.print("JSON object = ");
      Serial.println(data);
      Serial.println();

      // Extract values
      url = data[0]["url"];
      name = data[0]["name"];
      isplaying = data[0]["isPlaying"];
      volume = data[0]["volume"];

      
      // Convert to strings
      urlString = String(url);
      nameString = String(name);

      // Checking if there is any change in url and/or name
      if (urlString != oldUrlString){
        // New URL detected
        oldUrlString = urlString;
        oldNameString = nameString;
        Serial.println("New URL detected");

      } else if (nameString != oldNameString){
        // New name detected
        oldNameString = nameString;
        Serial.println("New name detected");

      } else {
        // Old URL
        Serial.println("No change in URL or name");
      }

      // Printing URL
      Serial.print("URL: ");
      Serial.println(oldUrlString);
      Serial.print("Name: ");
      Serial.println(oldNameString);

      Serial.print("isplaying: ");
      Serial.println(isplaying);
      Serial.print("Volume: ");
      Serial.println(volume);
      
      // Use oldUrlString and oldNameString for playing music

      //Rasmus has deleted JSON stuff with sensorReadingsArr

      // HTTP Post stuff
      batteryData = processBatteryLevel();
      Serial.println(batteryData);
      postResponseCode = httpPost(batteryData);

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

int httpPost(String payload){
  HTTPClient httpPost;
    
      // Your Domain name with URL path or IP address with path
      httpPost.begin(postServerName);
      
      httpPost.addHeader("Content-Type", "application/json");

      int httpResponseCode = httpPost.POST(payload);
     
      Serial.print("HTTP Post response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      httpPost.end();

      return httpResponseCode;

}

void connectToWiFi(void){

  WiFi.begin(ssid, password);
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
