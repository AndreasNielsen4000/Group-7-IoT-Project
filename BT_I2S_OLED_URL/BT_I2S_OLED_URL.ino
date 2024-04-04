#include <M5StickCPlus.h>
#include "BluetoothA2DPSink.h"
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#include "WifiCredentials.h"
//#include <WiFi.h>
#include "Audio.h"
#include <EEPROM.h>

#define I2S_BCK_PIN 13
#define I2S_WS_PIN 27
#define I2S_DOUT_PIN 14
#define SAMPLE_RATE 44100
#define BITS 16
#define CHANNELS 2

struct Metadata {
  char title[100];
  char artist[100];
  char album[100];
};

const char* deviceName = "BT-WiFi-I2S-OLED";

const uint8_t volumeMax = 21;

//128x64 display

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//TEST STATIONS:
/** Web radio stream URLs */
const String stationURLs[] = {
    "http://streams.radiobob.de/bob-national/mp3-192/streams.radiobob.de/",
    "http://stream.rockantenne.de/rockantenne/stream/mp3",
    "http://wdr-wdr2-ruhrgebiet.icecast.wdr.de/wdr/wdr2/ruhrgebiet/mp3/128/stream.mp3",
    "http://streams.br.de/bayern3_2.m3u",
    "http://play.antenne.de/antenne.m3u",
    "http://funkhaus-ingolstadt.stream24.net/radio-in.mp3"
};

/** Number of stations */
const uint8_t numStations = sizeof(stationURLs) / sizeof(stationURLs[0]);

Audio *pAudio_ = nullptr;
TaskHandle_t pAudioTask_ = nullptr;

Metadata metadata;

/**
 * Instance of the 'BluetoothA2DPSink' class from the 'ESP32-A2DP' library.
 * Using a pointer and dynamic creation of the instance causes the ESP32 to crash when a2dp_.start() is called.
 */ 
BluetoothA2DPSink a2dp_;

// Enumeration with possible device modes
enum DeviceMode {NONE = 0, RADIO = 1, A2DP = 2};

typedef enum DeviceMode t_DeviceMode;

// Current device mode (initialization as 'RADIO')
t_DeviceMode deviceMode_ = RADIO;

// Content in audio buffer (provided by esp32-audioI2S library)
uint32_t audioBufferFilled_ = 0;

// Size of audio buffer (provided by esp32-audioI2S library)
uint32_t audioBufferSize_ = 0;

// Current station index
uint8_t stationIndex_ = 0;

// Flag to indicate the user wants to changed the station
bool stationChanged_ = true;

// Flag to indicate that audio is muted after tuning to a new station
bool stationChangedMute_ = true;

// Name of the current station as provided by the stream header data
String stationStr_ = "";

// Flag indicating the station name has changed
bool stationUpdatedFlag_ = false;

// Flag indicating that the connection to a host could not be established
bool connectionError_ = false;

// Info about current song as provided by the stream meta data or from AVRC data
String infoStr_ = "";

// Song artist provided by AVRC data (bluetooth)
String artistStr_ = "";

// Song title provided by AVRC data (bluetooth)
String titleStr_ = "";

// Flag indicating the song title has changed
bool infoUpdatedFlag_ = false;

// Position of the song title sprite on the screen (used for scrolling)
int16_t titlePosX_ = SCREEN_WIDTH;

// Audio volume to be set by the audio task
uint8_t volumeCurrent_ = 0;

// Volume as float value for fading
float_t volumeCurrentF_ = 0.0f;

// Flag indicating the volume needs to be set by the audio task
bool volumeCurrentChangedFlag_ = true;

// Audio volume that is set during normal operation
uint8_t volumeNormal_ = volumeMax;

// Time in milliseconds at which the connection to the chosen stream has been established
uint64_t timeConnect_ = 0;

int16_t titleTextWidth_ = 0;

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  //Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
  if (data1 == 0x1) {
    strncpy(metadata.title, (char*)data2, sizeof(metadata.title) - 1);
  } else if (data1 == 0x2) {
    strncpy(metadata.artist, (char*)data2, sizeof(metadata.artist) - 1);
  } else if (data1 == 0x3) {
    strncpy(metadata.album, (char*)data2, sizeof(metadata.album) - 1);
  }
  infoUpdatedFlag_ = true;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE,0);

  display.setCursor(0,10);
  display.print(metadata.title);
  
  display.setCursor(0,20);
  display.print(metadata.artist);

  display.setCursor(0,30);
  display.print(metadata.album);

  //display.display();
}


// Forward declaration of the volume change callback in bluetooth sink mode
void avrc_volume_change_callback(int vol);

// Forward declaration of the connection state change callback in bluetooth sink mode
void a2dp_connection_state_changed(esp_a2d_connection_state_t state, void*);

/**
 * Shows a welcome message at startup of the device on the TFT display.
 */
void showWelcomeMessage() {
    // Show some information on the startup screen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE,0);
    display.setCursor(0,25);
    display.print(deviceName);
    display.display();
}

/**
 * Displays the current station name contained in 'stationStr_' on the TFT screen.
 */
void showStation() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE,0);
    display.setCursor(0,10);
    if (deviceMode_ == RADIO) {
        display.print("Station: ");
    }
    else {
        display.print("Bluetooth");
    }
    display.setCursor(0,20);
    display.print(stationStr_);
    display.display();
}

/**
 * Displays the current song information contained in 'infoStr_' on the TFT screen.
 * Each time the song info is updated, it starts scrolling from the right edge.
 */
void showSongInfo() {
    // Update the song title if flag is raised
    if (infoUpdatedFlag_) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE,0);
        display.setCursor(128,10);
        display.print(infoStr_);
        infoUpdatedFlag_ = false; // Clear update flag
        display.display();
    }
    else {
        //Scroll the song title from right to left
        titlePosX_ -= 6; // Move the sprite to the left by 6 pixels

        //After the sprite has passed by completely...
        if (titlePosX_ < titleTextWidth_) {
            titlePosX_ = SCREEN_WIDTH; // ...let the sprite start again at the right side of the screen 
        }
        display.setCursor(titlePosX_,10);
        display.print(infoStr_);
        display.display();        
    }
}

/**
 * Displays the volume on the TFT screen.
 * 
 * @param volume Volume to be displayed on the TFT screen.
 */
void showVolume(uint8_t volume) {
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE,0);
    display.setCursor(0,30);
    display.print("Volume: ");
    display.print(volume);
    display.display();
}

void showPlayState(bool isPlaying) {
    display.setTextSize(1);
    display.setCursor(0,0);

    if (isPlaying) {
        display.print("Playing");
    }
    else {
        display.print("Stopped");
    }
    display.display();
}

/**
 * Connects to the specified WiFi network and starts the device in internet radio mode.
 * Audio task is started.
 */
void startRadio() {
    //Serial.print("Begin: free heap = %d, max alloc heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
    if (pAudio_ == nullptr) {
        display.setCursor(0,10);
        display.print(" MAC: "); // Own network mac address
        display.setCursor(0,20);
        display.print(WiFi.macAddress().c_str());
        Serial.println("MAC: ");
        Serial.print(WiFi.macAddress().c_str());
        display.setCursor(0,30);
        display.println(" Connecting to WiFi...");
        display.setCursor(0,40);
        display.printf(" SSID: "); // WiFi network name
        display.setCursor(0,50);
        display.print(WifiCredentials::SSID);
        Serial.println("Connecting to:");
        Serial.println(WifiCredentials::SSID);
        display.display();

        // Initialize WiFi and connect to network
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(deviceName);
        WiFi.begin(WifiCredentials::SSID, WifiCredentials::PASSWORD);
        Serial.print("WiFi Connecting");
        while (!WiFi.isConnected()) {
            delay(100);
            Serial.print("...");
        }

        // Display own IP address after connecting
        display.clearDisplay();
        display.setCursor(0,10);
        display.println(" Connected to WiFi");
        display.setCursor(0,20);
        display.printf(" IP: %s", WiFi.localIP().toString().c_str());
        Serial.print("\nIP:");
        Serial.print(WiFi.localIP().toString().c_str());
        display.display();
        pAudio_ = new Audio(false); // Use external DAC

        // Setup audio
        pAudio_->setVolume(0); // 0...21
        pAudio_->setPinout(I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);

        deviceMode_ = RADIO;

        // Start the audio processing task
        xTaskCreate(audioProcessing, "Audio processing task", 4096, nullptr, configMAX_PRIORITIES - 4, &pAudioTask_);

        // Wait some time before wiping out the startup screen
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        display.clearDisplay();
    }
    else {
        log_w("'pAudio_' not cleaned up!");
    }

    //Serial.print("End: free heap = %d, max alloc heap = %d, min free heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
}

/**
 * Stops the internet radio including the audio tasks.
 */
void stopRadio() {
    //Serial.print("Begin : free heap = %d, max alloc heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

    if (pAudio_ != nullptr) {
        deviceMode_ = NONE;
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if (pAudioTask_ != nullptr) {
            vTaskDelete(pAudioTask_);
            pAudioTask_ = nullptr;
        }
        else {
            log_w("Cannot clean up 'pAudioTask_'!");
        }

        pAudio_->stopSong();

        delete pAudio_;

        pAudio_ = nullptr;

        // Set variables to default values
        audioBufferFilled_ = 0;
        audioBufferSize_ = 0;
        // stationIndex_ = 0;
        stationChanged_ = true;
        stationChangedMute_ = true;
        String stationStr_ = "";
        stationUpdatedFlag_ = false;
        connectionError_ = false;
        infoStr_ = "";
        infoUpdatedFlag_ = false;
        titleTextWidth_ = 0;
        titlePosX_ = SCREEN_WIDTH;
        volumeCurrent_ = 0;
        volumeCurrentF_ = 0.0f;
        volumeCurrentChangedFlag_ = true;

        display.clearDisplay();
    }
    else {
        log_w("Cannot clean up 'pAudio_'!");
    }

    //Serial.print("End: free heap = %d, max alloc heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
}

/**
 * Starts the device in bluetooth sink (A2DP) mode.
 */
void startA2dp() {
    //Serial.print("Begin: free heap = %d, max alloc heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

    i2s_pin_config_t pinConfig = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    a2dp_.set_pin_config(pinConfig);

    a2dp_.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
    a2dp_.set_avrc_metadata_callback(avrc_metadata_callback);
    //a2dp_.set_on_connection_state_changed(a2dp_connection_state_changed);
    //a2dp_.set_on_volumechange(avrc_volume_change_callback);

    //showWelcomeMessage();
    display.setCursor(0,10);
    display.print(" Starting bluetooth");

    a2dp_.start(deviceName);
    deviceMode_ = A2DP;
    
    esp_bt_controller_status_t btStatus = esp_bt_controller_get_status();

    if (btStatus == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        display.setCursor(0,20);
        display.print(" Bluetooth enabled");
        display.display();
    }
    else {
        display.setCursor(0,20);
        display.print(" Bluetooth not enabled");
        display.setCursor(0,30);
        display.print(" Status: ");
        display.println(btStatus);
        display.display();
    }

    stationStr_ = "Bluetooth";
    stationUpdatedFlag_ = true;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //display.clearDisplay();

    //Serial.print("End: free heap = %d, max alloc heap = %d, min free heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
}

/**
 * Currently the API does not support stopping the A2DP mode.
 */
void stopA2dp() {
    //log_w("Not possible to stop and cleanup 'a2dp_'!");
    return;
}

/**
 * Enable or disable the shutdown circuit of the amplifier.
 * Amplifier: M5Stack SPK hat with PAM8303.
 * - b = true  --> GPIO_0 = 0 : Shutdown enabled
 * - b = false --> GPIO_0 = 1 : Shutdown disabled
 */
void setAudioShutdown(bool b) {
    /*
    if (b) {
        gpio_set_level(GPIO_NUM_0, 0); // Enable shutdown circuit
    }
    else {
        gpio_set_level(GPIO_NUM_0, 1); // Disable shutdown circuit
    }
    */
}

void audioProcessing(void *p) {
    while (true) {
        if (deviceMode_ != RADIO) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }

        // Process requested change of audio volume
        if (volumeCurrentChangedFlag_) {
            pAudio_->setVolume(volumeCurrent_);
            volumeCurrentChangedFlag_ = false; // Clear flag
        }
        
        // Proces requested station change
        if (stationChanged_) {
            pAudio_->stopSong();
            setAudioShutdown(true); // Turn off amplifier
            stationChangedMute_ = true; // Mute audio until stream becomes stable

            // Establish HTTP connection to requested stream URL
            const char *streamUrl = stationURLs[stationIndex_].c_str();

            bool success = pAudio_->connecttohost( streamUrl );

            if (success) {
                stationChanged_ = false; // Clear flag
                connectionError_ = false; // Clear in case a connection error occured before

                timeConnect_ = millis(); // Store time in order to detect stream errors after connecting
            }
            else {
                stationChanged_ = false; // Clear flag
                connectionError_ = true; // Raise connection error flag
            }

            // Update buffer state variables
            audioBufferFilled_ = pAudio_->inBufferFilled(); // 0 after connecting
            audioBufferSize_ = pAudio_->inBufferFree() + audioBufferFilled_;
        }

        // After the buffer has been filled up sufficiently enable audio output
        if (stationChangedMute_) {
            if ( audioBufferFilled_ > 0.9f * audioBufferSize_) {
                setAudioShutdown(false);
                stationChangedMute_ = false;
                connectionError_ = false;
            }
            else {
                // If the stream does not build up within a few seconds something is wrong with the connection
                if ( millis() - timeConnect_ > 3000 ) {
                    if (!connectionError_) {
                        //Serial.printf("Audio buffer low: %u of %u bytes.\n", audioBufferFilled_, audioBufferSize_);
                        connectionError_ = true; // Raise connection error flag
                    }
                }
            }
        }

        // Let 'esp32-audioI2S' library process the web radio stream data
        pAudio_->loop();

        audioBufferFilled_ = pAudio_->inBufferFilled(); // Update used buffer capacity
        
        vTaskDelay(1 / portTICK_PERIOD_MS); // Let other tasks execute
    }
}

// Button pin numbers
const int BUTTON_PIN_1 = 4;
const int BUTTON_PIN_2 = 5;
// Button states
bool buttonState_1 = false;
bool buttonState_2 = false;

void readButtonStates() {
  buttonState_1 = digitalRead(BUTTON_PIN_1) == LOW;
  buttonState_2 = digitalRead(BUTTON_PIN_2) == LOW;
}

void setup() {

    pinMode(BUTTON_PIN_1, INPUT_PULLUP);
    pinMode(BUTTON_PIN_2, INPUT_PULLUP);


    /*
    // Setup GPIO ports for SPK hat
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT); // Shutdown circuit of M5Stack SPK hat
    setAudioShutdown(true);

    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT_OD); // Configure GPIO_26 for DAC output (DAC_CHANNEL_2)
    //dac_output_enable(DAC_CHANNEL_2);
    //dac_output_voltage(DAC_CHANNEL_2, 0);
    */

    // Serial.print("IDF version = %s", ESP.getSdkVersion());
    // Serial.print("Total heap = %d", ESP.getHeapSize());
    // Serial.print("Free heap = %d", ESP.getFreeHeap());
    // Serial.print("Max alloc heap = %d", ESP.getMaxAllocHeap());

    Serial.begin(115200);

    if(!display.begin(0x3C,true)) { // Address 0x3C for 128x32
        Serial.println(F("Adafruit SH1106G allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    if ( EEPROM.begin(1) ) {
        uint8_t mode = EEPROM.readByte(0);

        //Serial.print("EEPROM.readByte(0) = %d", mode);

        if (mode == 2) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SH110X_WHITE,0);
          display.setCursor(0,25);
          display.print("I2S BT");
          display.display();
          startA2dp();
        }
        else {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SH110X_WHITE,0);
          display.setCursor(0,25);
          display.print("I2S URL");
          display.display();
            startRadio();
        }
    }
    else {
        //log_w("EEPROM.begin() returned 'false'!");
        startRadio();
    }


}

void loop() {
    
    readButtonStates();

    if (buttonState_1) {
        if (deviceMode_ == RADIO) {
            EEPROM.writeByte(0, 2); // Enter A2DP mode after restart
            EEPROM.commit();
            Serial.println("Switching to BT!");
            stopRadio(); // Close connections and clean up
        }
        else {
            EEPROM.writeByte(0, 1); // Enter internet radio mode after restart
            EEPROM.commit();
            Serial.println("Switching to Radio!");
        }
        ESP.restart();
    }

    if (deviceMode_ == RADIO) {

        // Button A: Switch to next station
        if (buttonState_2) {
            
            // Turn down volume
            volumeCurrent_ = 0;
            volumeCurrentF_ = 0.0f;
            volumeCurrentChangedFlag_ = true; // Raise flag for the audio task

            // Advance station index to next station
            stationIndex_ = (stationIndex_ + 1) % numStations;
            stationChanged_ = true; // Raise flag for the audio task

            // Erase station name
            stationStr_ = "";
            stationUpdatedFlag_ = true; // Raise flag for display update routine

            // Erase stream info
            infoStr_ = "";
            infoUpdatedFlag_ = true; // Raise flag for display update routine
        }
        else {
            // Increase volume gradually after station change
            if (!stationChangedMute_ && volumeCurrent_ < volumeNormal_) {
                volumeCurrentF_ += 0.25;
                volumeCurrent_ = (uint8_t) volumeCurrentF_;
                volumeCurrentChangedFlag_ = true; // Raise flag for the audio task

                showVolume(volumeCurrent_);
            }
        }

        if (connectionError_) {
            display.setCursor(0,10);
            display.println("Stream unavailable");
            vTaskDelay(200 / portTICK_PERIOD_MS); // Wait until next cycle
        }
        else {
            // Update the station name if flag is raised
            if (stationUpdatedFlag_) {
                showStation();
                stationUpdatedFlag_ = false; // Clear update flag
            }

            showSongInfo();
            vTaskDelay(20 / portTICK_PERIOD_MS); // Wait until next cycle
        }
    }
    else {
        if (deviceMode_ == A2DP) {
            // Update the station name if flag is raised
            if (stationUpdatedFlag_) {
                showStation();
                stationUpdatedFlag_ = false; // Clear update flag
            }

            showSongInfo();

            if (volumeCurrentChangedFlag_) {
                showVolume(volumeCurrent_);
            } 
            
            showPlayState(a2dp_.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED);
            vTaskDelay(20 / portTICK_PERIOD_MS); // Wait until next cycle
        }
        else {
            // Neither radio mode nor A2DP mode
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}


// optional
void audio_info(const char *info){
    //Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    // Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    // Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    stationStr_ = info;
    stationUpdatedFlag_ = true; // Raise flag for the display update routine

    // Serial.print("station     ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    if (deviceMode_ == RADIO) {
        infoStr_ = info;
    }
    else {
        std::string title(metadata.title);
        std::string artist(metadata.artist);
        infoStr_ = (title + " - " + artist).c_str();
        //infoStr_ = metadata.title + " - " + metadata.artist;
    }
    infoUpdatedFlag_ = true; // Raise flag for the display update routine

    // Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    // Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    // Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    // Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    // Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    // Serial.print("eof_speech  ");Serial.println(info);
}

// void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
//     switch (id) {
//         case ESP_AVRC_MD_ATTR_TITLE:
//             titleStr_ = (char*) text;
//             break;
        
//         case ESP_AVRC_MD_ATTR_ARTIST:
//             artistStr_ = (char*) text;
//             break;
//     }    

//     if ( artistStr_.isEmpty() ) {
//         infoStr_ = titleStr_;
//     }
//     else {
//         if ( titleStr_.isEmpty() ) {
//             infoStr_ = artistStr_;
//         }
//         else {
//             infoStr_ = artistStr_ + " - " + titleStr_;
//         }
//     }
    
//     infoUpdatedFlag_ = true; // Raise flag for the display update routine
//     // Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
// }

void a2dp_connection_state_changed(esp_a2d_connection_state_t state, void*) {

    //Serial.print("Connection state: %d", state);

    if (state != ESP_A2D_CONNECTION_STATE_CONNECTED) {
        infoStr_ = "not connected";
        infoUpdatedFlag_ = true; // Raise flag for the display update routine
    }
}

void avrc_volume_change_callback(int vol) {
    volumeCurrent_ = vol;
    volumeCurrentChangedFlag_ = true;
}