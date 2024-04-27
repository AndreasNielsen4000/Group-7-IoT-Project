#include "BluetoothA2DPSink.h"
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#include "WifiCredentials.h"
#include "Audio.h"
#include <EEPROM.h>
#define _TIMERINTERRUPT_LOGLEVEL_     3
#include "ESP32_New_TimerInterrupt.h"
//#include <HTTPClient.h> //https://randomnerdtutorials.com/esp32-http-get-post-arduino/
#include <RF24.h>
#include <RF24Network.h>


//NRF PINS
#define PIN_NRF_CE 17
#define PIN_NRF_CSN 16
#define PIN_NRF_IRQ 4
#define PIN_NRF_MISO 19
#define PIN_NRF_MOSI 23
#define PIN_NRF_SCK 18

//I2S PINS & INFO
#define I2S_BCK_PIN 13
#define I2S_WS_PIN 26
#define I2S_DOUT_PIN 25
#define SAMPLE_RATE 44100
#define BITS 16
#define CHANNELS 2

//LED & BUTTON PINS
#define PIN_LED 0
#define PIN_BTN 35
#define PIN_ENC_A 27
#define PIN_ENC_B 14
#define PIN_LED_R 32
#define PIN_LED_G 15
#define PIN_LED_B 5

//I2C PINS
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

//AMP & PSU PINS
#define PIN_AMP_EN 2
#define PIN_PSU_EN 33
#define PD_MUTE 12 //MARK: PD_MUTE - CHECK

#define PD_I2C_ADR  0x28   //0101000
#define PD_I2C_FREQ 100000 //kb/s

#define TIMER_0_INTERVAL 1000000 //Blink & Tick interval (µs)
#define TIMER_3_INTERVAL 50000 //Power button refresh rate (µs)

#define PWM_FREQ 5000 //Hz
#define PWM_CNL_R 0 //Channel (0-16)
#define PWM_CNL_G 1 
#define PWM_CNL_B 3 
#define PWM_RES 8 //Resoluion (0-16 Bit)

#define ENC_VEL 5 //Encoder velocity

#define SW_THRESHOLD 300

#define BTN_SINGLE_PRESS 20
// #define BTN_LONG_PRESS 25 

#define PIN_BATT 39
#define PIN_CHG 36

ESP32Timer Timer0(0); //4 timers are available (from 0 to 3)
ESP32Timer Timer3(3);

struct Metadata {
  char title[100];
  char artist[100];
  char album[100];
};

const char* deviceName = "FrankenRadio";

const uint8_t volumeMax = 127;

static volatile char ENC_COUNT = 0;
static volatile long tick = 0; //Seconds from power on
static volatile bool encA = false;
static volatile bool encB = false;

// /*Interrupt Handlers*/
bool IRAM_ATTR Timer0_ISR(void * timerNo);
bool IRAM_ATTR Timer3_ISR(void * timerNo);
void IRAM_ATTR encA_ISR();
void IRAM_ATTR encB_ISR();

//128x64 display

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Initialize NRF
RF24 nrfLink(PIN_NRF_CE, PIN_NRF_CSN);
int8_t channel = 0x76;
RF24Network nrfNetwork(nrfLink);
const uint16_t thisNode = 01;
const uint16_t otherNode = 00;


//TEST STATIONS:
/** Web radio stream URLs */
const String stationURLs[] = { //MARK: stationURLs
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
enum DeviceMode {WIFI = 0, BT = 1, CHG = 2, NONE = 3};

typedef enum DeviceMode t_DeviceMode;

// Current device mode (initialization as 'NONE')
t_DeviceMode deviceMode_ = NONE;

// Flag indicating that deviceMode has changed
bool deviceModeChanged_ = false;

// MOD_IN true if bluetooth mode
static volatile int8_t MOD_IN_ = 0x01; //start in WiFi mode after programming

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
int16_t volumeCurrent_ = 0;

// Volume as float value for fading
float_t volumeCurrentF_ = 0.0f;

// Flag indicating the volume needs to be set by the audio task
bool volumeCurrentChangedFlag_ = true;

// Playstate of the audio stream
bool isPlaying_ = false;

// Audio volume that is set during normal operation
uint8_t volumeNormal_ = 20;

// Time in milliseconds at which the connection to the chosen stream has been established
uint64_t timeConnect_ = 0;

int16_t titleTextWidth_ = 0;

uint16_t holdCounter_ = 0;
int8_t volumeChange_ = 0;

int8_t batteryLevel_ = 0;

bool display_present = false; //MARK: display_present

// Forward declaration of the volume change callback in bluetooth sink mode
void avrc_volume_change_callback(int vol);

// Forward declaration of the connection state change callback in bluetooth sink mode
void a2dp_connection_state_changed(esp_a2d_connection_state_t state, void*);

void avrc_metadata_callback(uint8_t id, const uint8_t *text);

bool IRAM_ATTR Timer0_ISR(void * timerNo){ //MARK: Timer0_ISR
  /*Blink */
  static bool ledToggle = 0;
  digitalWrite(PIN_LED,ledToggle);
  ledToggle=!ledToggle;
  /*Tick*/
  tick++;
  return true;
}

unsigned long previousMillisCHG = 0;
const long intervalCHG = 1000;

//nRFLink receive variables
String url_ = "";
String urlPlaceholder_ = "";
bool isReadingUrl_ = false;

#define MAX_URL_LENGTH 141 // Define the maximum length of the URL

struct nrfReceivePayload_t {
    char url[MAX_URL_LENGTH];
    bool isPlaying;
    int8_t volume;
    uint8_t paramBitMask; // 0b00000000 <- 00 unused 0/1 url 0/1 isPlaying 0/1 volume 0/1 deviceModeChanged 00/01/10/11 deviceMode
};

struct nrfTransmitPayload_t {
    bool isPlaying;
    DeviceMode deviceMode;
    int8_t volume;
    int8_t batteryLevel;
};

bool IRAM_ATTR Timer3_ISR(void * timerNo){ //MARK: Timer3_ISR
    /*Power logic*/ 
    bool BTN_IN = analogRead(PIN_BTN) >= SW_THRESHOLD;
    if (BTN_IN) {
        holdCounter_++;
    }
    
    switch (deviceMode_) {
    case WIFI:
        if (!BTN_IN && holdCounter_ >= BTN_SINGLE_PRESS) {
            deviceMode_ = BT;
            MOD_IN_ = (deviceMode_ & 0x03) | ((deviceMode_ == BT ? 1 : 0) << 2);
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        } else if (!BTN_IN && holdCounter_ > 1 && holdCounter_ < BTN_SINGLE_PRESS) {
            //Save current mode in the third bit of MOD_IN_
            DeviceMode temp = deviceMode_;
            if (digitalRead(PIN_CHG)) {
                temp = NONE;
            } else {
                temp = CHG;
            }
            MOD_IN_ = (temp & 0x03) | ((deviceMode_ == BT ? 1 : 0) << 2);
            deviceMode_ = temp;
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        }
        break;
    case BT:
        if (!BTN_IN && holdCounter_ >= BTN_SINGLE_PRESS) {
            deviceMode_ = WIFI;
            MOD_IN_ = (deviceMode_ & 0x03) | ((deviceMode_ == BT ? 1 : 0) << 2);
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        } else if (!BTN_IN && holdCounter_ > 1 && holdCounter_ < BTN_SINGLE_PRESS) {
            //Save current mode in the third bit of MOD_IN_
            DeviceMode temp = deviceMode_;
            if (digitalRead(PIN_CHG)) {
                temp = NONE;
            } else {
                temp = CHG;
            }
            MOD_IN_ = (temp & 0x03) | ((deviceMode_ == BT ? 1 : 0) << 2);
            deviceMode_ = temp;
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        }
        break;
    case CHG:
        digitalWrite(PIN_PSU_EN,LOW); //MARK: Change to low
        if (!BTN_IN && holdCounter_ > 1 && holdCounter_ < BTN_SINGLE_PRESS) {
            deviceMode_ = static_cast<t_DeviceMode>((MOD_IN_ >> 2) & 0x03);
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        }
        if (holdCounter_ > BTN_SINGLE_PRESS) {
          holdCounter_ = 0;
        }
        if (digitalRead(PIN_CHG)) {
            deviceMode_ = NONE;
            MOD_IN_ = (deviceMode_ & 0x07); 
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        }
        break;
    case NONE:
        digitalWrite(PIN_PSU_EN,HIGH);
        if (!BTN_IN && holdCounter_ > 1 && holdCounter_ < BTN_SINGLE_PRESS) {
            deviceMode_ = static_cast<t_DeviceMode>((MOD_IN_ >> 2) & 0x03);
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        }
        if (holdCounter_ > BTN_SINGLE_PRESS) {
          holdCounter_ = 0;
        }
        if (!digitalRead(PIN_CHG)) {
            deviceMode_ = CHG;
            MOD_IN_ = (deviceMode_ & 0x07); 
            holdCounter_ = 0;
            deviceModeChanged_ = true;
        }
        break;
    default:
        break;
    }
    return true;
}


void IRAM_ATTR encA_ISR(){ //~7us //MARK: encA_ISR
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_A));
  encB = digitalRead(PIN_ENC_B);
  if(encB){
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encB_ISR, FALLING);
  }
  else{
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encB_ISR, RISING);
  }
  if(encA != encB){
    ENC_COUNT+=ENC_VEL;
    volumeChange_++;
  }
}

void IRAM_ATTR encB_ISR(){ //~7us //MARK: encB_ISR
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_B));
  encA = digitalRead(PIN_ENC_A);
  if(encA){
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encA_ISR, FALLING);
  }
  else{
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encA_ISR, RISING);
  }
  if(encA != encB){
    ENC_COUNT-=ENC_VEL;
    volumeChange_--;
  }
}



/**
 * Shows a welcome message at startup of the device on the TFT display.
 */
void showWelcomeMessage() { //MARK: showWelcomeMessage
    // Show some information on the startup screen
    if (display_present) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE,0);
        display.setCursor(0,25);
        display.print(deviceName);
        display.display();
    }
}

/**
 * Displays the volume on the TFT screen.
 * 
 * @param volume Volume to be displayed on the TFT screen.
 */
void showVolume(uint8_t volume) { //MARK: showVolume
    if (display_present) {
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE,0);
        display.setCursor(0,0);
        display.print("Volume: ");
        display.printf("%3d", volume);
        display.display();
    }
}

void showBattery() { //MARK: showBattery
    if (display_present) {
        display.setCursor(65,55);
        display.print("Batt: ");
        display.printf("%3d", batteryLevel_);
        display.print("%");
        display.display();
    }
}

/**
 * Displays the current station name contained in 'stationStr_' on the TFT screen.
 */
void showStation() { //MARK: showStation
   if (display_present) {
         display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE,0);
        display.setCursor(0,10);
        if (deviceMode_ == WIFI) {
            display.print("Station: ");
        }
        else {
            display.print("Bluetooth");
        }
        if (!connectionError_) {
            display.setCursor(0,20);
            display.print(stationStr_);
        }
        else {
            display.setCursor(0,20);
            display.print("Connection error");
        }
        if (deviceMode_ == WIFI) {
            display.setCursor(100,0);
            display.print("WiFi");
        }
        else {
            display.setCursor(110,0);
            display.print("BT");
        }
        display.display();
    } else {
        Serial.print("Station: ");
        Serial.println(stationStr_);
        if (connectionError_) {
            Serial.println("Connection error");
        }
    }
}

/**
 * Displays the current song information contained in 'infoStr_' on the TFT screen.
 * Each time the song info is updated, it starts scrolling from the right edge.
 */
void showSongInfo() { //MARK: showSongInfo
    // Update the song title if flag is raised
    if (display_present) {
        if (infoUpdatedFlag_) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE,0);
        display.setCursor(0,10);
        if (deviceMode_ == WIFI) {
            display.print("Station:");
            display.setCursor(0,20);
            display.print(stationStr_);
        }
        else {
            display.print("Bluetooth");
        }
        display.setCursor(0,30);
        display.print(infoStr_);
        infoUpdatedFlag_ = false; // Clear update flag
        if (deviceMode_ == WIFI) {
            display.setCursor(100,0);
            display.print("WiFi");
        }
        else {
            display.setCursor(110,0);
            display.print("BT");
        }
        display.display();
        showVolume(volumeCurrent_);
        showBattery();
        }
    }
}



void showPlayState(bool isConnected) { //MARK: showPlayState
    if (display_present) {
        display.setTextSize(1);
        display.setCursor(80,0);
        display.print("P:");
        if (isConnected) {
            display.print("Y");
            digitalWrite(PIN_AMP_EN,HIGH);
        }
        else {
            display.print("N");
            digitalWrite(PIN_AMP_EN,LOW);
        }
        display.display();
    }
}

/**
 * Connects to the specified WiFi network and starts the device in internet radio mode.
 * Audio task is started.
 */
void startRadio() { //MARK: startRadio()
    if (pAudio_ == nullptr) {
        if (display_present) {
            display.clearDisplay();
            display.setCursor(0,0);
            display.print("MAC:"); // Own network mac address
            display.setCursor(0,10);
            display.print(WiFi.macAddress().c_str());
            display.setCursor(0,30);
            display.println("Connecting to WiFi...");
            display.setCursor(0,40);
            display.printf("SSID: "); // WiFi network name
            display.setCursor(0,50);
            display.print(WifiCredentials::SSID);
            display.display();
        }
        Serial.println("MAC: ");
        Serial.print(WiFi.macAddress().c_str());
        Serial.println("Connecting to:");
        Serial.println(WifiCredentials::SSID);
        

        // Initialize WiFi and connect to network
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(deviceName);
        WiFi.begin(WifiCredentials::SSID, WifiCredentials::PASSWORD);
        Serial.print("WiFi Connecting");
        while (!WiFi.isConnected()) {
            if (deviceModeChanged_) {
                changeDeviceMode();
                deviceModeChanged_ = false;
            }
            delay(100);
            Serial.print(".");
        }

        // Display own IP address after connecting
        if (display_present) {
            display.clearDisplay();
            display.setCursor(0,0);
            display.print("MAC:"); // Own network mac address
            display.setCursor(0,10);
            display.print(WiFi.macAddress().c_str());
            display.setCursor(0,30);
            display.print("IP:"); // Own IP address
            display.setCursor(0,40);
            display.print(WiFi.localIP().toString().c_str());
            display.display();
        }
        Serial.print("\nIP:");
        Serial.println(WiFi.localIP().toString().c_str());

        pAudio_ = new Audio(false); // Use external DAC

        // Setup audio
        pAudio_->setVolume(0); // 0...volumeMax
        pAudio_->setVolumeSteps(volumeMax);
        pAudio_->setPinout(I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, -1);
        digitalWrite(PIN_AMP_EN, HIGH);
        deviceMode_ = WIFI;

        // Start the audio processing task
        xTaskCreate(audioProcessing, "Audio processing task", 4096, nullptr, configMAX_PRIORITIES - 4, &pAudioTask_);

        // Wait some time before wiping out the startup screen
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (display_present) {
            display.clearDisplay();
        }
        isPlaying_ = true;
    }
    else {
        log_w("'pAudio_' not cleaned up!");
    }
}

/**
 * Stops the internet radio including the audio tasks.
 */
void stopRadio() { //MARK: stopRadio()

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
        if (display_present) {
          display.clearDisplay();
        }
        
    }
    else {
        log_w("Cannot clean up 'pAudio_'!");
    }
    isPlaying_ = false;
}

/**
 * Starts the device in bluetooth sink (BT) mode.
 */
void startA2dp() { //MARK: startA2dp

    i2s_pin_config_t pinConfig = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num = -1 // Not used
    };

    a2dp_.set_pin_config(pinConfig);

    a2dp_.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
    a2dp_.set_avrc_metadata_callback(avrc_metadata_callback);
    a2dp_.set_on_connection_state_changed(a2dp_connection_state_changed);
    a2dp_.set_avrc_rn_volumechange(avrc_volume_change_callback);

    showWelcomeMessage();
    if (display_present) {
        display.setCursor(0,10);
        display.print("Starting bluetooth");
    }
    a2dp_.set_auto_reconnect(true, 1000); //Remove this line if you don't want to auto reconnect
    a2dp_.start(deviceName);
    digitalWrite(PIN_AMP_EN,HIGH);
    deviceMode_ = BT;
    
    esp_bt_controller_status_t btStatus = esp_bt_controller_get_status();

    if (btStatus == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        if (display_present) {
            display.clearDisplay();
            display.setCursor(0,10);
            display.print("Bluetooth enabled");
            display.display();
        }
        Serial.println("Bluetooth enabled");
    }
    else {
        if (display_present) {
            display.clearDisplay();
            display.setCursor(0,20);
            display.print("Bluetooth not enabled");
            display.setCursor(0,30);
            display.print("Status: ");
            display.println(btStatus);
            display.display();
        }
        Serial.print("Bluetooth not enabled: ");
    }

    stationStr_ = "Bluetooth";
    stationUpdatedFlag_ = true;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //display.clearDisplay();

    //Serial.print("End: free heap = %d, max alloc heap = %d, min free heap = %d", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
}

/**
 * Enable or disable the shutdown circuit of the amplifier.
 * Amplifier: M5Stack SPK hat with PAM8303.
 * - b = true  --> GPIO_0 = 0 : Shutdown enabled
 * - b = false --> GPIO_0 = 1 : Shutdown disabled
 */
void setAudioShutdown(bool b) { //MARK: setAudioShutdown
    if (b) {
        digitalWrite(PIN_AMP_EN, LOW);
    }
    else {
        digitalWrite(PIN_AMP_EN, HIGH);
    }
}

void audioProcessing(void *p) { //MARK: audioProcessing
    
    while (true) {
        if (deviceMode_ != WIFI) {
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
            //const char *streamUrl = stationURLs[stationIndex_].c_str();
            if (urlPlaceholder_ == "") {
                url_ = stationURLs[stationIndex_];
            } else {
                url_ = urlPlaceholder_;
            }
            const char *streamUrl = url_.c_str();

            bool success = pAudio_->connecttohost( streamUrl );

            if (success) {
                stationChanged_ = false; // Clear flag
                connectionError_ = false; // Clear in case a connection error occured before

                timeConnect_ = millis(); // Store time in order to detect stream errors after connecting
            }
            else {
                stationChanged_ = false; // Clear flag
                connectionError_ = true; // Raise connection error flag
                digitalWrite(PIN_AMP_EN,LOW);
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

void readEncState() { //MARK: readEncState
    
 if (volumeChange_ != 0) {
               
                if (deviceMode_ == BT){
                    if (a2dp_.get_audio_state() != ESP_A2D_AUDIO_STATE_STARTED) {
                        a2dp_.play();
                    }
                    volumeCurrent_ += volumeChange_;
                    if (volumeCurrent_ < 0) {
                        volumeCurrent_ = 0;
                        a2dp_.pause();
                    } else if (volumeCurrent_ > 127) {
                        volumeCurrent_ = 127;
                    } 
                    a2dp_.set_volume(volumeCurrent_);
                    showVolume(volumeCurrent_);
                } else if (deviceMode_ == WIFI) {
                    volumeCurrent_ += volumeChange_;
                    if (volumeCurrent_ < 0) {
                        volumeCurrent_ = 0;
                    } else if (volumeCurrent_ > volumeMax) {
                        volumeCurrent_ = volumeMax;
                    }
                    volumeCurrentChangedFlag_ = true; // Raise flag for the audio task
                }   
            
    }
    volumeChange_ = 0;
}

void setup() { //MARK: Setup
    
    pinMode(PIN_PSU_EN,OUTPUT);
    //Enable PSU latch
    digitalWrite(PIN_PSU_EN,LOW);
    pinMode(PIN_BTN,INPUT);
    pinMode(PIN_BATT,INPUT);
    pinMode(PD_MUTE,OUTPUT);
    pinMode(PIN_CHG,INPUT);
    /*Blink setup*/
    pinMode(PIN_LED, OUTPUT);
    Timer0.attachInterruptInterval(TIMER_0_INTERVAL, Timer0_ISR);

    /*Encoder Interrupt setup*/
    pinMode(PIN_ENC_A,INPUT);
    pinMode(PIN_ENC_B,INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encA_ISR, RISING);
    
    delay(1);
    pinMode(PIN_AMP_EN, OUTPUT);

    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    digitalWrite(PIN_LED_R,HIGH);
    digitalWrite(PIN_LED_G,HIGH);
    digitalWrite(PIN_LED_B,HIGH);
    digitalWrite(PIN_AMP_EN,LOW);

    Serial.begin(115200);

    if(!display.begin(0x3C,true)) { // Address 0x3C for 128x32
        Serial.println(F("Adafruit SH1106G allocation failed"));
        display_present = false;
    } else {
        display_present = true;
    }

    if ( EEPROM.begin(1) ) {
        //uint8_t mode = EEPROM.readByte(0);
        MOD_IN_ = EEPROM.readByte(0);
        Serial.print("EEPROM.readByte(0) = ");
        Serial.println(MOD_IN_,BIN);
        //Serial.print("EEPROM.readByte(0) = %d", mode);
        Timer3.attachInterruptInterval(TIMER_3_INTERVAL, Timer3_ISR);
        //MARK: TIMER3
        deviceMode_ = static_cast<t_DeviceMode>(MOD_IN_ & 0x03);
        Serial.println(deviceMode_);
        // enum DeviceMode {WIFI = 0, BT = 1, CHG = 2, NONE = 3};
        if (deviceMode_ == BT) {
            if (display_present) {
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SH110X_WHITE,0);
                display.setCursor(0,25);
                display.print("Bluetooth");
                display.display();
            }
            digitalWrite(PD_MUTE,HIGH); //MARK: UNMUTE
            startA2dp();
        }
        else if (deviceMode_ == WIFI) {
            if (display_present) {
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SH110X_WHITE,0);
                display.setCursor(0,25);
                display.print("WiFi Radio");
                display.display();
            }
            digitalWrite(PD_MUTE,HIGH); //MARK: UNMUTE
            startRadio();
        } else if (deviceMode_ == CHG) {
            if (display_present) {
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SH110X_WHITE,0);
                display.setCursor(0,25);
                display.print("Charging");
                display.display();    
            }
            showBattery();
        } else if (deviceMode_ == NONE) {
            if (display_present) {
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SH110X_WHITE,0);
                display.setCursor(0,25);
                display.print("Standby");
                display.display();
            }
        }
    } else {
        //log_w("EEPROM.begin() returned 'false'!");
        Timer3.attachInterruptInterval(TIMER_3_INTERVAL, Timer3_ISR);
    }
    delay(1000);
    pinMode(PIN_AMP_EN,OUTPUT);
    digitalWrite(PIN_AMP_EN,LOW);
    if (deviceMode_ == WIFI) {
        digitalWrite(PIN_LED_G,LOW);
    } else if (deviceMode_ == BT) {
        digitalWrite(PIN_LED_B,LOW);
    } else if (deviceMode_ == CHG) {
        digitalWrite(PIN_LED_R,LOW);
    } else {
        digitalWrite(PIN_LED_R,HIGH);
        digitalWrite(PIN_LED_G,HIGH);
        digitalWrite(PIN_LED_B,HIGH);
    }
    //Setup NRF
    if (!nrfLink.begin()) {
        Serial.println("Failed to communicate with nRF24L01");
        for (int i = 0; i < 3; i++) {
            Serial.println("Failed to communicate with nRF24L01");
            delay(1000);
        }
    } else {
        Serial.println("nRF24L01 communication established");
        delay(100);
        nrfLink.setChannel(channel);
        delay(100);
        nrfNetwork.begin(thisNode);
        delay(100);
    }
    
}

/* Handle volume control and other commands via serial interface - only for debugging 
*/
void handleSerialCommands() { //MARK: handleSerialCommands
    if (Serial.available()) {
        char input = Serial.read();
        if (input == '+') { //MARK: Volume Up
            // Increase volume
            if (volumeCurrent_ < 127) {
                volumeCurrent_++;
                if (deviceMode_ == BT){
                    a2dp_.set_volume(volumeCurrent_); 
                    showVolume(volumeCurrent_);
                } else if (deviceMode_ == WIFI) {
                    volumeCurrentChangedFlag_ = true; // Raise flag for the audio task
                }
                
            }
        } else if (input == '-') { //MARK: Volume Down
            // Decrease volume
            if (volumeCurrent_ > 0) {
                volumeCurrent_--;
                if (deviceMode_ == BT){
                    a2dp_.set_volume(volumeCurrent_);
                    showVolume(volumeCurrent_);
                } else if (deviceMode_ == WIFI) {
                    volumeCurrentChangedFlag_ = true; // Raise flag for the audio task
                }
            }
        } else if (input == 'm') { //MARK: Mute
            // Mute
            volumeCurrent_ = 0;
            infoUpdatedFlag_ = true; // Raise flag for the display update routine
            if (deviceMode_ == BT){
                a2dp_.set_volume(volumeCurrent_);
            } else if (deviceMode_ == WIFI) {
                    volumeCurrentChangedFlag_ = true; // Raise flag for the audio task
            }
        } else if (input == 'p') { //MARK: Pause
            // Play
            if (deviceMode_ == BT){
                a2dp_.pause();
            } else if (deviceMode_ == WIFI) {
                // Stop
                pAudio_->stopSong();
                setAudioShutdown(true); // Turn off amplifier
                stationChangedMute_ = true; // Mute audio until stream becomes stable
            }
        } else if (input == 's') { //MARK: Start
            if (deviceMode_ == BT){
                a2dp_.play();
            } else if (deviceMode_ == WIFI) {
                // Start
                setAudioShutdown(false); // Turn on amplifier
                stationChangedMute_ = false; // Unmute audio
                stationChanged_ = true; // Raise flag for the audio task
                stationUpdatedFlag_ = true; // Raise flag for display update routine
                infoUpdatedFlag_ = true; // Raise flag for display update routine
            }
        } else if (input == 'n') { //MARK: Next
            // Next
            if (deviceMode_ == BT){
                a2dp_.next();
            } else if (deviceMode_ == WIFI) {
                stationIndex_ = (stationIndex_ + 1) % numStations;
                stationChanged_ = true; // Raise flag for the audio task
            }
        } else if (input == 'b') { //MARK: Previous
            // Previous
            if (deviceMode_ == BT) {
                a2dp_.previous();
            } else if (deviceMode_ == WIFI) {
                stationIndex_ = (stationIndex_ + numStations - 1) % numStations;
                stationChanged_ = true; // Raise flag for the audio task
            }
        } else if (input == 'r') { //MARK: Restart
            // Restart
            ESP.restart();
        } else if (input == 'w') { //MARK: WiFi
            // Switch to WiFi mode
            if (deviceMode_ == BT) {
                deviceMode_ = WIFI;
                EEPROM.writeByte(0, 0); // Enter internet radio mode after restart
                EEPROM.commit();
                Serial.println("Switching to Radio!");
            }
        } else if (input == 'c') { //MARK: Bluetooth
            // Switch to Bluetooth mode
            if (deviceMode_ == WIFI) {
                deviceMode_ = BT;
                EEPROM.writeByte(0, 1); // Enter BT mode after restart
                EEPROM.commit();
                Serial.println("Switching to BT!");
                stopRadio(); // Close connections and clean up
            }
        }
    }
}        

void nrfTransmit() { //MARK: nrfTransmit
    bool playstate = false;
    if (deviceMode_ == BT) {
        playstate = a2dp_.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED;
    } else if (deviceMode_ == WIFI) {
        playstate = isPlaying_;
    } else {
        playstate = false;
    }
    nrfTransmitPayload_t payload;
    payload.isPlaying = playstate;
    payload.deviceMode = deviceMode_;
    payload.volume = volumeCurrent_;
    payload.batteryLevel = batteryLevel_;
    RF24NetworkHeader header(otherNode);
    bool ok = nrfNetwork.write(header, &payload, sizeof(payload));
    if (ok) {
        Serial.println("Message sent");
    } else {
        Serial.println("Message failed");
    }
}


void nrfCheck() { //TODO: if not working, print hex value
    nrfNetwork.update();
    while (nrfNetwork.available()) {
        RF24NetworkHeader header;
        nrfReceivePayload_t payload;
        nrfNetwork.read(header, &payload, sizeof(payload));
        Serial.print(F("Received packet: url="));
        Serial.print(payload.url);
        Serial.print(F(", isPlaying="));
        Serial.println(payload.isPlaying);
        Serial.print(F(", volume="));
        Serial.println(payload.volume);
        Serial.print(F(", changed="));
        Serial.println(payload.paramBitMask,BIN);
        if (payload.paramBitMask & (1 << 0)) { // Check bit 0
            Serial.println("URL has changed");
            urlPlaceholder_ = String(payload.url);
            stationChanged_ = true;
        }
        if (payload.paramBitMask & (1 << 1)) { // Check bit 1
            Serial.println("Play state has changed");
            if (payload.isPlaying) {
                if (deviceMode_ == BT){
                    a2dp_.play();
                } else if (deviceMode_ == WIFI) {
                    // Start
                    setAudioShutdown(false); // Turn on amplifier
                    stationChangedMute_ = false; // Unmute audio
                    stationChanged_ = true; // Raise flag for the audio task
                    stationUpdatedFlag_ = true; // Raise flag for display update routine
                    infoUpdatedFlag_ = true; // Raise flag for display update routine
                }
            } else {
                if (deviceMode_ == BT){
                    a2dp_.pause();
                } else if (deviceMode_ == WIFI) {
                    // Stop
                    pAudio_->stopSong();
                    setAudioShutdown(true); // Turn off amplifier
                    stationChangedMute_ = true; // Mute audio until stream becomes stable
                }
            }
        }
        if (payload.paramBitMask & (1 << 2)) { // Check bit 2
            Serial.println("Volume has changed");
            if (deviceMode_ == BT){
                a2dp_.set_volume(payload.volume);
            } else if (deviceMode_ == WIFI) {
                volumeCurrent_ = payload.volume;
                volumeCurrentChangedFlag_ = true;
            }
        }
    }
    
}


void processBatteryLevel(void){ //MARK: processBatteryLevel
  float voltage = analogRead(PIN_BATT)*3.3/4096*6;  // Reading an calculating voltage level
  int8_t percent = (voltage/12.4)*100;              // Converting to percent
  batteryLevel_ = percent;
}

void changeDeviceMode() {
    int8_t byteToWrite = MOD_IN_ & 0x04 | (deviceMode_ & 0x03);
    Serial.print("byteToWrite: ");
    Serial.println(byteToWrite,BIN);
    if (deviceMode_ == WIFI) {
        EEPROM.writeByte(0, byteToWrite); // Enter BT mode after restart
        EEPROM.commit();
        Serial.println("Switching to BT!");
        stopRadio(); // Close connections and clean up
        ESP.restart();
    }
    else if (deviceMode_ == BT) {
        EEPROM.writeByte(0, byteToWrite); // Enter internet radio mode after restart
        EEPROM.commit();
        Serial.println("Switching to WIFI!");
        ESP.restart();
    } else if (deviceMode_ == CHG) {
        Serial.println("Switching to CHG!");
        EEPROM.writeByte(0, byteToWrite); // Enter charging mode after restart
        EEPROM.commit();
        ESP.restart();
    } else if (deviceMode_ == NONE) {
        Serial.println("Switching to NONE!");
        EEPROM.writeByte(0, byteToWrite); // Enter standby mode after restart
        EEPROM.commit();
        ESP.restart();
    }
}

void loop() { //MARK: loop
    

    if (deviceModeChanged_) {
        changeDeviceMode();
        deviceModeChanged_ = false;
    } else {
      unsigned long currentMillisCHG = millis();
      nrfCheck();
      readEncState();
      
      if (deviceMode_ == WIFI || deviceMode_ == BT) {
        if (currentMillisCHG - previousMillisCHG >= intervalCHG) {
            previousMillisCHG = currentMillisCHG;
            processBatteryLevel();
            showBattery();
            nrfTransmit();
        }
      }
      if (deviceMode_ == WIFI) {
          if (volumeCurrentChangedFlag_) {
            showVolume(volumeCurrent_);
          }
          if (connectionError_) {
              showVolume(volumeCurrent_);
              display.setCursor(0,30);
              display.println("Stream unavailable");
              digitalWrite(PIN_AMP_EN,LOW); //MARK: DISABLE AMP NO STREAM
              vTaskDelay(200 / portTICK_PERIOD_MS); // Wait until next cycle
          }
          else {
              // Update the station name if flag is raised
              if (stationUpdatedFlag_ && !stationChanged_) {
                  volumeNormal_ = volumeCurrent_;
                  volumeCurrent_ = 0;
                  volumeCurrentChangedFlag_ = true;
                  showVolume(volumeCurrent_);
                  showStation();
                  stationUpdatedFlag_ = false; // Clear update flag  
                  digitalWrite(PIN_AMP_EN,HIGH); //MARK: ENABLE AMP NEW STATION
                  while (volumeCurrent_ < volumeNormal_){
                    volumeCurrentF_ += 0.25;
                    volumeCurrent_ = (uint8_t) volumeCurrentF_;
                    volumeCurrentChangedFlag_ = true;
                    showVolume(volumeCurrent_);
                  }
              }
              showSongInfo();
              vTaskDelay(20 / portTICK_PERIOD_MS); // Wait until next cycle
          }
      } else if (deviceMode_ == BT) {
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
      } else if (deviceMode_ == CHG) {
        if (currentMillisCHG - previousMillisCHG >= intervalCHG) {
            previousMillisCHG = currentMillisCHG;
            processBatteryLevel();
            showBattery();
            nrfTransmit();
            if (batteryLevel_ < 20) {
                digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R)); // Toggle red LED
                digitalWrite(PIN_LED_G, HIGH);  // Make sure green LED is off
                digitalWrite(PIN_LED_B, HIGH);  // Make sure blue LED is off
            } else if (batteryLevel_ == 20) {
                digitalWrite(PIN_LED_R, LOW); // Turn on red LED
                digitalWrite(PIN_LED_G, HIGH);  // Make sure green LED is off
                digitalWrite(PIN_LED_B, HIGH);  // Make sure blue LED is off
            } else if (batteryLevel_ > 20 && batteryLevel_ < 40) {
                digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R)); // Toggle red LED
                digitalWrite(PIN_LED_G, !digitalRead(PIN_LED_G)); // Toggle green LED
                digitalWrite(PIN_LED_B, HIGH);  // Make sure blue LED is off
            } else if (batteryLevel_ >= 40 && batteryLevel_ <= 60) {
                digitalWrite(PIN_LED_B, !digitalRead(PIN_LED_B)); // Toggle blue LED
                digitalWrite(PIN_LED_R, HIGH);  // Make sure red LED is off
                digitalWrite(PIN_LED_G, HIGH);  // Make sure green LED is off
            } else if (batteryLevel_ == 60) {
                digitalWrite(PIN_LED_B, LOW); // Toggle red LED
                digitalWrite(PIN_LED_G, HIGH); // Toggle green LED
                digitalWrite(PIN_LED_B, HIGH);  // Make sure blue LED is off
            } else if (batteryLevel_ > 60 && batteryLevel_ <= 80) {
                digitalWrite(PIN_LED_B, !digitalRead(PIN_LED_B)); // Toggle blue LED
                digitalWrite(PIN_LED_R, HIGH);  // Make sure red LED is off
                digitalWrite(PIN_LED_G, !digitalRead(PIN_LED_G));  // Make sure green LED is off
            } else if (batteryLevel_ > 80) {
                digitalWrite(PIN_LED_G, !digitalRead(PIN_LED_G)); // Toggle green LED
                digitalWrite(PIN_LED_R, HIGH);  // Make sure red LED is off
                digitalWrite(PIN_LED_B, HIGH);  // Make sure blue LED is off
            }

        }
      } else if (deviceMode_ == NONE) {
        // Neither WIFI, BT or CHG
        vTaskDelay(200 / portTICK_PERIOD_MS);
      }
      
    }
}


//MARK: optional functions
void audio_info(const char *info){ //MARK: audio_info
    //Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata //MARK: audio_id3data
    // Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file //MARK: audio_eof_mp3
    // Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){ //MARK: audio_showstation
    stationStr_ = info;
    stationUpdatedFlag_ = true; // Raise flag for the display update routine

    // Serial.print("station     ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){ //MARK: audio_showstreamtitle
    if (deviceMode_ == WIFI) {
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
void audio_bitrate(const char *info){ //MARK: audio_bitrate
    // Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec //audio_commercial
    // Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage //MARK: audio_icyurl
    // Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played //MARK: audio_lasthost
    // Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){ //MARK: audio_eof_speech
    // Serial.print("eof_speech  ");Serial.println(info);
}

void avrc_metadata_callback(uint8_t id, const uint8_t *text) { //MARK: avrc_metadata_callback
    switch (id) {
        case ESP_AVRC_MD_ATTR_TITLE:
            titleStr_ = (char*) text;
            break;
        
        case ESP_AVRC_MD_ATTR_ARTIST:
            artistStr_ = (char*) text;
            break;
    }    

    if ( artistStr_.isEmpty() ) {
        infoStr_ = titleStr_;
    }
    else {
        if ( titleStr_.isEmpty() ) {
            infoStr_ = artistStr_;
        }
        else {
            infoStr_ = artistStr_ + " - " + titleStr_;
        }
    }
    
    infoUpdatedFlag_ = true; // Raise flag for the display update routine
    // Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
}

void a2dp_connection_state_changed(esp_a2d_connection_state_t state, void*) { //MARK: a2dp_connection_state_changed

    //Serial.print("Connection state: %d", state);

    if (state != ESP_A2D_CONNECTION_STATE_CONNECTED) {
        infoStr_ = "not connected";
        infoUpdatedFlag_ = true; // Raise flag for the display update routine
    }
}

void avrc_volume_change_callback(int vol) { //MARK: avrc_volume_change_callback
    volumeCurrent_ = vol;
    volumeCurrentChangedFlag_ = true;
}