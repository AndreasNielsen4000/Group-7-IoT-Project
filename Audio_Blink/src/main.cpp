#include <Arduino.h>
#define _TIMERINTERRUPT_LOGLEVEL_     3
#include "ESP32_New_TimerInterrupt.h"

#define PIN_LED 0
#define PIN_PSU_EN 33
#define PIN_BTN 35
#define PIN_ENC_A 27
#define PIN_ENC_B 14
#define PIN_LED_R 32
#define PIN_LED_G 15
#define PIN_LED_B 5
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_AMP_EN 2

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

ESP32Timer Timer0(0); //4 timers are available (from 0 to 3)
ESP32Timer Timer3(3);

static volatile char ENC_COUNT = 0;
static volatile long tick = 0; //Seconds from power on
static volatile bool encA = false;
static volatile bool encB = false;
static volatile bool ampMode = false;
static volatile bool chgMode = false;

/*Interrupt Handlers*/
bool IRAM_ATTR Timer0_ISR(void * timerNo);
bool IRAM_ATTR Timer3_ISR(void * timerNo);
void IRAM_ATTR encA_ISR();
void IRAM_ATTR encB_ISR();

bool IRAM_ATTR Timer0_ISR(void * timerNo){
  /*Blink */
  static bool ledToggle = 0;
  digitalWrite(PIN_LED,ledToggle);
  ledToggle=!ledToggle;
  /*Tick*/
  tick++;
  return true;
}

bool IRAM_ATTR Timer3_ISR(void * timerNo){
  /*Power logic*/ 
  bool sw = analogRead(PIN_BTN) >= SW_THRESHOLD;
  if(ampMode && !chgMode && sw){
    digitalWrite(PIN_PSU_EN,LOW);
    ampMode = false;
  }
  else if(chgMode && !ampMode && sw){
    digitalWrite(PIN_PSU_EN,HIGH);
    ampMode = true;
  }
  else if(chgMode && ampMode && !sw){
    chgMode = false;
  }
  else if(!chgMode && !ampMode && !sw){
    chgMode = true;
  }
  return true;
}


void IRAM_ATTR encA_ISR(){ //~7us
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
  }
}

void IRAM_ATTR encB_ISR(){ //~7us
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
  }
}

void setup() {
  /*Power switch setup*/
  pinMode(PIN_PSU_EN,OUTPUT);
  pinMode(PIN_BTN,INPUT);
  chgMode = true;
  Timer3.attachInterruptInterval(TIMER_3_INTERVAL, Timer3_ISR);

  /*Serial setup*/
  Serial.begin(115200);

  /*Blink setup*/
  pinMode(PIN_LED, OUTPUT);
  Timer0.attachInterruptInterval(TIMER_0_INTERVAL, Timer0_ISR);

  /*Encoder Interrupt setup*/
  pinMode(PIN_ENC_A,INPUT);
  pinMode(PIN_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encA_ISR, RISING);

  delay(1);
}

void loop() {
delay(1000);
//String enc_s = String(ENC_COUNT);
//Serial.println(enc_s);
Serial.println(ENC_COUNT,DEC);
}

