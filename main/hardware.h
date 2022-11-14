#ifndef BIOCHEM_HARDWARE_H_
#define BIOCHEM_HARDWARE_H_

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"
#include "Preferences.h"
#include "esp32/rom/rtc.h"
#include "driver/rtc_io.h"

#define FIRMWARE_VERSION    "1.1"

#define HbPort      Serial
#define HB_BAUD     38400
#define HB_RX       26
#define HB_TX       1
#define HB_DET      27
#define HB_ON       14
#define HB_LED      2

#define GluPort     Serial1
#define GLU_BAUD    38400
#define GLU_RX      19
#define GLU_TX      21
#define GLU_DET     18
#define GLU_ON      17
#define GLU_LED     15

#define CholPort    Serial2
#define CHOL_BAUD   38400
#define CHOL_RX     32
#define CHOL_TX     12
#define CHOL_DET    33
#define CHOL_ON     25
#define CHOL_LED    5

#define SWITCH      39
#define LED         23
#define BUZZER      13
#define VOLTAGE     36
#define CHRG        22
#define ESP_PING    0

#define NUMPIXELS   2 
#define RED         Adafruit_NeoPixel::Color(150, 0, 0)
#define GREEN       Adafruit_NeoPixel::Color(0, 150, 0)
#define BLUE        Adafruit_NeoPixel::Color(0, 0, 150)
#define CYAN        Adafruit_NeoPixel::Color(0, 150, 150)
#define YELLOW      Adafruit_NeoPixel::Color(150, 150, 0)
#define MAGENTA     Adafruit_NeoPixel::Color(150, 0, 150)
#define WHITE       Adafruit_NeoPixel::Color(150, 150, 150)
#define ORANGE      Adafruit_NeoPixel::Color(150, 40, 0)
#define OFF         Adafruit_NeoPixel::Color(0, 0, 0)

#define BUZZER_PWM  0
#define BUZZER_TONE 4000
#define BUZZER_BITS 8

#define WDT_TIMEOUT         3
#define CHARGER_POLL_TIMER  10000000ULL
#define LO_CHARGER_POLL_TIMER  120000000ULL

extern SemaphoreHandle_t led_blink_semaphore;
extern SemaphoreHandle_t led_steady_semaphore;
extern QueueHandle_t led_colour_override_queue;

void setup_hardware();
void set_module_state(uint8_t state, uint8_t module);
void set_led_state(uint8_t color_code, uint8_t blink_state);
String get_parameter(uint8_t param_id);
void set_device_info(uint8_t* raw_data);
void set_led(uint32_t colour);
void check_wakeup_reason();
uint8_t get_battery_level(uint16_t adc_value);
void perform_beep(uint8_t state);
void set_led_colour(uint32_t colour);
uint32_t get_led_colour();
#endif