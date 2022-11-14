#include "hardware.h"
#include "task_manager.h"

RTC_DATA_ATTR bool chargerConnected = 0;
Adafruit_NeoPixel pixels(NUMPIXELS, LED, NEO_GRB + NEO_KHZ800);
Preferences prefs;


void setup_hardware() {
  Serial.begin(HB_BAUD, SERIAL_8N1, HB_RX, HB_TX);    //set Tx, Rx and Baudrate, serial comm(Uart 0) for Hemo(we're initialising when the test starts so not required to initialise here also i think)

  // Setup Tasks
  startBackgroundTasks();     // Setup tasks in the background

  // Configure Sleep Wakeup Options
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);        // wake-up due to switch is pressed
  esp_sleep_enable_timer_wakeup(CHARGER_POLL_TIMER);  // wake-up every 10s to check any changes in the device

  pinMode(HB_LED, OUTPUT);    // Set LED pin as output for hemo test
  pinMode(GLU_LED, OUTPUT);   // Set LED pin as output for glu test
  pinMode(CHOL_LED, OUTPUT);  // Set LED pin as output for chol test
  digitalWrite(HB_LED, LOW);  // Set LED pin low for hemo test
  digitalWrite(GLU_LED, LOW); // Set LED pin low for glu test
  digitalWrite(CHOL_LED, LOW);// Set LED pin low for chol test

  pixels.begin();   //  Select LED pixels
  pixels.clear();   //  Clear LED pixels

  prefs.begin("device_info"); // 
}

//Set colour of LED
void set_led_colour(uint32_t colour) {
    pixels.setPixelColor(0, colour);  //  no pixel selected
    pixels.setPixelColor(1, colour);  //  Set pixel colour
    pixels.show();  // Show pixel
}

//get colour of LED
uint32_t get_led_colour() {
    return pixels.getPixelColor(1);   //  Get colour of LED
}

//Set paramter into device
void set_device_info(uint8_t* raw_data) {
  uint8_t param_id = raw_data[4];           // Get param id from raw data
  uint8_t param_len = raw_data[2] - 4;      //get para len by removing start 2 bytes+len+command
  String param_value;
  for(int i = 0; i < param_len; i++) {
    param_value += (char)(raw_data[5+i]);   //get complete string from raw data
  }
  char data_value[15];
  param_value.toCharArray(data_value, 15);  //conversion of char to array
  log_e("Writing Param ID %d, Length = %d, Value = %s", param_id, param_len, data_value); //print the param 
  switch(param_id) {
    case 0x00:
      param_value = prefs.putString("model", param_value);          //if param id is 0x00 change the model number
      break;
    case 0x01:
      param_value = prefs.putString("serial", param_value);         //if param id is 0x01 change the serial number
      break;
    case 0x02:
      param_value = prefs.putString("fw", param_value);             //if param id is 0x02 change the fw version
      break;
    case 0x03:
      param_value = prefs.putString("hw", param_value);             //if param id is 0x03 change the hw version
      break;
    case 0x04:
      param_value = prefs.putString("glucose", param_value);        //if param id is 0x04
      break;
    case 0x05:
      param_value = prefs.putString("hemoglobin", param_value);     //if param id is 0x05
      break;
    case 0x06:
      param_value = prefs.putString("cholesterol", param_value);    //if param id is 0x06
      break;
    case 0x07:
      param_value = prefs.putString("display_number", param_value); //if param id is 0x07 change display number
      break;
    default:
      break;    
  }
}

// get parameters from device
String get_parameter(uint8_t param_id) {
  String param_value;
  switch(param_id) {  //get param id from fun call
    case 0x00:
      param_value = prefs.getString("model");             //if param id is 0x00 get model number of device
      break;
    case 0x01:
      param_value = prefs.getString("serial");            //if param id is 0x01 get serial number of device
      break;
    case 0x02:
      param_value = FIRMWARE_VERSION;                     //if param id is 0x02 get firmware version of the device
      break;
    case 0x03:  
      param_value = prefs.getString("hw");                //if param id is 0x03 get hardware version of the device
      break;
    case 0x04:
      param_value = prefs.getString("glucose");           //if param id is 0x04 
      break;
    case 0x05:
      param_value = prefs.getString("hemoglobin");        //if param id is 0x05
      break;
    case 0x06:
      param_value = prefs.getString("cholesterol");       //if param id is 0x06
      break;
    case 0x07:
      param_value = prefs.getString("display_number");    //if param id is 0x07 get display number of the device
      break;
    default:
      param_value = "---";                                //if param id is invalid show ntng
      break;    
  }
  return param_value;                                     //return updated param value
}

//Set module state of the device
void set_module_state(uint8_t state, uint8_t module) {
    switch(module) {
      case 0x00:  //case 0x00 is for all 3 tests simultaneously
        digitalWrite(GLU_ON, state);  //turn on Glu LED
        digitalWrite(HB_ON, state);   //turn on hemo LED
        digitalWrite(CHOL_ON, state); //turn on chol LED
        break;
      case 0x01:  // case 0x01 is for gluc test
        digitalWrite(GLU_ON, state);  //turn on glu LED
        break;
      case 0x02:  // case 0x02 is for hemo test
        digitalWrite(HB_ON, state);   //turn on Hemo LED
        break;
      case 0x03:  // case 0x03 is for chol test
        digitalWrite(CHOL_ON, state); //turn on chol LED
        break;
      default:  //for any other value do nothing
        break;
    }
}

//  Set main LED state 
void set_led_state(uint8_t color_code, uint8_t blink_state) {
  if(blink_state) {   //if LED is is in blinking state
    xSemaphoreGive(led_blink_semaphore);  //release the blinking task(stop the task)
  } else {
    xSemaphoreGive(led_steady_semaphore); //release the steady glow task(stop the task)
  }
  uint32_t led_colour;
  switch(color_code) {
    case 0x00:
      led_colour = OFF;   //if colour code is 0x00 turn off LED
      break;
    case 0x01:
      led_colour = RED;   //if colour code is 0x01 LED colour is RED
      break;
    case 0x02:
      led_colour = GREEN; //if colour code is 0x02 LED colour is GREEN
      break;  
    case 0x03:
      led_colour = BLUE;  //if colour code is 0x03 LED colour is BLUE
      break;  
    case 0x04:
      led_colour = CYAN;  // if colour code is 0x04 LED colour is CYAN
      break;  
    case 0x05:
      led_colour = YELLOW;  //if colour code is 0x05 LED colour is YELLOW
      break;  
    case 0x06:
      led_colour = MAGENTA; //if colour code is 0x06 LED colour is MAGENTA
      break;  
    case 0x07:
      led_colour = WHITE;   //if colour code is 0x07 LED colour is WHITE
      break;  
    default:
      led_colour = OFF;     // any other colour code turn off LED
      break;  
    
  }
  xQueueSend(led_colour_override_queue, &led_colour, 0);  //  Send LED colour in a queue manner
  log_i("Colour Code %d, Blink %d", color_code, blink_state); //print current LED colour and state of LED
}

// Set LED colour
void set_led(uint32_t colour){
  pixels.setPixelColor(0, colour);  // disable LED colour // colouring 0 pixels
  pixels.setPixelColor(1, colour);  // enable LED colour  //colouring 1 pixel
  pixels.show();  //display LED colour
}

// Waking up from sleep
void check_wakeup_reason(){
  pinMode(CHRG, INPUT_PULLUP);              // making pin as input and pulling it up       
  RESET_REASON reset_cause_0, reset_cause_1;  
  esp_sleep_wakeup_cause_t wakeup_reason;
  
  reset_cause_0 = rtc_get_reset_reason(0);  // check reason for reset of CPU 0
  reset_cause_1 = rtc_get_reset_reason(1);  // check reason for reset of CPU 1
  wakeup_reason = esp_sleep_get_wakeup_cause(); //check actual reason for waking up

  log_e("Reset Cause %d / %d", reset_cause_0, reset_cause_1); // print reset cause

  if(reset_cause_0 == reset_cause_1) {  // if both reason are same
    pixels.begin(); //enable pixels of LED
    pixels.clear(); //disable pixels of LED
    set_led(OFF);   // turn off LED
    log_e("Brownout. Sleeping.");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);  //wake up due to switch is pressed
    esp_sleep_enable_timer_wakeup(LO_CHARGER_POLL_TIMER); // wakeup timer for every 120s while in low charge to check any change in the device
    esp_deep_sleep_start(); //enter sleep
  }

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0: //wake up due to switch is pressed
      log_e("Power Switch Pressed");  //print switch pressed msg
      break;
    case ESP_SLEEP_WAKEUP_TIMER:  //wake-up due to wakeup-timer
      if(chargerConnected && digitalRead(CHRG) == true) { // if charger is removed
        chargerConnected = false; //update as charger has been removed
        log_e("Charger Removed"); //print charger removed
        pixels.begin();           // enable pixels of LED
        pixels.clear();           // disable pixels of LED
        set_led(OFF);             // turn off LED
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);        //wake-up due to switch is pressed
        esp_sleep_enable_timer_wakeup(CHARGER_POLL_TIMER);  //wake-up every 10s to check any changes in the device
        esp_deep_sleep_start();                             //go back to sleep
      } else if(!chargerConnected && digitalRead(CHRG) == false) {  //if charger is connected
        chargerConnected = true;      // update as charger is connected
        log_e("Charger Connected");   // print charger connected
        pixels.begin();               // enable pixels of LED
        pixels.clear();               // disable pixels of LED
        set_led(ORANGE);              // orange LED is ON
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);        // wake-up due to switch is pressed
        esp_sleep_enable_timer_wakeup(CHARGER_POLL_TIMER);  // wake-up every 10s to check any changes is the device
        esp_deep_sleep_start();                             // go back to sleep
      } else {
        log_e("No Charger Event");                          // print no charge event occured
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);        // wake-up due to switch is pressed
        esp_sleep_enable_timer_wakeup(CHARGER_POLL_TIMER);  // wake-up every 10s to check any changes is the device
        esp_deep_sleep_start();                             // go back to sleep
      }
      break;
    default: 
      log_e("Unknown Wakeup: %d", wakeup_reason);           // unknown wake-up
      break;
  }
}

// Battery level
uint8_t get_battery_level(uint16_t adc_value) {
  int batt_value_percentage=0;
   /* //log_d("ADC Val = %d", adc_value);
    if(adc_value < 3100) {          // Vbatt < 3.1 V
        return 0;
    } else if(adc_value < 3200) {   // Vbatt < 3.2 V
        return 10;
    } else if(adc_value < 3300) {   // Vbatt < 3.3 V
        return 20;
    } else if(adc_value < 3400) {   // Vbatt < 3.4 V
        return 30;
    } else if(adc_value < 3500) {   // Vbatt < 3.5 V
        return 40;
    } else if(adc_value < 3600) {   // Vbatt < 3.6 V
        return 50;
    } else if(adc_value < 3700) {   // Vbatt < 3.7 V
        return 60;
    } else if(adc_value < 3800) {   // Vbatt < 3.8 V
        return 70;
    } else if(adc_value < 3900) {   // Vbatt < 3.9 V
        return 80;
    } else if(adc_value < 4000) {   // Vbatt < 4.0 V
        return 90;
    } else {                        // Vbatt > 4.0 V
        return 100; 
    }*/

    batt_value_percentage = ((adc_value - 3333) * 100) / (4100 - 3333); // Calculation to find battery level
    if (batt_value_percentage > 100 ) // if batttery level is more than 100
    {
      batt_value_percentage = 100;    // return 100 as battery level
    }

    if (batt_value_percentage < 0 )   // if battery level is less than 0
    {
      batt_value_percentage = 0;      //return 0 as battery level
    }

    if(batt_value_percentage<5)       // if battery level is less than 5 percent
    {
    log_e("Low Battery Going Sleep");  // prit going to sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);          // wake-up due to switch is pressed
    esp_sleep_enable_timer_wakeup(LO_CHARGER_POLL_TIMER); // wake-up every 120s to check any changes is the device
    esp_deep_sleep_start();                               // go back to sleep
    }
    return batt_value_percentage;                         // return battery value

}
