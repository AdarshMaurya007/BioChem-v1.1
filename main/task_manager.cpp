#include "task_manager.h"
#include "hardware.h"
#include "ble_packets.h"
#include "ble_comm.h"

TaskHandle_t wdtTaskHandle = NULL;
TaskHandle_t batteryMonitorTaskHandle = NULL;
TaskHandle_t ledUpdateTaskHandle = NULL;
TaskHandle_t switchPressedTaskHandle = NULL;
TaskHandle_t CholesterolTaskHandle = NULL;
TaskHandle_t HemoglobinTaskHandle = NULL;
TaskHandle_t GlucoseTaskHandle = NULL;
TaskHandle_t bleMonitorTaskHandle = NULL;
TaskHandle_t buzzerTaskHandle = NULL;

QueueHandle_t led_colour_queue, led_colour_override_queue, beep_queue;

SemaphoreHandle_t led_blink_semaphore, led_steady_semaphore, switch_pressed_semaphore, sleep_semaphore;
SemaphoreHandle_t bleConnectedSemaphore, bleDisconnectedSemaphore;
SemaphoreHandle_t glucose_start_semaphore, hemoglobin_start_semaphore, cholesterol_start_semaphore;

uint8_t hemoglobin_state = TEST_NOT_STARTED;
uint8_t cholesterol_state = TEST_NOT_STARTED;
uint8_t glucose_state = TEST_NOT_STARTED;

void IRAM_ATTR switch_isr() {
  xSemaphoreGiveFromISR(switch_pressed_semaphore, NULL);
}

// Create a task for Watchdog
void createWatchdogTask() {
  xTaskCreatePinnedToCore(
    TaskWatchdog,   // function to implement the task
    "Watchdog",     // name of the task
    1024,           // stack size
    NULL,           // task input parameter
    2,              // priority of the task
    &wdtTaskHandle, // task handler
    0               // core where the task should run
  );
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(wdtTaskHandle);
}

// Create a task for Cholestrol test
void createCholesterolTask() {
  xTaskCreatePinnedToCore(
    TaskCholesterol,
    "Cholesterol",
    8192,
    NULL,
    1,
    &CholesterolTaskHandle,
    0
  );
}

// Create a task for Hemoglobin test
void createHemoglobinTask() {
  xTaskCreatePinnedToCore(
    TaskHemoglobin,
    "Hemoglobin",
    8192,
    NULL,
    1,
    &HemoglobinTaskHandle,
    0
  );
}

// Create task for Glucose test
void createGlucoseTask() {
  xTaskCreatePinnedToCore(
    TaskGlucose,
    "Glucose",
    8192,
    NULL,
    1,
    &GlucoseTaskHandle,
    0
  );
}

// Create a task for LED Update
void createLEDUpdateTask() {
  xTaskCreatePinnedToCore(
    TaskUpdateLED,
    "LED Update",
    2048,
    NULL,
    1,
    &ledUpdateTaskHandle,
    0
  );
}

// Create a task for BLE Monitor
void createBLEMonitorTask() {
  xTaskCreatePinnedToCore(
    TaskBLEMonitor,
    "BLE Monitor",
    2048,
    NULL,
    1,
    &bleMonitorTaskHandle,
    0
  );
}

// Create a task for Battry monitor
void createBatteryMonitorTask() {
  xTaskCreatePinnedToCore(
    TaskBatteryMonitor,
    "Battery Monitor",
    3072,
    NULL,
    1,
    &batteryMonitorTaskHandle,
    0
  );
}

// Create a task for Switch press
void createSwitchPressedTask() {
  xTaskCreatePinnedToCore(
    TaskSwitchPressed,
    "Switch Pressed",
    2048,
    NULL,
    2,
    &switchPressedTaskHandle,
    0
  );
}

// Create a task for Buzzer
void createBuzzerTask() {
  xTaskCreatePinnedToCore(
    TaskBuzzer,
    "Buzzer Beep",
    2048,
    NULL,
    1,
    &buzzerTaskHandle,
    0
  );
}

// Buzzer task
void TaskBuzzer(void *pvParameters) {
    // Configure Buzzer PWM and Play 1 long beep
  ledcSetup(BUZZER_PWM, BUZZER_TONE, BUZZER_BITS);              //setting up Buzzer channel and tone
  ledcAttachPin(BUZZER, BUZZER_PWM);                            //selecting Buzzer pin
  beep_queue = xQueueCreate(5, sizeof(BuzzerBeep));             //create buzzer

  BuzzerBeep current_beep = {1, 500};                           //beep count and time
  xQueueSend(beep_queue, &current_beep, 0);                     //send buzzer request

  BuzzerBeep beep_params;                                       //buzzer structure
  while(true) {
    if(xQueueReceive(beep_queue, &beep_params, 0) == pdTRUE) {  // receive buzzer request
      uint16_t beepCount = beep_params.beepCount;               //collect beep count  
      uint16_t beepInterval = beep_params.beepInterval;         //collect time interval
      for(; beepCount > 0; beepCount--) {                       //start from the beep count
        ledcWriteTone(BUZZER_PWM, BUZZER_TONE);                 //write the max buzzer tone
        vTaskDelay(beepInterval / portTICK_PERIOD_MS);          //wait for interval
        ledcWriteTone(BUZZER_PWM, 0);                           //write the min buzzer tone
        vTaskDelay(beepInterval / portTICK_PERIOD_MS);          //wait for interval
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);                       //wait 100ms
  }
}

// BLE Monitor task
void TaskBLEMonitor(void *pvParameters) {
  bool inStandby = true;
  int standbyTimer = 0;
  bleConnectedSemaphore = xSemaphoreCreateBinary();             //create a task for ble connection
  bleDisconnectedSemaphore = xSemaphoreCreateBinary();          //create a task for b;le disconnection
  while(true) {
    if (xSemaphoreTake(bleConnectedSemaphore, 0) == pdTRUE) {   //if ble connected
        inStandby = false;                                      //update the device is not in standby
        standbyTimer = 0;                                       //reset standby timer
        xSemaphoreGive(led_steady_semaphore);                   //get led steady glow task
        BuzzerBeep current_beep = {2, 200};                     //beep count and time
        xQueueSend(beep_queue, &current_beep, 0);               //start beep
    }

    if (xSemaphoreTake(bleDisconnectedSemaphore, 0) == pdTRUE) { //if ble is disconnected
        inStandby = true;                                        //update the device is in standby
        standbyTimer = 0;                                        //reset standby timer
        uint32_t led_colour = RED;                               //set initial LED colour as RED
        xQueueSend(led_colour_queue, &led_colour, 0);            //Set LED colour
        xSemaphoreGive(led_blink_semaphore);                     //get led blink task
        BuzzerBeep current_beep = {3, 200};                      //beep count and time
        xQueueSend(beep_queue, &current_beep, 0);                //start buzzer
    }
    if(inStandby) {                                              //if device is in standby
      standbyTimer++;                                            //increment standby timer
      if(standbyTimer > BLE_SLEEP_TIMEOUT) {                     //if standby timer exceeds BLE sleep_timeout
        startSleepMode();                                        //enter device to sleep
      }
    } 
    vTaskDelay(1000 / portTICK_PERIOD_MS);                       //wait 1s
  }
}

// Battery Monitor task
void TaskBatteryMonitor(void *pvParameters) {
  // Configure and read charging status and voltage 
  uint16_t adc_values[256];
  uint16_t adc_ptr = 0;
  uint32_t led_colour = RED;                             //initial LED colour is RED
  xQueueSend(led_colour_queue, &led_colour, 0);          //set LED colour

  bool onBattery;
  bool chargerConnected1 =0;

  pinMode(CHRG, INPUT_PULLUP);                           // making pin as input and pulling it up
  onBattery = digitalRead(CHRG);                         //check for charger connection(1 or 0)
  

  for (uint16_t i = 0; i < 256; i++) {
      adc_values[i] = analogReadMilliVolts(VOLTAGE) * 2; // read battery voltage for every 1ms 
      vTaskDelay(1 / portTICK_PERIOD_MS);                //wait 1ms
  }

  while(true) {
    onBattery = digitalRead(CHRG);                       //check for charger connection(1 or 0)
    log_d("Charger Val %d", onBattery);                  //print whether the device is in battery


   if(chargerConnected1 && digitalRead(CHRG) == true)    //if charger disconnected
   {
        chargerConnected1 = false;                       //update as charger disconnected
        log_d("Charger Removed");                        //print charger removed msg

       vTaskDelay(100 / portTICK_PERIOD_MS);            //wait 100ms

      for (uint16_t i = 0; i < 256; i++) {
      adc_values[i] = analogReadMilliVolts(VOLTAGE) * 2; // read battery voltage for every 1ms
      vTaskDelay(1 / portTICK_PERIOD_MS);                //wait 1ms
  }
        
      } 
      
      else if(!chargerConnected1 && digitalRead(CHRG) == false) //if charger connected
      { 
        chargerConnected1 = true;                               //update as charger connected
        log_d("Charger Connected");                             //print charger conected msg  
      }
    
    adc_values[adc_ptr] = analogReadMilliVolts(VOLTAGE) * 2;    //read battery voltage
    adc_ptr += 1;                                               //increment array element
    if(adc_ptr >= 256) {                                        //if array element value exceeds
      adc_ptr = 0;                                              //reset the value
    }

    uint32_t sum = 0;                                           //initialise sum value
    for(uint16_t i = 0; i < 256; i++) {
        sum += adc_values[i];                                   //add all the adc value
    }
    uint16_t adc_avg = (uint16_t)(sum / 256);                   //get average adc value
    uint8_t battery_level = get_battery_level(adc_avg);         //get battery percentage
    
    uint32_t led_colour;
    if(battery_level < 25 && battery_level > 15 && onBattery) {
      led_colour = WHITE;                                       //update LED colour as white if 25<battery level>15
    } 
    
    else if(battery_level < 16 && onBattery) {
      led_colour = RED;                                         //update LED colour as red if battery level<16
    } 

    else {
      led_colour = BLUE;                                        //for any other value LED colour is blue
    }
    xQueueSend(led_colour_queue, &led_colour, 0);               //set LED colour 

    log_d("Battery - %d, On Battery - %d", battery_level, onBattery);   //print Battery level and status
    battery_status_packet(battery_level, onBattery);                    //prepare packet for battery level and status
    send_ble_packet();                                                  //send packet
    vTaskDelay(1000 / portTICK_PERIOD_MS);                              //wait 1s
  }
}

// Task to update LED
void TaskUpdateLED(void *pvParameters) {
  led_colour_queue = xQueueCreate(5, sizeof(uint32_t));           //create a task for changing LED colour
  led_colour_override_queue = xQueueCreate(5, sizeof(uint32_t));  //create a task for overriding the LED colour
  led_blink_semaphore = xSemaphoreCreateBinary();                 //task for lED blinking
  led_steady_semaphore = xSemaphoreCreateBinary();                //task for steady led

  uint32_t led_colour = RED;                                      //initial LED colour is RED
  bool isLEDBlinking = true;                                      //initial LED status is blinking
  bool ledColourOverride = false;                                 //no LED colour overriding at start

  xQueueSend(led_colour_queue, &led_colour, 0);                   //set LED colour
  xSemaphoreGive(led_blink_semaphore);                            //get led blinking task

  uint32_t current_led_colour = RED;                              //initialise current led colour as RED

  while(true) {
    if(!ledColourOverride) {                                                    //if there's no LED overriding 
      if(xQueueReceive(led_colour_queue, &current_led_colour, 0) == pdTRUE) {   //updating LED colour 
          // log_d("Colour Updated to %d", current_led_colour);   
      }
    }

    if(xSemaphoreTake(led_blink_semaphore, 0) == pdTRUE) {                                //if led blink task is running 
        log_d("LED Blinking Enabled");                                                    //print LED blinking
        isLEDBlinking = true;                                                             //update LED blinking is true
    }

    if(xSemaphoreTake(led_steady_semaphore, 0) == pdTRUE) {                               //if LED steady task is running
        log_d("LED Blinking Disabled");                                                   // print LED blinking disabled
        isLEDBlinking = false;                                                            // LED blinking false (steady glowing LED)
    }

    if(xQueueReceive(led_colour_override_queue, &current_led_colour, 0) == pdTRUE) {      //updating LED colour
        log_e("Colour Override to %d", current_led_colour);                               //print current LED colour
        if(current_led_colour == OFF) {                                                   //if Curennt LED is OFF
          ledColourOverride = false;                                                      //LED colour override is not required
        } else {
          ledColourOverride = true;                                                       //LED colour override is required
        }   
    }

    if(get_led_colour() != OFF && isLEDBlinking) {                 //if LED is not OFF and it's blinking
        set_led_colour(OFF);                                       //set LED as OFF
    } else {
        set_led_colour(current_led_colour);                        //else update the LED colour
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);                         //wait 1s
  }
} 

// Task for switch press
void TaskSwitchPressed(void *pvParameters) {
  switch_pressed_semaphore = xSemaphoreCreateBinary();             //create task for when switch pressed
  sleep_semaphore = xSemaphoreCreateBinary();                      //create a task for entering sleep
  pinMode(SWITCH, INPUT);                                          //set switch pin as input
  attachInterrupt(SWITCH, switch_isr, RISING);                     //create an iterrupt process for switch
  uint8_t switch_pressed_duration = 0;                             //switch press time duration
  bool switch_pressed_flag = false;                                //flag for switch press
  while(true) {
    if(xSemaphoreTake(switch_pressed_semaphore, 0) == pdTRUE) {    //if switch pressed
        switch_pressed_flag = true;                                //set flag as switch is pressed
    }
    if(switch_pressed_flag) {                                      //if switch is pressed
        if(digitalRead(SWITCH)) {                                  //get state of switch
            switch_pressed_duration += 1;                          //increment switch pressed duration
        } else {
            switch_pressed_duration = 0;                           //reset switch press duration
            switch_pressed_flag = false;                           //reset the switch pressed flag
        }
    }
    if(switch_pressed_duration == 3) {                             //if switch pressed duration is 3
        BuzzerBeep current_beep = {1, 100};                        //buzzer count and time
        xQueueSend(beep_queue, &current_beep, 0);                  //start buzzer
    }
    if(switch_pressed_duration > 40 && (switch_pressed_duration % 2 == 0)) {    //if switch pressed duration is even and more than 40
        BuzzerBeep current_beep = {1, 100};                                     //buzzer count and time
        xQueueSend(beep_queue, &current_beep, 0);                               //sart buzzer
    }

    uint8_t long_press_threshold = 20;                              //peak value for long press
    if(isConnected()) {                                             //if ble connected
      long_press_threshold = 60;                                    //increase the peak value of long press
    }

    if(switch_pressed_duration >= long_press_threshold) {           //if switch pressed duration is greater than peak value
      startSleepMode();                                             //enter sleep mode
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);                           //wait 100ms
  }
}

// Task for Glucose test
void TaskGlucose(void *pvParameters) {
  GluPort.begin(GLU_BAUD, SERIAL_8N1, GLU_RX, GLU_TX);      //set Tx, Rx and Baudrate, serial comm(UART 1) for Glu
  GluPort.setTimeout(500);                                  //set timeout for Gluc test 500ms
  
  bool glu_strip_inserted = false;                          //initial value for strip insertion
  bool glu_led_status = false;                              //intial value for led status
  bool glu_led_blinking = false;                            //initial value for led blinking status
  BuzzerBeep current_beep;

  pinMode(GLU_ON, OUTPUT);                                  //set Glu pin as output pin
  pinMode(GLU_LED, OUTPUT);                                 //set Glu Led pin output 
  pinMode(GLU_DET, INPUT_PULLUP);                           //pullup means the pin is connected to Vcc // making pin as input and pulling it up

  const int GLU_RX_BUFFER_SIZE = 20;                        //glu Rx buffer size
  uint8_t glu_rx_buffer[GLU_RX_BUFFER_SIZE];                //Glu rx buffer
  glucose_start_semaphore = xSemaphoreCreateBinary();       //start glu task
  glucose_state = TEST_NOT_STARTED;                         //initial glu test state
  uint16_t glucose_value = 0x0000;                          //intial glu value
  log_e("Glu task Created");
  while(true) {
    if(xSemaphoreTake(glucose_start_semaphore, 0) == pdTRUE) {    //Glu test started
      log_e("GLU Test Started");                                  //print Glu test started
      glucose_state = TEST_STARTED;                               //change Glu test state as test started
      digitalWrite(GLU_ON, HIGH);                                 //set Glu test pin high
      vTaskDelay(3000 / portTICK_PERIOD_MS);                      //wait 3s
      glu_led_status = false;                                     //glucose led status as false
      glu_led_blinking = true;                                    //glucose LED blinking
      glu_strip_inserted = false;                                 //no strip inserted
      glucose_test_status(glucose_state, glucose_value);    //send test status
      send_ble_packet();                                          //send packet in Ble
    }

    if(glucose_state == TEST_STARTED) {                               //test started
      if(GluPort.available()) {                                       //if Glu port is available
        delay(500);                                                   //wait 500ms
        uint8_t rx_packet_length = 0;                                 //intialise Rx packet length
        while(GluPort.available()) {                                  //while Glu port is busy 
          glu_rx_buffer[rx_packet_length++] = GluPort.read();         //read data from the port
          log_e("GLU data : %x", glu_rx_buffer[rx_packet_length-1]);  //print read values
        }
        log_e("GLU data RX - %d bytes", rx_packet_length);            //print legth of the packet received

        if(rx_packet_length >= 8) {                                   //if packet length is greater than 8
          uint8_t module_function = glu_rx_buffer[4];                 //get module function
          log_e("GLU data RX - %d bytes", glu_rx_buffer[13]);         //
          uint8_t module_status = glu_rx_buffer[5];                   //get module status
          uint16_t module_value = ((uint16_t)(glu_rx_buffer[6]));     //get module value
          module_value += ((uint16_t)(glu_rx_buffer[7]))<<8;          //get module value

          if(module_function == 0x03) {                               //if module function is Glucose
              if(!(module_status == TEST_COMPLETED_AND_VALID && !glu_strip_inserted)) {   //if module status is completed
            glucose_test_status(module_status, module_value);                       //Update Gluc test status
            log_e("Glucose Blood Value Result :- %d mg/dl", module_value);                //print the Glu value of patient in mg
          }
          }else {
            glucose_test_status(WRONG_STRIP_ERROR, 0x0000);                         //if wrong strip is inserted then update test as with error value
            module_status = WRONG_STRIP_ERROR;                                            //update test status as strip error
          }
          send_ble_packet();                                                              //send ble packet

          if ((rx_packet_length >= 12) && (glu_rx_buffer[13] == CONTROL_SOLUTION_RESULT)) //if rx packet length is more than 12 and ??
          {
             // uint8_t control_status = glu_rx_buffer[5];
              uint16_t control_value = ((uint16_t)(glu_rx_buffer[14]));                   //get control sol value
              control_value += ((uint16_t)(glu_rx_buffer[15]))<<8;                        //get control sol value

              
              if(!(glu_rx_buffer[13] == CONTROL_SOLUTION_RESULT && !glu_strip_inserted)) {  //if test with control sol is completed
            glucose_test_status(CONTROL_SOLUTION_RESULT, control_value);              //update test results
            log_e("Glucose Control Solution Result :- %d mg/dl", control_value);            //print the glu value of patient in mg
          }
          
          else {
            glucose_test_status(WRONG_STRIP_ERROR, 0x0000);      //if wrong strip is inserted then update test with error value
            module_status = WRONG_STRIP_ERROR;                         //update test status as strip error
          }
          send_ble_packet();                                           //send ble packet
          }

          switch(module_status) {
            case STRIP_INSERTED:                            //if strip is inserted
                glu_strip_inserted = true;                  //update as strip inserted
                glu_led_blinking = false;                   //stop blinking glu LED
                glu_led_status = true;                      //steady glow LED
                break;

            case SAMPLE_APPLIED:                            //if sample applied
              current_beep = {3, 100};                      //beep count and time
              xQueueSend(beep_queue, &current_beep, 0);     //start beep
              break;
              
            case CALIBRATION_STRIP_INSERTED:                //for calibration strip inserted

            case CONTROL_SOLUTION_RESULT:                   //control solution result

            case TEST_COMPLETED_AND_VALID:                  //test completed
              current_beep = {2, 100};                      //beep coiunt and time
              xQueueSend(beep_queue, &current_beep, 0);     //start beep
              glucose_state = TEST_NOT_STARTED;             //update glu test status as test not started
              digitalWrite(GLU_ON, LOW);                    //set Glu test pin to low
              glu_led_status = false;                       //turn off led
              glu_led_blinking = false;                     //turn off blinking led
              break;

            case CALIBRATION_STRIP_ERROR_1:                 //calibration strip error

            case CALIBRATION_STRIP_ERROR_2:                 //calibartion strip error

            case STRIP_REMOVED_BEFORE_SAMPLE:               //strip removed before applying sample
              glu_strip_inserted = false;                   //update strip inserted as false
              //  current_beep = {4, 100};
              //  xQueueSend(beep_queue, &current_beep, 0);
              glu_led_status = true;                        //LED should be ON
              glu_led_blinking = true;                      //LED should be blinking
              log_e("Glucose : Strip Removed");             //print strip removed msg
              break;
            case WRONG_STRIP_ERROR:                         //wrong strip insertion

            case USED_STRIP_ERROR:                          //used strip insertion

            case STRIP_ERROR:                               //strip error
              current_beep = {4, 100};                      //beep count and time
              xQueueSend(beep_queue, &current_beep, 0);     //start beep
              glu_led_status = false;                       //update LED status as false
              glu_led_blinking = true;                      //update LEd Blinking as true
              log_e("GLU : Strip Error");                   //print strip error
              break;

              case MODULE_POWER_OFF:                        //module power OFF
              current_beep = {4, 100};                      //beep count and time
              xQueueSend(beep_queue, &current_beep, 0);     //start beep 
              glu_led_status = true;                        //update glu led status as true
              glu_led_blinking = true;                      //update led blinking as true
              log_e("Glucose : Module Power OFF");          //print glu module power is off
              break;

            default:                                                      //by default
              glucose_state = TEST_NOT_STARTED;                           //Glucose state is test not started
              digitalWrite(GLU_ON, LOW);                                  //Glu test pin is low
              glu_led_status = false;                                     //update glu led status as false
              glu_led_blinking = false;                                   //update glu led blinking as false
              
              glucose_test_status(glucose_state, glucose_value);  //update Gluc test status with Glucose test
              //glucose_test_status(glucose_state, glucose_value);
              log_e("GLU Test Terminated");                               //print Glu test teminated msg
              break;
          }
        }
      }
    } else {                                      //if glu test is not started
      glu_led_blinking = false;                   //disable glu led blinking
      glu_led_status = false;                     //disable led status
    }
    if(glu_led_blinking) {                        //if led blinking
      glu_led_status = !glu_led_status;           // reverse the LED status
    }
    digitalWrite(GLU_LED, glu_led_status);        //change the Glu LED status
    vTaskDelay(500 / portTICK_PERIOD_MS);         //wait 500ms
  }
}


// Task Cholestrol
void TaskCholesterol(void *pvParameters) {
  CholPort.begin(CHOL_BAUD, SERIAL_8N1, CHOL_RX, CHOL_TX);      //set Tx, Rx and Baudrate, serial comm(UART 2) for Chol
  CholPort.setTimeout(500);
 
  bool chol_strip_inserted = false;
  bool chol_led_status = false;
  bool chol_led_blinking = false;
  BuzzerBeep current_beep;

  pinMode(CHOL_ON, OUTPUT);
  pinMode(CHOL_LED, OUTPUT);
  pinMode(CHOL_DET, INPUT_PULLUP);

  const int CHOL_RX_BUFFER_SIZE = 20;
  uint8_t chol_rx_buffer[CHOL_RX_BUFFER_SIZE];
  cholesterol_start_semaphore = xSemaphoreCreateBinary();
  cholesterol_state = TEST_NOT_STARTED;
  uint16_t cholesterol_value = 0x0000;

  while(true) {
    if(xSemaphoreTake(cholesterol_start_semaphore, 0) == pdTRUE) {
      log_e("CHOL/Uric Test Started");
      cholesterol_state = TEST_STARTED;
      digitalWrite(CHOL_ON, HIGH);
      vTaskDelay(3000 / portTICK_PERIOD_MS);
      chol_led_status = false;
      chol_led_blinking = true;
      chol_strip_inserted = false;
      cholesterol_test_status(cholesterol_state, cholesterol_value, 0x00);
      send_ble_packet();
    }

    if(cholesterol_state == TEST_STARTED) {
      if(CholPort.available()) {
        delay(500);
        uint8_t rx_packet_length = 0;
        while(CholPort.available()) {
          chol_rx_buffer[rx_packet_length++] = CholPort.read();
          log_e("CHOL/Uric data : %x", chol_rx_buffer[rx_packet_length-1]);
        }
        log_e("CHOL/Uric data RX - %d bytes", rx_packet_length);

        if(rx_packet_length >= 8) {
          uint8_t module_function = chol_rx_buffer[4];
          uint8_t module_status = chol_rx_buffer[5];
          uint16_t module_value = ((uint16_t)(chol_rx_buffer[6])); 
          module_value += ((uint16_t)(chol_rx_buffer[7]))<<8; 

          if(module_function == 0x02) {
              if(!(module_status == TEST_COMPLETED_AND_VALID && !chol_strip_inserted)) {
            cholesterol_test_status(module_status, module_value, 0x00);
            log_e("Chol/Uric Blood value Result :- %d mg/dl", module_value);
              }
          } else if(module_function == 0x01) {
            cholesterol_test_status(module_status, module_value, 0x01);          
          }else {
            cholesterol_test_status(WRONG_STRIP_ERROR, 0x0000, 0x00);
            module_status = WRONG_STRIP_ERROR;
          }
          send_ble_packet();

          if ((rx_packet_length >= 12) && (chol_rx_buffer[13] == CONTROL_SOLUTION_RESULT))
          {
             // uint8_t control_status = glu_rx_buffer[5];
              uint16_t control_value = ((uint16_t)(chol_rx_buffer[14])); 
              control_value += ((uint16_t)(chol_rx_buffer[15]))<<8;

              
              if(!(chol_rx_buffer[13] == CONTROL_SOLUTION_RESULT && !chol_strip_inserted)) {
            cholesterol_test_status(CONTROL_SOLUTION_RESULT, control_value, 0x00);
            log_e("Chol/Uric Control Solution Result :- %d mg/dl", control_value);
          }
          
          else {
            cholesterol_test_status(WRONG_STRIP_ERROR, 0x0000, 0x00);
            module_status = WRONG_STRIP_ERROR;
          }
          send_ble_packet();
          }

          switch(module_status) {
            case STRIP_INSERTED:
                chol_strip_inserted = true;
                chol_led_blinking = false;
                chol_led_status = true;
                break;

            case SAMPLE_APPLIED:
              current_beep = {3, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              break;
              
            case CALIBRATION_STRIP_INSERTED:
            case CONTROL_SOLUTION_RESULT:
            case TEST_COMPLETED_AND_VALID:
              current_beep = {2, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              cholesterol_state = TEST_NOT_STARTED;
              digitalWrite(CHOL_ON, LOW);
              chol_led_status = false;
              chol_led_blinking = false;
              break;

            case CALIBRATION_STRIP_ERROR_1:
            case CALIBRATION_STRIP_ERROR_2:
            case STRIP_REMOVED_BEFORE_SAMPLE:
            chol_strip_inserted = false;
          //  current_beep = {4, 100};
              //xQueueSend(beep_queue, &current_beep, 0);
              chol_led_status = true;
              chol_led_blinking = true;
              log_e("CHOL/Uric : Strip Removed");
              break;
          
            case WRONG_STRIP_ERROR:
            case USED_STRIP_ERROR:
            case STRIP_ERROR:
              current_beep = {4, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              chol_led_status = false;
              chol_led_blinking = true;
              log_e("CHOL/Uric : Strip Error");
              break;

              case MODULE_POWER_OFF:
              current_beep = {4, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              chol_led_status = true;
              chol_led_blinking = true;
              log_e("CHOL/Uric : Module Power OFF");
              break;
            
            default:
              cholesterol_state = TEST_NOT_STARTED;
              digitalWrite(CHOL_ON, LOW);
              chol_led_status = false;
              chol_led_blinking = false;
              if(module_function == 0x02) {
                cholesterol_test_status(cholesterol_state, cholesterol_value, 0x00);
              } else {
                cholesterol_test_status(cholesterol_state, cholesterol_value, 0x01);
              }
              log_e("CHOL/Uric Test Terminated");
              break;
          }
        }
      }
    } else {
      chol_led_blinking = false;
      chol_led_status = false;
    }
    if(chol_led_blinking) {
      chol_led_status = !chol_led_status;
    }
    digitalWrite(CHOL_LED, chol_led_status);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task Hemoglobin
void TaskHemoglobin(void *pvParameters) {
  HbPort.begin(HB_BAUD, SERIAL_8N1, HB_RX, HB_TX);
  HbPort.setTimeout(500);

  bool hb_strip_inserted = false;
  bool hb_led_status = false;
  bool hb_led_blinking = false;
  BuzzerBeep current_beep;

  pinMode(HB_ON, OUTPUT);
  pinMode(HB_LED, hb_led_status);
  pinMode(HB_DET, INPUT_PULLUP);

  const int HB_RX_BUFFER_SIZE = 20;
  uint8_t hb_rx_buffer[HB_RX_BUFFER_SIZE];
  hemoglobin_start_semaphore = xSemaphoreCreateBinary();
  hemoglobin_state = TEST_NOT_STARTED;
  uint16_t hemoglobin_value = 0x0000;

  while(true) {
    if(xSemaphoreTake(hemoglobin_start_semaphore, 0) == pdTRUE) {
      log_e("HB Test Started");
      hemoglobin_state = TEST_STARTED;
      digitalWrite(HB_ON, HIGH);
      vTaskDelay(3000 / portTICK_PERIOD_MS);
      hb_led_status = false;
      hb_led_blinking = true;
      hb_strip_inserted = false;
      hemoglobin_test_status(hemoglobin_state, hemoglobin_value);
      send_ble_packet();
    }

    if(hemoglobin_state == TEST_STARTED) {

      if(HbPort.available()) {
        delay(500);
        uint8_t rx_packet_length = 0;
        while(HbPort.available()) {
          hb_rx_buffer[rx_packet_length++] = HbPort.read();
          log_e("HB data : %x", hb_rx_buffer[rx_packet_length-1]);
        }
        log_e("HB data RX - %d bytes", rx_packet_length);

        if(rx_packet_length >= 8) {
          uint8_t module_function = hb_rx_buffer[4];
          uint8_t module_status = hb_rx_buffer[5];
          uint16_t module_value = ((uint16_t)(hb_rx_buffer[6])); 
          module_value += ((uint16_t)(hb_rx_buffer[7]))<<8; 

          if(module_function == 0x0E) {
            if(!(module_status == TEST_COMPLETED_AND_VALID && !hb_strip_inserted)) {
              hemoglobin_test_status(module_status, module_value);
              log_e("Hb Bolood value Result :- %d mg/dl", module_value);
            }
          } else {
            hemoglobin_test_status(WRONG_STRIP_ERROR, 0x0000);
            module_status = WRONG_STRIP_ERROR;
          }
          send_ble_packet();

          if ((rx_packet_length >= 12) && (hb_rx_buffer[13] == CONTROL_SOLUTION_RESULT))
          {
             // uint8_t control_status = glu_rx_buffer[5];
              uint16_t control_value = ((uint16_t)(hb_rx_buffer[14])); 
              control_value += ((uint16_t)(hb_rx_buffer[15]))<<8;

              
              if(!(hb_rx_buffer[13] == CONTROL_SOLUTION_RESULT && !hb_strip_inserted)) {
            hemoglobin_test_status(CONTROL_SOLUTION_RESULT, control_value);
            log_e("Hb Control Solution Result :- %d mg/dl", control_value);
          }
          
          else {
            hemoglobin_test_status(WRONG_STRIP_ERROR, 0x0000);
            module_status = WRONG_STRIP_ERROR;
          }
          send_ble_packet();
          }

          switch(module_status) {
            case STRIP_INSERTED:
                hb_strip_inserted = true;
                hb_led_blinking = false;
                hb_led_status = true;
                break;

            case SAMPLE_APPLIED:
              current_beep = {3, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              break;
              
            case CALIBRATION_STRIP_INSERTED:
            case CONTROL_SOLUTION_RESULT:
            case TEST_COMPLETED_AND_VALID:
              current_beep = {2, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              hemoglobin_state = TEST_NOT_STARTED;
              digitalWrite(HB_ON, LOW);
              hb_led_status = false;
              hb_led_blinking = false;
              break;

            case CALIBRATION_STRIP_ERROR_1:
            case CALIBRATION_STRIP_ERROR_2:
            case STRIP_REMOVED_BEFORE_SAMPLE:
            hb_strip_inserted = false;
            hb_led_status = true;
            hb_led_blinking = true;
            log_e("Hb : Strip Removed");
            break;

            case WRONG_STRIP_ERROR:
            case USED_STRIP_ERROR:
            case STRIP_ERROR:
              current_beep = {4, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              hb_led_status = false;
              hb_led_blinking = true;
              log_e("HB : Strip Error");
              break;

            case MODULE_POWER_OFF:
              current_beep = {4, 100};
              xQueueSend(beep_queue, &current_beep, 0);
              hb_led_status = true;
              hb_led_blinking = true;
              log_e("Hb : Module Power OFF");
              break;
            
            default:
              hemoglobin_state = TEST_NOT_STARTED;
              digitalWrite(HB_ON, LOW);
              hb_led_status = false;
              hb_led_blinking = false;
              hemoglobin_test_status(hemoglobin_state, hemoglobin_value);
              log_e("HB Test Terminated");
              break;
          }
        }
      }
    } else {
      hb_led_blinking = false;
      hb_led_status = false;
    }
    
    if(hb_led_blinking) {
      hb_led_status = !hb_led_status;
    }
    digitalWrite(HB_LED, hb_led_status);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Starts test
void startTests(uint8_t glu_test_flag, uint8_t hb_test_flag, uint8_t chol_test_flag) {
  bool start_beep = false;                                          //start beep is off
  log_e("Starting Tests");
  clear_test_status();                                              //clear previous test status

  if(glu_test_flag && glucose_state == TEST_NOT_STARTED) {          //if glucose test is not started
    xSemaphoreGive(glucose_start_semaphore);                        // start task
    start_beep = true;                                              //activate start beep
  }
  if(hb_test_flag && hemoglobin_state == TEST_NOT_STARTED) {        //if hemoglobin test is not started
    hemoglobin_state = TEST_STARTED;                                //update hemoglobin state as test started
    start_beep = true;                                              //activate start beep
    xSemaphoreGive(hemoglobin_start_semaphore);                     //start hemoglobin test
  }
  if(chol_test_flag && cholesterol_state == TEST_NOT_STARTED) {     //if cholestrol test is not started
    cholesterol_state = TEST_STARTED;                               //update cholestrol state as test started
    start_beep = true;                                              //activate start beep
    xSemaphoreGive(cholesterol_start_semaphore);                    //start cholestrol test
  }
  if(start_beep) {                                                  //if beep activated
    BuzzerBeep current_beep = {2, 100};                             //beep count and time
    xQueueSend(beep_queue, &current_beep, 0);                       //start beep
  }
}

// Aborting ongoing tests
void abortTests(uint8_t glu_test_flag, uint8_t hb_test_flag, uint8_t chol_test_flag) {
  bool abort_beep = false;
  log_d("Abort Test : GLU = %x, HB = %x, CHOL = %x", glucose_state, hemoglobin_state, cholesterol_state); //print state of all tests
  
  if(glu_test_flag && glucose_state != TEST_NOT_STARTED) {          //if glu test is started
    glucose_state = TEST_NOT_STARTED;                               //update glu test  state
    digitalWrite(GLU_ON, LOW);                                      //disable glu test pin
    digitalWrite(GLU_LED, LOW);                                     //disable glu LED
    glucose_test_status(TEST_ABORTED, 0x0000);                //update glu test status as test aborted
    abort_beep = true;                                              //abort beep
    log_d("GLU Test Aborted");                                      //print test aborted
  }
  
  if(hb_test_flag && hemoglobin_state != TEST_NOT_STARTED) {        //if hemo test is started
    hemoglobin_state = TEST_NOT_STARTED;                            //update hemo test state
    digitalWrite(HB_ON, LOW);                                       //disable hemo test pin
    digitalWrite(HB_LED, LOW);                                      //disable hemo LED
    hemoglobin_test_status(TEST_ABORTED, 0x0000);                   //update hemo test status as test aborted
    abort_beep = true;                                              //abort beep
    log_d("HB Test Aborted");                                       //print test aborted
  }

  if(chol_test_flag && cholesterol_state != TEST_NOT_STARTED) {     //if chol test is started
    cholesterol_state = TEST_NOT_STARTED;                           //update chol test state
    digitalWrite(CHOL_ON, LOW);                                     //disable chol test pin
    digitalWrite(CHOL_LED, LOW);                                    //disable chol LED
    cholesterol_test_status(TEST_ABORTED, 0x0000, 0x00);            //update chol test status as test aborted
    abort_beep = true;                                              //abort beep
    log_d("CHOL Test Aborted");                                     //print chol test aborted
  }
  if (abort_beep) {                                                 //if abort beep is true
    BuzzerBeep current_beep = {4, 100};                             //beep count and time
    xQueueSend(beep_queue, &current_beep, 0);                       //start beep
    send_ble_packet();                                              //send ble packet
  }
}

// Send previous result
void send_previous_result() {
  load_previous_values();     //get previous test values
  send_ble_packet();          //send test result via ble
}

// Watchdog 
void TaskWatchdog(void *pvParameters) {
  bool pingState = HIGH;

  pinMode(ESP_PING, OUTPUT);                //select a pin for watchdog // making pin as output and pulling it up
  digitalWrite(ESP_PING, pingState);        //ping esp32

  while(true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);  //wait 1s
    pingState = !pingState;                 //reverse pin state
    digitalWrite(ESP_PING, pingState);      //ping esp32
    esp_task_wdt_reset();                   //reset watchdog timer
  }
}

// Enter sleep mode
void startSleepMode() {
  detachInterrupt(SWITCH);                        //disable interrupt from switch
  log_e("Sleeping");                              //print device is sleeping

  if(batteryMonitorTaskHandle != NULL) {          //if Battery monitor task is running
    vTaskDelete(batteryMonitorTaskHandle);        //delete battery monitoring task
    batteryMonitorTaskHandle = NULL;              //battery monitor stopped
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);          //wait 1s

  uint32_t led_colour = OFF;                      //update LED colour to off
  xSemaphoreGive(led_steady_semaphore);           //
  xQueueSend(led_colour_queue, &led_colour, 0);   //set LED OFF
  log_d("Turning LED OFF......");                 //print turning off led


  BuzzerBeep current_beep = {3, 500};             //beep count and time
  xQueueSend(beep_queue, &current_beep, 0);       //start beep

  vTaskDelay(4000 / portTICK_PERIOD_MS);          //wait 4s

  if(ledUpdateTaskHandle != NULL) {               //if LED upate task is running
    vTaskDelete(ledUpdateTaskHandle);             //delete LED update task
    ledUpdateTaskHandle = NULL;                   //LED update task is stopped
  }

  if(buzzerTaskHandle != NULL) {                  //if buzzer task is running
    vTaskDelete(buzzerTaskHandle);                //delete buzzer task
    buzzerTaskHandle = NULL;                      //buzzer task is deleted
  }

  bool onBattery = digitalRead(CHRG);             //check for charger connection
  if(!onBattery) {                                //if charger connected
      led_colour = ORANGE;                        //update LED colour as orange
      xQueueSend(led_colour_queue, &led_colour, 0); //send LED udpating task
      log_d("Setting LED Orange......");            //print msg
  }
  
  set_led(led_colour);                                //set led colour
  vTaskDelay(100 / portTICK_PERIOD_MS);               //wait 100ms
  esp_sleep_enable_timer_wakeup(CHARGER_POLL_TIMER);  //wake-up device every 10s to check any changes in th device
  vTaskDelay(3000 / portTICK_PERIOD_MS);              //wait 3s
  esp_deep_sleep_start();                             //enter sleep
}

// Stop module task
void stopModuleTasks() {
  digitalWrite(GLU_ON, LOW);            //Set glucose test pin as low(disable glucose test)
  digitalWrite(HB_ON, LOW);             //Set Hemoglobin test pin as low(disable hemoglobin test)
  digitalWrite(CHOL_ON, LOW);           //Set cholestrol test pin as low(disable cholestrol test)

  if(GlucoseTaskHandle != NULL) {       //if glucose task is running
    vTaskDelete(GlucoseTaskHandle);     //delete glucose task
    GlucoseTaskHandle = NULL;           //glucose task is stopped
  }

  if(HemoglobinTaskHandle != NULL) {    //if hemoglobin task is running
    vTaskDelete(HemoglobinTaskHandle);  //delete hemo task
    HemoglobinTaskHandle = NULL;        //hemo task is stopped
  }

  if(CholesterolTaskHandle != NULL) {   //if chol task is running
    vTaskDelete(CholesterolTaskHandle); //delete chol task
    CholesterolTaskHandle = NULL;       //chol task is stopped
  }
}
// Start module tasks
void startModuleTasks() {
  clear_test_status();     //Clear previous test status
  createGlucoseTask();     //create a task for Glucose
  createHemoglobinTask();  //create a task for Hemoglobin
  createCholesterolTask(); //create a task for Cholestrol
}

// Suspending ongoing Tasks
void suspendTasks() {
  stopModuleTasks();  //stop all the tasks
  vTaskSuspend(switchPressedTaskHandle);  //Suspend a particular task
}

// Resume pending tasks
void resumeTasks() {
  startModuleTasks();                   //Start module tasks
  vTaskResume(switchPressedTaskHandle); //Start a particular task
}

/* Starting all the tasks in the background */
void startBackgroundTasks() {
  createWatchdogTask();        // Create a task for watchdog
  createLEDUpdateTask();       // Create a task for LED update
  createBatteryMonitorTask();  // Create a task for Battery monitor
  createSwitchPressedTask();   // Create a task for Switch pressed
  createBLEMonitorTask();      // Create a task for BLE monitor
  createBuzzerTask();          // Create a task for Buzzer
}
