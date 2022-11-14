#include "ble_comm.h"
#include "ble_packets.h"
#include "hardware.h"
#include "task_manager.h"
#include <string>

BLEServer*          pServer = NULL;
BLECharacteristic*  pCharacteristic = NULL;
BLECharacteristic*  pOTACharacteristic = NULL;

esp_ota_handle_t otaHandler = 0;

volatile bool bleConnected = false;     //variable which stores only 0 or 1
volatile bool otaEnabledFlag = false;
volatile bool otaStartedFlag = false;

// When the device is connected to server it will enter this function
void MyServerCallbacks::onConnect(BLEServer* pServer) {
  bleConnected = true;       //update the variable after conection
  xSemaphoreGive(bleConnectedSemaphore);  // assign a thread for the task
  startModuleTasks();        // Start the Tasks
  log_e("BLE Connected");    // Print Ble connected msg
}

// When server disconnected it will enter this function
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  bleConnected = false;                      //update variable after disconnection
  xSemaphoreGive(bleDisconnectedSemaphore);  //assign a thread for Ble disconnection
  log_e("BLE Disconnected");                 //print Ble disnconnected msg
  stopModuleTasks();                         //Stop all the tasks
  pServer->startAdvertising();               //start advertising for device connection (Show visibility to other devices)
}

//  when OTA data receives it will enter this function
void OTACallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  if(otaEnabledFlag) {                                //checking whther the OTA flag is enabled or not
    std::string rxData = pCharacteristic->getValue(); //storing received data in a string
    if (!otaStartedFlag) {                            //
      log_e("Begin OTA");                             //Begin OTA msg
      suspendTasks();                                 //suspend all the ongoing tasks
      esp_ota_begin(esp_ota_get_next_update_partition(NULL), OTA_SIZE_UNKNOWN, &otaHandler);  //start receiving data
      otaStartedFlag = true;                          // update the variable after starting OTA
    }
    if (rxData.length() > 0)                          //if data received means length will be more than 0
    {
      esp_ota_write(otaHandler, rxData.c_str(), rxData.length());   //Write OTA data on esp32
      if (rxData.length() != OTA_PACKET_SIZE)         //if received data legth is not equal to expected data length
      {
        esp_ota_end(otaHandler);                      //stop receiving data
        log_e("End OTA");
        if (ESP_OK == esp_ota_set_boot_partition(esp_ota_get_next_update_partition(NULL))) {    //Firmware is not working properly
          vTaskDelay(2000 / portTICK_PERIOD_MS);      //wait for 2s
          esp_restart();                              //restart the device
        } else {
          log_e("OTA Error");                         //print OTA error
          otaStartedFlag = false;                     //disable flag
          resumeTasks();                              //resume tasks
        }
      }
    }
    uint8_t ackData[5] = {0x55, 0xAA, 0x03, 0xBB, 0x41};  //prepare acknowledgement data
    pOTACharacteristic->setValue((uint8_t*)ackData, 5);   //send ack data
    pOTACharacteristic->notify();                         //check data is received or not
  }
}

// when data received serially it will enter this function
void DataCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  uint8_t* data = pCharacteristic->getData();         // Collect data byte by byte
  std::string value = pCharacteristic->getValue();    //collect value from bluetooth

  if (value.length() > 4) {                           //if value length is greater than 4
    uint8_t packet_length = (uint8_t)(data[2]);       //copy the packet length to a variable
    log_e("packet_length=%x",data[2]);
    uint8_t command = (uint8_t)(data[3]);             //copy the command to a variable
    log_e("command=%x",data[3]);

    switch(command) {                                 //switch case for the command received
      case 0xB1:
        set_module_state(data[4], data[5]);           //set the state of the module
        log_e("data4=%x data5=%x",data[4],data[5]);
        send_ble_packet(data, packet_length+2);       //send the packet through bluetooth
        break;
      
      case 0xB2:
        send_ble_packet(data, packet_length+2);       //send the packet to the bluetooth connected device
        set_led_state(data[4], data[5]);              //set led color and blink state
        log_e("data4=%x data5=%x",data[4],data[5]);
        break;
      
      case 0xB3:
        //TODO - Implement Command Passthrough
        break;

      case 0xB4:
        device_info_packet(data[4]);                  //collect device info
        log_e("data4=%x",data[4]);
        send_ble_packet();                            //send that info through bluetooth
        break;

      case 0xB5:
        set_device_info(data);                        //set device information
        device_info_packet(data[4]);                  // read device info
        log_e("data4=%x",data[4]);
        send_ble_packet();                            //send that info througb bluetooth
        break;

      case 0xB6:
        send_ble_packet(data, packet_length+2);       //send data and it's packet length including checksum
        startSleepMode();                             //enter sleepmhode
        break;

      case 0xBB:
        otaEnabledFlag = true;                        //update ota enabled flag as true
        otaStartedFlag = false;                       //update ota start flag as false
        stopModuleTasks();                            //stop module tasks
        send_ble_packet(data, packet_length+2);       //send data via bluetooth
        break;

      case 0xA0:
        startTests(data[4], data[5], data[6]);        //start particular tests(probably multitest not sure)
        log_e("data4=%x data5=%x data6=%x",data[4],data[5],data[6]);
        break;

      case 0xA1:
        send_previous_result();                       // send previous test results
        break;

      case 0xA9:
        abortTests(data[4], data[5], data[6]);        //abort all tests
        log_e("data4=%x data5=%x data6=%x",data[4],data[5],data[6]);
        break;

      default:      
        log_e("Unknown Command");                                  //do nothing on default
        break;

    }
  }
}
// Setting up bluetooth connection
void setup_ble() {
  String display_number = get_parameter(0x07);                // get display number of the device
  String device_display_name = DEVICE_NAME + display_number;  // display name and id in available devices
  BLEDevice::init(device_display_name.c_str());               // initialize bluetooth device
  BLEDevice::setPower(ESP_PWR_LVL_N12);                       // set power required to achieve bluetooth communication
  pServer = BLEDevice::createServer();                        // create a server for the bluetooth communication
  pServer->setCallbacks(new MyServerCallbacks());             //
  BLEService *pService = pServer->createService(SERVICE_UUID);//

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new DataCallbacks());

  pOTACharacteristic = pService->createCharacteristic(
                      OTA_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pOTACharacteristic->addDescriptor(new BLE2902());
  pOTACharacteristic->setCallbacks(new OTACallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);      //respond true to a scan
  pAdvertising->setMinPreferred(0x06);      //min time for scan response
  pAdvertising->setMaxPreferred(0x10);      //max time for scan response
  BLEDevice::startAdvertising();            // show visibility to nearby devices
}

// Send complete packet
void send_ble_packet() {
  if(bleConnected) {                //if device is connected to the App
    pCharacteristic->setValue(get_packet_data(), get_packet_length());  //Get data and length of the packet
    pCharacteristic->notify();      // get acknowldgement after sending the data successfully
  }
}

// Send packet of user defined length
void send_ble_packet(uint8_t* packet, uint8_t packet_length) {
  if(bleConnected) {                //if device is connected to the App
    pCharacteristic->setValue(packet, packet_length);       //Set value
    pCharacteristic->notify();      // get acknowldgement after sending the data successfully
  }
}
// Checking Ble is connected or not
bool isConnected() {
  return bleConnected;              //return bleConnected value
}
