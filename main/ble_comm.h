#ifndef BIOCHEM_BLE_COMM_H_
#define BIOCHEM_BLE_COMM_H_

#include "ble_packets.h"
#include "Arduino.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

#include "esp_ota_ops.h"

#define DEVICE_NAME             "Biochem - "
#define SERVICE_UUID            "e623efa7-195b-4d49-8840-ae20bfc888a8"
#define CHARACTERISTIC_UUID     "2356a338-be1d-41dc-9915-82852faf3d70"
#define OTA_CHARACTERISTIC_UUID "ffea6668-3006-47fa-9acc-1a2cd9fe6f8e"

#define OTA_PACKET_SIZE         512

extern SemaphoreHandle_t bleConnectedSemaphore;
extern SemaphoreHandle_t bleDisconnectedSemaphore;


class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
};

class DataCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic);
};

class OTACallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic);
};


void setup_ble();
void send_ble_packet();
void send_ble_packet(uint8_t* packet, uint8_t packet_length);
bool isConnected();

#endif