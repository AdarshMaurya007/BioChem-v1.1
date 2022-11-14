#ifndef BIOCHEM_BLE_PACKETS_H_
#define BIOCHEM_BLE_PACKETS_H_

#include "Arduino.h"

enum TEST_STATUS {
    TEST_STARTED = 0x00,
    STRIP_INSERTED = 0x01,
    SAMPLE_APPLIED = 0x02,
    TEST_COMPLETED_AND_VALID = 0x03,
    CALIBRATION_STRIP_INSERTED = 0x04,
    STRIP_REMOVED_BEFORE_SAMPLE = 0x05,
    MODULE_POWER_OFF = 0x06,
    CONTROL_SOLUTION_RESULT = 0x07,
    WRONG_STRIP_ERROR = 0x10,
    LOW_VOLTAGE_ERROR = 0x11,
    CALIBRATION_STRIP_ERROR_1 = 0x13,
    TEMPERATURE_ERROR = 0x14,
    USED_STRIP_ERROR = 0x15,
    STRIP_REMOVED_AFTER_SAMPLE_ERROR = 0x16,
    STRIP_ERROR = 0x17,
    CALIBRATION_STRIP_ERROR_2 = 0x18,
    INSUFFICIENT_VOLUME_ERROR = 0x19,
    TEST_ABORTED = 0x99,
    TEST_NOT_STARTED = 0xFF
};

uint8_t generate_checksum();
uint8_t get_packet_length();
uint8_t* get_packet_data();
void battery_status_packet(uint8_t battery_level, bool onBattery);
void device_info_packet(uint8_t param_id);
void test_status_packet();
void clear_test_status();
void glucose_test_status(uint8_t status, uint16_t value);
void hemoglobin_test_status(uint8_t status, uint16_t value);
void cholesterol_test_status(uint8_t status, uint16_t value, uint8_t flag);
void load_previous_values();
#endif