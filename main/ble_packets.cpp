#include "ble_packets.h"
#include "hardware.h"

uint8_t packet[20];
uint8_t glucose_status_value, hemoglobin_status_value, cholesterol_status_value, chol_ua_flag;
uint16_t glucose_data_value, hemoglobin_data_value, cholesterol_data_value;
uint16_t prev_glucose_data_value, prev_hemoglobin_data_value, prev_cholesterol_data_value;
uint8_t prev_chol_ua_flag;

// generate checksum
uint8_t generate_checksum() {
  uint8_t packet_length = packet[2];              //get packet length
  uint16_t checksum = 0;                          //initialise checksum value
  for(uint8_t i = 0; i < packet_length-1; i++) {
     checksum += packet[2+i];                     //checksum calculation
  }
  checksum = ~checksum;                           //increment one and get negative of the value
  return (uint8_t)checksum;                       //return checksum value
}

// get packet length 
uint8_t get_packet_length() {
  return packet[2] + 2;       //packet[2] will have the length of packet except start bit so include start bits also
}

// get packet data
uint8_t* get_packet_data() {
  return packet;              //return packet data
}

// get battery status
void battery_status_packet(uint8_t battery_level, bool onBattery) {
  packet[0] = 0x55;                   //start byte
  packet[1] = 0xAA;                   //start byte
  packet[2] = 0x05;                   //packet length
  packet[3] = 0xB0;                   //command
  if(onBattery) {
    packet[4] = 0x00;                 //if device is running on battery update 0
  } else {
    packet[4] = 0x01;                 //if device is running on direct supply update 1
  }
  packet[5] = battery_level;          //Get battery level
  packet[6] = generate_checksum();    //get checksum

}

// Device info (already defined)
void device_info_packet(uint8_t param_id) {
  packet[0] = 0x55;                               //start byte
  packet[1] = 0xAA;                               //start byte

  packet[3] = 0xB4;                               //command byte
  packet[4] = param_id;                           //parameter ID

  if (param_id == 0x01)                           //if param id is 0x01
  {

  String param_val = get_parameter(param_id);     //get parameter for this particular param id from the device
  packet[2] = (param_val.length()-2) + 4;         //get parameter length
  log_d("data len = %d", (param_val.length()) );  //print param length

  int i=0;
  for(i = 0; i < (param_val.length()-2); i++) {   //for loop
     
    packet[5+i] = param_val.charAt(i+2);          // get each element from the string and store it in array
  }
  packet[5+i] = generate_checksum();              //generate checksum for the values
  log_d ("index = %d", (5+i));                    //print start index of the serial number
  }
  else                                            //if param id is not 0x01
  {
  String param_val = get_parameter(param_id);     //get parameters for particular param id
  packet[2] = param_val.length() + 4;             //get packet length
  log_d("data len = %d", (param_val.length()) );  // print packet length

  int i;
  for(i = 0; i < (param_val.length()); i++) {
     
    packet[5+i] = param_val.charAt(i);            //get each element from the string and store it in array
  }
  packet[5+i] = generate_checksum();              //generate checksum for 
  log_d ("index = %d", (5+i));                    //display the index

  }
}

void test_status_packet() {
  packet[0] = 0x55;                                             //start byte
  packet[1] = 0xAA;                                             //start byte
  packet[2] = 0x0D;                                             //Length of the packet
  packet[3] = 0xA0;                                             // Command
  packet[4] = glucose_status_value;                             // Glu status
  packet[5] = hemoglobin_status_value;                          //Hb status
  packet[6] = cholesterol_status_value;                         //chol status
  packet[7] = (uint8_t)(glucose_data_value & 0x00FF);           //split 2 bytes of Glucose data into 1 byte and store it in packet
  packet[8] = (uint8_t)((glucose_data_value & 0xFF00)>>8);
  packet[9] = (uint8_t)(hemoglobin_data_value & 0x00FF);        //split 2 bytes of Hemoglobin data into 1 byte and store it in packet
  packet[10] = (uint8_t)((hemoglobin_data_value & 0xFF00)>>8);
  packet[11] = (uint8_t)(cholesterol_data_value & 0x00FF);      //split 2 bytes of Cholestrol/Uric acid data into 1 byte and store it in packet
  packet[12] = (uint8_t)((cholesterol_data_value & 0xFF00)>>8);
  packet[13] = chol_ua_flag;                                    //get chol uric acid flag
  packet[14] = generate_checksum();                             //generate checksum
}

// Clear previous test result
void clear_test_status() {
  glucose_status_value = 0xFF;      // reset glucose test status value
  hemoglobin_status_value = 0xFF;   // reset hemoglobin test status value
  cholesterol_status_value = 0xFF;  // reset Cholestrol test status value
  glucose_data_value = 0x0000;      // reset glucose data
  hemoglobin_data_value = 0x0000;   // reset hemoglobin data
  cholesterol_data_value = 0x0000;  // reset cholestrol data
  chol_ua_flag = 0x00;              // reset chol uric acid flag
}

// glucose test status
void glucose_test_status(uint8_t status, uint16_t value) {
  glucose_status_value = status;                //get glucose test status
  glucose_data_value = value;                   //get glucose data
  if(status == TEST_COMPLETED_AND_VALID) {      //if test is completed
    prev_glucose_data_value = value;            //update the current glu value as prev glu data
  }
  test_status_packet();                         //arrange the data in packet
}

// hemoglobin test status
void hemoglobin_test_status(uint8_t status, uint16_t value) {
  hemoglobin_status_value = status;             //get hemo test status
  hemoglobin_data_value = value;                //get hemo test data
  if(status == TEST_COMPLETED_AND_VALID) {      //if test is completed
    prev_hemoglobin_data_value = value;         //update the current hemo value as prev hemo data
  }
  test_status_packet();                         //arrange data in packet 
}

// cholesterol test status
void cholesterol_test_status(uint8_t status, uint16_t value, uint8_t flag) {
  cholesterol_status_value = status;              //get chol test status
  cholesterol_data_value = value;                 //get chol test data
  chol_ua_flag = flag;                            //check chol/ua test falg
  if(status == TEST_COMPLETED_AND_VALID) {        //if test is completed
    prev_cholesterol_data_value = value;          //update the current chol value as prev chol data
    prev_chol_ua_flag = flag;                     //update chol/ua flag as prev chol/ua flag
  }
  test_status_packet();                           //arrange data in packet
}

// get previous test values
void load_previous_values() {
  glucose_status_value = TEST_COMPLETED_AND_VALID;      //update glucose test status as completed
  hemoglobin_status_value = TEST_COMPLETED_AND_VALID;   //update hemo test status as completed
  cholesterol_status_value = TEST_COMPLETED_AND_VALID;  //update chol test status as completed
  glucose_data_value = prev_glucose_data_value;         //update glu data from it's prev data
  hemoglobin_data_value = prev_hemoglobin_data_value;   //update hemo data from it's prev data
  cholesterol_data_value = prev_cholesterol_data_value; //update chol data from it's prev data
  chol_ua_flag = prev_chol_ua_flag;                     //update chol/uric flag from prev chol/uric flag
  test_status_packet();                                 //arrange data in packets
}