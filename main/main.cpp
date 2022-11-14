#include "Arduino.h"
#include "task_manager.h"
#include "ble_comm.h"
#include "hardware.h"


//SemaphoreHandle_t glucose_start_semaphore, hemoglobin_start_semaphore, cholesterol_start_semaphore;

extern "C" {
	void app_main(void);
}

 void app_main()
{
  /* Arduino initializtion*/
  initArduino();

    /* wake-up device */
    check_wakeup_reason();

    /* Hardware Initialization */
    setup_hardware();

    /* BLE initialization */
    setup_ble();

    /* Device Initialized */
    log_e("Device Initialized");
}