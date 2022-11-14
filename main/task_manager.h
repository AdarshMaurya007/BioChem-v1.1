#ifndef BIOCHEM_TASK_MANAGER_H_
#define BIOCHEM_TASK_MANAGER_H_

#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define BLE_SLEEP_TIMEOUT  120

struct BuzzerBeep
{
   uint16_t beepCount, beepInterval;
};

void startSleepMode();
void startModuleTasks();
void stopModuleTasks();
void startBackgroundTasks();
void suspendTasks();
void resumeTasks();
void createWatchdogTask();
void createPM4100Task();
void createGlucoseTask();
void createLEDUpdateTask();
void createBLEMonitorTask();
void createBatteryMonitorTask();
void createSwitchPressedTask();
void createBuzzerTask();
void TaskBLEMonitor(void *pvParameters);
void TaskBatteryMonitor(void *pvParameters);
void TaskUpdateLED(void *pvParameters);
void TaskSwitchPressed(void *pvParameters);
void TaskGlucose(void *pvParameters);
void TaskHemoglobin(void *pvParameters);
void TaskCholesterol(void *pvParameters);
void TaskWatchdog(void *pvParameters);
void TaskBuzzer(void *pvParameters);
void startTests(uint8_t glu_test_flag, uint8_t hb_test_flag, uint8_t chol_test_flag);
void abortTests(uint8_t glu_test_flag, uint8_t hb_test_flag, uint8_t chol_test_flag);
void send_previous_result();
#endif