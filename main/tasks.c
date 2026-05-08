/*
   Espresso — RTOS application tasks

   task_500ms: temperature acquisition, brew state machine, heating control.
   task_5000ms: RainMaker status reporting.

   Both tasks are spawned by tasks_start() after all hardware and network
   initialisation is complete (called as the last step of app_main).
*/

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "control.h"
#include "tasks.h"

static const char *TAG = "espresso";

#define TASK_500MS_STACK_DEPTH      8192
#define TASK_5000MS_STACK_DEPTH     4096
#define TASK_500MS_PRIORITY         (tskIDLE_PRIORITY + 1)
#define TASK_5000MS_PRIORITY        (tskIDLE_PRIORITY)

/*****************************************************************************
 * Task implementations
 *****************************************************************************/

static void task_500ms(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        spiComm();
        brewProgram();
        heatingControl();
        xSemaphoreGive(g_state_mutex);
    }
}

static void task_5000ms(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5000));

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        if (powerOn) {
            /* Only report when the device is on to conserve MQTT budget. */
            temp_status_line_report();
            boiler_status_report();
#if ADAPTIVE_WARMUP_ENABLE
            dither_disp_report();
#endif
        }
        xSemaphoreGive(g_state_mutex);
    }
}

/*****************************************************************************
 * Public init — call as the last step in app_main()
 *****************************************************************************/

void tasks_start(void)
{
    BaseType_t ret;

    ret = xTaskCreate(task_500ms, "ctrl500ms", TASK_500MS_STACK_DEPTH,
                      NULL, TASK_500MS_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task_500ms!");
    }

    ret = xTaskCreate(task_5000ms, "rpt5000ms", TASK_5000MS_STACK_DEPTH,
                      NULL, TASK_5000MS_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task_5000ms!");
    }

    ESP_LOGI(TAG, "Application tasks started.");
}
