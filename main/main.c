/*
   Espresso PID/P Controller  v2.0.1

   This project implements a PID/P controller to effectively control the
   temperature of an Espresso machine boiler for a stable setpoint.
   A pre-infusion and brew program controls the water pump, and combined
   with the temperature control delivers improved espresso extraction.

   https://github.com/raffarost/espresso
   March 2024 – April 2026

   Raffael Rostagno
   raffael.rostagno@gmail.com
*/

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_rmaker_core.h"
#include "esp_rmaker_ota.h"
#include "esp_rmaker_schedule.h"
#include "esp_rmaker_scenes.h"
#include "esp_rmaker_console.h"
#include "app_wifi.h"
#include "app_insights.h"
#include "spi_mod.h"
#include "private.h"
#include "control.h"
#include "nvs.h"
#include "tasks.h"

static const char *TAG = "espresso";

/*****************************************************************************
 * Free-running 1 µs timer
 *****************************************************************************/

static gptimer_handle_t s_free_run_timer;

static const gptimer_config_t s_timer_cfg = {
    .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
    .direction     = GPTIMER_COUNT_UP,
    .resolution_hz = 1 * 1000 * 1000,  /* 1 MHz → 1 tick = 1 µs */
};

unsigned long long getAbsTime1us(void)
{
    unsigned long long val = 0;
    gptimer_get_raw_count(s_free_run_timer, &val);
    return val;
}

/*****************************************************************************
 * GPIO configuration — call first so pads are never floating
 *****************************************************************************/

void gpioConfig(void)
{
    const gpio_config_t out_cfg = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&out_cfg);
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    gpio_set_level(TRIAC_1_GPIO, 0);
}

/*****************************************************************************
 * Shared mutex
 *****************************************************************************/

SemaphoreHandle_t g_state_mutex;

/*****************************************************************************
 * Application entry point
 *
 * Init sequence (strict order — do NOT reorder):
 *   1.  gpioConfig()          outputs low; pads never float
 *   2.  gptimer init          1 µs free-running clock
 *   3.  NVS flash init        + nvsRead() restores persisted params
 *   4.  app_wifi_init()
 *   5.  rainmaker_init()      registers events, creates device + params
 *   6.  RainMaker services    OTA, timezone, schedule, scenes, insights
 *   7.  esp_rmaker_start()    RainMaker agent running
 *   8.  app_wifi_start()      BLOCKS until connected / provisioned
 *   9.  spi_init()            SPI driver for thermocouple
 *  10.  pidInit()             PID state (no-op for LOOKUP mode)
 *  11.  dimInit()             triac dimmer driver
 *  12.  mutex create          g_state_mutex
 *  13.  tasks_start()         spawn application tasks — app begins here
 *****************************************************************************/

void app_main(void)
{
    /* 1 — GPIO outputs low before anything else */
    gpioConfig();

    /* 2 — Free-running 1 µs timer */
    ESP_ERROR_CHECK(gptimer_new_timer(&s_timer_cfg, &s_free_run_timer));
    ESP_ERROR_CHECK(gptimer_enable(s_free_run_timer));
    ESP_ERROR_CHECK(gptimer_start(s_free_run_timer));

    /* 3 — NVS */
    esp_rmaker_console_init();
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    nvsRead();

    /* 4 — Wi-Fi (must precede esp_rmaker_node_init) */
    app_wifi_init();

    /* 5 — RainMaker node + device + params */
    esp_rmaker_config_t rmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rmaker_cfg,
                                                   "ESP RainMaker Device", "Espresso");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise RainMaker node. Aborting!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        abort();
    }
    rainmaker_init(node);   /* registers events, creates device, params, write_cb */

    /* 6 — RainMaker services */
    esp_rmaker_ota_enable_default();
    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    app_insights_enable();

    /* 7 — Start RainMaker agent */
    esp_rmaker_start();

    /* 8 — Start Wi-Fi (blocks until connected or provisioned) */
    err = app_wifi_set_custom_mfg_data(MFG_DATA_DEVICE_TYPE_SWITCH,
                                       MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    err = app_wifi_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wi-Fi. Aborting!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        abort();
    }

    /* 9-11 — Hardware drivers (after WiFi so SPI is not racing with init) */
    spi = spi_init();

#if (CONTROL_TYPE == PID) || (CONTROL_TYPE == PID_LOOKUP)
    pidInit();
#endif

    dimInit();

    /* 12 — Shared mutex (before any task touches shared state) */
    g_state_mutex = xSemaphoreCreateMutex();

    /* 13 — Spawn application tasks; app_main returns, scheduler takes over */
    ESP_LOGI(TAG, "Setpoint (C) | Temp (C) | pidOut | control (%%) | power (%%)");
    tasks_start();
}
