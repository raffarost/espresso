/*
   Espresso — NVS persistence

   Stores and restores user-configurable parameters and the adaptive
   dither step that survives power cycles.
*/

#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
/* IDF NVS API — included before our local nvs.h to avoid shadowing */
#include "nvs_flash.h"
#include "control.h"

static const char *TAG = "espresso";

#if ADAPTIVE_WARMUP_ENABLE
static void nvs_read_dither_step(nvs_handle_t h);
#endif

/*****************************************************************************
 * Public functions
 *****************************************************************************/

void nvsRead(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    uint8_t u8 = 0;
    nvs_get_u8(nvs_handle, "tempLock", &u8);
    tempLock = u8 != 0;

    nvs_get_i32(nvs_handle, "tempSetpoint", &tempSetpoint);
    if (tempSetpoint < TEMP_SETPOINT_MIN) {
        tempSetpoint = TEMP_SETPOINT_MIN;
    } else if (tempSetpoint > TEMP_SETPOINT_MAX) {
        tempSetpoint = TEMP_SETPOINT_MAX;
    }

    nvs_get_i32(nvs_handle, "brewTime", &brewTime);
    nvs_get_i32(nvs_handle, "flushTime", &flushTime);

    u8 = 0;
    nvs_get_u8(nvs_handle, "preInfusion", &u8);
    preInfusion = u8 != 0;

    nvs_get_i32(nvs_handle, "preInfOnTime", &preInfOnTime);
    nvs_get_i32(nvs_handle, "preInfOffTime", &preInfOffTime);

#if ADAPTIVE_WARMUP_ENABLE
    nvs_read_dither_step(nvs_handle);
#endif

    nvs_close(nvs_handle);
}

void nvsWrite(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(nvs_set_u8(nvs_handle,  "tempLock",     (uint8_t)tempLock));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "tempSetpoint", tempSetpoint));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "brewTime",     brewTime));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "flushTime",    flushTime));
    ESP_ERROR_CHECK(nvs_set_u8(nvs_handle,  "preInfusion",  (uint8_t)preInfusion));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "preInfOnTime",  preInfOnTime));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "preInfOffTime", preInfOffTime));

    /* dither step is written only from nvs_persist_dither_step() on commit */

    ESP_LOGI(TAG, "Committing updates in NVS...");
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

/*****************************************************************************
 * Adaptive dither step NVS helpers
 *****************************************************************************/

#if ADAPTIVE_WARMUP_ENABLE

static void nvs_read_dither_step(nvs_handle_t h)
{
    int32_t step = DITHER_STEP_DEFAULT;
    if (nvs_get_i32(h, "dithStep", &step) == ESP_OK) {
        if (step < 0) step = 0;
        if (step >= DITHER_STEP_COUNT) step = DITHER_STEP_COUNT - 1;
        dither_step = (int)step;
        ESP_LOGI(TAG, "Dither step loaded: %d", dither_step);
    } else {
        dither_step = DITHER_STEP_DEFAULT;
    }
    /* Legacy keys (trimMp, pkOsm) are left in NVS and ignored. */
}

void nvs_persist_dither_step(void)
{
    nvs_handle_t h;

    if (nvs_open("storage", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    esp_err_t e = nvs_set_i32(h, "dithStep", (int32_t)dither_step);
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "dithStep NVS save failed: %s", esp_err_to_name(e));
    }
    nvs_close(h);
}

#endif /* ADAPTIVE_WARMUP_ENABLE */
