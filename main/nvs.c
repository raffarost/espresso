/*
   Espresso — NVS persistence

   Stores and restores user-configurable parameters and the overshoot
   power-trim value that survives power cycles.
*/

#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
/* IDF NVS API — included before our local nvs.h to avoid shadowing */
#include "nvs_flash.h"
#include "control.h"

static const char *TAG = "espresso";

#if OVERSHOOT_DETECT_ENABLE
static void nvs_read_overshoot_trim(nvs_handle_t h);
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

#if OVERSHOOT_DETECT_ENABLE
    nvs_read_overshoot_trim(nvs_handle);
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

    /* overshoot trim is written only from nvs_persist_overshoot_trim() on commit */

    ESP_LOGI(TAG, "Committing updates in NVS...");
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

/*****************************************************************************
 * Overshoot trim NVS helpers
 *****************************************************************************/

#if OVERSHOOT_DETECT_ENABLE

/* Trim stored as milli-percent in NVS (int32); 100 = 0.1% */
static void nvs_read_overshoot_trim(nvs_handle_t h)
{
    int32_t mpct = 0;

    if (nvs_get_i32(h, "trimMp", &mpct) == ESP_OK && mpct >= 0) {
        overshoot_trim_stored = fminf(OVERSHOOT_TRIM_MPCT_TO_FRAC(mpct),
                                      OVERSHOOT_TRIM_FRAC_MAX);
        ESP_LOGI(TAG, "Warmup trim loaded: -%.2f%%",
                 (double)(overshoot_trim_stored * 100.0f));
        return;
    }

    /* Legacy: "pkOsm" stored cumulative °C; migrate once to "trimMp" then erase. */
    int32_t milli_c = 0;
    if (nvs_get_i32(h, "pkOsm", &milli_c) == ESP_OK && milli_c > 0) {
        const float cumulative_c = (float)milli_c * 0.001f;
        overshoot_trim_stored = fminf(OVERSHOOT_TRIM_FRAC_PER_DEG * cumulative_c,
                                      OVERSHOOT_TRIM_FRAC_MAX);
        esp_err_t e = nvs_set_i32(h, "trimMp",
                                  OVERSHOOT_TRIM_FRAC_TO_MPCT(overshoot_trim_stored));
        if (e == ESP_OK) {
            nvs_erase_key(h, "pkOsm");
            nvs_commit(h);
        }
        ESP_LOGI(TAG, "Warmup trim migrated: pkOsm=%.2f°C -> -%.2f%%",
                 (double)cumulative_c, (double)(overshoot_trim_stored * 100.0f));
        return;
    }

    overshoot_trim_stored = 0.f;
}

void nvs_persist_overshoot_trim(void)
{
    nvs_handle_t h;

    if (nvs_open("storage", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    esp_err_t e = nvs_set_i32(h, "trimMp",
                               OVERSHOOT_TRIM_FRAC_TO_MPCT(overshoot_trim_stored));
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "trimMp NVS save failed: %s", esp_err_to_name(e));
    }
    nvs_close(h);
}

#endif /* OVERSHOOT_DETECT_ENABLE */
