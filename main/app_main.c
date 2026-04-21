/*
   Espresso Machine PID/P Controller

   This project implements a PID/P controller to effectively control the
   temperature of an Espresso machine boiler, for a stable setpoint.
   Another feature implemented is the control of the water pump and
   pre-infusion settings. Combined, these functionalities allow for
   improved espresso extraction and consistency.

   https://github.com/raffarost/espresso
   March 2024 - April 2026

   Raffael Rostagno
   raffael.rostagno@gmail.com
*/

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "esp_err.h"
#include "soc/soc_caps.h"
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "esp32-triac-dimmer-driver.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "pins.h"
#include "spi_mod.h"
#include "driver/gptimer.h"

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>

#include <esp_rmaker_common_events.h>

#include <app_wifi.h>
#include <app_insights.h>

#include "app_priv.h"

/*****************************************************************************
 * Configuration defines
 *****************************************************************************/

/* heating element control */
#define GRID_FREQ               60          /* power grid frequency for power control */
#define ZEROCROSS_GPIO          GPIO_NUM_5
#define TRIAC_1_GPIO            GPIO_NUM_33

/* Tasks config */
#define TASK_100ms_PERIOD_MS    100
#define TASK_500ms_PERIOD_MS    500
#define TASK_100ms_PRIORITY     tskIDLE_PRIORITY
#define TASK_500ms_PRIORITY     tskIDLE_PRIORITY

/* GPIO config */

#define GPIO_OUTPUT_IO_0    GPIO_NUM_4
#define GPIO_OUTPUT_IO_1    0
/* Pump + triac gate: outputs defined early so pads are not floating through Wi-Fi / RainMaker init. */
#define GPIO_OUTPUT_PIN_SEL  ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << TRIAC_1_GPIO))

static const gpio_config_t gpio_outcfg = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
};

gptimer_handle_t freeRunTimer = NULL;
gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
};

#define CHECK_RET(ret,msg)  \
if (ret != ESP_OK)  \
{   \
    ESP_LOGE(TAG, msg); \
}

#define PID         0
#define LOOKUP      1
#define PID_LOOKUP  2

#define CONTROL_TYPE    LOOKUP

/* Overshoot learn/trim for LOOKUP only; on by default. Off: -DOVERSHOOT_DETECT_ENABLE=0 or #define 0 above. */
#if (CONTROL_TYPE == LOOKUP) && !defined(OVERSHOOT_DETECT_ENABLE)
#define OVERSHOOT_DETECT_ENABLE  1
#endif

#define BKP_NUM       10

static float   deltaBkp[BKP_NUM] = {-10,  0, 0.5,  1,  2,  4, 10,  25,  50, 100};  /* temperature delta */
static float controlSet[BKP_NUM] = {  0,  0,   1,  1,  1,  1,  1,  80, 100, 100};  /* power setpoint in percentage */

#define SEC_TO_US(x)        (x * 1000000)

#define BREW_OFF            0
#define BREW_PREINF_ON      1
#define BREW_PREINF_OFF     2
#define BREW_ON             3
#define BREW_POWER_OFF      4
#define BREW_FLUSH          5

#define PUMP_ON_HEAT_BUFF_LEN  20  /* must be >= max brew time (s) and max flush time (s) */

/* Each BKP is seconds of pump-on time; output is heater power (%). */
static int pumpOnHeatBuff[PUMP_ON_HEAT_BUFF_LEN] = {
/*  s:   0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19 */
       100, 100,  80,  80,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70
};

#define TEMP_DELTA          2   /* temp delta from setpoint for good brewing temperature */
#define TEMP_SETPOINT_MIN   88
#define TEMP_SETPOINT_MAX   96
#define MAX_TEMP_THR      110   /* maximum temperature threshold for safety control */

#define TEMP_READ_OK_STREAK  3  /* consecutive good samples required before trusting */
#define TEMP_VALID_MIN_C     (-2.0f)
#define TEMP_VALID_MAX_C     (125.0f)
#define TEMP_DIAG_SPI        (1u << 0)
#define TEMP_DIAG_OPEN_TC    (1u << 1)
#define TEMP_DIAG_RANGE      (1u << 2)

/* Stuck sensor: high heater demand but temp unchanged for STUCK_DIAG_SAMPLES consecutive reads.
 * Warmup grace delays counting until the sensor has proven responsive or the grace period elapses
 * (the heater is physically far from the sensor, so a cold ramp reads flat for several seconds). */
#define STUCK_DIAG_MIN_POWER_PCT     10
#define STUCK_DIAG_SAMPLES           5     /* 2.5s @ 500ms cadence */
#define STUCK_DIAG_TEMP_EPS          0.21f /* < half MAX6675 LSB (0.25°C) */
#define STUCK_DIAG_CLEAR_MAX_C       110.f
#define STUCK_DIAG_WARMUP_GRACE_SEC  10.f

/* divisor factors for power reduction around temperature target */
#define POWER_50            2
#define POWER_33            3
#define POWER_25            4

#define POWER_FACTOR        POWER_33
#define TEMP_PWR_TOGGLE    10   /* °C window around setpoint where power is dithered */

#define POWERON_MIN         (15 * SEC_TO_US(60))    /* standby timeout (15 min) */

#if OVERSHOOT_DETECT_ENABLE
#define OVERSHOOT_SOFT_DISARM_SEC          90.f
#define OVERSHOOT_TRIM_DEMAND_PCT_THRESHOLD 20.0f  /* apply trim when control (%) is above this */
#define OVERSHOOT_TRIM_FRAC_PER_DEG        0.02f  /* trim fraction added per °C of measured excursion */
#define OVERSHOOT_TRIM_FRAC_MAX            0.15f  /* max cumulative trim fraction (0.15 = 15% cut) */
#define OVERSHOOT_COLD_START_MAX_C         60.0f  /* arm learn only if boiler at/below this temp (°C) */
#endif


/*****************************************************************************
 * Module variables
 *****************************************************************************/

static const char *TAG = "espresso";
esp_rmaker_device_t *espresso_device;

esp_rmaker_param_t *primary;
static esp_rmaker_param_t *status_param;

static char s_temp_line_str[40];
static char s_status_str[96];

/* Status latch: worst-severity issue shown until ~60s of clean Okay reports */
#define STATUS_PRI_ZERO       25
#define STATUS_PRI_UNTRUSTED  50
#define STATUS_PRI_TOO_HIGH   75
#define STATUS_PRI_STUCK      100
#define STATUS_WORST_CLEAR_OK_REPORTS  12  /* Task5000ms ticks (~60s) */

dimmertyp *ptr_dimmer;

static int count100ms = 0;
static int count500ms = 0;

spi_device_handle_t spi;
uint16_t data;
spi_transaction_t tM = {
    .tx_buffer = NULL,
    .rx_buffer = &data,
    .length = 16,
    .rxlength = 16,
};

static float tempCelsius;
static int32_t tempSetpoint = 96;
static float pidOut;
static float delta;
static int control;
static int powerToggle = 0;
static unsigned long long pumpTimer = 0;
static bool pump_active;
static unsigned long long pump_start_us;
static unsigned long long powerOnTimer = 0;
static bool brewSignal = false;
static bool flushSignal = false;
static int brewState = BREW_POWER_OFF;
static int32_t brewTime = 6;
static bool tempRangeOk = false;
static bool tempLock = true;
static bool powerOn = false;
static bool preInfusion = true;
static int32_t preInfOnTime = 3;
static int32_t preInfOffTime = 10;
static int32_t flushTime = 5;

static bool temp_read_trusted;
static uint8_t temp_read_good_streak;
static uint32_t temp_read_fault_latch;

static int s_status_worst_pri;
static char s_status_worst_str[48];
static uint16_t s_status_ok_clean_reports;

static bool temp_stuck_diag;
static float temp_stuck_prev_c;
static uint8_t temp_stuck_same_ct;
static bool temp_sensor_responsive;          /* latched once the sensor moves under heat demand */
static unsigned long long temp_stuck_warmup_start_us; /* t0 of the cold-start warmup grace window */

#if OVERSHOOT_DETECT_ENABLE
static esp_rmaker_param_t *overshoot_disp_param;
static char s_overshoot_disp_str[28];
static float overshoot_trim_stored;         /* NVS-persisted power cut fraction (0..TRIM_FRAC_MAX) */
static float overshoot_excursion_pk;        /* peak (temp - setpoint) °C of the active excursion */
static bool overshoot_excursion_latched;    /* clears at temp <= setpoint; triggers commit once */
static bool overshoot_detected;             /* sticky: crossed setpoint+TEMP_DELTA this arm cycle */
static bool overshoot_learn_allowed = false;
static bool overshoot_learn_ever_armed;     /* cold arm succeeded at least once this power session */
#define OS_DISARM_SOFT_TIMEOUT  (1u << 0)
#define OS_DISARM_COMMIT        (1u << 1)
static uint32_t overshoot_learn_disarm_mask;
static bool overshoot_boot_temp_sampled;
static unsigned long long overshoot_soft_timer_start_us;
#endif

static esp_rmaker_param_t *poweron_param;

/*****************************************************************************
 * Function headers
 *****************************************************************************/

void TaskBackground(void);
void Task100ms(void);
void Task500ms(void);
void Task5000ms(void);
void spiComm(void);
void heatingControl(void);
void brewProgram(void);
void nvsRead(void);
void nvsWrite(void);
static void temp_stuck_diag_update(int heating_demand_pct);
static bool boiler_heater_holdoff(void);
static void temp_status_line_report(void);
static void boiler_status_report(void);
#if OVERSHOOT_DETECT_ENABLE
static void overshoot_learn_try_arm_cold(const char *site);
static void overshoot_peak_detector_update(void);
static void overshoot_disp_format_str(void);
static void overshoot_disp_report(void);
static void nvs_read_overshoot_trim(nvs_handle_t h);
static void nvs_persist_overshoot_trim(void);
static void overshoot_apply_trim_to_control(int *p_control);
static void overshoot_learn_set_disallowed(uint32_t reason_bits);
#endif

/*****************************************************************************
 * Function declaration
 *****************************************************************************/

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    
    /* Save to local variable */
    if (strcmp(esp_rmaker_param_get_name(param), "Temperature setpoint (\xc2\xb0""C)") == 0)
    {
        int32_t sp = val.val.i;
        if (sp < TEMP_SETPOINT_MIN) {
            sp = TEMP_SETPOINT_MIN;
        } else if (sp > TEMP_SETPOINT_MAX) {
            sp = TEMP_SETPOINT_MAX;
        }
        tempSetpoint = sp;
        ESP_LOGI(TAG, "New temperature setpoint: %d", tempSetpoint);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Brew time (s)") == 0)
    {
        brewTime = val.val.i;
        ESP_LOGI(TAG, "Brew Time: %d s", brewTime);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Flush time (s)") == 0)
    {
        flushTime = val.val.i;
        ESP_LOGI(TAG, "Flush Time: %d s", flushTime);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Brew Signal") == 0)
    {
        brewSignal = val.val.b ? !brewSignal : brewSignal;
        ESP_LOGI(TAG, "Brew %s!", brewSignal ? "start" : "stop");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Flush") == 0)
    {
        flushSignal = val.val.b ? !flushSignal : flushSignal;
        ESP_LOGI(TAG, "Flush %s!", flushSignal ? "on" : "off");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Pre-Infusion") == 0)
    {
        preInfusion = val.val.b;
        ESP_LOGI(TAG, "Pre-Infusion set to %s", preInfusion ? "ON" : "OFF");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Power") == 0)
    {
        const bool was_on = powerOn;
        powerOn = val.val.b;
        ESP_LOGI(TAG, "Power is set to %s", powerOn ? "ON" : "OFF");
#if OVERSHOOT_DETECT_ENABLE
        if (!powerOn && was_on) {
            overshoot_learn_ever_armed = false;
        }
        if (powerOn && !was_on) {
            overshoot_learn_try_arm_cold("Power cb");
        }
#endif
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Temp Lock") == 0)
    {
        tempLock = val.val.b;
        ESP_LOGI(TAG, "Temp Lock set to %s", tempLock ? "ON" : "OFF");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Pre-Infusion on time (s)") == 0)
    {
        preInfOnTime = val.val.i;
        ESP_LOGI(TAG, "Pre-Infusion On Time: %d s", preInfOnTime);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Pre-Infusion off time (s)") == 0)
    {
        preInfOffTime = val.val.i;
        ESP_LOGI(TAG, "Pre-Infusion Off Time: %d s", preInfOffTime);
    }

#if OVERSHOOT_DETECT_ENABLE
    if (strcmp(esp_rmaker_param_get_name(param), "Reset power trim") == 0)
    {
        if (val.val.b) {
            overshoot_trim_stored = 0.f;
            overshoot_learn_allowed = true;
            overshoot_learn_ever_armed = false;
            overshoot_learn_disarm_mask = 0u;
            overshoot_soft_timer_start_us = 0;
            overshoot_detected = false;
            overshoot_excursion_latched = false;
            overshoot_excursion_pk = 0.f;
            nvs_persist_overshoot_trim();
            ESP_LOGI(TAG, "Warmup trim reset (NVS cleared, learn re-armed if cold)");
            overshoot_disp_report();
        }
    }
#endif

    return ESP_OK;
}

/* Event handler for catching RainMaker events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGI(TAG, "RainMaker Claim Failed.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STARTED:
                ESP_LOGI(TAG, "Local Control Started.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
                ESP_LOGI(TAG, "Local Control Stopped.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected.");
                break;
            case RMAKER_MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else if (event_base == APP_WIFI_EVENT) {
        switch (event_id) {
            case APP_WIFI_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            case APP_WIFI_EVENT_PROV_TIMEOUT:
                ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
                break;
            case APP_WIFI_EVENT_PROV_RESTART:
                ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled App Wi-Fi Event: %"PRIi32, event_id);
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch(event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "Starting OTA.");
                break;
            case RMAKER_OTA_EVENT_IN_PROGRESS:
                ESP_LOGI(TAG, "OTA is in progress.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA Failed.");
                break;
            case RMAKER_OTA_EVENT_REJECTED:
                ESP_LOGI(TAG, "OTA Rejected.");
                break;
            case RMAKER_OTA_EVENT_DELAYED:
                ESP_LOGI(TAG, "OTA Delayed.");
                break;
            case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
                ESP_LOGI(TAG, "Firmware image downloaded. Please reboot your device to apply the upgrade.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled OTA Event: %"PRIi32, event_id);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

void dimInit(void)
{
    ptr_dimmer = createDimmer(TRIAC_1_GPIO, ZEROCROSS_GPIO);
    begin(ptr_dimmer, NORMAL_MODE, ON, GRID_FREQ);
    setPower(ptr_dimmer, 0);    // set power to 0%
    
    ESP_LOGI(TAG, "Dimmer initialized");
}

void gpioConfig(void)
{
    /* Outputs: pump off, triac gate held low (dimmer driver re-configures GPIO33 later). */
    gpio_config(&gpio_outcfg);
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    gpio_set_level(TRIAC_1_GPIO, 0);
}

void nvsRead(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        /* Read (NVS APIs use fixed-width types; not bool* / int*) */
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
        nvs_get_u8(nvs_handle, "preInfusion", &u8);
        preInfusion = u8 != 0;
        nvs_get_i32(nvs_handle, "preInfOnTime", &preInfOnTime);
        nvs_get_i32(nvs_handle, "preInfOffTime", &preInfOffTime);

#if OVERSHOOT_DETECT_ENABLE
        nvs_read_overshoot_trim(nvs_handle);
#endif

        nvs_close(nvs_handle);
    }    
}

void nvsWrite(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        /* Write */
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "tempLock", (uint8_t)tempLock));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "tempSetpoint", tempSetpoint));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "brewTime", brewTime));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "flushTime", flushTime));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "preInfusion", (uint8_t)preInfusion));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "preInfOnTime", preInfOnTime));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "preInfOffTime", preInfOffTime));

        /* trimMp is written only from nvs_persist_overshoot_trim() on learn commit. */

        ESP_LOGI(TAG, "Committing updates in NVS ... ");
        
        ESP_ERROR_CHECK(nvs_commit(nvs_handle));

        nvs_close(nvs_handle);
    }    
}

#if OVERSHOOT_DETECT_ENABLE
/* Trim stored as milli-percent in NVS (int32); 100 = 0.1% */
#define OVERSHOOT_TRIM_FRAC_TO_MPCT(f) ((int32_t)lroundf((f) * 100000.0f))
#define OVERSHOOT_TRIM_MPCT_TO_FRAC(i) ((float)(i) * 0.00001f)

static void nvs_read_overshoot_trim(nvs_handle_t h)
{
    int32_t mpct = 0;
    if (nvs_get_i32(h, "trimMp", &mpct) == ESP_OK && mpct >= 0) {
        overshoot_trim_stored = fminf(OVERSHOOT_TRIM_MPCT_TO_FRAC(mpct), OVERSHOOT_TRIM_FRAC_MAX);
        ESP_LOGI(TAG, "Warmup trim loaded: -%.2f%%", (double)(overshoot_trim_stored * 100.0f));
        return;
    }

    /* Legacy: "pkOsm" was cumulative °C; migrate once to "trimMp" then erase. */
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

static void nvs_persist_overshoot_trim(void)
{
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    esp_err_t e = nvs_set_i32(h, "trimMp", OVERSHOOT_TRIM_FRAC_TO_MPCT(overshoot_trim_stored));
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "trimMp NVS save failed: %s", esp_err_to_name(e));
    }
    nvs_close(h);
}

static void overshoot_apply_trim_to_control(int *p_control)
{
    if ((float)*p_control > (float)OVERSHOOT_TRIM_DEMAND_PCT_THRESHOLD) {
        float cf = (float)*p_control * (1.f - overshoot_trim_stored);
        *p_control = (int)(cf + 0.5f);
    }
}

static void overshoot_learn_set_disallowed(uint32_t reason_bits)
{
    overshoot_learn_disarm_mask |= reason_bits;
    overshoot_learn_allowed = false;
}

static void overshoot_learn_try_arm_cold(const char *site)
{
    if (!powerOn) {
        return;
    }
    if (!temp_read_trusted) {
        return;
    }
    if (tempCelsius > OVERSHOOT_COLD_START_MAX_C) {
        return;
    }
    overshoot_learn_allowed = true;
    overshoot_learn_ever_armed = true;
    overshoot_learn_disarm_mask = 0u;
    overshoot_excursion_latched = false;
    overshoot_excursion_pk = 0.f;
    overshoot_soft_timer_start_us = 0;
    overshoot_detected = false;
    ESP_LOGI(TAG, "OS arm[%s] OK: cold start @ %.1f°C (trim pre-loaded -%.2f%%)",
             site, (double)tempCelsius, (double)(overshoot_trim_stored * 100.0f));
}


static void overshoot_peak_detector_update(void)
{
    if (!overshoot_learn_allowed) {
        return;
    }
    if (temp_stuck_diag) {
        return;
    }

    const float temp_ok_max_threshold = (float)tempSetpoint + (float)TEMP_DELTA;

    if (tempCelsius > temp_ok_max_threshold) {
        overshoot_detected = true;
        overshoot_excursion_latched = true;
    }

    if (!overshoot_detected) {
        if (overshoot_soft_timer_start_us == 0ULL && tempCelsius >= (float)tempSetpoint) {
            overshoot_soft_timer_start_us = getAbsTime1us();
        } else if (overshoot_soft_timer_start_us != 0ULL &&
                   (getAbsTime1us() - overshoot_soft_timer_start_us) >=
                       (unsigned long long)SEC_TO_US(OVERSHOOT_SOFT_DISARM_SEC)) {
            overshoot_learn_set_disallowed(OS_DISARM_SOFT_TIMEOUT);
            overshoot_soft_timer_start_us = 0;
            overshoot_detected = false;
            overshoot_excursion_latched = false;
            overshoot_excursion_pk = 0.f;
            ESP_LOGI(TAG,
                     "OS learn disarmed (soft): no temp above setpoint+%d°C within %.0fs",
                     TEMP_DELTA, (double)OVERSHOOT_SOFT_DISARM_SEC);
            overshoot_disp_report();
            return;
        }
    }

    if (overshoot_excursion_latched && tempCelsius > (float)tempSetpoint) {
        const float temp_above_setpoint_c = tempCelsius - (float)tempSetpoint;
        overshoot_excursion_pk = fmaxf(overshoot_excursion_pk, temp_above_setpoint_c);
    }

    if (tempCelsius <= (float)tempSetpoint) {
        if (overshoot_excursion_latched) {
            if (overshoot_excursion_pk > 0.01f) {
                const float peak_c = overshoot_excursion_pk;
                const float trim_add = OVERSHOOT_TRIM_FRAC_PER_DEG * peak_c;
                const float trim_prev = overshoot_trim_stored;
                overshoot_trim_stored = fminf(overshoot_trim_stored + trim_add,
                                              OVERSHOOT_TRIM_FRAC_MAX);
                ESP_LOGI(TAG,
                         "OS commit: peak +%.2f°C -> cut +%.2f%% (-%.2f%% -> -%.2f%%%s)",
                         (double)peak_c,
                         (double)(trim_add * 100.0f),
                         (double)(trim_prev * 100.0f),
                         (double)(overshoot_trim_stored * 100.0f),
                         (overshoot_trim_stored >= OVERSHOOT_TRIM_FRAC_MAX - 1e-6f) ? ", CAPPED" : "");
                nvs_persist_overshoot_trim();
                overshoot_learn_set_disallowed(OS_DISARM_COMMIT);
                overshoot_soft_timer_start_us = 0;
                overshoot_detected = false;
            }
            overshoot_excursion_pk = 0.f;
        }
        overshoot_excursion_latched = false;
    }
}

static void overshoot_disp_format_str(void)
{
    const float cut_pct = overshoot_trim_stored * 100.0f;
    if (overshoot_learn_allowed && overshoot_excursion_latched && overshoot_excursion_pk > 0.01f) {
        snprintf(s_overshoot_disp_str, sizeof(s_overshoot_disp_str),
                 "-%.1f%% +%.1f°C", (double)cut_pct, (double)overshoot_excursion_pk);
    } else {
        snprintf(s_overshoot_disp_str, sizeof(s_overshoot_disp_str), "-%.1f%%", (double)cut_pct);
    }
}

static void overshoot_disp_report(void)
{
    if (!powerOn) {
        return;
    }
    overshoot_disp_format_str();
    if (overshoot_disp_param) {
        esp_err_t e = esp_rmaker_param_update_and_report(overshoot_disp_param,
                                                         esp_rmaker_str(s_overshoot_disp_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Overshoot display: %s", esp_err_to_name(e));
        }
    }
}

#endif

static void temp_stuck_diag_update(int heating_demand_pct)
{
    const float t = tempCelsius;

    /* Reset session state on power off; stuck latch clears only when the reading moves. */
    if (!powerOn) {
        temp_stuck_same_ct = 0u;
        temp_stuck_warmup_start_us = 0ULL;
        temp_sensor_responsive = false;
        temp_stuck_prev_c = t;
        return;
    }

    if (temp_stuck_diag) {
        if (temp_read_trusted && t > 0.f && t < STUCK_DIAG_CLEAR_MAX_C &&
            fabsf(t - temp_stuck_prev_c) >= STUCK_DIAG_TEMP_EPS) {
            temp_stuck_diag = false;
            temp_stuck_same_ct = 1u;
            temp_stuck_prev_c = t;
            temp_sensor_responsive = true;
            ESP_LOGI(TAG, "Boiler temp stuck diag cleared (reading moved in range)");
        }
        return;
    }

    if (!temp_read_trusted || t <= 0.f || t >= STUCK_DIAG_CLEAR_MAX_C) {
        temp_stuck_same_ct = 0u;
        temp_stuck_prev_c = t;
        return;
    }

    if (heating_demand_pct < STUCK_DIAG_MIN_POWER_PCT) {
        temp_stuck_same_ct = 0u;
        temp_stuck_prev_c = t;
        if (!temp_sensor_responsive) {
            temp_stuck_warmup_start_us = 0ULL;
        }
        return;
    }

    /* First heating sample pre-responsive: anchor and start warmup timer. */
    if (!temp_sensor_responsive && temp_stuck_warmup_start_us == 0ULL) {
        temp_stuck_warmup_start_us = getAbsTime1us();
        temp_stuck_prev_c = t;
        temp_stuck_same_ct = 0u;
        return;
    }

    if (fabsf(t - temp_stuck_prev_c) >= STUCK_DIAG_TEMP_EPS) {
        if (!temp_sensor_responsive) {
            ESP_LOGI(TAG, "Boiler temp sensor responsive (%.2f -> %.2f C)",
                     (double)temp_stuck_prev_c, (double)t);
        }
        temp_sensor_responsive = true;
        temp_stuck_same_ct = 1u;
        temp_stuck_prev_c = t;
        return;
    }

    /* Pre-responsive: skip counting until warmup grace elapses. */
    if (!temp_sensor_responsive) {
        const unsigned long long warmup_elapsed = getAbsTime1us() - temp_stuck_warmup_start_us;
        if (warmup_elapsed < (unsigned long long)SEC_TO_US(STUCK_DIAG_WARMUP_GRACE_SEC)) {
            return;
        }
    }

    if (temp_stuck_same_ct < 255) {
        temp_stuck_same_ct++;
    }
    if (temp_stuck_same_ct >= STUCK_DIAG_SAMPLES) {
        temp_stuck_diag = true;
        ESP_LOGW(TAG,
                 "Boiler temp stuck diag: demand >= %d%%, %d identical samples (~%.2f C, %s)",
                 STUCK_DIAG_MIN_POWER_PCT, STUCK_DIAG_SAMPLES, (double)t,
                 temp_sensor_responsive ? "post-movement" : "warmup grace elapsed");
    }
}

static bool boiler_heater_holdoff(void)
{
    return !temp_read_trusted || (tempCelsius == 0.f) || (tempCelsius > (float)MAX_TEMP_THR) ||
           temp_stuck_diag;
}

static void temp_status_line_report(void)
{
    const char *state;
    if (tempRangeOk) {
        state = "Ready";
    } else if (tempCelsius < (float)(tempSetpoint - TEMP_DELTA)) {
        state = "Low";
    } else {
        state = "High";
    }
    snprintf(s_temp_line_str, sizeof(s_temp_line_str), "%.1f \xc2\xb7 %s", (double)tempCelsius, state);
    if (primary) {
        esp_err_t e = esp_rmaker_param_update_and_report(primary, esp_rmaker_str(s_temp_line_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Temperature line: %s", esp_err_to_name(e));
        }
    }
}

static void boiler_status_report(void)
{
    int cur_pri = 0;
    const char *cur_msg = "Okay";

    if (temp_stuck_diag) {
        cur_pri = STATUS_PRI_STUCK;
        cur_msg = "Temp sensor stuck";
    } else if (!temp_read_trusted) {
        cur_pri = STATUS_PRI_UNTRUSTED;
        cur_msg = "Temp not trusted";
    } else if (tempCelsius <= 0.f) {
        cur_pri = STATUS_PRI_ZERO;
        cur_msg = "Temp zero";
    } else if (tempCelsius > (float)MAX_TEMP_THR) {
        cur_pri = STATUS_PRI_TOO_HIGH;
        cur_msg = "Temp too high";
    }

    if (cur_pri > s_status_worst_pri) {
        s_status_worst_pri = cur_pri;
        snprintf(s_status_worst_str, sizeof(s_status_worst_str), "%s", cur_msg);
    }

    if (cur_pri == 0) {
        if (s_status_worst_pri > 0) {
            if (s_status_ok_clean_reports + 1u >= (uint16_t)STATUS_WORST_CLEAR_OK_REPORTS) {
                s_status_worst_pri = 0;
                s_status_worst_str[0] = '\0';
                s_status_ok_clean_reports = 0;
                snprintf(s_status_str, sizeof(s_status_str), "Okay");
            } else {
                s_status_ok_clean_reports++;
                snprintf(s_status_str, sizeof(s_status_str), "Okay (%s)", s_status_worst_str);
            }
        } else {
            snprintf(s_status_str, sizeof(s_status_str), "Okay");
            s_status_ok_clean_reports = 0;
        }
    } else {
        s_status_ok_clean_reports = 0;
        if (s_status_worst_pri > cur_pri && s_status_worst_str[0] != '\0') {
            snprintf(s_status_str, sizeof(s_status_str), "%s (%s)", cur_msg, s_status_worst_str);
        } else {
            snprintf(s_status_str, sizeof(s_status_str), "%s", cur_msg);
        }
    }

    if (status_param) {
        esp_err_t e = esp_rmaker_param_update_and_report(status_param, esp_rmaker_str(s_status_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Status: %s", esp_err_to_name(e));
        }
    }
}

unsigned long long getAbsTime1us(void)
{
    unsigned long long timerVal = 0;

    gptimer_get_raw_count(freeRunTimer, &timerVal);

    return timerVal;
}

static void temp_read_note_fault(uint32_t bit)
{
    temp_read_good_streak = 0;
    temp_read_trusted = false;
    const uint32_t prev = temp_read_fault_latch;
    temp_read_fault_latch |= bit;
    if (temp_read_fault_latch != prev) {
        ESP_LOGW(TAG, "Boiler temp read fault (latch 0x%02" PRIx32 ")", temp_read_fault_latch);
    }
}

static void temp_read_note_good(float celsius)
{
    tempCelsius = celsius;
    if (temp_read_good_streak < 255) {
        temp_read_good_streak++;
    }
    if (temp_read_good_streak >= TEMP_READ_OK_STREAK) {
        if (!temp_read_trusted) {
            ESP_LOGI(TAG, "Boiler temp read now trusted");
        }
        temp_read_trusted = true;
        temp_read_fault_latch = 0;
    }
}

void spiComm(void)
{
    esp_err_t ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI acquire bus failed: %s", esp_err_to_name(ret));
        temp_read_note_fault(TEMP_DIAG_SPI);
        return;
    }
    ret = spi_device_transmit(spi, &tM);
    spi_device_release_bus(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        temp_read_note_fault(TEMP_DIAG_SPI);
        return;
    }

    const int16_t res = (int16_t) SPI_SWAP_DATA_RX(data, 16);

    if (res & (1 << 2)) {
        ESP_LOGE(TAG, "Thermocouple open (MAX6675 fault bit)");
        temp_read_note_fault(TEMP_DIAG_OPEN_TC);
        return;
    }

    const int16_t shifted = (int16_t)(res >> 3);
    const float c = (float)shifted * 0.25f;
    if (c < TEMP_VALID_MIN_C || c > TEMP_VALID_MAX_C) {
        ESP_LOGW(TAG, "Boiler temp out of range: %.2f C", (double)c);
        temp_read_note_fault(TEMP_DIAG_RANGE);
        return;
    }

    temp_read_note_good(c);
}

bool boiler_temp_is_trusted(void)
{
    return temp_read_trusted;
}

uint32_t boiler_temp_fault_bits(void)
{
    return temp_read_fault_latch;
}

void app_main()
{
    /* Pump + triac gate: outputs low before any timers, NVS, or network (pads not floating). */
    gpioConfig();

    /* Init general purpose free running timer (1us resolution)*/
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &freeRunTimer));
    ESP_ERROR_CHECK(gptimer_enable(freeRunTimer));
    ESP_ERROR_CHECK(gptimer_start(freeRunTimer));

    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    esp_rmaker_console_init();

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK( err );

    /* Reading app stored NVS data */
    nvsRead();

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_wifi_init();

    /* Register an event handler to catch RainMaker events */
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_wifi_init() but before app_wifi_start()
     */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Espresso");

    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create a Switch device.
     * You can optionally use the helper API esp_rmaker_espresso_device_create() to
     * avoid writing code for adding the name and power parameters.
     */
    espresso_device = esp_rmaker_device_create("Espresso", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    esp_rmaker_node_add_device(node, espresso_device);

    /* Add the write callback for the device. We aren't registering any read callback yet as
     * it is for future use.
     */
    esp_rmaker_device_add_cb(espresso_device, write_cb, NULL);

    /* No ESP_RMAKER_DEF_NAME_PARAM — saves a redundant name row in the phone UI. */

    snprintf(s_temp_line_str, sizeof(s_temp_line_str), "--");
    primary = esp_rmaker_param_create("Temperature (\xc2\xb0""C)", NULL, esp_rmaker_str(s_temp_line_str), PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(primary, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, primary);
    esp_rmaker_device_assign_primary_param(espresso_device, primary);

    /* Creating Power Up toggle switch */
    poweron_param = esp_rmaker_param_create("Power", NULL, esp_rmaker_bool(powerOn), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(poweron_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, poweron_param);

    /* Creating brew button */
    esp_rmaker_param_t *brewSig_param = esp_rmaker_param_create("Brew Signal", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(brewSig_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, brewSig_param);

    esp_rmaker_param_t *flush_param = esp_rmaker_param_create("Flush", NULL, esp_rmaker_bool(false),
                                                              PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(flush_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, flush_param);
  
    /* Creating Temperature Lock toggle switch */
    esp_rmaker_param_t *templock_param = esp_rmaker_param_create("Temp Lock", NULL, esp_rmaker_bool(tempLock), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(templock_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, templock_param);

    /* Creating slider object */
    esp_rmaker_param_t *temp_param = esp_rmaker_param_create("Temperature setpoint (\xc2\xb0""C)", NULL,
                                                             esp_rmaker_int(tempSetpoint), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(temp_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(temp_param, esp_rmaker_int(TEMP_SETPOINT_MIN), esp_rmaker_int(TEMP_SETPOINT_MAX),
                                esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, temp_param);

    /* Creating slider object */
    esp_rmaker_param_t *brewtime_param = esp_rmaker_param_create("Brew time (s)", NULL, esp_rmaker_int(brewTime),
                                                                 PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(brewtime_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(brewtime_param, esp_rmaker_int(6), esp_rmaker_int(20), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, brewtime_param);

    /* Creating Pre-Infusion toggle switch */
    esp_rmaker_param_t *preinf_param = esp_rmaker_param_create("Pre-Infusion", NULL, esp_rmaker_bool(preInfusion), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(preinf_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, preinf_param);

    /* Pre-Infusion On Time slider object */
    esp_rmaker_param_t *preinfOn_param = esp_rmaker_param_create("Pre-Infusion on time (s)", NULL,
                                                                 esp_rmaker_int(preInfOnTime), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(preinfOn_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(preinfOn_param, esp_rmaker_int(2), esp_rmaker_int(10), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, preinfOn_param);

    /* Pre-Infusion Off Time slider object */
    esp_rmaker_param_t *preinfOff_param = esp_rmaker_param_create("Pre-Infusion off time (s)", NULL,
                                                                   esp_rmaker_int(preInfOffTime), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(preinfOff_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(preinfOff_param, esp_rmaker_int(2), esp_rmaker_int(30), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, preinfOff_param);

    esp_rmaker_param_t *flushtime_param = esp_rmaker_param_create("Flush time (s)", NULL, esp_rmaker_int(flushTime),
                                                                   PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(flushtime_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(flushtime_param, esp_rmaker_int(3), esp_rmaker_int(20), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, flushtime_param);

#if OVERSHOOT_DETECT_ENABLE
    /* Build initial string from loaded NVS trim (nvsRead ran earlier). */
    overshoot_disp_format_str();
    /* Stored cumulative trim (%). While an excursion is latched, appends live peak in °C. */
    overshoot_disp_param = esp_rmaker_param_create("Warmup power trim", NULL, esp_rmaker_str(s_overshoot_disp_str),
                                                   PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(overshoot_disp_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, overshoot_disp_param);

    esp_rmaker_param_t *reset_os_param = esp_rmaker_param_create("Reset power trim", NULL, esp_rmaker_bool(false),
                                                                 PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(reset_os_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, reset_os_param);

#endif

    snprintf(s_status_str, sizeof(s_status_str), "Okay");
    status_param = esp_rmaker_param_create("Status", NULL, esp_rmaker_str(s_status_str), PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(status_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, status_param);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable timezone service which will be require for setting appropriate timezone
     * from the phone apps for scheduling to work correctly.
     * For more information on the various ways of setting timezone, please check
     * https://rainmaker.espressif.com/docs/time-service.html
     */
    esp_rmaker_timezone_service_enable();

    /* Enable scheduling. */
    esp_rmaker_schedule_enable();

    /* Enable Scenes */
    esp_rmaker_scenes_enable();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    err = app_wifi_set_custom_mfg_data(MFG_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    
    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_wifi_start(POP_TYPE_RANDOM);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Init SPI driver */
    spi = spi_init();

    /* PID Controller init */
#if (CONTROL_TYPE == PID) || (CONTROL_TYPE == PID_LOOKUP)
    pidInit();
#endif

    /* initialize heating element control (dimmer) */
    dimInit();

    /* Start counting time to enter standby mode */
    powerOnTimer = getAbsTime1us();
    esp_rmaker_param_update_and_report(poweron_param, esp_rmaker_bool(powerOn));

    ESP_LOGI(TAG, "Setpoint (C) | Temp (C) | pidOut (C) | control (%%) | power (%%)");

    while(1)
    {
        TaskBackground();

        count100ms++;
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void TaskBackground(void)
{
    /* 
     * This simple scheduler is used because creating FreeRTOS tasks
     * caused some issues with RainMaker. As it is a simple application
     * it is enough for the implemented features.
     */
    if (count100ms > 0)
    {
        if ((count100ms % 10) == 0)
        {
            /* Each 100 ms */
            Task100ms();
        }

        if ((count100ms % 5) == 0)
        {
            /* Each 500 ms*/
            Task500ms();
            
            /* Task counter */
            count500ms++;
        }

        if ((count100ms % 50) == 0)
        {
            /* Each 5000 ms */
            Task5000ms();
        }            
    }
}

void Task100ms(void)
{
    /* empty for now */
}

void Task500ms(void)
{
    /* temperature acquisition */
    spiComm();

    /* brewing program (and water pump) control */
    brewProgram();

    /* heating element control */
    heatingControl();
}

void heatingControl(void)
{
#if (CONTROL_TYPE == PID)

        pidOut = pidUpdate((float)tempSetpoint, tempCelsius);
        control = (int)pidOut;

#elif (CONTROL_TYPE == LOOKUP)

        delta = ((float)tempSetpoint - tempCelsius);

        float ir = indexRatio(deltaBkp, BKP_NUM, delta);
        const int control_lookup = (int)interp1D(controlSet, BKP_NUM, ir);
#if OVERSHOOT_DETECT_ENABLE
        if (temp_read_trusted && !pump_active) {
            if (!overshoot_boot_temp_sampled) {
                overshoot_boot_temp_sampled = true;
                overshoot_learn_try_arm_cold("boot trusted");
            }
            overshoot_peak_detector_update();
        }
        int control_after_trim = control_lookup;
        overshoot_apply_trim_to_control(&control_after_trim);
        control = control_after_trim;
#else
        control = control_lookup;
#endif

        /* reduce power to a factor by toggling power each task (idle only) */
        if (!pump_active && (delta > 0) && (delta <= TEMP_PWR_TOGGLE))
        {
            powerToggle = (int)((count500ms % POWER_FACTOR) == 0);
            control = (control * powerToggle);
        }

#elif (CONTROL_TYPE == PID_LOOKUP)

        pidOut = pidUpdate((float)tempSetpoint, tempCelsius);

        float ir = indexRatio(deltaBkp, BKP_NUM, pidOut);
        control = (int)interp1D(controlSet, BKP_NUM, ir);

#endif

    if (!pump_active) {
        temp_stuck_diag_update(control);
    }

    if (boiler_heater_holdoff()) {
        control = 0;
        ESP_LOGW(TAG, "Boiler heater held off: unsafe read, range, or stuck-temp diag.");
    }
    else if (brewState == BREW_POWER_OFF)
    {
        /* switch off heating element and remain in standby */
        control = 0;
    }
    else if (pump_active)
    {
        unsigned s = (unsigned)((getAbsTime1us() - pump_start_us) / 1000000ULL);
        control = pumpOnHeatBuff[s];
    }

    setPower(ptr_dimmer, (int)control);


    ESP_LOGI(TAG, "%d | %.2f | %.2f | %d | %d", tempSetpoint, tempCelsius, pidOut, control, getPower(ptr_dimmer));
}

void Task5000ms(void)
{
    if (powerOn)
    {
        /* only report data to app when device is on, to save MQTT budget */
        temp_status_line_report();
        boiler_status_report();
#if OVERSHOOT_DETECT_ENABLE
        overshoot_disp_report();
#endif
    }
}

void brewProgram(void)
{
    /*
     * This signal (enabled or disabled by the Temp Lock switch) will cancel a
     * brewing attempt if temperature is not within an acceptable range (calibrated
     * by TEMP_DELTA).
     */
    if (!temp_read_trusted || temp_stuck_diag) {
        tempRangeOk = false;
    } else if ((tempCelsius < (tempSetpoint - TEMP_DELTA)) || (tempCelsius > (tempSetpoint + TEMP_DELTA)))
    {
        tempRangeOk = false;
    }
    else
    {
        tempRangeOk = true;
    }

    /* Stop pump: power off; or user cleared Brew during brew; or cleared Flush during flush. */
    if (brewState != BREW_POWER_OFF) {
        const bool stop_brew = !brewSignal &&
                               (brewState == BREW_PREINF_ON || brewState == BREW_PREINF_OFF ||
                                brewState == BREW_ON);
        const bool stop_flush = !flushSignal && brewState == BREW_FLUSH;

        if (!powerOn || stop_brew || stop_flush) {
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
            pump_active = false;
            brewSignal = false;
            flushSignal = false;
            brewState = BREW_OFF;
        }
    }

    /*
     * Brewing phases and water pump control
     */
    switch (brewState)
    {
        case BREW_OFF:

            if (brewSignal == true)
            {
                if ((tempRangeOk) || (tempLock == false))
                {
                    pumpTimer = getAbsTime1us();

                    if (preInfusion)
                    {
                        brewState = BREW_PREINF_ON;
                    }
                    else
                    {
                        brewState = BREW_ON;
                    }
                    pump_active = true;
                    pump_start_us = pumpTimer;

                    /* reset power on timer */
                    powerOnTimer = getAbsTime1us();
                }
                else
                {
                    /* abort brew attempt and reset button */
                    ESP_LOGI(TAG, "Brew aborted! Setpoint temperature not reached");
                    brewSignal = false;
                    flushSignal = false;
                }
            }
            else if (flushSignal == true)
            {
                pumpTimer = getAbsTime1us();
                brewState = BREW_FLUSH;
                pump_active = true;
                pump_start_us = pumpTimer;
                powerOnTimer = getAbsTime1us();
                ESP_LOGI(TAG, "Flush started");
            }
            else
            {
                /* turn power off if Brew Signal is not pressed after POWERON_MIN (default 15 min) */
                if ((powerOn == false) || ((getAbsTime1us() - powerOnTimer) > POWERON_MIN))
                {
                    /* switch off heating element and remain in standby */
                    brewState = BREW_POWER_OFF;

                    powerOn = false;
#if OVERSHOOT_DETECT_ENABLE
                    overshoot_learn_ever_armed = false;
#endif
                    esp_rmaker_param_update_and_report(poweron_param, esp_rmaker_bool(powerOn));

                    ESP_LOGI(TAG, "Switching power OFF!");
                }
            }

        break;

        case BREW_PREINF_ON:

            if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(preInfOnTime))
            {
                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
            }
            else
            {
                brewState = BREW_PREINF_OFF;
                pump_active = false;
                pumpTimer = getAbsTime1us();
            }

        break;

        case BREW_PREINF_OFF:

            if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(preInfOffTime))
            {
                /* keep pump off */
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
            }
            else
            {
                brewState = BREW_ON;
                pumpTimer = getAbsTime1us();
                pump_active = true;
                pump_start_us = pumpTimer;
            }

        break;

        case BREW_ON:

            if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(brewTime))
            {
                /* brew */
                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
            }
            else
            {
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                pump_active = false;
                brewState = BREW_OFF;

                ESP_LOGI(TAG, "Brew cycle completed!");
                brewSignal = false;     /* reset for next press */
                flushSignal = false;

                /* Save brewing parameters to NVS */
                nvsWrite();
            }

        break;

        case BREW_POWER_OFF:

            /* Standby: leaving on Power on — reset latches so a clean idle is guaranteed. */
            if (powerOn == true)
            {
                powerOnTimer = getAbsTime1us();
                brewState = BREW_OFF;
                brewSignal = false;
                flushSignal = false;
#if OVERSHOOT_DETECT_ENABLE
                overshoot_learn_try_arm_cold("standby->on");
#endif

                ESP_LOGI(TAG, "Switching power ON!");
            }

        break;

        case BREW_FLUSH:

            if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(flushTime)) {
                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
            } else {
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                pump_active = false;
                brewState = BREW_OFF;
                brewSignal = false;
                flushSignal = false;
                ESP_LOGI(TAG, "Flush completed");
            }

        break;

        default:
        break;
    }
}
