/*
   Espresso — Control logic

   Owns all boiler temperature and brew-program state. Provides:
     - Shared state variables (extern'd in control.h)
     - SPI thermocouple acquisition (spiComm)
     - Heating element control via lookup / PID (heatingControl)
     - Brew and pre-infusion state machine (brewProgram)
     - Temperature diagnostics and stuck-sensor detection
     - Overshoot learn / trim (LOOKUP mode only)
     - RainMaker status reporting helpers
*/

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"
#include "esp32-triac-dimmer-driver.h"
#include "spi_mod.h"
#include "private.h"
#include "nvs.h"
#include "control.h"

static const char *TAG = "espresso";

/*****************************************************************************
 * Private configuration defines
 *****************************************************************************/

#define BKP_NUM     10

#define SEC_TO_US(x)        ((x) * 1000000ULL)

#define BREW_OFF            0
#define BREW_PREINF_ON      1
#define BREW_PREINF_OFF     2
#define BREW_ON             3
#define BREW_POWER_OFF      4
#define BREW_FLUSH          5

#define PUMP_ON_HEAT_BUFF_LEN   20  /* must be >= max brew time (s) */

#define TEMP_DELTA              2   /* ±°C window around setpoint for ready state */
#define MAX_TEMP_THR            110 /* hard safety cut-off (°C) */

#define TEMP_READ_OK_STREAK     3   /* consecutive good samples before trusting */
#define TEMP_VALID_MIN_C        (-2.0f)
#define TEMP_VALID_MAX_C        (125.0f)
#define TEMP_DIAG_SPI           (1u << 0)
#define TEMP_DIAG_OPEN_TC       (1u << 1)
#define TEMP_DIAG_RANGE         (1u << 2)

/* Stuck sensor: high demand but temp unchanged for STUCK_DIAG_SAMPLES reads */
#define STUCK_DIAG_MIN_POWER_PCT    10
#define STUCK_DIAG_SAMPLES          5       /* 2.5 s @ 500 ms cadence */
#define STUCK_DIAG_TEMP_EPS         0.21f   /* < half MAX6675 LSB (0.25 °C) */
#define STUCK_DIAG_CLEAR_MAX_C      110.f
#define STUCK_DIAG_WARMUP_GRACE_SEC 10.f

/* Power dithering around setpoint */
#define POWER_50        2
#define POWER_33        3
#define POWER_25        4
#define POWER_FACTOR    POWER_50
#define TEMP_PWR_TOGGLE 2   /* °C window where power is dithered */

#define POWERON_MIN     (15 * SEC_TO_US(60))    /* standby timeout (15 min) */

/* Status severity priorities */
#define STATUS_PRI_ZERO         25
#define STATUS_PRI_UNTRUSTED    50
#define STATUS_PRI_TOO_HIGH     75
#define STATUS_PRI_STUCK        100
#define STATUS_WORST_CLEAR_OK_REPORTS   12  /* Task5000ms ticks (~60 s) */

#if OVERSHOOT_DETECT_ENABLE
#define OVERSHOOT_SOFT_DISARM_SEC               90.f
#define OVERSHOOT_TRIM_DEMAND_PCT_THRESHOLD     30.0f
#define OVERSHOOT_COLD_START_MAX_C              60.0f
#define OS_DISARM_SOFT_TIMEOUT  (1u << 0)
#define OS_DISARM_COMMIT        (1u << 1)
#endif

/*****************************************************************************
 * Lookup tables
 *****************************************************************************/

/* Temperature delta vs power setpoint breakpoints (LOOKUP mode) */
static float   deltaBkp[BKP_NUM] = {-10,  0, 0.5,  1,  2,  4,  10,  25,  50,  70};
static float controlSet[BKP_NUM] = {  0,  0,   1,  1,  1,  2,  15,  30,  60,  80};

/*
 * Per-second heater power (%) while the pump is on.
 * pumpTimer resets at BREW_PREINF_ON and again at BREW_ON.
 *
 * Phase 1 (~4 s) : free-flow before puck resistance builds
 * Phase 2 (~4 s) : puck compressing, flow becoming restricted
 * Phase 3 (~6 s) : puck at maximum compression
 * Phase 4 (~6 s+): puck deteriorates, flow recovers
 */
static int pumpOnHeatBuff[PUMP_ON_HEAT_BUFF_LEN] = {
/*  s:  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19 */
       100, 100, 100, 100,  80,  80,  40,  40,  30,  30,  30,  40,  40,  40,  40,  60,  60,  60,  60,  60
};

/*****************************************************************************
 * Exported shared state (extern'd in control.h)
 *****************************************************************************/

spi_device_handle_t spi;
dimmertyp          *ptr_dimmer;

int32_t tempSetpoint = 96;
int32_t brewTime     = 6;
int32_t flushTime    = 5;
int32_t preInfOnTime  = 3;
int32_t preInfOffTime = 10;
bool    brewSignal  = false;
bool    flushSignal = false;
bool    powerOn     = false;
bool    tempLock    = true;
bool    preInfusion = true;
float   tempCelsius;

#if OVERSHOOT_DETECT_ENABLE
float overshoot_trim_stored;
#endif

/* RainMaker param handles — defined here, created by rainmaker_init() */
esp_rmaker_param_t *primary;
esp_rmaker_param_t *status_param;
esp_rmaker_param_t *poweron_param;
#if OVERSHOOT_DETECT_ENABLE
esp_rmaker_param_t *overshoot_disp_param;
#endif

/*****************************************************************************
 * Private module state
 *****************************************************************************/

static spi_transaction_t s_spi_txn = {
    .tx_buffer = NULL,
    .rx_buffer = NULL,      /* set to &s_spi_data in spiComm() */
    .length    = 16,
    .rxlength  = 16,
};
static uint16_t s_spi_data;

static float   delta;
static float   pidOut;
static int     control;
static int     powerToggle;
static unsigned long long pumpTimer;
static bool    pump_active;
static unsigned long long powerOnTimer;
static int     brewState = BREW_POWER_OFF;
static bool    tempRangeOk;

static bool     temp_read_trusted;
static uint8_t  temp_read_good_streak;
static uint32_t temp_read_fault_latch;

static bool              temp_stuck_diag;
static float             temp_stuck_prev_c;
static uint8_t           temp_stuck_same_ct;
static bool              temp_sensor_responsive;
static unsigned long long temp_stuck_warmup_start_us;

static char     s_temp_line_str[40];
static char     s_status_str[96];
static int      s_status_worst_pri;
static char     s_status_worst_str[48];
static uint16_t s_status_ok_clean_reports;

#if OVERSHOOT_DETECT_ENABLE
static char              s_overshoot_disp_str[28];
static float             overshoot_excursion_pk;
static bool              overshoot_excursion_latched;
static bool              overshoot_detected;
static bool              overshoot_learn_allowed;
static bool              overshoot_learn_ever_armed;
static uint32_t          overshoot_learn_disarm_mask;
static bool              overshoot_boot_temp_sampled;
static unsigned long long overshoot_soft_timer_start_us;
#endif

/*****************************************************************************
 * Forward declarations (private)
 *****************************************************************************/

static void temp_read_note_fault(uint32_t bit);
static void temp_read_note_good(float celsius);
static void temp_stuck_diag_update(int heating_demand_pct);
static bool boiler_heater_holdoff(void);

#if OVERSHOOT_DETECT_ENABLE
static void overshoot_learn_try_arm_cold(const char *site);
static void overshoot_peak_detector_update(void);
static void overshoot_disp_format_str(void);
static void overshoot_apply_trim_to_control(int *p_control);
static void overshoot_learn_set_disallowed(uint32_t reason_bits);
#endif

/*****************************************************************************
 * Hardware init
 *****************************************************************************/

void dimInit(void)
{
    ptr_dimmer = createDimmer(TRIAC_1_GPIO, ZEROCROSS_GPIO);
    begin(ptr_dimmer, NORMAL_MODE, ON, GRID_FREQ);
    setPower(ptr_dimmer, 0);
    ESP_LOGI(TAG, "Dimmer initialized");
}

/*****************************************************************************
 * Temperature read helpers
 *****************************************************************************/

static void temp_read_note_fault(uint32_t bit)
{
    temp_read_good_streak = 0;
    temp_read_trusted = false;
    const uint32_t prev = temp_read_fault_latch;
    temp_read_fault_latch |= bit;
    if (temp_read_fault_latch != prev) {
        ESP_LOGW(TAG, "Boiler temp read fault (latch 0x%02" PRIx32 ")",
                 temp_read_fault_latch);
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

bool boiler_temp_is_trusted(void)
{
    return temp_read_trusted;
}

uint32_t boiler_temp_fault_bits(void)
{
    return temp_read_fault_latch;
}

/*****************************************************************************
 * SPI thermocouple acquisition
 *****************************************************************************/

void spiComm(void)
{
    s_spi_txn.rx_buffer = &s_spi_data;

    esp_err_t ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI acquire bus failed: %s", esp_err_to_name(ret));
        temp_read_note_fault(TEMP_DIAG_SPI);
        return;
    }

    ret = spi_device_transmit(spi, &s_spi_txn);
    spi_device_release_bus(spi);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        temp_read_note_fault(TEMP_DIAG_SPI);
        return;
    }

    const int16_t res = (int16_t)SPI_SWAP_DATA_RX(s_spi_data, 16);

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

/*****************************************************************************
 * Stuck-sensor diagnostic
 *****************************************************************************/

static void temp_stuck_diag_update(int heating_demand_pct)
{
    const float t = tempCelsius;

    /* Reset session state on power off; latch clears only when reading moves. */
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

    /* First heating sample before sensor responsive: start warmup grace timer. */
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
        const unsigned long long elapsed = getAbsTime1us() - temp_stuck_warmup_start_us;
        if (elapsed < (unsigned long long)SEC_TO_US(STUCK_DIAG_WARMUP_GRACE_SEC)) {
            return;
        }
    }

    if (temp_stuck_same_ct < 255) {
        temp_stuck_same_ct++;
    }
    if (temp_stuck_same_ct >= STUCK_DIAG_SAMPLES) {
        temp_stuck_diag = true;
        ESP_LOGW(TAG,
                 "Boiler temp stuck: demand >= %d%%, %d identical samples (~%.2f C, %s)",
                 STUCK_DIAG_MIN_POWER_PCT, STUCK_DIAG_SAMPLES, (double)t,
                 temp_sensor_responsive ? "post-movement" : "warmup grace elapsed");
    }
}

static bool boiler_heater_holdoff(void)
{
    return !temp_read_trusted || (tempCelsius == 0.f) ||
           (tempCelsius > (float)MAX_TEMP_THR) || temp_stuck_diag;
}

/*****************************************************************************
 * RainMaker status reporting
 *****************************************************************************/

void temp_status_line_report(void)
{
    const char *state;

    if (tempRangeOk) {
        state = "Ready";
    } else if (tempCelsius < (float)(tempSetpoint - TEMP_DELTA)) {
        state = "Low";
    } else {
        state = "High";
    }

    snprintf(s_temp_line_str, sizeof(s_temp_line_str),
             "%.1f \xc2\xb7 %s", (double)tempCelsius, state);

    if (primary) {
        esp_err_t e = esp_rmaker_param_update_and_report(primary,
                                                         esp_rmaker_str(s_temp_line_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Temperature line report: %s", esp_err_to_name(e));
        }
    }
}

void boiler_status_report(void)
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
                snprintf(s_status_str, sizeof(s_status_str),
                         "Okay (%s)", s_status_worst_str);
            }
        } else {
            snprintf(s_status_str, sizeof(s_status_str), "Okay");
            s_status_ok_clean_reports = 0;
        }
    } else {
        s_status_ok_clean_reports = 0;
        if (s_status_worst_pri > cur_pri && s_status_worst_str[0] != '\0') {
            snprintf(s_status_str, sizeof(s_status_str),
                     "%s (%s)", cur_msg, s_status_worst_str);
        } else {
            snprintf(s_status_str, sizeof(s_status_str), "%s", cur_msg);
        }
    }

    if (status_param) {
        esp_err_t e = esp_rmaker_param_update_and_report(status_param,
                                                          esp_rmaker_str(s_status_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Status report: %s", esp_err_to_name(e));
        }
    }
}

/*****************************************************************************
 * Overshoot learn / trim (LOOKUP mode only)
 *****************************************************************************/

#if OVERSHOOT_DETECT_ENABLE

static void overshoot_learn_set_disallowed(uint32_t reason_bits)
{
    overshoot_learn_disarm_mask |= reason_bits;
    overshoot_learn_allowed = false;
}

static void overshoot_learn_try_arm_cold(const char *site)
{
    if (!powerOn || !temp_read_trusted || tempCelsius > OVERSHOOT_COLD_START_MAX_C) {
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
    if (!overshoot_learn_allowed || temp_stuck_diag) {
        return;
    }

    const float temp_ok_max = (float)tempSetpoint + (float)TEMP_DELTA;

    if (tempCelsius > temp_ok_max) {
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
            ESP_LOGI(TAG, "OS learn disarmed (soft): no excursion above setpoint+%d°C "
                     "within %.0f s", TEMP_DELTA, (double)OVERSHOOT_SOFT_DISARM_SEC);
            overshoot_disp_report();
            return;
        }
    }

    if (overshoot_excursion_latched && tempCelsius > (float)tempSetpoint) {
        const float above = tempCelsius - (float)tempSetpoint;
        overshoot_excursion_pk = fmaxf(overshoot_excursion_pk, above);
    }

    if (tempCelsius <= (float)tempSetpoint) {
        if (overshoot_excursion_latched && overshoot_excursion_pk > 0.01f) {
            const float peak_c   = overshoot_excursion_pk;
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
                     (overshoot_trim_stored >= OVERSHOOT_TRIM_FRAC_MAX - 1e-6f)
                         ? ", CAPPED" : "");
            nvs_persist_overshoot_trim();
            overshoot_learn_set_disallowed(OS_DISARM_COMMIT);
            overshoot_soft_timer_start_us = 0;
            overshoot_detected = false;
            overshoot_excursion_pk = 0.f;
        }
        overshoot_excursion_latched = false;
    }
}

static void overshoot_disp_format_str(void)
{
    const float cut_pct = overshoot_trim_stored * 100.0f;

    if (overshoot_learn_allowed && overshoot_excursion_latched &&
        overshoot_excursion_pk > 0.01f) {
        snprintf(s_overshoot_disp_str, sizeof(s_overshoot_disp_str),
                 "%s%.1f%% +%.1f°C",
                 cut_pct > 0.05f ? "-" : "",
                 (double)cut_pct, (double)overshoot_excursion_pk);
    } else {
        snprintf(s_overshoot_disp_str, sizeof(s_overshoot_disp_str),
                 "%s%.1f%%",
                 cut_pct > 0.05f ? "-" : "", (double)cut_pct);
    }
}

void overshoot_disp_report(void)
{
    if (!powerOn) {
        return;
    }
    overshoot_disp_format_str();
    if (overshoot_disp_param) {
        esp_err_t e = esp_rmaker_param_update_and_report(overshoot_disp_param,
                                                          esp_rmaker_str(s_overshoot_disp_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Overshoot display report: %s", esp_err_to_name(e));
        }
    }
}

static void overshoot_apply_trim_to_control(int *p_control)
{
    if ((float)*p_control > (float)OVERSHOOT_TRIM_DEMAND_PCT_THRESHOLD) {
        float cf = (float)*p_control * (1.f - overshoot_trim_stored);
        *p_control = (int)(cf + 0.5f);
    }
}

/* Public wrappers called from write_cb (rainmaker.c) */
void control_on_power_on(void)
{
    overshoot_learn_try_arm_cold("Power cb");
}

void control_on_power_off(void)
{
    overshoot_learn_ever_armed = false;
}

void control_reset_overshoot_trim(void)
{
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

#else /* !OVERSHOOT_DETECT_ENABLE */

void control_on_power_on(void) {}
void control_on_power_off(void) {}

#endif /* OVERSHOOT_DETECT_ENABLE */

/*****************************************************************************
 * Heating element control
 *****************************************************************************/

void heatingControl(void)
{
    static int tick = 0;
    tick++;

#if (CONTROL_TYPE == PID)
    pidOut  = pidUpdate((float)tempSetpoint, tempCelsius);
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

    /* Dither power within TEMP_PWR_TOGGLE window around setpoint (idle only) */
    if (!pump_active && (delta > 0) && (delta <= TEMP_PWR_TOGGLE)) {
        powerToggle = (int)((tick % POWER_FACTOR) == 0);
        control = control * powerToggle;
    }

#elif (CONTROL_TYPE == PID_LOOKUP)
    pidOut = pidUpdate((float)tempSetpoint, tempCelsius);
    float ir = indexRatio(deltaBkp, BKP_NUM, pidOut);
    control  = (int)interp1D(controlSet, BKP_NUM, ir);
#endif

    if (!pump_active) {
        temp_stuck_diag_update(control);
    }

    if (boiler_heater_holdoff()) {
        control = 0;
        ESP_LOGW(TAG, "Boiler heater held off: unsafe read, range, or stuck-temp diag.");
    } else if (brewState == BREW_POWER_OFF) {
        control = 0;
    } else if (pump_active) {
        if (brewState == BREW_FLUSH) {
            /* Flush: free-flow with no puck, run heater at full power. */
            control = 100;
        } else {
            /* Pre-infusion / brew: puck restricts flow, use calibrated buffer. */
            unsigned s = (unsigned)((getAbsTime1us() - pumpTimer) / 1000000ULL);
            if (s >= PUMP_ON_HEAT_BUFF_LEN) {
                s = PUMP_ON_HEAT_BUFF_LEN - 1;
            }
            control = pumpOnHeatBuff[s];
        }
    }

    setPower(ptr_dimmer, control);

    ESP_LOGI(TAG, "%d | %.2f | %.2f | %d | %d",
             tempSetpoint, tempCelsius, pidOut, control, getPower(ptr_dimmer));
}

/*****************************************************************************
 * Brew and pre-infusion state machine
 *****************************************************************************/

void brewProgram(void)
{
    /* Evaluate temperature readiness. */
    if (!temp_read_trusted || temp_stuck_diag) {
        tempRangeOk = false;
    } else if ((tempCelsius < (tempSetpoint - TEMP_DELTA)) ||
               (tempCelsius > (tempSetpoint + TEMP_DELTA))) {
        tempRangeOk = false;
    } else {
        tempRangeOk = true;
    }

    /* Unified stop condition: power off, or user cleared Brew/Flush signal. */
    if (brewState != BREW_POWER_OFF) {
        const bool stop_brew = !brewSignal &&
                               (brewState == BREW_PREINF_ON ||
                                brewState == BREW_PREINF_OFF ||
                                brewState == BREW_ON);
        const bool stop_flush = !flushSignal && brewState == BREW_FLUSH;

        if (!powerOn || stop_brew || stop_flush) {
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
            pump_active = false;
            brewSignal  = false;
            flushSignal = false;
            brewState   = BREW_OFF;
        }
    }

    switch (brewState) {
    case BREW_OFF:
        if (brewSignal) {
            if (tempRangeOk || !tempLock) {
                pumpTimer = getAbsTime1us();
                brewState = preInfusion ? BREW_PREINF_ON : BREW_ON;
                pump_active = true;
                powerOnTimer = getAbsTime1us();
            } else {
                ESP_LOGI(TAG, "Brew aborted! Setpoint temperature not reached");
                brewSignal  = false;
                flushSignal = false;
            }
        } else if (flushSignal) {
            pumpTimer    = getAbsTime1us();
            brewState    = BREW_FLUSH;
            pump_active  = true;
            powerOnTimer = getAbsTime1us();
            ESP_LOGI(TAG, "Flush started");
        } else {
            /* Auto standby after POWERON_MIN or explicit power-off. */
            if (!powerOn || (getAbsTime1us() - powerOnTimer) > POWERON_MIN) {
                brewState = BREW_POWER_OFF;
                powerOn   = false;
#if OVERSHOOT_DETECT_ENABLE
                overshoot_learn_ever_armed = false;
#endif
                esp_rmaker_param_update_and_report(poweron_param,
                                                   esp_rmaker_bool(powerOn));
                ESP_LOGI(TAG, "Switching power OFF!");
            }
        }
        break;

    case BREW_PREINF_ON:
        if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(preInfOnTime)) {
            gpio_set_level(GPIO_OUTPUT_IO_0, 1);
        } else {
            brewState   = BREW_PREINF_OFF;
            pump_active = false;
            pumpTimer   = getAbsTime1us();
        }
        break;

    case BREW_PREINF_OFF:
        if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(preInfOffTime)) {
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
        } else {
            brewState   = BREW_ON;
            pumpTimer   = getAbsTime1us();
            pump_active = true;
        }
        break;

    case BREW_ON:
        if ((getAbsTime1us() - pumpTimer) < SEC_TO_US(brewTime)) {
            gpio_set_level(GPIO_OUTPUT_IO_0, 1);
        } else {
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
            pump_active = false;
            brewState   = BREW_OFF;
            brewSignal  = false;
            flushSignal = false;
            ESP_LOGI(TAG, "Brew cycle completed!");
            nvsWrite();
        }
        break;

    case BREW_POWER_OFF:
        if (powerOn) {
            powerOnTimer = getAbsTime1us();
            brewState    = BREW_OFF;
            brewSignal   = false;
            flushSignal  = false;
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
            brewState   = BREW_OFF;
            brewSignal  = false;
            flushSignal = false;
            ESP_LOGI(TAG, "Flush completed");
        }
        break;

    default:
        break;
    }
}
