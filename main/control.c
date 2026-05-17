/*
   Espresso — Control logic

   Owns all boiler temperature and brew-program state. Provides:
     - Shared state variables (extern'd in control.h)
     - SPI thermocouple acquisition (spiComm)
     - Heating element control via lookup / PID (heatingControl)
     - Brew and pre-infusion state machine (brewProgram)
     - Temperature diagnostics and stuck-sensor detection
     - Adaptive dither: overshoot and stall detection (LOOKUP mode only)
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

/* Minimum safe driver value — see controlSet[] comment for the gate-overflow rationale */
#define CONTROL_MIN_NONZERO     5

/* Approach window: dithering and stall detection active within this delta */
#define DELTA_PWR_TOGGLE    10   /* °C below setpoint */

#define POWERON_MIN         (15 * SEC_TO_US(60))    /* standby timeout (15 min) */

/* Status severity priorities */
#define STATUS_PRI_WARMUP_STALL 10
#define STATUS_PRI_ZERO         25
#define STATUS_PRI_UNTRUSTED    50
#define STATUS_PRI_TOO_HIGH     75
#define STATUS_PRI_STUCK        100
#define STATUS_WORST_CLEAR_OK_REPORTS   12  /* Task5000ms ticks (~60 s) */

#if ADAPTIVE_WARMUP_ENABLE
#define DITHER_STALL_SEC        30.0f   /* no delta progress → stall */
#define DITHER_SOFT_SETTLE_SEC  60.0f   /* wait at setpoint before clean commit */
#define OVERSHOOT_SEVERE_PEAK_C  5.0f   /* peak ≥ this → step+2, else step+1 */

/* Named aliases for dither_steps[] indices */
#define POWER_100   0
#define POWER_75    1
#define POWER_50    2
#define POWER_25    3

/* Power factor applied within TEMP_DELTA of setpoint for accurate
 * temperature maintenance — lower than the adaptive approach step. */
#define POWER_NEAR_SETPOINT  POWER_50
#endif

/*****************************************************************************
 * Lookup tables
 *****************************************************************************/

/*
 * Driver constraint: use 0 (fully off) or ≥ 5.  Values 1–4 are not functional.
 * The fixed gate pulse (4 timer steps) overflows the half-cycle boundary for
 * those values, re-latching the TRIAC and delivering ~25 % unintended power.
 * Actual power delivery is highly nonlinear — see README for the reference table.
 */
/* Temperature delta vs power setpoint breakpoints (LOOKUP mode) */
static float   deltaBkp[BKP_NUM] = {-10,   0,  0.5,   1,   2,   4,   10,   25,   50,   70};
static float controlSet[BKP_NUM] = {  0,   0,   10,  10,  20,  30,   40,   50,   80,  100};

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
/*  s:  0    1    2    3     4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19 */
       100, 100, 100, 100,  80,  80,  50,  50,  40,  40,  40,  50,  50,  50,  50,  70,  70,  70,  70,  70
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

#if ADAPTIVE_WARMUP_ENABLE
int dither_step = DITHER_STEP_DEFAULT;
#endif

/* RainMaker param handles — defined here, created by rainmaker_init() */
esp_rmaker_param_t *primary;
esp_rmaker_param_t *status_param;
esp_rmaker_param_t *poweron_param;
#if ADAPTIVE_WARMUP_ENABLE
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

#if ADAPTIVE_WARMUP_ENABLE
static char               s_dither_disp_str[20];
static float              overshoot_excursion_pk;
static bool               overshoot_excursion_latched;
static bool               overshoot_detected;
static bool               learn_armed;
static bool               learn_ever_armed;
static bool               boot_temp_sampled;
static unsigned long long soft_timer_us;
static bool               stall_active;
static float              stall_delta_min;
static unsigned long long stall_timer_us;
static bool               stall_armed;
static bool               stall_nvs_pending;
static bool               warmup_nv_committed; /* true after first NVS write; maintenance mode */
static int                dither_step_nv;      /* shadow of the last NVS-persisted step */
#endif

/*****************************************************************************
 * Forward declarations (private)
 *****************************************************************************/

static void temp_read_note_fault(uint32_t bit);
static void temp_read_note_good(float celsius);
static void temp_stuck_diag_update(int heating_demand_pct);
static bool boiler_heater_holdoff(void);

#if ADAPTIVE_WARMUP_ENABLE
static void adaptive_try_arm_cold(void);
static void stall_detector_update(void);
static void overshoot_peak_detector_update(void);
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
#if ADAPTIVE_WARMUP_ENABLE
    } else if (stall_active) {
        cur_pri = STATUS_PRI_WARMUP_STALL;
        cur_msg = "Warmup stall";
#endif
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
 * Adaptive dither — overshoot and stall detection (LOOKUP mode only)
 *****************************************************************************/

#if ADAPTIVE_WARMUP_ENABLE

/* Duty step table: (period, on_count) → duty = on_count / period */
typedef struct { uint8_t period; uint8_t on_count; } dither_step_t;
static const dither_step_t dither_steps[DITHER_STEP_COUNT] = {
    {1, 1},  /* step 0: 100 % */
    {4, 3},  /* step 1:  75 % */
    {2, 1},  /* step 2:  50 % — default */
    {4, 1},  /* step 3:  25 % */
};
_Static_assert(sizeof(dither_steps) / sizeof(dither_steps[0]) == DITHER_STEP_COUNT,
               "dither_steps[] size mismatch with DITHER_STEP_COUNT");

/* ---- arm / disarm ---------------------------------------------------- */

static void adaptive_try_arm_cold(void)
{
    if (!powerOn || !temp_read_trusted) {
        return;
    }
    if (!learn_ever_armed) {
        dither_step_nv = dither_step; /* capture the NVS value loaded at boot */
    }
    learn_armed             = true;
    learn_ever_armed        = true;
    overshoot_excursion_latched = false;
    overshoot_excursion_pk  = 0.f;
    soft_timer_us           = 0;
    overshoot_detected      = false;
    stall_armed             = false;
    stall_active            = false;
    stall_nvs_pending       = false;
    warmup_nv_committed     = false;
}

/* ---- stall detector -------------------------------------------------- */

static void stall_detector_update(void)
{
    if (!learn_armed || temp_stuck_diag) {
        stall_armed = false;
        return;
    }

    const bool in_approach = (delta > 0.f && delta <= (float)DELTA_PWR_TOGGLE);

    if (!in_approach) {
        stall_armed  = false;
        stall_active = false; /* left approach zone — clear status */
        return;
    }

    if (!stall_armed) {
        /* Entering approach window: snapshot best delta and start timer */
        stall_delta_min = delta;
        stall_timer_us  = getAbsTime1us();
        stall_armed     = true;
        return;
    }

    if (delta < stall_delta_min) {
        /* Progress: reset timer */
        stall_delta_min = delta;
        stall_timer_us  = getAbsTime1us();
        return;
    }

    if ((getAbsTime1us() - stall_timer_us) >=
            (unsigned long long)SEC_TO_US(DITHER_STALL_SEC)) {
        stall_armed = false; /* one fire per approach entry */
        if (dither_step > 0) {
            dither_step--;
            if (!warmup_nv_committed) {
                stall_nvs_pending = true; /* deferred NVS write for cold warmup only */
            }
        }
        stall_active = true;
        ESP_LOGI(TAG, "Warmup stall: step -> %d (%d%% duty)%s",
                 dither_step,
                 dither_steps[dither_step].on_count * 100 / dither_steps[dither_step].period,
                 warmup_nv_committed ? " [maintenance]" : "");
        dither_disp_report();
    }
}

/* ---- overshoot peak detector ----------------------------------------- */

static void overshoot_peak_detector_update(void)
{
    if (!learn_armed || temp_stuck_diag) {
        return;
    }

    /* Discard excursion state on setpoint change — prevents false latch
     * when setpoint is moved down below the current temperature. */
    static int32_t last_sp = 0;
    if (tempSetpoint != last_sp) {
        last_sp                     = tempSetpoint;
        overshoot_excursion_latched = false;
        overshoot_excursion_pk      = 0.f;
        overshoot_detected          = false;
        soft_timer_us               = 0;
    }

    const float sp = (float)tempSetpoint;

    if (tempCelsius > sp + (float)TEMP_DELTA) {
        overshoot_detected      = true;
        overshoot_excursion_latched = true;
        soft_timer_us           = 0; /* cancel soft timer */
    }

    /* Soft timer: cold warmup only — started on first clean reach of setpoint.
     * If no overshoot fires within DITHER_SOFT_SETTLE_SEC, commit any pending
     * stall step and disarm. Skipped in maintenance (warmup_nv_committed). */
    if (!overshoot_detected && !warmup_nv_committed) {
        if (soft_timer_us == 0ULL && tempCelsius >= sp) {
            soft_timer_us = getAbsTime1us();
        } else if (soft_timer_us != 0ULL &&
                   (getAbsTime1us() - soft_timer_us) >=
                       (unsigned long long)SEC_TO_US(DITHER_SOFT_SETTLE_SEC)) {
            if (stall_nvs_pending) {
                stall_nvs_pending = false;
                nvs_persist_dither_step();
                dither_step_nv = dither_step;
                ESP_LOGI(TAG, "Stall step committed: step %d (%d%%)",
                         dither_step,
                         dither_steps[dither_step].on_count * 100 /
                             dither_steps[dither_step].period);
            }
            warmup_nv_committed = true;
            soft_timer_us       = 0;
            dither_disp_report(); /* learn_armed stays true — maintenance begins */
        }
    }

    /* Track peak excursion above setpoint */
    if (overshoot_excursion_latched && tempCelsius > sp) {
        overshoot_excursion_pk = fmaxf(overshoot_excursion_pk, tempCelsius - sp);
    }

    /* Commit when temp drops back to setpoint after an overshoot */
    if (tempCelsius <= sp && overshoot_excursion_latched &&
            overshoot_excursion_pk > 0.01f) {
        const float peak   = overshoot_excursion_pk;
        const int step_inc = (peak >= OVERSHOOT_SEVERE_PEAK_C) ? 2 : 1;
        const int new_step = dither_step + step_inc;
        dither_step         = (new_step < DITHER_STEP_COUNT) ? new_step
                                                              : DITHER_STEP_COUNT - 1;
        stall_nvs_pending   = false;
        soft_timer_us       = 0;
        overshoot_detected  = false;
        overshoot_excursion_pk      = 0.f;
        overshoot_excursion_latched = false;
        if (!warmup_nv_committed) {
            nvs_persist_dither_step();
            dither_step_nv      = dither_step;
            warmup_nv_committed = true; /* learn_armed stays true — maintenance begins */
        }
        /* maintenance: learn_armed stays true, step change is RAM-only */
        ESP_LOGI(TAG, "Overshoot: peak +%.2f°C -> step+%d -> step %d (%d%%)%s",
                 (double)peak, step_inc, dither_step,
                 dither_steps[dither_step].on_count * 100 /
                     dither_steps[dither_step].period,
                 warmup_nv_committed ? " [maintenance]" : "");
        dither_disp_report();
        return;
    }

    if (tempCelsius <= sp) {
        overshoot_excursion_latched = false;
    }
}

/* ---- display --------------------------------------------------------- */

void dither_disp_report(void)
{
    if (!powerOn) {
        return;
    }
    snprintf(s_dither_disp_str, sizeof(s_dither_disp_str), "%d%%",
             dither_steps[dither_step].on_count * 100 /
                 dither_steps[dither_step].period);
    if (overshoot_disp_param) {
        esp_err_t e = esp_rmaker_param_update_and_report(overshoot_disp_param,
                                                          esp_rmaker_str(s_dither_disp_str));
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Dither display report: %s", esp_err_to_name(e));
        }
    }
}

int dither_step_pct(void)
{
    return dither_steps[dither_step].on_count * 100 / dither_steps[dither_step].period;
}

/* ---- public API ------------------------------------------------------ */

void control_on_power_on(void)
{
    adaptive_try_arm_cold();
}

void control_on_power_off(void)
{
    dither_step         = dither_step_nv; /* discard maintenance-only RAM adjustments */
    learn_armed         = false;
    learn_ever_armed    = false;
    stall_armed         = false;
    stall_active        = false;
    stall_nvs_pending   = false;
    warmup_nv_committed = false;
}

void control_reset_dither(void)
{
    dither_step             = DITHER_STEP_DEFAULT;
    learn_armed             = false;
    learn_ever_armed        = false;
    soft_timer_us           = 0;
    overshoot_detected      = false;
    overshoot_excursion_latched = false;
    overshoot_excursion_pk  = 0.f;
    stall_armed             = false;
    stall_active            = false;
    stall_nvs_pending       = false;
    nvs_persist_dither_step();
    dither_step_nv          = dither_step;
    ESP_LOGI(TAG, "Dither reset: step %d (%d%%)", DITHER_STEP_DEFAULT,
             dither_steps[DITHER_STEP_DEFAULT].on_count * 100 /
                 dither_steps[DITHER_STEP_DEFAULT].period);
    dither_disp_report();
}

#else /* !ADAPTIVE_WARMUP_ENABLE */

void control_on_power_on(void) {}
void control_on_power_off(void) {}

#endif /* ADAPTIVE_WARMUP_ENABLE */

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

#if ADAPTIVE_WARMUP_ENABLE
    if (temp_read_trusted && !pump_active) {
        if (!boot_temp_sampled) {
            boot_temp_sampled = true;
            adaptive_try_arm_cold();
        }
        if (learn_ever_armed) {
            stall_detector_update();
            overshoot_peak_detector_update();
        }
    }
    /* Adaptive dithering — always active in the approach window.
     * Within TEMP_DELTA of setpoint, lock to step 3 (33 %) to cap power
     * and prevent stall/overshoot from fighting each other near target. */
    if (!pump_active && delta > 0.f && delta <= (float)DELTA_PWR_TOGGLE) {
        const dither_step_t *s = (delta <= (float)TEMP_DELTA)
                                 ? &dither_steps[POWER_NEAR_SETPOINT]
                                 : &dither_steps[dither_step];
        control = ((tick % (int)s->period) < (int)s->on_count) ? control_lookup : 0;
    } else {
        control = control_lookup;
    }
#else
    control = control_lookup;
#endif

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

    if (control > 0 && control < CONTROL_MIN_NONZERO) {
        control = CONTROL_MIN_NONZERO;
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
#if ADAPTIVE_WARMUP_ENABLE
                learn_ever_armed = false;
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
#if ADAPTIVE_WARMUP_ENABLE
            adaptive_try_arm_cold();
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
