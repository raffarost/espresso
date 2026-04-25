/*
   Espresso — shared state, configuration, and control API

   This header is included by all modules that share control state or call
   control functions. It owns the CONTROL_TYPE / OVERSHOOT_DETECT_ENABLE
   compile-time knobs so every translation unit sees the same setting.
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp32-triac-dimmer-driver.h"
#include "esp_rmaker_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*****************************************************************************
 * Control mode selection
 *****************************************************************************/
#define PID         0
#define LOOKUP      1
#define PID_LOOKUP  2

#define CONTROL_TYPE    LOOKUP

/* Overshoot learn/trim for LOOKUP only; on by default. */
#if (CONTROL_TYPE == LOOKUP) && !defined(OVERSHOOT_DETECT_ENABLE)
#define OVERSHOOT_DETECT_ENABLE  1
#endif

/*****************************************************************************
 * Hardware pin assignments
 *****************************************************************************/
#define GRID_FREQ           60          /* power grid frequency (Hz) */
#define ZEROCROSS_GPIO      GPIO_NUM_5
#define TRIAC_1_GPIO        GPIO_NUM_33
#define GPIO_OUTPUT_IO_0    GPIO_NUM_4  /* water pump relay */
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << TRIAC_1_GPIO))

/*****************************************************************************
 * Temperature setpoint limits (shared with write_cb and NVS clamping)
 *****************************************************************************/
#define TEMP_SETPOINT_MIN   88
#define TEMP_SETPOINT_MAX   96

/*****************************************************************************
 * Overshoot NVS encoding helpers (also used by nvs.c)
 *****************************************************************************/
#if OVERSHOOT_DETECT_ENABLE
#define OVERSHOOT_TRIM_FRAC_MAX             0.20f
#define OVERSHOOT_TRIM_FRAC_PER_DEG         0.04f
#define OVERSHOOT_TRIM_FRAC_TO_MPCT(f)      ((int32_t)lroundf((f) * 100000.0f))
#define OVERSHOOT_TRIM_MPCT_TO_FRAC(i)      ((float)(i) * 0.00001f)
#endif

/*****************************************************************************
 * Shared mutex — created in main.c before tasks_start()
 *****************************************************************************/
extern SemaphoreHandle_t g_state_mutex;

/*****************************************************************************
 * Hardware handles — initialised in main.c / control.c init functions
 *****************************************************************************/
extern spi_device_handle_t spi;
extern dimmertyp *ptr_dimmer;

/*****************************************************************************
 * Shared parameters
 * Written from write_cb (rainmaker.c), read by control tasks.
 * Callers must hold g_state_mutex when accessing from a task context.
 *****************************************************************************/
extern int32_t tempSetpoint;
extern int32_t brewTime;
extern int32_t flushTime;
extern int32_t preInfOnTime;
extern int32_t preInfOffTime;
extern bool    brewSignal;
extern bool    flushSignal;
extern bool    powerOn;
extern bool    tempLock;
extern bool    preInfusion;
extern float   tempCelsius;

#if OVERSHOOT_DETECT_ENABLE
extern float overshoot_trim_stored;
#endif

/*****************************************************************************
 * RainMaker param handles
 * Defined in rainmaker.c, used by control.c reporting functions.
 *****************************************************************************/
extern esp_rmaker_param_t *primary;
extern esp_rmaker_param_t *status_param;
extern esp_rmaker_param_t *poweron_param;
#if OVERSHOOT_DETECT_ENABLE
extern esp_rmaker_param_t *overshoot_disp_param;
#endif

/*****************************************************************************
 * Control API
 *****************************************************************************/
void dimInit(void);
void spiComm(void);
void heatingControl(void);
void brewProgram(void);
void temp_status_line_report(void);
void boiler_status_report(void);

/* Called from write_cb when the Power parameter changes */
void control_on_power_on(void);
void control_on_power_off(void);

#if OVERSHOOT_DETECT_ENABLE
void overshoot_disp_report(void);
void control_reset_overshoot_trim(void);
#endif
