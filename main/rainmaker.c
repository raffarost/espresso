/*
   Espresso — ESP RainMaker integration

   Owns the RainMaker device, all parameter handles, the write callback, and
   the event handler. rainmaker_init() must be called after
   esp_rmaker_node_init() but before esp_rmaker_start().
*/

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_standard_devices.h"
#include "esp_rmaker_ota.h"
#include "esp_rmaker_common_events.h"
#include "app_wifi.h"
#include "control.h"

static const char *TAG = "espresso";

static esp_rmaker_device_t *espresso_device;

/*****************************************************************************
 * Write callback — called from RainMaker's MQTT task
 *****************************************************************************/

static esp_err_t write_cb(const esp_rmaker_device_t *device,
                           const esp_rmaker_param_t *param,
                           const esp_rmaker_param_val_t val,
                           void *priv_data,
                           esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via: %s",
                 esp_rmaker_device_cb_src_to_str(ctx->src));
    }

    const char *name = esp_rmaker_param_get_name(param);

    /* Take mutex with a short timeout; if the control task is busy, don't stall MQTT. */
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "write_cb: could not acquire mutex (param: %s)", name);
        return ESP_OK;
    }

    if (strcmp(name, "Temperature setpoint (\xc2\xb0""C)") == 0) {
        int32_t sp = val.val.i;
        if (sp < TEMP_SETPOINT_MIN) {
            sp = TEMP_SETPOINT_MIN;
        } else if (sp > TEMP_SETPOINT_MAX) {
            sp = TEMP_SETPOINT_MAX;
        }
        tempSetpoint = sp;
        ESP_LOGI(TAG, "New temperature setpoint: %d", tempSetpoint);

    } else if (strcmp(name, "Brew time (s)") == 0) {
        brewTime = val.val.i;
        ESP_LOGI(TAG, "Brew time: %d s", brewTime);

    } else if (strcmp(name, "Flush time (s)") == 0) {
        flushTime = val.val.i;
        ESP_LOGI(TAG, "Flush time: %d s", flushTime);

    } else if (strcmp(name, "Brew Signal") == 0) {
        brewSignal = val.val.b ? !brewSignal : brewSignal;
        ESP_LOGI(TAG, "Brew %s!", brewSignal ? "start" : "stop");

    } else if (strcmp(name, "Flush") == 0) {
        flushSignal = val.val.b ? !flushSignal : flushSignal;
        ESP_LOGI(TAG, "Flush %s!", flushSignal ? "on" : "off");

    } else if (strcmp(name, "Pre-Infusion") == 0) {
        preInfusion = val.val.b;
        ESP_LOGI(TAG, "Pre-Infusion: %s", preInfusion ? "ON" : "OFF");

    } else if (strcmp(name, "Power") == 0) {
        const bool was_on = powerOn;
        powerOn = val.val.b;
        ESP_LOGI(TAG, "Power: %s", powerOn ? "ON" : "OFF");
        if (!powerOn && was_on) {
            control_on_power_off();
        } else if (powerOn && !was_on) {
            control_on_power_on();
        }

    } else if (strcmp(name, "Temp Lock") == 0) {
        tempLock = val.val.b;
        ESP_LOGI(TAG, "Temp Lock: %s", tempLock ? "ON" : "OFF");

    } else if (strcmp(name, "Pre-Infusion on time (s)") == 0) {
        preInfOnTime = val.val.i;
        ESP_LOGI(TAG, "Pre-Infusion on time: %d s", preInfOnTime);

    } else if (strcmp(name, "Pre-Infusion off time (s)") == 0) {
        preInfOffTime = val.val.i;
        ESP_LOGI(TAG, "Pre-Infusion off time: %d s", preInfOffTime);

#if ADAPTIVE_WARMUP_ENABLE
    } else if (strcmp(name, "Reset power factor") == 0) {
        if (val.val.b) {
            control_reset_dither();
        }
#endif
    }

    xSemaphoreGive(g_state_mutex);
    return ESP_OK;
}

/*****************************************************************************
 * Event handler
 *****************************************************************************/

static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
        case RMAKER_EVENT_INIT_DONE:
            ESP_LOGI(TAG, "RainMaker initialised.");
            break;
        case RMAKER_EVENT_CLAIM_STARTED:
            ESP_LOGI(TAG, "RainMaker claim started.");
            break;
        case RMAKER_EVENT_CLAIM_SUCCESSFUL:
            ESP_LOGI(TAG, "RainMaker claim successful.");
            break;
        case RMAKER_EVENT_CLAIM_FAILED:
            ESP_LOGI(TAG, "RainMaker claim failed.");
            break;
        case RMAKER_EVENT_LOCAL_CTRL_STARTED:
            ESP_LOGI(TAG, "Local control started.");
            break;
        case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
            ESP_LOGI(TAG, "Local control stopped.");
            break;
        default:
            ESP_LOGW(TAG, "Unhandled RainMaker event: %" PRIi32, event_id);
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
            ESP_LOGI(TAG, "MQTT connected.");
            break;
        case RMAKER_MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected.");
            break;
        case RMAKER_MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT published. Msg id: %d.", *((int *)event_data));
            break;
        default:
            ESP_LOGW(TAG, "Unhandled RainMaker common event: %" PRIi32, event_id);
        }
    } else if (event_base == APP_WIFI_EVENT) {
        switch (event_id) {
        case APP_WIFI_EVENT_QR_DISPLAY:
            ESP_LOGI(TAG, "Provisioning QR: %s", (char *)event_data);
            break;
        case APP_WIFI_EVENT_PROV_TIMEOUT:
            ESP_LOGI(TAG, "Provisioning timed out. Please reboot.");
            break;
        case APP_WIFI_EVENT_PROV_RESTART:
            ESP_LOGI(TAG, "Provisioning restarted due to failures.");
            break;
        default:
            ESP_LOGW(TAG, "Unhandled Wi-Fi event: %" PRIi32, event_id);
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch (event_id) {
        case RMAKER_OTA_EVENT_STARTING:
            ESP_LOGI(TAG, "Starting OTA.");
            break;
        case RMAKER_OTA_EVENT_IN_PROGRESS:
            ESP_LOGI(TAG, "OTA in progress.");
            break;
        case RMAKER_OTA_EVENT_SUCCESSFUL:
            ESP_LOGI(TAG, "OTA successful.");
            break;
        case RMAKER_OTA_EVENT_FAILED:
            ESP_LOGI(TAG, "OTA failed.");
            break;
        case RMAKER_OTA_EVENT_REJECTED:
            ESP_LOGI(TAG, "OTA rejected.");
            break;
        case RMAKER_OTA_EVENT_DELAYED:
            ESP_LOGI(TAG, "OTA delayed.");
            break;
        case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
            ESP_LOGI(TAG, "Firmware downloaded. Reboot to apply.");
            break;
        default:
            ESP_LOGW(TAG, "Unhandled OTA event: %" PRIi32, event_id);
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

/*****************************************************************************
 * Public init — call after esp_rmaker_node_init(), before esp_rmaker_start()
 *****************************************************************************/

void rainmaker_init(esp_rmaker_node_t *node)
{
    static char temp_line_str[40];
    static char status_str[96];

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT,        ESP_EVENT_ANY_ID,
                                               &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID,
                                               &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_WIFI_EVENT,      ESP_EVENT_ANY_ID,
                                               &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT,    ESP_EVENT_ANY_ID,
                                               &event_handler, NULL));

    /* Create device */
    espresso_device = esp_rmaker_device_create("Espresso",
                                               ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    esp_rmaker_node_add_device(node, espresso_device);
    esp_rmaker_device_add_cb(espresso_device, write_cb, NULL);

    /* Temperature display (primary param) */
    snprintf(temp_line_str, sizeof(temp_line_str), "--");
    primary = esp_rmaker_param_create("Temperature (\xc2\xb0""C)", NULL,
                                      esp_rmaker_str(temp_line_str), PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(primary, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, primary);
    esp_rmaker_device_assign_primary_param(espresso_device, primary);

    /* Power toggle */
    poweron_param = esp_rmaker_param_create("Power", NULL,
                                            esp_rmaker_bool(powerOn),
                                            PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(poweron_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, poweron_param);

    /* Brew trigger */
    esp_rmaker_param_t *p;
    p = esp_rmaker_param_create("Brew Signal", NULL,
                                esp_rmaker_bool(false),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, p);

    /* Flush trigger */
    p = esp_rmaker_param_create("Flush", NULL,
                                esp_rmaker_bool(false),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, p);

    /* Temp Lock toggle */
    p = esp_rmaker_param_create("Temp Lock", NULL,
                                esp_rmaker_bool(tempLock),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, p);

    /* Temperature setpoint slider */
    p = esp_rmaker_param_create("Temperature setpoint (\xc2\xb0""C)", NULL,
                                esp_rmaker_int(tempSetpoint),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(p, esp_rmaker_int(TEMP_SETPOINT_MIN),
                                esp_rmaker_int(TEMP_SETPOINT_MAX), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, p);

    /* Brew time slider */
    p = esp_rmaker_param_create("Brew time (s)", NULL,
                                esp_rmaker_int(brewTime),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(p, esp_rmaker_int(6), esp_rmaker_int(20),
                                esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, p);

    /* Pre-Infusion toggle */
    p = esp_rmaker_param_create("Pre-Infusion", NULL,
                                esp_rmaker_bool(preInfusion),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, p);

    /* Pre-Infusion on time slider */
    p = esp_rmaker_param_create("Pre-Infusion on time (s)", NULL,
                                esp_rmaker_int(preInfOnTime),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(p, esp_rmaker_int(2), esp_rmaker_int(10),
                                esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, p);

    /* Pre-Infusion off time slider */
    p = esp_rmaker_param_create("Pre-Infusion off time (s)", NULL,
                                esp_rmaker_int(preInfOffTime),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(p, esp_rmaker_int(2), esp_rmaker_int(30),
                                esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, p);

    /* Flush time slider */
    p = esp_rmaker_param_create("Flush time (s)", NULL,
                                esp_rmaker_int(flushTime),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(p, esp_rmaker_int(3), esp_rmaker_int(10),
                                esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, p);

#if ADAPTIVE_WARMUP_ENABLE
    /* Approach dither display */
    static char dith_disp_str[20];
    snprintf(dith_disp_str, sizeof(dith_disp_str), "%d%%", dither_step_pct());
    overshoot_disp_param = esp_rmaker_param_create("Power factor (dither)", NULL,
                                                   esp_rmaker_str(dith_disp_str),
                                                   PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(overshoot_disp_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, overshoot_disp_param);

    /* Reset dither trigger */
    p = esp_rmaker_param_create("Reset power factor", NULL,
                                esp_rmaker_bool(false),
                                PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(p, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, p);
#endif

    /* Status text */
    snprintf(status_str, sizeof(status_str), "Okay");
    status_param = esp_rmaker_param_create("Status", NULL,
                                           esp_rmaker_str(status_str),
                                           PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(status_param, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, status_param);
}
