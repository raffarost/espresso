/*
   Espresso Machine PID/P Controller

   This project implements a PID/P controller to effectively control the
   temperature of an Espresso machine boiler, for a stable setpoint.
   Another feature implemented is the control of the water pump and
   pre-infusion settings. Combined, these functionalities allow for
   improved espresso extraction and consistency.

   https://github.com/raffarost/espresso
   March 2024

   Raffael Rostagno
   raffael.rostagno@gmail.com
*/

#include <string.h>
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
#define GPIO_OUTPUT_PIN_SEL  (1ULL << GPIO_OUTPUT_IO_0)

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

#define BKP_NUM       10

static float   deltaBkp[BKP_NUM] = {-10,  0, 0.5,  1,  2,  4, 10,  25,  50, 100};  /* temperature delta */
static float controlSet[BKP_NUM] = {  0,  0,   1,  1,  1,  1,  1,  80, 100, 100};  /* power setpoint in percentage */

#define SEC_TO_US(x)        (x * 1000000)

#define BREW_OFF            0
#define BREW_PREINF_ON      1
#define BREW_PREINF_OFF     2
#define BREW_ON             3
#define BREW_POWER_OFF      4

#define TEMP_DELTA          2   /* temp delta from setpoint for good brewing temperature */
#define MAX_TEMP_THR      110   /* maximum temperature threshold for safety control */

/* divisor factors for power reduction around temperature target */
#define POWER_50            2
#define POWER_33            3
#define POWER_25            4

#define POWER_FACTOR        POWER_33
#define TEMP_PWR_TOGGLE    10   /* temperature range around setpoint in which power is reduced to
                                  a factor (above) of calibrated values for better stability */

#define POWERON_MIN         (15 * SEC_TO_US(60))    /* 15 minutes before entering standby (power off) mode */

/*****************************************************************************
 * Module variables
 *****************************************************************************/

static const char *TAG = "espresso";
esp_rmaker_device_t *espresso_device;

/* these are used to report events/data to UI */
esp_rmaker_param_t *primary;
esp_rmaker_param_t *tempOk_param;

/* Heating element control */
dimmertyp *ptr_dimmer;

/* Task control */
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
static int tempSetpoint = 96;
static float pidOut;
static float delta;
static int control;
static int powerToggle = 0;
static unsigned long long pumpTimer = 0;
static unsigned long long powerOnTimer = 0;
static bool brewSignal = false;
static int brewState = BREW_OFF;
static int brewTime = 6;
static bool tempRangeOk = false;
static bool tempLock = true;
static bool powerOn = true;
static bool preInfusion = true;
static int preInfOnTime = 3;
static int preInfOffTime = 10;

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
    if (strcmp(esp_rmaker_param_get_name(param), "Temperature Setpoint") == 0)
    {
        tempSetpoint = val.val.i;
        ESP_LOGI(TAG, "New temperature setpoint: %d", tempSetpoint);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Brew Time") == 0)
    {
        brewTime = val.val.i;
        ESP_LOGI(TAG, "Brew Time: %d s", brewTime);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Brew Signal") == 0)
    {
        brewSignal = val.val.b ? !brewSignal : brewSignal;
        ESP_LOGI(TAG, "Brew %s!", brewSignal ? "start" : "stop");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Pre-Infusion") == 0)
    {
        preInfusion = val.val.b;
        ESP_LOGI(TAG, "Pre-Infusion set to %s", preInfusion ? "ON" : "OFF");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Power") == 0)
    {
        powerOn = val.val.b;
        ESP_LOGI(TAG, "Power is set to %s", powerOn ? "ON" : "OFF");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Temp Lock") == 0)
    {
        tempLock = val.val.b;
        ESP_LOGI(TAG, "Temp Lock set to %s", tempLock ? "ON" : "OFF");
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Pre-Infusion On Time") == 0)
    {
        preInfOnTime = val.val.i;
        ESP_LOGI(TAG, "Pre-Infusion On Time: %d s", preInfOnTime);
    }

    if (strcmp(esp_rmaker_param_get_name(param), "Pre-Infusion Off Time") == 0)
    {
        preInfOffTime = val.val.i;
        ESP_LOGI(TAG, "Pre-Infusion Off Time: %d s", preInfOffTime);
    }

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
    /* configure GPIO output pins */
    gpio_config(&gpio_outcfg); 
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
        /* Read */
        nvs_get_u8(nvs_handle, "tempLock", &tempLock);
        nvs_get_i32(nvs_handle, "tempSetpoint", &tempSetpoint);
        nvs_get_i32(nvs_handle, "brewTime", &brewTime);
        nvs_get_u8(nvs_handle, "preInfusion", &preInfusion);
        nvs_get_i32(nvs_handle, "preInfOnTime", &preInfOnTime);
        nvs_get_i32(nvs_handle, "preInfOffTime", &preInfOffTime);

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
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "tempLock", tempLock));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "tempSetpoint", tempSetpoint));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "brewTime", brewTime));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "preInfusion", preInfusion));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "preInfOnTime", preInfOnTime));
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "preInfOffTime", preInfOffTime));

        ESP_LOGI(TAG, "Committing updates in NVS ... ");
        
        ESP_ERROR_CHECK(nvs_commit(nvs_handle));

        nvs_close(nvs_handle);
    }    
}

unsigned long long getAbsTime1us(void)
{
    unsigned long long timerVal = 0;

    gptimer_get_raw_count(freeRunTimer, &timerVal);

    return timerVal;
}

void spiComm(void)
{
    esp_err_t ret;

    ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    CHECK_RET(ret, "Error opening SPI");
    ret = spi_device_transmit(spi, &tM);
    CHECK_RET(ret, "Error TX SPI");
    spi_device_release_bus(spi);

    int16_t res = (int16_t) SPI_SWAP_DATA_RX(data, 16);

    if (res & (1 << 2)) {
      res = 0;
      ESP_LOGE(TAG, "Sensor is not connected");
    }
    else {
      res >>= 3;
      tempCelsius = (float)(res * 0.25);
    }
}

void app_main()
{
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

    /* Adding temperature display to device */
    esp_rmaker_device_add_param(espresso_device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Espresso"));

    primary = esp_rmaker_param_create("Temperature", ESP_RMAKER_PARAM_TEMPERATURE, esp_rmaker_float(0), PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(primary, ESP_RMAKER_UI_TEXT);
    esp_rmaker_device_add_param(espresso_device, primary);
    esp_rmaker_device_assign_primary_param(espresso_device, primary);

    /* Creating temperature setpoint OK */
    tempOk_param = esp_rmaker_param_create("Temperature OK", NULL, esp_rmaker_bool(false), PROP_FLAG_READ);
    esp_rmaker_param_add_ui_type(tempOk_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, tempOk_param);  
  
    /* Creating Power Up toggle switch */
    poweron_param = esp_rmaker_param_create("Power", NULL, esp_rmaker_bool(powerOn), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(poweron_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, poweron_param);

    /* Creating brew button */
    esp_rmaker_param_t *brewSig_param = esp_rmaker_param_create("Brew Signal", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(brewSig_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(espresso_device, brewSig_param);
  
    /* Creating Temperature Lock toggle switch */
    esp_rmaker_param_t *templock_param = esp_rmaker_param_create("Temp Lock", NULL, esp_rmaker_bool(tempLock), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(templock_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, templock_param);

    /* Creating slider object */
    esp_rmaker_param_t *temp_param = esp_rmaker_param_create("Temperature Setpoint", NULL, esp_rmaker_int(tempSetpoint), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(temp_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(temp_param, esp_rmaker_int(90), esp_rmaker_int(110), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, temp_param);

    /* Creating slider object */
    esp_rmaker_param_t *brewtime_param = esp_rmaker_param_create("Brew Time", NULL, esp_rmaker_int(brewTime), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(brewtime_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(brewtime_param, esp_rmaker_int(6), esp_rmaker_int(20), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, brewtime_param);
  
    /* Creating Pre-Infusion toggle switch */
    esp_rmaker_param_t *preinf_param = esp_rmaker_param_create("Pre-Infusion", NULL, esp_rmaker_bool(preInfusion), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(preinf_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(espresso_device, preinf_param);

    /* Pre-Infusion On Time slider object */
    esp_rmaker_param_t *preinfOn_param = esp_rmaker_param_create("Pre-Infusion On Time", NULL, esp_rmaker_int(preInfOnTime), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(preinfOn_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(preinfOn_param, esp_rmaker_int(2), esp_rmaker_int(10), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, preinfOn_param);

    /* Pre-Infusion Off Time slider object */
    esp_rmaker_param_t *preinfOff_param = esp_rmaker_param_create("Pre-Infusion Off Time", NULL, esp_rmaker_int(preInfOffTime), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(preinfOff_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(preinfOff_param, esp_rmaker_int(2), esp_rmaker_int(30), esp_rmaker_int(1));
    esp_rmaker_device_add_param(espresso_device, preinfOff_param);

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

    err = app_wifi_set_custom_mfg_data(MGF_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    
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

    gpioConfig();
    
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
        control = (int)interp1D(controlSet, BKP_NUM, ir);
        
        if ((delta > 0) && (delta <= TEMP_PWR_TOGGLE))
        {
            /* reduce power to a factor by toggling power each task */
            powerToggle = (int)((count500ms % POWER_FACTOR) == 0);
            control = (control * powerToggle);
        }

    #elif (CONTROL_TYPE == PID_LOOKUP)

        pidOut = pidUpdate((float)tempSetpoint, tempCelsius);

        float ir = indexRatio(deltaBkp, BKP_NUM, pidOut);
        control = (int)interp1D(controlSet, BKP_NUM, ir);

    #endif

    if ((tempCelsius == 0) || (tempCelsius > MAX_TEMP_THR))
    {
        /* switch off heating element when temperature sensor is disconnected or in overheat condition */
        control = 0;
        ESP_LOGE(TAG, "Recovery mode, overheat detected or temperature sensor failed!");
    }
    else if (brewState == BREW_POWER_OFF)
    {
        /* switch off heating element and remain in standby */
        control = 0;
    }
    else
    {
        /*
         * Set full power while water pump is ON for to keep temperature stable.
         * To better buffer temperature after starting pre-infusion, we'll leave heating element ON for
         * half the time of preinfusion ON at the start of pre-infusion OFF.
         */

        #define TIME_TEMP_BUFF  (unsigned long long)SEC_TO_US((float)preInfOnTime / 2)

        control = ((brewState == BREW_PREINF_ON) || (brewState == BREW_ON) ||
                    ((brewState == BREW_PREINF_OFF) && ((getAbsTime1us() - pumpTimer) < TIME_TEMP_BUFF))) ? (int)100 : control;
    }

    setPower(ptr_dimmer, (int)control);

    ESP_LOGI(TAG, "%d | %.2f | %.2f | %d | %d", tempSetpoint, tempCelsius, pidOut, control, getPower(ptr_dimmer));
}

void Task5000ms(void)
{
    if (powerOn)
    {
        /* only report data to app when device is on, to save MQTT budget */
        esp_rmaker_param_update_and_report(primary, esp_rmaker_float(tempCelsius));
        esp_rmaker_param_update_and_report(tempOk_param, esp_rmaker_bool(tempRangeOk));
    }
}

void brewProgram(void)
{
    /*
     * This signal (enabled or disabled by the Temp Lock switch) will cancel a
     * brewing attempt if temperature is not within an acceptable range (calibrated
     * by TEMP_DELTA).
     */
    if ((tempCelsius < (tempSetpoint - TEMP_DELTA)) || (tempCelsius > (tempSetpoint + TEMP_DELTA)))
    {
        tempRangeOk = false;
    }
    else
    {
        tempRangeOk = true;
    }

    /* A brewing operation is stopped if user presses Brew Signal again */
    if ((brewSignal == false) && (brewState != BREW_POWER_OFF))
    {
        /* abort brew at any stage */
        gpio_set_level(GPIO_OUTPUT_IO_0, 0);
        brewState = BREW_OFF;
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
                    
                    /* reset power on timer */
                    powerOnTimer = getAbsTime1us();
                }
                else
                {
                    /* abort brew attempt and reset button */
                    ESP_LOGI(TAG, "Brew aborted! Setpoint temperature not reached");
                    brewSignal = false;
                }
            }
            else
            {
                /* turn power off if Brew Signal is not pressed after POWERON_MIN (default 15 min) */
                if ((powerOn == false) || ((getAbsTime1us() - powerOnTimer) > POWERON_MIN))
                {
                    /* switch off heating element and remain in standby */
                    brewState = BREW_POWER_OFF;

                    powerOn = false;
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
                brewState = BREW_OFF;

                ESP_LOGI(TAG, "Brew cycle completed!");
                brewSignal = false;     /* reset for next press */
                
                /* Save brewing parameters to NVS */
                nvsWrite();
            } 

        break;

        case BREW_POWER_OFF:

            /* Standby mode. Leaving here only if Power switch is toggled */
            if (powerOn == true)
            {
                powerOnTimer = getAbsTime1us();
                brewState = BREW_OFF;

                ESP_LOGI(TAG, "Switching power ON!");
            }

        break;

        default:
        break;
    }
}
