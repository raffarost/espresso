#include "driver/spi_master.h"

#include "spi_mod.h"
#include <esp_log.h>

/* SPI Config */
spi_device_handle_t spi_init(void) {
  spi_device_handle_t spi;
  esp_err_t ret;
  spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = (4 * 8)
  };

  //Initialize the SPI bus
  ret = spi_bus_initialize(VSPI_HOST, &buscfg, DMA_CHAN);
  ESP_ERROR_CHECK(ret);

  if (ret == ESP_OK)
  {
    ESP_LOGI("SPI", "SPI initilized successfully");
  }
  else
  {
    ESP_LOGE("SPI", "Failed initializing SPI");
  }

  spi_device_interface_config_t devCfg={
    .mode = 0,
    .clock_speed_hz = 2*1000*1000,
    .spics_io_num=25,
    .queue_size=3
  };

  ret = spi_bus_add_device(VSPI_HOST, &devCfg, &spi);
  ESP_ERROR_CHECK(ret);
  
  if (ret == ESP_OK)
  {
    ESP_LOGI("SPI", "SPI device added successfully");
  }
  else
  {
    ESP_LOGE("SPI", "Failed adding SPI device");
  }
  
  return spi;
}
