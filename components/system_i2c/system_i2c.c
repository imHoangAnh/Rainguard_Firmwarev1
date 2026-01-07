/**
 * @file system_i2c.c
 * @brief Thread-safe I2C master bus implementation
 */

#include "system_i2c.h"
#include "pin_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stddef.h>

static const char *TAG = "system_i2c";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static SemaphoreHandle_t i2c_mutex = NULL;

bool system_i2c_init(void) {
  if (i2c_bus_handle != NULL) {
    ESP_LOGW(TAG, "I2C bus already initialized");
    return true;
  }

  // Create mutex for thread safety
  i2c_mutex = xSemaphoreCreateMutex();
  if (i2c_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C mutex");
    return false;
  }

  // I2C bus configuration
  i2c_master_bus_config_t i2c_bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags =
          {
              .enable_internal_pullup = true,
          },
  };

  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
    vSemaphoreDelete(i2c_mutex);
    i2c_mutex = NULL;
    return false;
  }

  ESP_LOGI(TAG, "I2C bus initialized successfully (SDA=%d, SCL=%d, Freq=%d Hz)",
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  return true;
}

i2c_master_bus_handle_t system_i2c_get_bus_handle(void) {
  return i2c_bus_handle;
}

void system_i2c_deinit(void) {
  if (i2c_bus_handle != NULL) {
    esp_err_t ret = i2c_del_master_bus(i2c_bus_handle);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
    }
    i2c_bus_handle = NULL;
  }

  if (i2c_mutex != NULL) {
    vSemaphoreDelete(i2c_mutex);
    i2c_mutex = NULL;
  }

  ESP_LOGI(TAG, "I2C bus deinitialized");
}
