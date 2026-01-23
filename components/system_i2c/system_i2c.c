/**
 * @file system_i2c.c
 * @brief Thread-safe I2C master bus implementation
 * @details Optimized for ESP-IDF with modern i2c_master API
 *
 * Improvements:
 * - Proper mutex handling for thread safety
 * - Device add/remove helpers for sensors
 * - Lock/unlock API for transaction batching
 * - Better error reporting and diagnostics
 */

#include "system_i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pin_config.h"
#include <stdbool.h>
#include <stddef.h>

static const char *TAG = "system_i2c";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static SemaphoreHandle_t i2c_mutex = NULL;

#define DEFAULT_MUTEX_TIMEOUT_MS 1000

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
  // Note: Internal pullups are weak (~45kΩ), external 4.7kΩ pullups recommended
  i2c_master_bus_config_t i2c_bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
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

uint8_t system_i2c_scan(void) {
  if (i2c_bus_handle == NULL) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return 0;
  }

  // Common sensor I2C addresses to scan
  // BME680/BME280/BMP280: 0x76, 0x77
  // MPU6050/MPU9250: 0x68, 0x69
  // OLED SSD1306: 0x3C, 0x3D
  // AHT10/AHT20: 0x38
  // BH1750: 0x23, 0x5C
  // INA219: 0x40-0x4F
  static const uint8_t common_addresses[] = {
      0x23, // BH1750 (ADDR LOW)
      0x38, // AHT10/AHT20
      0x3C, // OLED SSD1306
      0x3D, // OLED SSD1306 (alt)
      0x40, // INA219
      0x44, // SHT31
      0x45, // SHT31 (alt)
      0x48, // ADS1115
      0x5C, // BH1750 (ADDR HIGH)
      0x68, // MPU6050/MPU9250/DS3231
      0x69, // MPU6050 (AD0 HIGH)
      0x76, // BME680/BME280/BMP280 (SDO LOW)
      0x77, // BME680/BME280/BMP280 (SDO HIGH)
  };

  static const char *device_names[] = {
      "BH1750",
      "AHT10/AHT20",
      "SSD1306 OLED",
      "SSD1306 OLED",
      "INA219",
      "SHT31",
      "SHT31",
      "ADS1115",
      "BH1750",
      "MPU6050/MPU9250/DS3231",
      "MPU6050 (AD0=HIGH)",
      "BME680/BME280/BMP280",
      "BME680/BME280/BMP280",
  };

  ESP_LOGI(TAG, "========== I2C SCAN START ==========");
  ESP_LOGI(TAG, "Scanning %d common sensor addresses (SDA=%d, SCL=%d)...",
           (int)sizeof(common_addresses), I2C_SDA_PIN, I2C_SCL_PIN);

  uint8_t devices_found = 0;

  // Temporarily reduce log level to suppress timeout errors during scan
  esp_log_level_t old_level = esp_log_level_get("i2c.master");
  esp_log_level_set("i2c.master", ESP_LOG_WARN);

  for (size_t i = 0; i < sizeof(common_addresses); i++) {
    uint8_t addr = common_addresses[i];

    // Use i2c_master_probe to check if device responds
    // Use longer timeout (100ms) for more reliable detection
    esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, pdMS_TO_TICKS(100));

    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "  [FOUND] 0x%02X - %s", addr, device_names[i]);
      devices_found++;
    }
  }

  // Restore log level
  esp_log_level_set("i2c.master", old_level);

  ESP_LOGI(TAG, "========== I2C SCAN COMPLETE ==========");
  ESP_LOGI(TAG, "Total devices found: %d", devices_found);

  if (devices_found == 0) {
    ESP_LOGW(TAG, "No I2C devices found! Check:");
    ESP_LOGW(TAG, "  - SDA/SCL wiring (SDA=%d, SCL=%d)", I2C_SDA_PIN,
             I2C_SCL_PIN);
  }

  return devices_found;
}

esp_err_t system_i2c_lock(uint32_t timeout_ms) {
  if (i2c_mutex == NULL) {
    ESP_LOGE(TAG, "I2C mutex not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t timeout = (timeout_ms == 0) ? DEFAULT_MUTEX_TIMEOUT_MS : timeout_ms;

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(timeout)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire I2C mutex within %lu ms",
             (unsigned long)timeout);
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

esp_err_t system_i2c_unlock(void) {
  if (i2c_mutex == NULL) {
    ESP_LOGE(TAG, "I2C mutex not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreGive(i2c_mutex) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to release I2C mutex");
    return ESP_FAIL;
  }

  return ESP_OK;
}

bool system_i2c_is_initialized(void) {
  return (i2c_bus_handle != NULL && i2c_mutex != NULL);
}

esp_err_t system_i2c_add_device(uint8_t dev_addr, uint32_t scl_speed_hz,
                                i2c_master_dev_handle_t *dev_handle) {
  if (i2c_bus_handle == NULL) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (dev_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = dev_addr,
      .scl_speed_hz = scl_speed_hz,
  };

  esp_err_t ret =
      i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", dev_addr,
             esp_err_to_name(ret));
  } else {
    ESP_LOGD(TAG, "Added device 0x%02X at %lu Hz", dev_addr,
             (unsigned long)scl_speed_hz);
  }

  return ret;
}

esp_err_t system_i2c_remove_device(i2c_master_dev_handle_t dev_handle) {
  if (dev_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
  }

  return ret;
}
