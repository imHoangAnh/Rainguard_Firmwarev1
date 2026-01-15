/**
 * @file sensor_bme680.c
 * @brief BME680 environmental sensor driver using official Bosch BME68x API
 * @details Wrapper driver utilizing Bosch Sensortec BME68x sensor API
 *          All compensation and calculations are done by the official Bosch
 * driver
 */

#include "sensor_bme680.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "system_i2c.h"
#include <string.h>

static const char *TAG = "sensor_bme680";

// I2C device handle
static i2c_master_dev_handle_t bme680_i2c_handle = NULL;

// Bosch BME68x device structure
static struct bme68x_dev bme68x_sensor;
static bool sensor_initialized = false;

// Sensor configuration cache
static struct {
  bme680_oversampling_t temp_os;
  bme680_oversampling_t press_os;
  bme680_oversampling_t hum_os;
  uint8_t filter;
  uint16_t gas_wait_ms;
  uint16_t gas_heater_temp;
  bool gas_enabled;
} sensor_config = {.temp_os = BME680_OS_2X,
                   .press_os = BME680_OS_16X,
                   .hum_os = BME680_OS_1X,
                   .filter = BME68X_FILTER_SIZE_3,
                   .gas_wait_ms = 100,
                   .gas_heater_temp = 320,
                   .gas_enabled = true};

// IAQ calculation state (for accuracy tracking)
static struct {
  float gas_baseline;
  uint32_t gas_baseline_count;
  bool gas_baseline_valid;
  uint32_t stabilization_count;
} iaq_state = {0};

/*============================================================================
 * I2C Communication Callbacks for Bosch BME68x API
 *============================================================================*/

/**
 * @brief I2C read callback for Bosch BME68x API
 */
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                            uint32_t len, void *intf_ptr) {
  (void)intf_ptr;

  if (bme680_i2c_handle == NULL) {
    return BME68X_E_COM_FAIL;
  }

  esp_err_t ret = i2c_master_transmit_receive(
      bme680_i2c_handle, &reg_addr, 1, reg_data, len, pdMS_TO_TICKS(100));

  return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

/**
 * @brief I2C write callback for Bosch BME68x API
 */
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len, void *intf_ptr) {
  (void)intf_ptr;

  if (bme680_i2c_handle == NULL) {
    return BME68X_E_COM_FAIL;
  }

  // Combine register address and data into one buffer
  uint8_t *buf = malloc(len + 1);
  if (buf == NULL) {
    return BME68X_E_COM_FAIL;
  }

  buf[0] = reg_addr;
  memcpy(buf + 1, reg_data, len);

  esp_err_t ret =
      i2c_master_transmit(bme680_i2c_handle, buf, len + 1, pdMS_TO_TICKS(100));
  free(buf);

  return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

/**
 * @brief Delay callback for Bosch BME68x API
 */
static void bme68x_delay_us(uint32_t period, void *intf_ptr) {
  (void)intf_ptr;

  if (period >= 1000) {
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
  } else {
    esp_rom_delay_us(period);
  }
}

/*============================================================================
 * IAQ Calculation
 *============================================================================*/

/**
 * @brief Calculate IAQ using Bosch-inspired algorithm
 * @param temp Temperature in Celsius
 * @param hum Humidity in %
 * @param gas_res Gas resistance in Ohms
 * @param accuracy Output parameter for IAQ accuracy
 * @return IAQ value (0-500)
 */
static float calculate_iaq(float temp, float hum, float gas_res,
                           uint8_t *accuracy) {
  // Establish gas baseline (first 10 minutes of operation)
  if (!iaq_state.gas_baseline_valid) {
    if (iaq_state.gas_baseline_count < 600) { // ~10 minutes at 1Hz
      iaq_state.gas_baseline += gas_res;
      iaq_state.gas_baseline_count++;
      *accuracy = 0; // Stabilizing
      return 25.0f;  // Default value during stabilization
    } else {
      iaq_state.gas_baseline /= iaq_state.gas_baseline_count;
      iaq_state.gas_baseline_valid = true;
      *accuracy = 1; // Low accuracy
    }
  }

  // Update baseline with exponential moving average (EMA)
  if (gas_res > 0) {
    iaq_state.gas_baseline =
        (iaq_state.gas_baseline * 0.95f) + (gas_res * 0.05f);
  }

  // Calculate humidity contribution
  float hum_score = 0.0f;
  if (hum >= 38.0f && hum <= 42.0f) {
    hum_score = 0.25f * (hum - 38.0f); // Optimal range
  } else if (hum < 38.0f) {
    hum_score = 0.25f + 0.25f * ((38.0f - hum) / 38.0f);
  } else {
    hum_score = 0.5f + 0.5f * ((hum - 42.0f) / 58.0f);
  }

  // Calculate gas contribution (inverse relationship - lower resistance = worse
  // air)
  float gas_score = 0.0f;
  if (iaq_state.gas_baseline > 0.0f && gas_res > 0.0f) {
    float gas_ratio = gas_res / iaq_state.gas_baseline;
    if (gas_ratio > 1.0f) {
      gas_score = 0.0f; // Better than baseline
    } else {
      gas_score = (1.0f - gas_ratio) * 100.0f; // Worse than baseline
    }
  }

  // Calculate temperature contribution
  float temp_score = 0.0f;
  if (temp >= 20.0f && temp <= 25.0f) {
    temp_score = 0.0f; // Optimal
  } else if (temp < 20.0f) {
    temp_score = ((20.0f - temp) / 20.0f) * 50.0f;
  } else {
    temp_score = ((temp - 25.0f) / 15.0f) * 50.0f;
  }

  // Combine scores (weighted)
  float iaq =
      (gas_score * 0.5f) + (hum_score * 100.0f * 0.25f) + (temp_score * 0.25f);

  // Clamp to 0-500 range
  if (iaq > 500.0f)
    iaq = 500.0f;
  if (iaq < 0.0f)
    iaq = 0.0f;

  // Update accuracy based on stabilization time
  if (iaq_state.stabilization_count < 3600) { // 1 hour at 1Hz
    iaq_state.stabilization_count++;
    if (*accuracy < 3) {
      *accuracy = (iaq_state.stabilization_count > 1800) ? 3 : 2;
    }
  } else {
    *accuracy = 3; // High accuracy
  }

  return iaq;
}

/*============================================================================
 * Public API Implementation
 *============================================================================*/

bool sensor_bme680_init(void) {
  if (sensor_initialized) {
    ESP_LOGW(TAG, "BME680 already initialized");
    return true;
  }

  // Get I2C bus handle
  i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return false;
  }

  // Try both I2C addresses (0x76 and 0x77)
  uint8_t addresses[] = {BME680_I2C_ADDR,
                         BME680_I2C_ADDR == 0x77 ? 0x76 : 0x77};
  bool device_found = false;
  uint8_t actual_address = 0;

  ESP_LOGI(TAG, "Searching for BME680 sensor...");

  for (int addr_idx = 0; addr_idx < 2; addr_idx++) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addresses[addr_idx],
        .scl_speed_hz = 100000,
    };

    esp_err_t ret =
        i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bme680_i2c_handle);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to add device at 0x%02X: %s", addresses[addr_idx],
               esp_err_to_name(ret));
      continue;
    }

    // Test communication by reading chip ID
    uint8_t chip_id = 0;
    uint8_t reg = BME68X_REG_CHIP_ID;
    for (int retry = 0; retry < 3; retry++) {
      vTaskDelay(pdMS_TO_TICKS(10));
      ret = i2c_master_transmit_receive(bme680_i2c_handle, &reg, 1, &chip_id, 1,
                                        pdMS_TO_TICKS(100));
      if (ret == ESP_OK && chip_id == BME68X_CHIP_ID) {
        device_found = true;
        actual_address = addresses[addr_idx];
        ESP_LOGI(TAG, "BME680 found at I2C address 0x%02X (chip_id=0x%02X)",
                 actual_address, chip_id);
        break;
      }
    }

    if (device_found)
      break;

    if (chip_id != 0 && chip_id != 0xFF) {
      ESP_LOGW(TAG, "Device at 0x%02X has chip ID 0x%02X (expected 0x61)",
               addresses[addr_idx], chip_id);
    }

    i2c_master_bus_rm_device(bme680_i2c_handle);
    bme680_i2c_handle = NULL;
  }

  if (!device_found) {
    ESP_LOGE(TAG, "BME680 not found. Check:\n"
                  "  - I2C wiring (SDA, SCL)\n"
                  "  - Pull-up resistors (4.7k-10k ohm)\n"
                  "  - Power supply (VCC=3.3V)\n"
                  "  - SDO pin (GND=0x76, VCC=0x77)");
    return false;
  }

  // Initialize Bosch BME68x API
  memset(&bme68x_sensor, 0, sizeof(bme68x_sensor));
  bme68x_sensor.intf = BME68X_I2C_INTF;
  bme68x_sensor.intf_ptr = &actual_address;
  bme68x_sensor.read = bme68x_i2c_read;
  bme68x_sensor.write = bme68x_i2c_write;
  bme68x_sensor.delay_us = bme68x_delay_us;
  bme68x_sensor.amb_temp = 25; // Default ambient temperature

  int8_t rslt = bme68x_init(&bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Bosch BME68x init failed: %d", rslt);
    i2c_master_bus_rm_device(bme680_i2c_handle);
    bme680_i2c_handle = NULL;
    return false;
  }

  ESP_LOGI(TAG, "Bosch BME68x API initialized (variant_id=0x%02X)",
           (unsigned int)bme68x_sensor.variant_id);

  // Configure default settings
  sensor_bme680_configure(sensor_config.temp_os, sensor_config.press_os,
                          sensor_config.hum_os, sensor_config.filter,
                          sensor_config.gas_wait_ms,
                          sensor_config.gas_heater_temp);

  sensor_initialized = true;
  ESP_LOGI(TAG, "BME680 initialized successfully using official Bosch driver");

  return true;
}

bool sensor_bme680_configure(bme680_oversampling_t temp_os,
                             bme680_oversampling_t press_os,
                             bme680_oversampling_t hum_os, uint8_t filter,
                             uint16_t gas_wait_ms, uint16_t gas_heater_temp) {
  if (bme680_i2c_handle == NULL) {
    return false;
  }

  // Store configuration
  sensor_config.temp_os = temp_os;
  sensor_config.press_os = press_os;
  sensor_config.hum_os = hum_os;
  sensor_config.filter = filter & 0x07;
  sensor_config.gas_wait_ms = gas_wait_ms;
  sensor_config.gas_heater_temp = (gas_heater_temp < 200)   ? 200
                                  : (gas_heater_temp > 400) ? 400
                                                            : gas_heater_temp;
  sensor_config.gas_enabled = (gas_wait_ms > 0);

  // Configure sensor using Bosch API
  struct bme68x_conf conf;
  int8_t rslt = bme68x_get_conf(&conf, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to get config: %d", rslt);
    return false;
  }

  conf.os_temp = temp_os;
  conf.os_pres = press_os;
  conf.os_hum = hum_os;
  conf.filter = filter;
  conf.odr = BME68X_ODR_NONE;

  rslt = bme68x_set_conf(&conf, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to set config: %d", rslt);
    return false;
  }

  // Configure gas heater
  if (sensor_config.gas_enabled) {
    struct bme68x_heatr_conf heatr_conf;
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = sensor_config.gas_heater_temp;
    heatr_conf.heatr_dur = sensor_config.gas_wait_ms;

    rslt =
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme68x_sensor);
    if (rslt != BME68X_OK) {
      ESP_LOGE(TAG, "Failed to set heater config: %d", rslt);
      return false;
    }
  }

  ESP_LOGI(TAG,
           "Configured: T_os=%d, P_os=%d, H_os=%d, Filter=%d, Gas=%dms@%d°C",
           temp_os, press_os, hum_os, filter, gas_wait_ms, gas_heater_temp);

  return true;
}

bool sensor_bme680_set_power_mode(bme680_power_mode_t mode) {
  if (bme680_i2c_handle == NULL) {
    return false;
  }

  int8_t rslt = bme68x_set_op_mode((uint8_t)mode, &bme68x_sensor);
  return (rslt == BME68X_OK);
}

bool sensor_bme680_read(bme680_data_t *data) {
  if (bme680_i2c_handle == NULL || data == NULL || !sensor_initialized) {
    return false;
  }

  // Set forced mode to trigger measurement
  int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to set forced mode: %d", rslt);
    return false;
  }

  // Calculate measurement duration and wait
  struct bme68x_conf conf;
  bme68x_get_conf(&conf, &bme68x_sensor);
  uint32_t meas_dur =
      bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme68x_sensor);

  // Add heater duration if gas is enabled
  if (sensor_config.gas_enabled) {
    meas_dur += sensor_config.gas_wait_ms * 1000; // Convert to microseconds
  }

  // Wait for measurement to complete (with small margin)
  bme68x_sensor.delay_us(meas_dur + 1000, bme68x_sensor.intf_ptr);

  // Read data using Bosch API
  struct bme68x_data sensor_data[3];
  uint8_t n_data = 0;

  rslt =
      bme68x_get_data(BME68X_FORCED_MODE, sensor_data, &n_data, &bme68x_sensor);
  if (rslt != BME68X_OK || n_data == 0) {
    ESP_LOGE(TAG, "Failed to get data: rslt=%d, n_data=%d", rslt, n_data);
    return false;
  }

  // Use the first (and only in forced mode) data sample
  struct bme68x_data *d = &sensor_data[0];

  // Copy data to output structure
#ifdef BME68X_USE_FPU
  data->temperature = d->temperature;
  data->humidity = d->humidity;
  data->pressure = d->pressure / 100.0f; // Convert Pa to hPa
  data->gas_resistance = d->gas_resistance;
#else
  data->temperature = d->temperature / 100.0f;
  data->humidity = d->humidity / 1000.0f;
  data->pressure = d->pressure / 100.0f; // Convert Pa to hPa
  data->gas_resistance = (float)d->gas_resistance;
#endif

  // Check validity flags
  data->gas_valid = (d->status & BME68X_GASM_VALID_MSK) != 0;
  data->heat_stable = (d->status & BME68X_HEAT_STAB_MSK) != 0;

  // Calculate IAQ
  data->iaq = calculate_iaq(data->temperature, data->humidity,
                            data->gas_resistance, &data->iaq_accuracy);

  ESP_LOGD(
      TAG,
      "Read: T=%.2f°C, H=%.2f%%, P=%.2fhPa, Gas=%.0fΩ (valid=%d, stable=%d)",
      data->temperature, data->humidity, data->pressure, data->gas_resistance,
      data->gas_valid, data->heat_stable);

  return true;
}

bool sensor_bme680_self_test(void) {
  if (bme680_i2c_handle == NULL || !sensor_initialized) {
    ESP_LOGE(TAG, "Sensor not initialized");
    return false;
  }

  // Use Bosch's built-in self-test
  int8_t rslt = bme68x_selftest_check(&bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Bosch self-test failed: %d", rslt);

    // Fall back to manual verification
    bme680_data_t test_data;
    if (!sensor_bme680_read(&test_data)) {
      ESP_LOGE(TAG, "Manual self-test failed: measurement error");
      return false;
    }

    // Validate data ranges
    if (test_data.temperature < -40.0f || test_data.temperature > 85.0f) {
      ESP_LOGW(TAG, "Self-test warning: temperature out of range (%.2f°C)",
               test_data.temperature);
      return false;
    }
    if (test_data.humidity < 0.0f || test_data.humidity > 100.0f) {
      ESP_LOGW(TAG, "Self-test warning: humidity out of range (%.2f%%)",
               test_data.humidity);
      return false;
    }
    if (test_data.pressure < 300.0f || test_data.pressure > 1100.0f) {
      ESP_LOGW(TAG, "Self-test warning: pressure out of range (%.2f hPa)",
               test_data.pressure);
      return false;
    }

    ESP_LOGI(TAG, "Manual self-test passed: T=%.2f°C, H=%.2f%%, P=%.2f hPa",
             test_data.temperature, test_data.humidity, test_data.pressure);
    return true;
  }

  ESP_LOGI(TAG, "Bosch self-test passed");
  return true;
}

void sensor_bme680_reset_iaq_baseline(void) {
  iaq_state.gas_baseline = 0.0f;
  iaq_state.gas_baseline_count = 0;
  iaq_state.gas_baseline_valid = false;
  iaq_state.stabilization_count = 0;
  ESP_LOGI(TAG, "IAQ baseline reset, recalibration will start");
}

bool sensor_bme680_get_status(bool *gas_valid, bool *heat_stable) {
  if (bme680_i2c_handle == NULL) {
    return false;
  }

  uint8_t status = 0;
  int8_t rslt =
      bme68x_get_regs(BME68X_REG_FIELD0 + 14, &status, 1, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    return false;
  }

  if (gas_valid != NULL) {
    *gas_valid = (status & BME68X_GASM_VALID_MSK) != 0;
  }
  if (heat_stable != NULL) {
    *heat_stable = (status & BME68X_HEAT_STAB_MSK) != 0;
  }

  return true;
}

struct bme68x_dev *sensor_bme680_get_dev(void) {
  if (!sensor_initialized) {
    return NULL;
  }
  return &bme68x_sensor;
}

void sensor_bme680_deinit(void) {
  if (bme680_i2c_handle != NULL) {
    // Put sensor in sleep mode
    bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme68x_sensor);

    // Remove I2C device
    i2c_master_bus_rm_device(bme680_i2c_handle);
    bme680_i2c_handle = NULL;
    sensor_initialized = false;

    // Reset IAQ state
    sensor_bme680_reset_iaq_baseline();

    ESP_LOGI(TAG, "BME680 deinitialized");
  }
}
