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

// IAQ Algorithm Constants (based on reference implementation)
#define IAQ_GAS_BASELINE_DEFAULT                                               \
  50000.0f                     // Default baseline (adjusted for typical values)
#define IAQ_BURN_IN_SAMPLES 50 // Samples needed for initial calibration
#define IAQ_CALIBRATION_RATE 0.005f // Slow adaptation rate for baseline
#define IAQ_GAS_HISTORY_SIZE 10     // Moving average window
#define IAQ_TEMP_COMP_COEFF 0.003f  // 0.3% per °C deviation from 25°C
#define IAQ_HUM_COMP_COEFF 0.015f   // 1.5% per %RH deviation from 40%

// IAQ calculation state (enhanced for better accuracy)
static struct {
  float gas_baseline;
  float gas_sum;
  float gas_min;
  float gas_max;
  float gas_history[IAQ_GAS_HISTORY_SIZE];
  uint8_t gas_history_idx;
  uint32_t samples_count;
  bool baseline_valid;
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
 * IAQ Calculation (Improved algorithm based on reference implementation)
 *============================================================================*/

/**
 * @brief Apply temperature and humidity compensation to gas resistance
 * @param gas_resistance Raw gas resistance in Ohms
 * @param temperature Temperature in Celsius
 * @param humidity Relative humidity in %
 * @return Compensated gas resistance
 */
static float compensate_gas_resistance(float gas_resistance, float temperature,
                                       float humidity) {
  // Compensation for temperature deviation from reference (25°C)
  float temp_factor = 1.0f + IAQ_TEMP_COMP_COEFF * (temperature - 25.0f);

  // Compensation for humidity deviation from reference (40% RH)
  float hum_factor = 1.0f + IAQ_HUM_COMP_COEFF * (humidity - 40.0f);

  // Apply compensation (higher humidity = lower apparent resistance)
  float comp_resistance = gas_resistance * temp_factor / hum_factor;

  return comp_resistance;
}

/**
 * @brief Update gas baseline using exponential moving average
 * @param gas_resistance Compensated gas resistance in Ohms
 */
static void update_gas_baseline(float gas_resistance) {
  // Add to history buffer
  iaq_state.gas_history[iaq_state.gas_history_idx] = gas_resistance;
  iaq_state.gas_history_idx =
      (iaq_state.gas_history_idx + 1) % IAQ_GAS_HISTORY_SIZE;

  // Update running statistics
  iaq_state.gas_sum += gas_resistance;
  if (gas_resistance > iaq_state.gas_max) {
    iaq_state.gas_max = gas_resistance;
  }
  if (gas_resistance < iaq_state.gas_min || iaq_state.gas_min == 0) {
    iaq_state.gas_min = gas_resistance;
  }

  iaq_state.samples_count++;

  // Update baseline with slow adaptation
  if (iaq_state.samples_count <= IAQ_BURN_IN_SAMPLES) {
    // During burn-in, use simple average
    iaq_state.gas_baseline = iaq_state.gas_sum / iaq_state.samples_count;
    iaq_state.baseline_valid = false;
  } else {
    iaq_state.baseline_valid = true;
    // After burn-in, use exponential moving average with slow adaptation
    // Only update if current reading suggests cleaner air (higher resistance)
    if (gas_resistance > iaq_state.gas_baseline) {
      iaq_state.gas_baseline =
          iaq_state.gas_baseline * (1.0f - IAQ_CALIBRATION_RATE) +
          gas_resistance * IAQ_CALIBRATION_RATE;
    }
  }
}

/**
 * @brief Calculate IAQ score from compensated gas resistance
 * @param comp_gas_resistance Compensated gas resistance in Ohms
 * @return IAQ score (0-500, lower is better)
 */
static float calculate_iaq_score(float comp_gas_resistance) {
  float baseline = iaq_state.gas_baseline;

  // Use default baseline if not yet established
  if (baseline <= 0) {
    baseline = IAQ_GAS_BASELINE_DEFAULT;
  }

  // Calculate gas ratio (higher is better)
  float gas_ratio = comp_gas_resistance / baseline;

  // Convert to IAQ score (0-500 scale, lower is better)
  float iaq;

  if (gas_ratio >= 1.0f) {
    // Clean air (ratio >= 1.0)
    // IAQ = 0-50 for very clean, 50-100 for clean
    float clamped_ratio = gas_ratio;
    if (clamped_ratio > 2.0f)
      clamped_ratio = 2.0f;
    iaq = 50.0f * (2.0f - clamped_ratio);
  } else if (gas_ratio >= 0.5f) {
    // Slightly polluted (0.5 <= ratio < 1.0)
    // IAQ = 50-150
    iaq = 50.0f + 100.0f * (1.0f - gas_ratio) * 2.0f;
  } else if (gas_ratio >= 0.2f) {
    // Moderately polluted (0.2 <= ratio < 0.5)
    // IAQ = 150-250
    iaq = 150.0f + 100.0f * ((0.5f - gas_ratio) / 0.3f);
  } else if (gas_ratio >= 0.1f) {
    // Heavily polluted (0.1 <= ratio < 0.2)
    // IAQ = 250-350
    iaq = 250.0f + 100.0f * ((0.2f - gas_ratio) / 0.1f);
  } else {
    // Severely polluted (ratio < 0.1)
    // IAQ = 350-500
    float severity = (0.1f - gas_ratio) / 0.1f;
    if (severity > 1.0f)
      severity = 1.0f;
    iaq = 350.0f + 150.0f * severity;
  }

  // Clamp to valid range
  if (iaq < 0)
    iaq = 0;
  if (iaq > 500)
    iaq = 500;

  return iaq;
}

/**
 * @brief Determine accuracy based on calibration progress
 * @return Accuracy level (0-3)
 */
static uint8_t determine_accuracy(void) {
  uint32_t samples = iaq_state.samples_count;

  if (samples < IAQ_BURN_IN_SAMPLES / 4) {
    return 0; // Unreliable
  } else if (samples < IAQ_BURN_IN_SAMPLES / 2) {
    return 1; // Low
  } else if (samples < IAQ_BURN_IN_SAMPLES) {
    return 2; // Medium
  }
  return 3; // High
}

/**
 * @brief Calculate IAQ using improved algorithm based on reference
 * implementation
 * @param temp Temperature in Celsius
 * @param hum Humidity in %
 * @param gas_res Gas resistance in Ohms
 * @param accuracy Output parameter for IAQ accuracy (0-3)
 * @return IAQ value (0-500, lower is better)
 */
static float calculate_iaq(float temp, float hum, float gas_res,
                           uint8_t *accuracy) {
  // Skip calculation if gas resistance is invalid
  if (gas_res <= 0) {
    *accuracy = 0;
    return 0.0f;
  }

  // Step 1: Apply temperature/humidity compensation
  float comp_gas = compensate_gas_resistance(gas_res, temp, hum);

  // Step 2: Update baseline calibration
  update_gas_baseline(comp_gas);

  // Step 3: Calculate IAQ score
  float iaq_score = calculate_iaq_score(comp_gas);

  // Step 4: Determine accuracy
  *accuracy = determine_accuracy();

  // Debug logging for calibration progress
  if (iaq_state.samples_count <= IAQ_BURN_IN_SAMPLES &&
      (iaq_state.samples_count % 10 == 0)) {
    ESP_LOGI(TAG, "IAQ Calibration: %lu/%d samples, baseline=%.0f Ohms",
             (unsigned long)iaq_state.samples_count, IAQ_BURN_IN_SAMPLES,
             iaq_state.gas_baseline);
  }

  return iaq_score;
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

  // Debug: Print calibration coefficients to verify they were read correctly
  ESP_LOGI(TAG, "Calibration T: par_t1=%u, par_t2=%d, par_t3=%d",
           bme68x_sensor.calib.par_t1, bme68x_sensor.calib.par_t2,
           bme68x_sensor.calib.par_t3);
  ESP_LOGI(TAG, "Calibration P: par_p1=%u, par_p2=%d, par_p3=%d",
           bme68x_sensor.calib.par_p1, bme68x_sensor.calib.par_p2,
           bme68x_sensor.calib.par_p3);
  ESP_LOGI(TAG, "Calibration H: par_h1=%u, par_h2=%u, par_h3=%d",
           bme68x_sensor.calib.par_h1, bme68x_sensor.calib.par_h2,
           bme68x_sensor.calib.par_h3);

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

  int8_t rslt;

  // CRITICAL FIX: Re-apply FULL sensor configuration before EACH measurement
  // The bme68x_set_heatr_conf() internally calls bme68x_set_op_mode(SLEEP)
  // which may cause TPH oversampling settings to not be applied correctly
  // in the next measurement cycle. We must reconfigure EVERYTHING.

  // Step 1: Configure TPH oversampling settings
  struct bme68x_conf conf = {0};
  conf.os_temp = sensor_config.temp_os;
  conf.os_pres = sensor_config.press_os;
  conf.os_hum = sensor_config.hum_os;
  conf.filter = sensor_config.filter;
  conf.odr = BME68X_ODR_NONE; // No standby in forced mode

  rslt = bme68x_set_conf(&conf, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to set TPH config: %d", rslt);
    return false;
  }

  // Step 2: Configure heater for gas measurement
  if (sensor_config.gas_enabled) {
    struct bme68x_heatr_conf heatr_conf = {0};
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

  // Step 3: Set forced mode to trigger a NEW measurement
  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to set forced mode: %d", rslt);
    return false;
  }

  // Step 4: Calculate and wait for measurement duration
  uint32_t meas_dur =
      bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme68x_sensor);

  // Add heater duration if gas is enabled
  if (sensor_config.gas_enabled) {
    meas_dur += sensor_config.gas_wait_ms * 1000; // Convert ms to us
  }

  // Wait for measurement to complete with extra margin (20ms) for reliability
  bme68x_sensor.delay_us(meas_dur + 20000, bme68x_sensor.intf_ptr);

  // Step 5: Poll for new data with retries
  struct bme68x_data sensor_data[3];
  uint8_t n_data = 0;
  uint8_t max_retries = 10;

  for (uint8_t retry = 0; retry < max_retries; retry++) {
    rslt = bme68x_get_data(BME68X_FORCED_MODE, sensor_data, &n_data,
                           &bme68x_sensor);

    // Check if we got valid new data
    if (rslt == BME68X_OK && n_data > 0) {
      break; // Success - we have new data
    }

    // If warning indicates no new data, wait a bit and retry
    if (rslt == BME68X_W_NO_NEW_DATA) {
      ESP_LOGD(TAG, "No new data yet, retry %d/%d", retry + 1, max_retries);
      bme68x_sensor.delay_us(10000, bme68x_sensor.intf_ptr); // Wait 10ms
      continue;
    }

    // Any other error is a real failure
    if (rslt != BME68X_OK && rslt != BME68X_W_NO_NEW_DATA) {
      ESP_LOGE(TAG, "Failed to get data: rslt=%d", rslt);
      return false;
    }
  }

  // Final check if we got data
  if (n_data == 0) {
    ESP_LOGE(TAG, "No new data after %d retries", max_retries);
    return false;
  }

  // Use the first (and only in forced mode) data sample
  struct bme68x_data *d = &sensor_data[0];

  // Copy data to output structure
#ifdef BME68X_USE_FPU
  // FPU mode: temperature is in Celsius, pressure in Pa, humidity in %
  data->temperature = d->temperature;
  data->humidity = d->humidity;
  data->pressure = d->pressure / 100.0f; // Convert Pa to hPa
  data->gas_resistance = d->gas_resistance;
#else
  // Integer mode: temperature in centidegrees, humidity in milli-percent
  data->temperature = d->temperature / 100.0f;
  data->humidity = d->humidity / 1000.0f;
  data->pressure = d->pressure / 100.0f; // Convert Pa to hPa
  data->gas_resistance = (float)d->gas_resistance;
#endif

  // Sanity check the readings - if values are clearly wrong, log a warning
  if (data->temperature < -40.0f || data->temperature > 85.0f) {
    ESP_LOGW(TAG, "Temperature out of range: %.2f°C", data->temperature);
  }
  if (data->humidity < 0.0f || data->humidity > 100.0f) {
    ESP_LOGW(TAG, "Humidity out of range: %.2f%%", data->humidity);
  }
  if (data->pressure < 300.0f || data->pressure > 1100.0f) {
    ESP_LOGW(TAG, "Pressure out of range: %.2f hPa", data->pressure);
  }

  // Check validity flags
  data->gas_valid = (d->status & BME68X_GASM_VALID_MSK) != 0;
  data->heat_stable = (d->status & BME68X_HEAT_STAB_MSK) != 0;

  // Calculate IAQ
  data->iaq = calculate_iaq(data->temperature, data->humidity,
                            data->gas_resistance, &data->iaq_accuracy);

  ESP_LOGI(TAG, "BME680 Raw: status=0x%02X, meas_idx=%d, gas_idx=%d", d->status,
           d->meas_index, d->gas_index);
  ESP_LOGI(TAG,
           "BME680 Data: T=%.2f°C, H=%.2f%%, P=%.2fhPa, Gas=%.0fΩ (valid=%d, "
           "stable=%d)",
           data->temperature, data->humidity, data->pressure,
           data->gas_resistance, data->gas_valid, data->heat_stable);

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
  iaq_state.gas_sum = 0.0f;
  iaq_state.gas_min = 0.0f;
  iaq_state.gas_max = 0.0f;
  iaq_state.gas_history_idx = 0;
  iaq_state.samples_count = 0;
  iaq_state.baseline_valid = false;
  iaq_state.stabilization_count = 0;
  memset(iaq_state.gas_history, 0, sizeof(iaq_state.gas_history));
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

/**
 * @brief Debug function to read and print raw sensor data registers
 * This helps diagnose if raw ADC values are changing between reads
 */
void sensor_bme680_debug_raw_data(void) {
  if (bme680_i2c_handle == NULL || !sensor_initialized) {
    ESP_LOGE(TAG, "Sensor not initialized for debug");
    return;
  }

  // Read raw field data from register 0x1D (BME68X_REG_FIELD0)
  uint8_t buff[17] = {0};
  int8_t rslt = bme68x_get_regs(BME68X_REG_FIELD0, buff, 17, &bme68x_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to read raw registers: %d", rslt);
    return;
  }

  // Parse raw ADC values (same as in read_field_data)
  uint32_t adc_pres =
      (uint32_t)(((uint32_t)buff[2] * 4096) | ((uint32_t)buff[3] * 16) |
                 ((uint32_t)buff[4] / 16));
  uint32_t adc_temp =
      (uint32_t)(((uint32_t)buff[5] * 4096) | ((uint32_t)buff[6] * 16) |
                 ((uint32_t)buff[7] / 16));
  uint16_t adc_hum = (uint16_t)(((uint32_t)buff[8] * 256) | (uint32_t)buff[9]);
  uint16_t adc_gas_low =
      (uint16_t)((uint32_t)buff[13] * 4 | (((uint32_t)buff[14]) / 64));
  uint16_t adc_gas_high =
      (uint16_t)((uint32_t)buff[15] * 4 | (((uint32_t)buff[16]) / 64));
  uint8_t gas_range = buff[14] & 0x0F;

  ESP_LOGI(TAG, "=== BME680 DEBUG ===");
  ESP_LOGI(TAG, "Status: 0x%02X, meas_index: %d, gas_index: %d", buff[0],
           buff[1], buff[0] & 0x0F);
  ESP_LOGI(TAG, "ADC Pressure: %lu (raw bytes: %02X %02X %02X)",
           (unsigned long)adc_pres, buff[2], buff[3], buff[4]);
  ESP_LOGI(TAG, "ADC Temperature: %lu (raw bytes: %02X %02X %02X)",
           (unsigned long)adc_temp, buff[5], buff[6], buff[7]);
  ESP_LOGI(TAG, "ADC Humidity: %u (raw bytes: %02X %02X)", adc_hum, buff[8],
           buff[9]);
  ESP_LOGI(TAG, "ADC Gas Low: %u, High: %u, Range: %d", adc_gas_low,
           adc_gas_high, gas_range);
  ESP_LOGI(TAG, "Gas status byte[14]: 0x%02X, byte[16]: 0x%02X", buff[14],
           buff[16]);
  ESP_LOGI(TAG, "=====================");
}
