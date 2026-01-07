/**
 * @file sensor_bme680.c
 * @brief BME680 environmental sensor implementation
 */

#include "sensor_bme680.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "system_i2c.h"
#include <string.h>

static const char *TAG = "sensor_bme680";
static i2c_master_dev_handle_t bme680_handle = NULL;

// BME680 Register Addresses
#define BME680_REG_CHIP_ID 0xD0
#define BME680_REG_RESET 0xE0
#define BME680_REG_CTRL_HUM 0x72
#define BME680_REG_CTRL_MEAS 0x74
#define BME680_REG_CONFIG 0x75
#define BME680_REG_GAS_WAIT_0 0x64
#define BME680_REG_CTRL_GAS_1 0x71
#define BME680_REG_STATUS 0x1D
#define BME680_REG_TEMP_MSB 0x22
#define BME680_REG_HUM_MSB 0x25
#define BME680_REG_PRESS_MSB 0x1F
#define BME680_REG_GAS_R_MSB 0x2A

#define BME680_CHIP_ID 0x61 // Actual BME680 chip ID
#define BME680_RESET_CMD 0xB6

// Calibration data structure
typedef struct {
  uint16_t T1, T2, T3;
  uint16_t P1, P2, P3, P4, P5, P6, P7, P8, P9, P10;
  uint8_t H1, H2, H3, H4, H5, H6, H7;
  int8_t GH1, GH2, GH3;
} bme680_calib_t;

static bme680_calib_t calib_data;
static bool calib_loaded = false;
static bme680_power_mode_t current_power_mode = BME680_SLEEP_MODE;
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
                   .filter = 3,
                   .gas_wait_ms = 100,
                   .gas_heater_temp = 320,
                   .gas_enabled = true};

static bool bme680_read_calibration(void);
static float bme680_compensate_temperature(int32_t adc_temp);
static float bme680_compensate_pressure(int32_t adc_press, int32_t t_fine);
static float bme680_compensate_humidity(int32_t adc_hum, int32_t t_fine);
static float bme680_calculate_iaq(float temp, float hum, float gas_res,
                                  uint8_t *accuracy);

static esp_err_t bme680_read_reg(uint8_t reg, uint8_t *data, size_t len) {
  if (bme680_handle == NULL)
    return ESP_FAIL;

  return i2c_master_transmit_receive(bme680_handle, &reg, 1, data, len,
                                     pdMS_TO_TICKS(100));
}

static esp_err_t bme680_write_reg(uint8_t reg, uint8_t data) {
  if (bme680_handle == NULL)
    return ESP_FAIL;

  uint8_t buf[2] = {reg, data};
  return i2c_master_transmit(bme680_handle, buf, 2, pdMS_TO_TICKS(100));
}

static bool bme680_read_calibration(void) {
  uint8_t calib[42];
  esp_err_t ret;

  // Read temperature calibration (0xE1-0xE7)
  ret = bme680_read_reg(0xE1, calib, 7);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read temperature calibration");
    return false;
  }
  calib_data.T1 = (uint16_t)((calib[1] << 8) | calib[0]);
  calib_data.T2 = (int16_t)((calib[3] << 8) | calib[2]);
  calib_data.T3 = (int8_t)calib[4];

  // Read pressure calibration (0x8E-0x9F)
  ret = bme680_read_reg(0x8E, calib, 18);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read pressure calibration");
    return false;
  }
  calib_data.P1 = (uint16_t)((calib[1] << 8) | calib[0]);
  calib_data.P2 = (int16_t)((calib[3] << 8) | calib[2]);
  calib_data.P3 = (int8_t)calib[4];
  calib_data.P4 = (int16_t)((calib[6] << 8) | calib[5]);
  calib_data.P5 = (int16_t)((calib[8] << 8) | calib[7]);
  calib_data.P6 = (int8_t)calib[9];
  calib_data.P7 = (int8_t)calib[10];
  calib_data.P8 = (int16_t)((calib[12] << 8) | calib[11]);
  calib_data.P9 = (int16_t)((calib[14] << 8) | calib[13]);
  calib_data.P10 = (uint8_t)calib[15];

  // Read humidity calibration (0xE1-0xE8, 0xE3-0xE8)
  ret = bme680_read_reg(0xE1, calib, 8);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read humidity calibration H1");
    return false;
  }
  calib_data.H1 = calib[7];
  ret = bme680_read_reg(0xE3, calib, 6);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read humidity calibration H2-H6");
    return false;
  }
  calib_data.H2 = (int16_t)((calib[1] << 8) | calib[0]);
  calib_data.H3 = calib[2];
  calib_data.H4 = (int16_t)((calib[3] << 4) | (calib[4] & 0x0F));
  calib_data.H5 = (int16_t)((calib[5] << 4) | (calib[4] >> 4));
  calib_data.H6 = (int8_t)calib[6];
  ret = bme680_read_reg(0xE8, calib, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read humidity calibration H7");
    return false;
  }
  calib_data.H7 = calib[0];

  // Read gas calibration (0xED-0xF6)
  ret = bme680_read_reg(0xED, calib, 10);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read gas calibration");
    return false;
  }
  calib_data.GH1 = (int8_t)calib[1];
  calib_data.GH2 = (int16_t)((calib[3] << 8) | calib[2]);
  calib_data.GH3 = (int8_t)calib[4];

  calib_loaded = true;
  return true;
}

static float bme680_compensate_temperature(int32_t adc_temp) {
  int32_t var1, var2, t_fine;

  var1 = ((((adc_temp >> 3) - ((int32_t)calib_data.T1 << 1))) *
          ((int32_t)calib_data.T2)) >>
         11;

  var2 = (((((adc_temp >> 4) - ((int32_t)calib_data.T1)) *
            ((adc_temp >> 4) - ((int32_t)calib_data.T1))) >>
           12) *
          ((int32_t)calib_data.T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100.0f;
}

static float bme680_compensate_pressure(int32_t adc_press, int32_t t_fine) {
  int32_t var1, var2, var3, p;

  var1 = (t_fine >> 1) - 64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_data.P6);
  var2 = var2 + ((var1 * ((int32_t)calib_data.P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)calib_data.P4) << 16);
  var1 = (((calib_data.P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
          ((((int32_t)calib_data.P2) * var1) >> 1)) >>
         18;
  var1 = ((((32768 + var1)) * ((int32_t)calib_data.P1)) >> 15);

  if (var1 == 0) {
    return 0;
  }

  p = (((uint32_t)(1048576 - adc_press) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t)var1);
  } else {
    p = (p / (uint32_t)var1) * 2;
  }

  var1 =
      (((int32_t)calib_data.P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >>
      12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_data.P8)) >> 13;
  var3 = (((int32_t)(p >> 8)) * ((int32_t)(p >> 8)) * ((int32_t)(p >> 8)) *
          ((int32_t)calib_data.P10)) >>
         17;

  p = (int32_t)((int32_t)p +
                ((var1 + var2 + var3 + ((int32_t)calib_data.P7 << 7)) >> 4));

  return (float)p / 256.0f;
}

static float bme680_compensate_humidity(int32_t adc_hum, int32_t t_fine) {
  int32_t var1, var2, var3, var4, var5, var6, h;

  var1 = adc_hum - (((int32_t)calib_data.H1 * 16) +
                    (((int32_t)calib_data.H3 * t_fine) / 100));
  var2 =
      (((int32_t)calib_data.H2 *
        (((t_fine * (int32_t)calib_data.H4) / 100) +
         (((t_fine * ((t_fine * (int32_t)calib_data.H5) / 100)) >> 6) / 100) +
         (1 << 14))) >>
       10);
  var3 = var1 * var2;
  var4 = ((int32_t)calib_data.H6 << 7) +
         (((t_fine * (int32_t)calib_data.H7) / 100));
  var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
  var6 = (var4 * var5) >> 1;
  h = (var3 + var6) >> 12;
  h = (h * 1000) >> 12;

  if (h > 100000)
    h = 100000;
  if (h < 0)
    h = 0;

  return (float)h / 1000.0f;
}

// IAQ calculation state (for accuracy tracking)
static struct {
  float gas_baseline; // Baseline gas resistance
  uint32_t gas_baseline_count;
  bool gas_baseline_valid;
  uint32_t stabilization_count;
} iaq_state = {0};

/**
 * @brief Calculate IAQ using optimized Bosch algorithm
 * @param temp Temperature in Celsius
 * @param hum Humidity in %
 * @param gas_res Gas resistance in Ohms
 * @param accuracy Output parameter for IAQ accuracy
 * @return IAQ value (0-500)
 */
static float bme680_calculate_iaq(float temp, float hum, float gas_res,
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
  iaq_state.gas_baseline = (iaq_state.gas_baseline * 0.95f) + (gas_res * 0.05f);

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
  if (iaq_state.gas_baseline > 0.0f) {
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

bool sensor_bme680_init(void) {
  if (bme680_handle != NULL) {
    ESP_LOGW(TAG, "BME680 already initialized");
    return true;
  }

  i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return false;
  }

  // Try both common BME680 I2C addresses (0x76 and 0x77)
  // BME680 address depends on SDO pin: SDO->GND = 0x76, SDO->VCC = 0x77
  uint8_t addresses[] = {BME680_I2C_ADDR,
                         BME680_I2C_ADDR == 0x77 ? 0x76 : 0x77};
  uint8_t chip_id = 0;
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
        i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bme680_handle);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to add BME680 device at 0x%02X: %s",
               addresses[addr_idx], esp_err_to_name(ret));
      continue;
    }

    // Try reading chip ID with retries (I2C may need time to stabilize)
    for (int retry = 0; retry < 3; retry++) {
      chip_id = 0;
      vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before each attempt
      ret = bme680_read_reg(BME680_REG_CHIP_ID, &chip_id, 1);

      if (ret == ESP_OK && chip_id == BME680_CHIP_ID) {
        device_found = true;
        actual_address = addresses[addr_idx];
        ESP_LOGI(TAG, "BME680 found at I2C address 0x%02X with chip ID 0x%02X",
                 actual_address, chip_id);
        break;
      }

      if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C read error at 0x%02X (retry %d): %s",
                 addresses[addr_idx], retry, esp_err_to_name(ret));
      }
    }

    if (device_found) {
      break;
    }

    // Log what we found (could be useful for debugging)
    if (chip_id != 0 && chip_id != 0xFF) {
      ESP_LOGW(TAG,
               "Device at 0x%02X has chip ID 0x%02X (BME680 expects 0x%02X)",
               addresses[addr_idx], chip_id, BME680_CHIP_ID);
      ESP_LOGW(TAG, "  Note: BME280=0x60, BMP280=0x58, BMP180=0x55");
    }

    i2c_master_bus_rm_device(bme680_handle);
    bme680_handle = NULL;
  }

  if (!device_found) {
    ESP_LOGE(TAG, "BME680 not found. Please check:");
    ESP_LOGE(TAG, "  - I2C wiring (SDA, SCL connections)");
    ESP_LOGE(TAG, "  - Pull-up resistors (4.7k-10k ohm)");
    ESP_LOGE(TAG, "  - Power supply (VCC=3.3V, GND)");
    ESP_LOGE(TAG, "  - SDO pin (GND=0x76, VCC=0x77)");
    return false;
  }

  // Soft reset
  bme680_write_reg(BME680_REG_RESET, BME680_RESET_CMD);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Configure sensor with default optimized settings
  sensor_bme680_configure(
      BME680_OS_2X,  // Temperature oversampling
      BME680_OS_16X, // Pressure oversampling (higher for accuracy)
      BME680_OS_1X,  // Humidity oversampling
      3,             // Filter coefficient
      100,           // Gas wait time (ms)
      320            // Gas heater temperature (°C)
  );

  // Read calibration data
  if (!bme680_read_calibration()) {
    ESP_LOGE(TAG, "Failed to read calibration data");
    i2c_master_bus_rm_device(bme680_handle);
    bme680_handle = NULL;
    return false;
  }

  ESP_LOGI(TAG, "BME680 initialized successfully");
  return true;
}

bool sensor_bme680_configure(bme680_oversampling_t temp_os,
                             bme680_oversampling_t press_os,
                             bme680_oversampling_t hum_os, uint8_t filter,
                             uint16_t gas_wait_ms, uint16_t gas_heater_temp) {
  if (bme680_handle == NULL) {
    return false;
  }

  // Store configuration
  sensor_config.temp_os = temp_os;
  sensor_config.press_os = press_os;
  sensor_config.hum_os = hum_os;
  sensor_config.filter = filter & 0x07;             // 3 bits only
  sensor_config.gas_wait_ms = gas_wait_ms & 0x0FFF; // 12 bits
  sensor_config.gas_heater_temp = (gas_heater_temp < 200)   ? 200
                                  : (gas_heater_temp > 400) ? 400
                                                            : gas_heater_temp;
  sensor_config.gas_enabled = (gas_wait_ms > 0);

  // Configure humidity oversampling
  bme680_write_reg(BME680_REG_CTRL_HUM, hum_os & 0x07);

  // Configure gas sensor
  if (sensor_config.gas_enabled) {
    // Calculate gas wait register value (5 bits: multiplier + 4 bits: value)
    uint8_t gas_wait_mult = 0;
    uint16_t wait_time = gas_wait_ms;
    if (wait_time > 4095)
      wait_time = 4095;

    // Find multiplier
    while (wait_time > 63 && gas_wait_mult < 4) {
      wait_time >>= 1;
      gas_wait_mult++;
    }

    uint8_t gas_wait_reg = (gas_wait_mult << 5) | (wait_time & 0x1F);
    bme680_write_reg(BME680_REG_GAS_WAIT_0, gas_wait_reg);

    // Calculate heater temperature register
    // Formula: T = 200 + (reg_val * 0.5)
    uint8_t heater_temp_reg = (uint8_t)((gas_heater_temp - 200) * 2);
    bme680_write_reg(0x5A, heater_temp_reg); // RES_HEAT_0 register

    // Enable gas sensor
    bme680_write_reg(BME680_REG_CTRL_GAS_1,
                     0x10); // Enable gas, use heater profile 0
  } else {
    bme680_write_reg(BME680_REG_CTRL_GAS_1, 0x00); // Disable gas
  }

  // Configure filter and standby time
  uint8_t config_reg = (sensor_config.filter & 0x07) << 2;
  bme680_write_reg(BME680_REG_CONFIG, config_reg);

  return true;
}

bool sensor_bme680_set_power_mode(bme680_power_mode_t mode) {
  if (bme680_handle == NULL) {
    return false;
  }

  uint8_t ctrl_meas = 0;
  bme680_read_reg(BME680_REG_CTRL_MEAS, &ctrl_meas, 1);
  ctrl_meas &= 0xFC; // Clear mode bits
  ctrl_meas |= (mode & 0x03);
  bme680_write_reg(BME680_REG_CTRL_MEAS, ctrl_meas);

  current_power_mode = mode;
  return true;
}

bool sensor_bme680_read(bme680_data_t *data) {
  if (bme680_handle == NULL || data == NULL) {
    return false;
  }

  if (!calib_loaded) {
    ESP_LOGE(TAG, "Calibration data not loaded");
    return false;
  }

  // Trigger forced measurement
  uint8_t ctrl_meas = (sensor_config.temp_os << 5) |
                      (sensor_config.press_os << 2) | BME680_FORCED_MODE;
  bme680_write_reg(BME680_REG_CTRL_MEAS, ctrl_meas);

  // Wait for measurement (optimized: check status instead of fixed delay)
  uint32_t timeout = 0;
  uint8_t status = 0;
  do {
    vTaskDelay(pdMS_TO_TICKS(10));
    bme680_read_reg(BME680_REG_STATUS, &status, 1);
    timeout++;
  } while (!(status & 0x80) && timeout < 50); // Max 500ms

  if (!(status & 0x80)) {
    ESP_LOGE(TAG, "Measurement timeout");
    return false;
  }

  // Read raw data (pressure, temperature, humidity, gas)
  uint8_t raw_data[15];
  bme680_read_reg(BME680_REG_PRESS_MSB, raw_data, 15);

  // Parse pressure (20 bits)
  int32_t adc_press =
      (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));

  // Parse temperature (20 bits)
  int32_t adc_temp =
      (int32_t)((raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4));

  // Parse humidity (16 bits)
  int32_t adc_hum = (int32_t)((raw_data[6] << 8) | raw_data[7]);

  // Parse gas resistance (17 bits)
  uint32_t adc_gas = (uint32_t)((raw_data[13] << 2) | (raw_data[14] >> 6));
  uint8_t gas_range = raw_data[14] & 0x0F;

  // Calculate gas resistance in Ohms
  float gas_res = 0.0f;
  if (sensor_config.gas_enabled && (status & 0x20)) { // Gas measurement valid
    const float lookup_k1_range[16] = {0.0f, 0.0f,  0.0f, 0.0f, 0.0f,  -1.0f,
                                       0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f,
                                       0.0f, -1.0f, 0.0f, 0.0f};
    const float lookup_k2_range[16] = {0.0f, 0.0f,  0.0f,  0.0f, 0.1f, 0.7f,
                                       0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f,
                                       0.0f, 0.0f,  0.0f,  0.0f};

    float var1 = (1340.0f + (5.0f * lookup_k2_range[gas_range])) *
                 (1.0f + lookup_k1_range[gas_range] / 100.0f);
    float var2 = var1 * (1.0f + (lookup_k1_range[gas_range] / 100.0f));
    gas_res = var2 * (1.0f / (float)adc_gas - 1.0f);
  }

  // Compensate temperature (returns t_fine for use in other compensations)
  float temp = bme680_compensate_temperature(adc_temp);

  // Recalculate t_fine for pressure/humidity compensation
  int32_t var1 = ((((adc_temp >> 3) - ((int32_t)calib_data.T1 << 1))) *
                  ((int32_t)calib_data.T2)) >>
                 11;
  int32_t var2 = (((((adc_temp >> 4) - ((int32_t)calib_data.T1)) *
                    ((adc_temp >> 4) - ((int32_t)calib_data.T1))) >>
                   12) *
                  ((int32_t)calib_data.T3)) >>
                 14;
  int32_t t_fine = var1 + var2;

  // Store compensated values
  data->temperature = temp;
  data->pressure =
      bme680_compensate_pressure(adc_press, t_fine) / 100.0f; // Convert to hPa
  data->humidity = bme680_compensate_humidity(adc_hum, t_fine);
  data->gas_resistance = gas_res;
  data->iaq =
      bme680_calculate_iaq(temp, data->humidity, gas_res, &data->iaq_accuracy);

  return true;
}

void sensor_bme680_deinit(void) {
  if (bme680_handle != NULL) {
    i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
    if (i2c_bus != NULL) {
      i2c_master_bus_rm_device(bme680_handle);
    }
    bme680_handle = NULL;
    calib_loaded = false;
    ESP_LOGI(TAG, "BME680 deinitialized");
  }
}
