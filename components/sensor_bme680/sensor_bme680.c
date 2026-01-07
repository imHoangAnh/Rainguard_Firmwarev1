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

#define BME680_CHIP_ID 0x61
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

static bool bme680_read_calibration(void);
static float bme680_compensate_temperature(int32_t adc_temp);
static float bme680_compensate_pressure(int32_t adc_press, int32_t t_fine);
static float bme680_compensate_humidity(int32_t adc_hum, int32_t t_fine);
static float bme680_calculate_iaq(float temp, float hum, float press);

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

static float bme680_calculate_iaq(float temp, float hum, float press) {
  // Simplified IAQ calculation based on temperature, humidity, and pressure
  // This is a basic implementation - real IAQ requires gas sensor readings
  float iaq = 25.0f; // Base value

  // Adjust based on humidity (optimal range: 40-60%)
  if (hum < 40.0f || hum > 60.0f) {
    iaq += 10.0f;
  }

  // Adjust based on temperature (optimal range: 20-25°C)
  if (temp < 18.0f || temp > 26.0f) {
    iaq += 15.0f;
  }

  // Normalize to 0-500 range
  if (iaq > 500.0f)
    iaq = 500.0f;
  if (iaq < 0.0f)
    iaq = 0.0f;

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

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = BME680_I2C_ADDR,
      .scl_speed_hz = 100000,
  };

  esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bme680_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add BME680 device: %s", esp_err_to_name(ret));
    return false;
  }

  // Check chip ID
  uint8_t chip_id = 0;
  bme680_read_reg(BME680_REG_CHIP_ID, &chip_id, 1);
  if (chip_id != BME680_CHIP_ID) {
    ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id,
             BME680_CHIP_ID);
    i2c_master_bus_rm_device(bme680_handle);
    bme680_handle = NULL;
    return false;
  }

  // Soft reset
  bme680_write_reg(BME680_REG_RESET, BME680_RESET_CMD);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Configure sensor
  bme680_write_reg(BME680_REG_CTRL_HUM, 0x01); // Humidity oversampling x1
  bme680_write_reg(BME680_REG_CTRL_MEAS,
                   0x25); // Temperature/Pressure oversampling, normal mode
  bme680_write_reg(BME680_REG_CONFIG, 0x00);     // Standby time 0.5ms
  bme680_write_reg(BME680_REG_CTRL_GAS_1, 0x00); // Disable gas sensor for now

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

bool sensor_bme680_read(bme680_data_t *data) {
  if (bme680_handle == NULL || data == NULL) {
    return false;
  }

  if (!calib_loaded) {
    ESP_LOGE(TAG, "Calibration data not loaded");
    return false;
  }

  // Trigger measurement
  bme680_write_reg(BME680_REG_CTRL_MEAS, 0x25);
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for measurement

  // Check status
  uint8_t status = 0;
  bme680_read_reg(BME680_REG_STATUS, &status, 1);
  if (!(status & 0x80)) {
    ESP_LOGE(TAG, "Measurement not ready");
    return false;
  }

  // Read raw data
  uint8_t raw_data[8];
  bme680_read_reg(BME680_REG_PRESS_MSB, raw_data, 8);

  int32_t adc_press =
      (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
  int32_t adc_temp =
      (int32_t)((raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4));
  int32_t adc_hum = (int32_t)((raw_data[6] << 8) | raw_data[7]);

  // Compensate
  int32_t t_fine = 0;
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
  t_fine = var1 + var2;

  data->temperature = temp;
  data->pressure =
      bme680_compensate_pressure(adc_press, t_fine) / 100.0f; // Convert to hPa
  data->humidity = bme680_compensate_humidity(adc_hum, t_fine);
  data->iaq = bme680_calculate_iaq(temp, data->humidity, data->pressure);

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
