/**
 * @file sensor_mpu6050.c
 * @brief MPU6050 accelerometer and gyroscope implementation
 */

#include "sensor_mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "system_i2c.h"
#include <math.h>
#include <string.h>

static const char *TAG = "mpu6050";
static i2c_master_dev_handle_t mpu6050_handle = NULL;

// MPU6050 Register Addresses
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

#define MPU6050_WHO_AM_I_VALUE 0x68
#define MPU6050_PWR_MGMT_1_RESET 0x80
#define MPU6050_PWR_MGMT_1_WAKEUP 0x00

// Full scale ranges
#define MPU6050_ACCEL_FS_2G 0x00
#define MPU6050_ACCEL_FS_4G 0x01
#define MPU6050_ACCEL_FS_8G 0x02
#define MPU6050_ACCEL_FS_16G 0x03
#define MPU6050_GYRO_FS_250DPS 0x00
#define MPU6050_GYRO_FS_500DPS 0x01
#define MPU6050_GYRO_FS_1000DPS 0x02
#define MPU6050_GYRO_FS_2000DPS 0x03

// Temperature register
#define MPU6050_REG_TEMP_OUT_H 0x41

// Sensitivity scales (LSB/g or LSB/(deg/s))
static float accel_sensitivity = 16384.0f; // Default for 2G
static float gyro_sensitivity = 131.0f;    // Default for 250DPS

// Sensitivity lookup tables
static const float accel_sens_table[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float gyro_sens_table[] = {131.0f, 65.5f, 32.8f, 16.4f};

#define MPU6050_ACCEL_SENS_2G 16384.0f
#define MPU6050_GYRO_SENS_250DPS 131.0f

// Kalman filter structure for attitude estimation
typedef struct {
  float Q_angle; // Process noise covariance for angle
  float Q_gyro;  // Process noise covariance for gyro bias
  float R_angle; // Measurement noise covariance
  float angle;   // Estimated angle
  float bias;    // Estimated gyro bias
  float P[2][2]; // Error covariance matrix
} kalman_t;

// Sensor state
static struct {
  float gyro_offset_x;
  float gyro_offset_y;
  float gyro_offset_z;
  bool gyro_calibrated;
  kalman_t kalman_x; // For pitch
  kalman_t kalman_y; // For roll
  uint32_t last_update_us;
} sensor_state = {.gyro_offset_x = 0.0f,
                  .gyro_offset_y = 0.0f,
                  .gyro_offset_z = 0.0f,
                  .gyro_calibrated = false,
                  .last_update_us = 0};

// Initialize Kalman filter
static void kalman_init(kalman_t *k) {
  k->Q_angle = 0.001f;
  k->Q_gyro = 0.003f;
  k->R_angle = 0.03f;
  k->angle = 0.0f;
  k->bias = 0.0f;
  k->P[0][0] = 0.0f;
  k->P[0][1] = 0.0f;
  k->P[1][0] = 0.0f;
  k->P[1][1] = 0.0f;
}

// Kalman filter update
static float kalman_update(kalman_t *k, float newAngle, float newRate,
                           float dt) {
  // Predict
  k->angle += dt * (newRate - k->bias);
  k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
  k->P[0][1] -= dt * k->P[1][1];
  k->P[1][0] -= dt * k->P[1][1];
  k->P[1][1] += k->Q_gyro * dt;

  // Update
  float S = k->P[0][0] + k->R_angle;
  float K[2];
  K[0] = k->P[0][0] / S;
  K[1] = k->P[1][0] / S;

  float y = newAngle - k->angle;
  k->angle += K[0] * y;
  k->bias += K[1] * y;

  float P00_temp = k->P[0][0];
  float P01_temp = k->P[0][1];

  k->P[0][0] -= K[0] * P00_temp;
  k->P[0][1] -= K[0] * P01_temp;
  k->P[1][0] -= K[1] * P00_temp;
  k->P[1][1] -= K[1] * P01_temp;

  return k->angle;
}

static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data, size_t len) {
  if (mpu6050_handle == NULL)
    return ESP_FAIL;

  return i2c_master_transmit_receive(mpu6050_handle, &reg, 1, data, len,
                                     pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data) {
  if (mpu6050_handle == NULL)
    return ESP_FAIL;

  uint8_t buf[2] = {reg, data};
  return i2c_master_transmit(mpu6050_handle, buf, 2, pdMS_TO_TICKS(100));
}

bool sensor_mpu6050_init(void) {
  if (mpu6050_handle != NULL) {
    ESP_LOGW(TAG, "MPU6050 already initialized");
    return true;
  }

  i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return false;
  }

  ESP_LOGI(TAG, "Searching for MPU6050 sensor at address 0x%02X...",
           MPU6050_I2C_ADDR);

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_I2C_ADDR,
      .scl_speed_hz = 100000,
  };

  esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpu6050_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add MPU6050 device: %s", esp_err_to_name(ret));
    return false;
  }

  // Check WHO_AM_I register with retries
  uint8_t who_am_i = 0;
  bool device_found = false;

  for (int retry = 0; retry < 3; retry++) {
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i, 1);

    if (ret == ESP_OK && who_am_i == MPU6050_WHO_AM_I_VALUE) {
      device_found = true;
      break;
    }

    if (ret != ESP_OK) {
      ESP_LOGD(TAG, "I2C read error (retry %d): %s", retry,
               esp_err_to_name(ret));
    } else if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
      ESP_LOGD(TAG, "WHO_AM_I mismatch (retry %d): got 0x%02X", retry,
               who_am_i);
    }
  }

  if (!device_found) {
    ESP_LOGE(TAG, "MPU6050 not found. WHO_AM_I: 0x%02X (expected 0x%02X)",
             who_am_i, MPU6050_WHO_AM_I_VALUE);
    ESP_LOGE(TAG, "Please check:");
    ESP_LOGE(TAG, "  - I2C wiring (SDA, SCL connections)");
    ESP_LOGE(TAG, "  - Pull-up resistors (4.7k-10k ohm)");
    ESP_LOGE(TAG, "  - Power supply (VCC=3.3V, GND)");
    ESP_LOGE(TAG, "  - AD0 pin (GND=0x68, VCC=0x69)");
    i2c_master_bus_rm_device(mpu6050_handle);
    mpu6050_handle = NULL;
    return false;
  }

  ESP_LOGI(TAG, "MPU6050 found with Address: 0x%02X", who_am_i);

  // Reset device
  ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset MPU6050: %s", esp_err_to_name(ret));
    i2c_master_bus_rm_device(mpu6050_handle);
    mpu6050_handle = NULL;
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // Wake up device (clear sleep bit)
  ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKEUP);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
    i2c_master_bus_rm_device(mpu6050_handle);
    mpu6050_handle = NULL;
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  // Configure accelerometer (±2g)
  mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G << 3);

  // Configure gyroscope (±250°/s)
  mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250DPS << 3);

  // Configure DLPF (Digital Low Pass Filter) - 44Hz bandwidth
  mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03);

  // Initialize Kalman filters
  kalman_init(&sensor_state.kalman_x);
  kalman_init(&sensor_state.kalman_y);

  ESP_LOGI(TAG, "MPU6050 initialized successfully");
  return true;
}

bool sensor_mpu6050_calibrate_gyro(uint16_t samples) {
  if (mpu6050_handle == NULL) {
    return false;
  }

  ESP_LOGI(TAG, "Calibrating gyroscope (%d samples)...", samples);

  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

  for (uint16_t i = 0; i < samples; i++) {
    uint8_t raw_data[6];
    mpu6050_read_reg(MPU6050_REG_GYRO_XOUT_H, raw_data, 6);

    int16_t gyro_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t gyro_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t gyro_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    sum_x += (float)gyro_x_raw / MPU6050_GYRO_SENS_250DPS;
    sum_y += (float)gyro_y_raw / MPU6050_GYRO_SENS_250DPS;
    sum_z += (float)gyro_z_raw / MPU6050_GYRO_SENS_250DPS;

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  sensor_state.gyro_offset_x = sum_x / samples;
  sensor_state.gyro_offset_y = sum_y / samples;
  sensor_state.gyro_offset_z = sum_z / samples;
  sensor_state.gyro_calibrated = true;

  ESP_LOGI(TAG, "Gyro calibration complete: X=%.3f, Y=%.3f, Z=%.3f deg/s",
           sensor_state.gyro_offset_x, sensor_state.gyro_offset_y,
           sensor_state.gyro_offset_z);

  return true;
}

bool sensor_mpu6050_set_low_power(bool enable) {
  if (mpu6050_handle == NULL) {
    return false;
  }

  uint8_t pwr_mgmt = enable ? 0x20 : 0x00; // Cycle mode for low power
  mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, pwr_mgmt);

  return true;
}

bool sensor_mpu6050_read(mpu6050_data_t *data) {
  if (mpu6050_handle == NULL || data == NULL) {
    return false;
  }

  uint8_t raw_data[14];
  esp_err_t ret = mpu6050_read_reg(MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
  if (ret != ESP_OK) {
    return false;
  }

  // Read accelerometer data (16-bit, big-endian)
  int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  // Read gyroscope data (16-bit, big-endian)
  int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
  int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
  int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);

  // Convert to physical units using current sensitivity
  data->accel_x = (float)accel_x_raw / accel_sensitivity;
  data->accel_y = (float)accel_y_raw / accel_sensitivity;
  data->accel_z = (float)accel_z_raw / accel_sensitivity;

  // Apply gyro calibration offsets
  data->gyro_x =
      ((float)gyro_x_raw / gyro_sensitivity) - sensor_state.gyro_offset_x;
  data->gyro_y =
      ((float)gyro_y_raw / gyro_sensitivity) - sensor_state.gyro_offset_y;
  data->gyro_z =
      ((float)gyro_z_raw / gyro_sensitivity) - sensor_state.gyro_offset_z;

  // Parse temperature data (bytes 6-7)
  int16_t temp_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);
  data->temperature = (float)temp_raw / 340.0f + 36.53f;

  // Calculate acceleration magnitude for vibration
  float accel_magnitude =
      sqrtf(data->accel_x * data->accel_x + data->accel_y * data->accel_y +
            data->accel_z * data->accel_z);

  // Calculate time delta for Kalman filter
  uint32_t current_us = esp_timer_get_time();
  float dt = 0.0f;
  if (sensor_state.last_update_us > 0) {
    dt = (current_us - sensor_state.last_update_us) / 1000000.0f;
  } else {
    dt = 0.01f; // Default 10ms
  }
  sensor_state.last_update_us = current_us;

  // Calculate angles from accelerometer (pitch and roll)
  float accel_pitch = atan2f(data->accel_y, data->accel_z) * 180.0f / M_PI;
  float accel_roll =
      atan2f(-data->accel_x, sqrtf(data->accel_y * data->accel_y +
                                   data->accel_z * data->accel_z)) *
      180.0f / M_PI;

  // Apply Kalman filter to fuse accelerometer and gyroscope data
  data->pitch =
      kalman_update(&sensor_state.kalman_x, accel_pitch, data->gyro_x, dt);
  data->roll =
      kalman_update(&sensor_state.kalman_y, accel_roll, data->gyro_y, dt);

  // Yaw cannot be calculated from accelerometer alone (requires magnetometer)
  // For now, integrate gyro_z (will drift over time)
  static float yaw_angle = 0.0f;
  yaw_angle += data->gyro_z * dt;
  data->yaw = yaw_angle;

  // Calculate vibration index (0-100 scale)
  // Based on dynamic acceleration (total - gravity) and gyroscope
  float dyn_accel = fabsf(accel_magnitude - 1.0f); // Remove gravity
  float gyro_mag =
      sqrtf(data->gyro_x * data->gyro_x + data->gyro_y * data->gyro_y +
            data->gyro_z * data->gyro_z);

  // Normalize and combine (accel weight 70%, gyro weight 30%)
  float accel_norm = (dyn_accel / 0.5f) * 100.0f; // 0.5g = 100%
  float gyro_norm = (gyro_mag / 50.0f) * 100.0f;  // 50 deg/s = 100%

  data->vibration = 0.7f * accel_norm + 0.3f * gyro_norm;
  if (data->vibration > 100.0f)
    data->vibration = 100.0f;
  if (data->vibration < 0.0f)
    data->vibration = 0.0f;

  // Add timestamp
  data->timestamp_us = current_us;

  return true;
}

bool sensor_mpu6050_configure(const mpu6050_config_params_t *config) {
  if (mpu6050_handle == NULL || config == NULL) {
    return false;
  }

  // Configure accelerometer range
  mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, config->accel_range << 3);
  accel_sensitivity = accel_sens_table[config->accel_range];

  // Configure gyroscope range
  mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, config->gyro_range << 3);
  gyro_sensitivity = gyro_sens_table[config->gyro_range];

  // Configure DLPF
  mpu6050_write_reg(MPU6050_REG_CONFIG, config->dlpf);

  ESP_LOGI(TAG, "MPU6050 configured: Accel=%dG, Gyro=%dDPS, DLPF=%d",
           2 << config->accel_range, 250 << config->gyro_range, config->dlpf);

  return true;
}

bool sensor_mpu6050_read_raw(float *accel_x, float *accel_y, float *accel_z,
                             float *gyro_x, float *gyro_y, float *gyro_z) {
  if (mpu6050_handle == NULL) {
    return false;
  }

  uint8_t raw_data[14];
  esp_err_t ret = mpu6050_read_reg(MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
  if (ret != ESP_OK) {
    return false;
  }

  // Parse accelerometer data
  int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  // Parse gyroscope data
  int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
  int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
  int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);

  // Convert to physical units
  if (accel_x)
    *accel_x = (float)accel_x_raw / accel_sensitivity;
  if (accel_y)
    *accel_y = (float)accel_y_raw / accel_sensitivity;
  if (accel_z)
    *accel_z = (float)accel_z_raw / accel_sensitivity;
  if (gyro_x)
    *gyro_x = (float)gyro_x_raw / gyro_sensitivity - sensor_state.gyro_offset_x;
  if (gyro_y)
    *gyro_y = (float)gyro_y_raw / gyro_sensitivity - sensor_state.gyro_offset_y;
  if (gyro_z)
    *gyro_z = (float)gyro_z_raw / gyro_sensitivity - sensor_state.gyro_offset_z;

  return true;
}

bool sensor_mpu6050_read_temperature(float *temperature) {
  if (mpu6050_handle == NULL || temperature == NULL) {
    return false;
  }

  uint8_t raw_data[2];
  esp_err_t ret = mpu6050_read_reg(MPU6050_REG_TEMP_OUT_H, raw_data, 2);
  if (ret != ESP_OK) {
    return false;
  }

  int16_t temp_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  // Temperature formula from datasheet: Temp = (raw / 340.0) + 36.53
  *temperature = (float)temp_raw / 340.0f + 36.53f;

  return true;
}

bool sensor_mpu6050_self_test(void) {
  if (mpu6050_handle == NULL) {
    ESP_LOGE(TAG, "Sensor not initialized");
    return false;
  }

  // Read WHO_AM_I to verify communication
  uint8_t who_am_i = 0;
  esp_err_t ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
  if (ret != ESP_OK || who_am_i != MPU6050_WHO_AM_I_VALUE) {
    ESP_LOGE(TAG, "Self-test failed: WHO_AM_I mismatch");
    return false;
  }

  // Read sensor data to verify operation
  mpu6050_data_t test_data;
  if (!sensor_mpu6050_read(&test_data)) {
    ESP_LOGE(TAG, "Self-test failed: read error");
    return false;
  }

  // Check if accelerometer is reading reasonable values
  float accel_mag = sqrtf(test_data.accel_x * test_data.accel_x +
                          test_data.accel_y * test_data.accel_y +
                          test_data.accel_z * test_data.accel_z);

  // When stationary, magnitude should be close to 1g
  if (accel_mag < 0.5f || accel_mag > 1.5f) {
    ESP_LOGW(TAG, "Self-test warning: accel magnitude %.2fg (expected ~1g)",
             accel_mag);
  }

  ESP_LOGI(TAG,
           "Self-test passed: accel_mag=%.2fg, gyro=[%.1f, %.1f, %.1f] deg/s",
           accel_mag, test_data.gyro_x, test_data.gyro_y, test_data.gyro_z);

  return true;
}

void sensor_mpu6050_reset_attitude(void) {
  kalman_init(&sensor_state.kalman_x);
  kalman_init(&sensor_state.kalman_y);
  sensor_state.last_update_us = 0;
  ESP_LOGI(TAG, "Attitude filter reset");
}

void sensor_mpu6050_deinit(void) {
  if (mpu6050_handle != NULL) {
    // Put sensor in sleep mode to save power
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x40); // Set SLEEP bit

    i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
    if (i2c_bus != NULL) {
      i2c_master_bus_rm_device(mpu6050_handle);
    }
    mpu6050_handle = NULL;
    sensor_state.gyro_calibrated = false;
    ESP_LOGI(TAG, "MPU6050 deinitialized");
  }
}
