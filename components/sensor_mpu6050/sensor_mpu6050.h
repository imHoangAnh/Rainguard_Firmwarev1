/**
 * @file sensor_mpu6050.h
 * @brief MPU6050 accelerometer and gyroscope sensor driver
 * @details Optimized driver with Kalman filtering, motion detection, 
 *          and configurable full-scale ranges
 * 
 * Features:
 * - 6-axis motion sensing (accel + gyro)
 * - Kalman filter for attitude estimation (pitch/roll)
 * - Motion detection with configurable threshold
 * - Temperature sensor support
 * - Self-test capability
 * - Low power mode support
 */

#ifndef SENSOR_MPU6050_H
#define SENSOR_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Accelerometer full-scale range
 */
typedef enum {
  MPU6050_ACCEL_RANGE_2G = 0,
  MPU6050_ACCEL_RANGE_4G = 1,
  MPU6050_ACCEL_RANGE_8G = 2,
  MPU6050_ACCEL_RANGE_16G = 3
} mpu6050_accel_range_t;

/**
 * @brief Gyroscope full-scale range
 */
typedef enum {
  MPU6050_GYRO_RANGE_250DPS = 0,
  MPU6050_GYRO_RANGE_500DPS = 1,
  MPU6050_GYRO_RANGE_1000DPS = 2,
  MPU6050_GYRO_RANGE_2000DPS = 3
} mpu6050_gyro_range_t;

/**
 * @brief Digital Low Pass Filter configuration
 */
typedef enum {
  MPU6050_DLPF_260HZ = 0,
  MPU6050_DLPF_184HZ = 1,
  MPU6050_DLPF_94HZ = 2,
  MPU6050_DLPF_44HZ = 3,
  MPU6050_DLPF_21HZ = 4,
  MPU6050_DLPF_10HZ = 5,
  MPU6050_DLPF_5HZ = 6
} mpu6050_dlpf_t;

/**
 * @brief MPU6050 sensor data structure
 */
typedef struct {
  float accel_x;        // Acceleration X-axis in g
  float accel_y;        // Acceleration Y-axis in g
  float accel_z;        // Acceleration Z-axis in g
  float gyro_x;         // Gyroscope X-axis in deg/s
  float gyro_y;         // Gyroscope Y-axis in deg/s
  float gyro_z;         // Gyroscope Z-axis in deg/s
  float temperature;    // Temperature in Celsius
  float pitch;          // Calculated pitch angle in degrees (Kalman filtered)
  float roll;           // Calculated roll angle in degrees (Kalman filtered)
  float yaw;            // Calculated yaw angle in degrees (integrated)
  bool motion_detected; // Motion detection flag
  uint32_t timestamp_us; // Measurement timestamp in microseconds
} mpu6050_data_t;

/**
 * @brief MPU6050 configuration structure
 */
typedef struct {
  mpu6050_accel_range_t accel_range;
  mpu6050_gyro_range_t gyro_range;
  mpu6050_dlpf_t dlpf;
  uint8_t sample_rate_div; // Sample rate = 1kHz / (1 + div)
} mpu6050_config_params_t;

/**
 * @brief Initialize MPU6050 sensor with default configuration
 * @return true on success, false on failure
 */
bool sensor_mpu6050_init(void);

/**
 * @brief Configure MPU6050 sensor parameters
 * @param config Configuration parameters
 * @return true on success, false on failure
 */
bool sensor_mpu6050_configure(const mpu6050_config_params_t *config);

/**
 * @brief Calibrate gyroscope (should be called when sensor is stationary)
 * @param samples Number of samples to average (recommended: 100-500)
 * @return true on success, false on failure
 */
bool sensor_mpu6050_calibrate_gyro(uint16_t samples);

/**
 * @brief Read sensor data with Kalman filtering
 * @param data Pointer to mpu6050_data_t structure to store readings
 * @return true on success, false on failure
 */
bool sensor_mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Read raw sensor data without filtering
 * @param accel_x Pointer to X acceleration (or NULL)
 * @param accel_y Pointer to Y acceleration (or NULL)
 * @param accel_z Pointer to Z acceleration (or NULL)
 * @param gyro_x Pointer to X gyro rate (or NULL)
 * @param gyro_y Pointer to Y gyro rate (or NULL)
 * @param gyro_z Pointer to Z gyro rate (or NULL)
 * @return true on success, false on failure
 */
bool sensor_mpu6050_read_raw(float *accel_x, float *accel_y, float *accel_z,
                              float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief Read temperature from internal sensor
 * @param temperature Pointer to store temperature in Celsius
 * @return true on success, false on failure
 */
bool sensor_mpu6050_read_temperature(float *temperature);

/**
 * @brief Set motion detection threshold
 * @param threshold Acceleration threshold in g (default: 0.2g)
 */
void sensor_mpu6050_set_motion_threshold(float threshold);

/**
 * @brief Enable/disable low power mode
 * @param enable true to enable low power mode
 * @return true on success, false on failure
 */
bool sensor_mpu6050_set_low_power(bool enable);

/**
 * @brief Perform sensor self-test
 * @return true if self-test passed, false otherwise
 */
bool sensor_mpu6050_self_test(void);

/**
 * @brief Reset Kalman filter and yaw integration
 */
void sensor_mpu6050_reset_attitude(void);

/**
 * @brief Deinitialize sensor
 */
void sensor_mpu6050_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_MPU6050_H
