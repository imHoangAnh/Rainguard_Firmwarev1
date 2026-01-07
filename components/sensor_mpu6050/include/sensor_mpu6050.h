/**
 * @file sensor_mpu6050.h
 * @brief MPU6050 accelerometer and gyroscope sensor driver
 */

#ifndef SENSOR_MPU6050_H
#define SENSOR_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
  float pitch;          // Calculated pitch angle in degrees (Kalman filtered)
  float roll;           // Calculated roll angle in degrees (Kalman filtered)
  float yaw;            // Calculated yaw angle in degrees (Kalman filtered)
  bool motion_detected; // Motion detection flag
} mpu6050_data_t;

/**
 * @brief Initialize MPU6050 sensor
 * @return true on success, false on failure
 */
bool sensor_mpu6050_init(void);

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
 * @brief Deinitialize sensor
 */
void sensor_mpu6050_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_MPU6050_H
