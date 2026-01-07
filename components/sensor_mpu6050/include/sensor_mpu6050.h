/**
 * @file sensor_mpu6050.h
 * @brief MPU6050 accelerometer and gyroscope sensor driver
 */

#ifndef SENSOR_MPU6050_H
#define SENSOR_MPU6050_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MPU6050 sensor data structure
 */
typedef struct {
    float accel_x;  // Acceleration X-axis in g
    float accel_y;  // Acceleration Y-axis in g
    float accel_z;  // Acceleration Z-axis in g
    float gyro_x;   // Gyroscope X-axis in deg/s
    float gyro_y;   // Gyroscope Y-axis in deg/s
    float gyro_z;   // Gyroscope Z-axis in deg/s
} mpu6050_data_t;

/**
 * @brief Initialize MPU6050 sensor
 * @return true on success, false on failure
 */
bool sensor_mpu6050_init(void);

/**
 * @brief Read sensor data
 * @param data Pointer to mpu6050_data_t structure to store readings
 * @return true on success, false on failure
 */
bool sensor_mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Deinitialize sensor
 */
void sensor_mpu6050_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_MPU6050_H

