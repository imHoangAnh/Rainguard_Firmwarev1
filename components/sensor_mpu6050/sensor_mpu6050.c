/**
 * @file sensor_mpu6050.c
 * @brief MPU6050 accelerometer and gyroscope implementation
 */

#include "sensor_mpu6050.h"
#include "system_i2c.h"
#include "pin_config.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "sensor_mpu6050";
static i2c_master_dev_handle_t mpu6050_handle = NULL;

// MPU6050 Register Addresses
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C

#define MPU6050_WHO_AM_I_VALUE      0x68
#define MPU6050_PWR_MGMT_1_RESET    0x80
#define MPU6050_PWR_MGMT_1_WAKEUP   0x00

// Full scale ranges
#define MPU6050_ACCEL_FS_2G         0x00
#define MPU6050_GYRO_FS_250DPS      0x00

// Sensitivity scales (LSB/g or LSB/(deg/s))
#define MPU6050_ACCEL_SENS_2G       16384.0f
#define MPU6050_GYRO_SENS_250DPS    131.0f

static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    if (mpu6050_handle == NULL) return ESP_FAIL;
    
    return i2c_master_transmit_receive(
        mpu6050_handle,
        &reg, 1,
        data, len,
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    if (mpu6050_handle == NULL) return ESP_FAIL;
    
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(
        mpu6050_handle,
        buf, 2,
        pdMS_TO_TICKS(100)
    );
}

bool sensor_mpu6050_init(void)
{
    if (mpu6050_handle != NULL) {
        ESP_LOGW(TAG, "MPU6050 already initialized");
        return true;
    }

    i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return false;
    }

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

    // Check WHO_AM_I register
    uint8_t who_am_i = 0;
    mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: 0x%02X (expected 0x%02X)", who_am_i, MPU6050_WHO_AM_I_VALUE);
        i2c_master_bus_rm_device(mpu6050_handle);
        mpu6050_handle = NULL;
        return false;
    }

    // Reset device
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKEUP);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure accelerometer (±2g)
    mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G << 3);

    // Configure gyroscope (±250°/s)
    mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250DPS << 3);

    // Configure DLPF (Digital Low Pass Filter)
    mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03); // 44Hz bandwidth

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return true;
}

bool sensor_mpu6050_read(mpu6050_data_t *data)
{
    if (mpu6050_handle == NULL || data == NULL) {
        return false;
    }

    uint8_t raw_data[14];
    mpu6050_read_reg(MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);

    // Read accelerometer data (16-bit, big-endian)
    int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // Read gyroscope data (16-bit, big-endian)
    int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    // Convert to physical units
    data->accel_x = (float)accel_x_raw / MPU6050_ACCEL_SENS_2G;
    data->accel_y = (float)accel_y_raw / MPU6050_ACCEL_SENS_2G;
    data->accel_z = (float)accel_z_raw / MPU6050_ACCEL_SENS_2G;

    data->gyro_x = (float)gyro_x_raw / MPU6050_GYRO_SENS_250DPS;
    data->gyro_y = (float)gyro_y_raw / MPU6050_GYRO_SENS_250DPS;
    data->gyro_z = (float)gyro_z_raw / MPU6050_GYRO_SENS_250DPS;

    return true;
}

void sensor_mpu6050_deinit(void)
{
    if (mpu6050_handle != NULL) {
        i2c_master_bus_handle_t i2c_bus = system_i2c_get_bus_handle();
        if (i2c_bus != NULL) {
            i2c_master_bus_rm_device(mpu6050_handle);
        }
        mpu6050_handle = NULL;
        ESP_LOGI(TAG, "MPU6050 deinitialized");
    }
}

