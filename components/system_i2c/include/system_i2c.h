/**
 * @file system_i2c.h
 * @brief Thread-safe I2C master bus abstraction for ESP-IDF
 * @details Provides unified I2C bus management with mutex protection
 *          for multi-sensor access (BME680, MPU6050, etc.)
 * 
 * Features:
 * - Thread-safe bus access via mutex
 * - Automatic sensor detection and address scanning
 * - Compatible with new ESP-IDF i2c_master API
 * - Lock/unlock mechanism for transaction batching
 */

#ifndef SYSTEM_I2C_H
#define SYSTEM_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize I2C master bus
 * @return true on success, false on failure
 */
bool system_i2c_init(void);

/**
 * @brief Get I2C master bus handle
 * @return I2C master bus handle or NULL if not initialized
 */
i2c_master_bus_handle_t system_i2c_get_bus_handle(void);

/**
 * @brief Deinitialize I2C master bus
 */
void system_i2c_deinit(void);

/**
 * @brief Scan common I2C addresses for sensors
 * @note Scans addresses commonly used by BME680, BME280, MPU6050, etc.
 *       Results are printed to log output.
 * @return Number of devices found
 */
uint8_t system_i2c_scan(void);

/**
 * @brief Acquire I2C bus mutex for exclusive access
 * @param timeout_ms Timeout in milliseconds (0 for default)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 * @note Use for transaction batching - ensures atomic multi-register operations
 */
esp_err_t system_i2c_lock(uint32_t timeout_ms);

/**
 * @brief Release I2C bus mutex
 * @return ESP_OK on success
 */
esp_err_t system_i2c_unlock(void);

/**
 * @brief Check if I2C bus is initialized
 * @return true if initialized, false otherwise
 */
bool system_i2c_is_initialized(void);

/**
 * @brief Add a device to the I2C bus
 * @param dev_addr 7-bit device address
 * @param scl_speed_hz I2C clock speed in Hz (typically 100000 or 400000)
 * @param[out] dev_handle Pointer to store device handle
 * @return ESP_OK on success
 */
esp_err_t system_i2c_add_device(uint8_t dev_addr, uint32_t scl_speed_hz,
                                 i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Remove a device from the I2C bus
 * @param dev_handle Device handle to remove
 * @return ESP_OK on success
 */
esp_err_t system_i2c_remove_device(i2c_master_dev_handle_t dev_handle);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_I2C_H

