/**
 * @file system_i2c.h
 * @brief Thread-safe I2C master bus abstraction
 */

#ifndef SYSTEM_I2C_H
#define SYSTEM_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

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

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_I2C_H

