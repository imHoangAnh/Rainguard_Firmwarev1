/**
 * @file sensor_bme680.h
 * @brief BME680 environmental sensor driver
 */

#ifndef SENSOR_BME680_H
#define SENSOR_BME680_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BME680 sensor data structure
 */
typedef struct {
    float temperature;    // Temperature in Celsius
    float humidity;       // Relative humidity in %
    float pressure;       // Pressure in hPa
    float iaq;            // Indoor Air Quality index
} bme680_data_t;

/**
 * @brief Initialize BME680 sensor
 * @return true on success, false on failure
 */
bool sensor_bme680_init(void);

/**
 * @brief Read sensor data
 * @param data Pointer to bme680_data_t structure to store readings
 * @return true on success, false on failure
 */
bool sensor_bme680_read(bme680_data_t *data);

/**
 * @brief Deinitialize sensor
 */
void sensor_bme680_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_BME680_H

