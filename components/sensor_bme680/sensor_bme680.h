/**
 * @file sensor_bme680.h
 * @brief BME680/BME688 environmental sensor driver using official Bosch BME68x
 * API
 * @details Wrapper driver utilizing Bosch Sensortec BME68x sensor API
 *
 * Features:
 * - Full T/P/H/G measurement support via official Bosch API
 * - IAQ (Indoor Air Quality) calculation
 * - Low power mode support
 * - Self-test capability
 * - Compatible with both BME680 and BME688
 */

#ifndef SENSOR_BME680_H
#define SENSOR_BME680_H

#include "bme68x.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default I2C address (can be 0x76 or 0x77 depending on SDO pin)
 */
#ifndef BME680_I2C_ADDR
#define BME680_I2C_ADDR BME68X_I2C_ADDR_HIGH // 0x77
#endif

/**
 * @brief BME680 sensor data structure
 */
typedef struct {
  float temperature;    // Temperature in Celsius
  float humidity;       // Relative humidity in %
  float pressure;       // Pressure in hPa
  float gas_resistance; // Gas resistance in Ohms
  float iaq;            // Indoor Air Quality index (0-500)
  uint8_t iaq_accuracy; // IAQ accuracy (0-3: 0=stabilizing, 3=high accuracy)
  bool gas_valid;       // Gas measurement validity
  bool heat_stable;     // Heater stability status
} bme680_data_t;

/**
 * @brief BME680 oversampling configuration (maps to BME68x API)
 */
typedef enum {
  BME680_OS_NONE = BME68X_OS_NONE,
  BME680_OS_1X = BME68X_OS_1X,
  BME680_OS_2X = BME68X_OS_2X,
  BME680_OS_4X = BME68X_OS_4X,
  BME680_OS_8X = BME68X_OS_8X,
  BME680_OS_16X = BME68X_OS_16X
} bme680_oversampling_t;

/**
 * @brief BME680 IIR filter coefficient (maps to BME68x API)
 */
typedef enum {
  BME680_FILTER_OFF = BME68X_FILTER_OFF,
  BME680_FILTER_SIZE_1 = BME68X_FILTER_SIZE_1,
  BME680_FILTER_SIZE_3 = BME68X_FILTER_SIZE_3,
  BME680_FILTER_SIZE_7 = BME68X_FILTER_SIZE_7,
  BME680_FILTER_SIZE_15 = BME68X_FILTER_SIZE_15,
  BME680_FILTER_SIZE_31 = BME68X_FILTER_SIZE_31,
  BME680_FILTER_SIZE_63 = BME68X_FILTER_SIZE_63,
  BME680_FILTER_SIZE_127 = BME68X_FILTER_SIZE_127
} bme680_filter_t;

/**
 * @brief BME680 power mode (maps to BME68x API)
 */
typedef enum {
  BME680_SLEEP_MODE = BME68X_SLEEP_MODE,
  BME680_FORCED_MODE = BME68X_FORCED_MODE
} bme680_power_mode_t;

/**
 * @brief Initialize BME680 sensor using Bosch BME68x API
 * @return true on success, false on failure
 */
bool sensor_bme680_init(void);

/**
 * @brief Configure BME680 sensor parameters
 * @param temp_os Temperature oversampling
 * @param press_os Pressure oversampling
 * @param hum_os Humidity oversampling
 * @param filter Filter coefficient
 * @param gas_wait_ms Gas sensor wait time in ms
 * @param gas_heater_temp Gas heater temperature in Celsius (200-400°C)
 * @return true on success, false on failure
 */
bool sensor_bme680_configure(bme680_oversampling_t temp_os,
                             bme680_oversampling_t press_os,
                             bme680_oversampling_t hum_os, uint8_t filter,
                             uint16_t gas_wait_ms, uint16_t gas_heater_temp);

/**
 * @brief Read sensor data using Bosch BME68x API
 * @param data Pointer to bme680_data_t structure to store readings
 * @return true on success, false on failure
 */
bool sensor_bme680_read(bme680_data_t *data);

/**
 * @brief Set power mode
 * @param mode Power mode (SLEEP or FORCED)
 * @return true on success, false on failure
 */
bool sensor_bme680_set_power_mode(bme680_power_mode_t mode);

/**
 * @brief Perform sensor self-test using Bosch API
 * @return true if self-test passed, false otherwise
 */
bool sensor_bme680_self_test(void);

/**
 * @brief Reset IAQ baseline for recalibration
 */
void sensor_bme680_reset_iaq_baseline(void);

/**
 * @brief Get sensor status
 * @param[out] gas_valid True if gas measurement is valid
 * @param[out] heat_stable True if heater is stable
 * @return true on success
 */
bool sensor_bme680_get_status(bool *gas_valid, bool *heat_stable);

/**
 * @brief Get BME68x device structure (for advanced usage)
 * @return Pointer to bme68x_dev structure, or NULL if not initialized
 */
struct bme68x_dev *sensor_bme680_get_dev(void);

/**
 * @brief Deinitialize sensor
 */
void sensor_bme680_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_BME680_H
