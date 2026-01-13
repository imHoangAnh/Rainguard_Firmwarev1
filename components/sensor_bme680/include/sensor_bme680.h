/**
 * @file sensor_bme680.h
 * @brief BME680/BME688 environmental sensor driver with Bosch BSEC support
 * @details Optimized driver using official Bosch BME68x API
 * 
 * Features:
 * - Full T/P/H/G measurement support
 * - IAQ (Indoor Air Quality) calculation
 * - Low power mode support
 * - Self-test capability
 * - Compatible with both BME680 and BME688
 */

#ifndef SENSOR_BME680_H
#define SENSOR_BME680_H

#include <stdbool.h>
#include <stdint.h>

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
  float gas_resistance; // Gas resistance in Ohms
  float iaq;            // Indoor Air Quality index (0-500)
  uint8_t iaq_accuracy; // IAQ accuracy (0-3: 0=stabilizing, 3=high accuracy)
  bool gas_valid;       // Gas measurement validity
  bool heat_stable;     // Heater stability status
} bme680_data_t;

/**
 * @brief BME680 power mode
 */
typedef enum {
  BME680_SLEEP_MODE = 0x00,
  BME680_FORCED_MODE = 0x01,
  BME680_NORMAL_MODE = 0x03
} bme680_power_mode_t;

/**
 * @brief BME680 oversampling configuration
 */
typedef enum {
  BME680_OS_NONE = 0x00,
  BME680_OS_1X = 0x01,
  BME680_OS_2X = 0x02,
  BME680_OS_4X = 0x03,
  BME680_OS_8X = 0x04,
  BME680_OS_16X = 0x05
} bme680_oversampling_t;

/**
 * @brief BME680 IIR filter coefficient
 */
typedef enum {
  BME680_FILTER_OFF = 0,
  BME680_FILTER_SIZE_1 = 1,
  BME680_FILTER_SIZE_3 = 2,
  BME680_FILTER_SIZE_7 = 3,
  BME680_FILTER_SIZE_15 = 4,
  BME680_FILTER_SIZE_31 = 5,
  BME680_FILTER_SIZE_63 = 6,
  BME680_FILTER_SIZE_127 = 7
} bme680_filter_t;

/**
 * @brief Initialize BME680 sensor
 * @return true on success, false on failure
 */
bool sensor_bme680_init(void);

/**
 * @brief Configure BME680 sensor parameters
 * @param temp_os Temperature oversampling
 * @param press_os Pressure oversampling
 * @param hum_os Humidity oversampling
 * @param filter Filter coefficient (0-7, higher = more filtering)
 * @param gas_wait_ms Gas sensor wait time in ms (0-4095ms)
 * @param gas_heater_temp Gas heater temperature in Celsius (200-400°C)
 * @return true on success, false on failure
 */
bool sensor_bme680_configure(bme680_oversampling_t temp_os,
                             bme680_oversampling_t press_os,
                             bme680_oversampling_t hum_os, uint8_t filter,
                             uint16_t gas_wait_ms, uint16_t gas_heater_temp);

/**
 * @brief Read sensor data (optimized with gas sensor support)
 * @param data Pointer to bme680_data_t structure to store readings
 * @return true on success, false on failure
 */
bool sensor_bme680_read(bme680_data_t *data);

/**
 * @brief Set power mode
 * @param mode Power mode (SLEEP, FORCED, NORMAL)
 * @return true on success, false on failure
 */
bool sensor_bme680_set_power_mode(bme680_power_mode_t mode);

/**
 * @brief Perform sensor self-test
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
 * @brief Deinitialize sensor
 */
void sensor_bme680_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_BME680_H
