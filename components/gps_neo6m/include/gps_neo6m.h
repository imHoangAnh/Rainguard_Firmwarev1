/**
 * @file gps_neo6m.h
 * @brief NEO-77M GPS module NMEA parser
 */

#ifndef GPS_NEO7M_H
#define GPS_NEO7M_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS data structure
 */
typedef struct {
  float latitude;     // Latitude in degrees
  float longitude;    // Longitude in degrees
  float altitude;     // Altitude in meters
  float speed;        // Speed in km/h
  float course;       // Course over ground in degrees
  uint8_t satellites; // Number of satellites in view
  float hdop;         // Horizontal dilution of precision
  bool valid;         // True if GPS fix is valid
  uint32_t fix_time;  // UTC time (HHMMSS)
  uint32_t fix_date;  // UTC date (DDMMYY)
} gps_data_t;

/**
 * @brief Initialize GPS module (UART)
 * @return true on success, false on failure
 */
bool gps_neo6m_init(void);

/**
 * @brief Read and parse GPS data from NMEA sentences (optimized parsing)
 * @param data Pointer to gps_data_t structure to store readings
 * @param timeout_ms Timeout in milliseconds
 * @return true on success, false on failure
 */
bool gps_neo6m_read(gps_data_t *data, uint32_t timeout_ms);

/**
 * @brief Configure GPS power mode
 * @param enable true to enable GPS, false to put in low power mode
 * @return true on success, false on failure
 */
bool gps_neo6m_set_power_mode(bool enable);

/**
 * @brief Send UBX command to GPS module (for advanced configuration)
 * @param cmd Command bytes
 * @param cmd_len Command length
 * @return true on success, false on failure
 */
bool gps_neo6m_send_ubx_command(const uint8_t *cmd, size_t cmd_len);

/**
 * @brief Deinitialize GPS module
 */
void gps_neo6m_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_NEO6M_H
