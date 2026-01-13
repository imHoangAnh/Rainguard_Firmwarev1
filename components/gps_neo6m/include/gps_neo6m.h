/**
 * @file gps_neo6m.h
 * @brief NEO-6M/7M GPS module NMEA parser and UBX configuration
 * @details Optimized NMEA parsing with UBX protocol support
 *
 * Features:
 * - High-performance NMEA parsing (no dynamic allocation)
 * - UBX protocol configuration support
 * - Power management modes
 * - Navigation mode configuration
 * - Fix quality and DOP reporting
 */

#ifndef GPS_NEO6M_H
#define GPS_NEO6M_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS fix type
 */
typedef enum {
  GPS_FIX_NONE = 0,
  GPS_FIX_2D = 2,
  GPS_FIX_3D = 3
} gps_fix_type_t;

/**
 * @brief GPS UTC time structure
 */
typedef struct {
  uint8_t hour;         // Hour (0-23)
  uint8_t minute;       // Minute (0-59)
  uint8_t second;       // Second (0-59)
  uint16_t millisecond; // Millisecond (0-999)
} gps_time_t;

/**
 * @brief GPS UTC date structure
 */
typedef struct {
  uint8_t day;   // Day (1-31)
  uint8_t month; // Month (1-12)
  uint16_t year; // Year (e.g., 2026)
} gps_date_t;

/**
 * @brief GPS data structure
 */
typedef struct {
  float latitude;          // Latitude in degrees (positive = North)
  float longitude;         // Longitude in degrees (positive = East)
  float altitude;          // Altitude in meters (above mean sea level)
  float speed;             // Speed over ground in km/h
  float course;            // Course over ground in degrees
  uint8_t satellites;      // Number of satellites used in fix
  float hdop;              // Horizontal dilution of precision
  float vdop;              // Vertical dilution of precision (from GSA)
  float pdop;              // Position dilution of precision (from GSA)
  bool valid;              // True if GPS fix is valid
  gps_fix_type_t fix_type; // Fix type (none/2D/3D)
  uint32_t fix_time;       // UTC time (HHMMSS) - legacy format
  uint32_t fix_date;       // UTC date (DDMMYY) - legacy format
  gps_time_t utc_time;     // Parsed UTC time
  gps_date_t utc_date;     // Parsed UTC date
  float geoid_sep;         // Geoid separation in meters
} gps_data_t;

/**
 * @brief GPS navigation mode (for UBX configuration)
 */
typedef enum {
  GPS_NAV_MODE_PORTABLE = 0,
  GPS_NAV_MODE_STATIONARY = 2,
  GPS_NAV_MODE_PEDESTRIAN = 3,
  GPS_NAV_MODE_AUTOMOTIVE = 4,
  GPS_NAV_MODE_SEA = 5,
  GPS_NAV_MODE_AIRBORNE = 6
} gps_nav_mode_t;

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
 * @brief Configure navigation update rate
 * @param rate_hz Update rate in Hz (1-10 for NEO-6M, max 5 recommended)
 * @return true on success, false on failure
 */
bool gps_neo6m_set_update_rate(uint8_t rate_hz);

/**
 * @brief Configure navigation mode
 * @param mode Navigation mode optimized for use case
 * @return true on success, false on failure
 */
bool gps_neo6m_set_nav_mode(gps_nav_mode_t mode);

/**
 * @brief Perform cold start (clear all GPS data)
 * @return true on success, false on failure
 */
bool gps_neo6m_cold_start(void);

/**
 * @brief Send UBX command to GPS module (for advanced configuration)
 * @param cmd Command bytes
 * @param cmd_len Command length
 * @return true on success, false on failure
 */
bool gps_neo6m_send_ubx_command(const uint8_t *cmd, size_t cmd_len);

/**
 * @brief Check if GPS has valid fix
 * @return true if valid fix, false otherwise
 */
bool gps_neo6m_has_fix(void);

/**
 * @brief Get time since last valid fix in milliseconds
 * @return Time in ms, or UINT32_MAX if never had fix
 */
uint32_t gps_neo6m_time_since_fix(void);

/**
 * @brief Deinitialize GPS module
 */
void gps_neo6m_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_NEO6M_H
