/**
 * @file gps_neo7m.h
 * @brief u-blox NEO-7M GNSS Module Driver (Ultra-Lightweight) for ESP-IDF
 * @details Minimal driver for real-time GPS tracking
 *
 * Features:
 * - NMEA parsing (GGA, RMC, VTG, GSA)
 * - Position, speed, heading, time/date
 * - Optimized for sending data to backend
 *
 * @author TrainGuard Team
 * @version 3.0.0 (Ultra-Lightweight)
 * @date 2026-01-19
 */

#ifndef GPS_NEO7M_H
#define GPS_NEO7M_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CONFIGURATION
// =============================================================================

/** GPS read buffer size */
#define GPS_BUFFER_SIZE 512

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @brief GPS UTC time structure
 */
typedef struct {
  uint8_t hour;         /**< Hour (0-23) */
  uint8_t minute;       /**< Minute (0-59) */
  uint8_t second;       /**< Second (0-59) */
  uint16_t millisecond; /**< Millisecond (0-999) */
} gps_time_t;

/**
 * @brief GPS UTC date structure
 */
typedef struct {
  uint8_t day;   /**< Day of month (1-31) */
  uint8_t month; /**< Month (1-12) */
  uint16_t year; /**< Year (e.g., 2026) */
} gps_date_t;

/**
 * @brief Main GPS data structure for real-time tracking
 */
typedef struct {
  /* Position - Essential for tracking */
  double latitude;  /**< Latitude in degrees (positive = North) */
  double longitude; /**< Longitude in degrees (positive = East) */
  float altitude;   /**< Altitude above MSL in meters */

  /* Velocity - For tracking animation */
  float speed_kmh; /**< Speed over ground in km/h */
  float course;    /**< Course/heading in degrees (0-360) */

  /* Time and date - For timestamps */
  gps_time_t utc_time; /**< UTC time */
  gps_date_t utc_date; /**< UTC date */

  /* Status */
  bool valid;              /**< True if fix is valid */
  uint8_t satellites_used; /**< Number of satellites used */
  float hdop;              /**< Horizontal dilution of precision */
} gps_data_t;

// =============================================================================
// API FUNCTIONS
// =============================================================================

/**
 * @brief Initialize GPS module
 * @return true on success, false on failure
 */
bool gps_neo7m_init(void);

/**
 * @brief Deinitialize GPS module and release resources
 */
void gps_neo7m_deinit(void);

/**
 * @brief Read GPS data
 * @param data Pointer to GPS data structure to populate
 * @param timeout_ms Read timeout in milliseconds
 * @return true if data was received (check data->valid for fix status)
 */
bool gps_neo7m_read(gps_data_t *data, uint32_t timeout_ms);

/**
 * @brief Check if GPS has valid fix
 * @return true if valid fix is available
 */
bool gps_neo7m_has_fix(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_NEO7M_H
