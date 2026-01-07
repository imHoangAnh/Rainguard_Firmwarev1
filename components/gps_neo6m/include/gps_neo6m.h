/**
 * @file gps_neo6m.h
 * @brief NEO-6M GPS module NMEA parser
 */

#ifndef GPS_NEO6M_H
#define GPS_NEO6M_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS data structure
 */
typedef struct {
    float latitude;   // Latitude in degrees
    float longitude;  // Longitude in degrees
    float speed;      // Speed in km/h
    bool valid;       // True if GPS fix is valid
} gps_data_t;

/**
 * @brief Initialize GPS module (UART)
 * @return true on success, false on failure
 */
bool gps_neo6m_init(void);

/**
 * @brief Read and parse GPS data from NMEA sentences
 * @param data Pointer to gps_data_t structure to store readings
 * @return true on success, false on failure
 */
bool gps_neo6m_read(gps_data_t *data);

/**
 * @brief Deinitialize GPS module
 */
void gps_neo6m_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_NEO6M_H

