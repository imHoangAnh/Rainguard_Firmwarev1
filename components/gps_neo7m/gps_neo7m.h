/**
 * @file gps_neo7m.h
 * @brief u-blox NEO-7M GNSS Module Driver for ESP-IDF 5.5.2
 * @details Advanced GNSS driver supporting GPS, GLONASS, QZSS with UBX protocol
 *
 * Features:
 * - High-performance NMEA parsing (GGA, RMC, VTG, GSA, GSV)
 * - UBX protocol configuration support
 * - Multi-GNSS support (GPS + GLONASS)
 * - AssistNow Autonomous for faster TTFF
 * - Power management modes (Continuous, Power Save, Backup)
 * - Navigation mode configuration
 * - Fix quality and full DOP reporting (HDOP, VDOP, PDOP)
 * - Satellite information tracking
 *
 * Based on u-blox NEO-7 Data Sheet (UBX-13003830)
 * Default UART: 9600 Baud, 8N1
 *
 * @author RainGuard Team
 * @version 1.0.0
 * @date 2026-01-17
 */

#ifndef GPS_NEO7M_H
#define GPS_NEO7M_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CONFIGURATION CONSTANTS
// =============================================================================

/** Maximum number of satellites to track */
#define GPS_MAX_SATELLITES 32

/** Maximum NMEA sentence length per NMEA-0183 spec */
#define GPS_NMEA_MAX_LEN 82

/** GPS read buffer size */
#define GPS_BUFFER_SIZE 1024

// =============================================================================
// ENUMERATIONS
// =============================================================================

/**
 * @brief GPS fix type enumeration
 */
typedef enum {
  GPS_FIX_NONE = 0,           /**< No fix available */
  GPS_FIX_DEAD_RECKONING = 1, /**< Dead reckoning only */
  GPS_FIX_2D = 2,             /**< 2D fix (no altitude) */
  GPS_FIX_3D = 3,             /**< 3D fix (with altitude) */
  GPS_FIX_GPS_DR = 4,         /**< GPS + Dead reckoning combined */
  GPS_FIX_TIME_ONLY = 5       /**< Time only fix */
} gps_fix_type_t;

/**
 * @brief GPS fix quality (from GGA sentence)
 */
typedef enum {
  GPS_QUALITY_INVALID = 0,   /**< Invalid/No fix */
  GPS_QUALITY_GPS = 1,       /**< GPS fix (SPS) */
  GPS_QUALITY_DGPS = 2,      /**< DGPS fix */
  GPS_QUALITY_PPS = 3,       /**< PPS fix */
  GPS_QUALITY_RTK_FIXED = 4, /**< Real Time Kinematic, fixed */
  GPS_QUALITY_RTK_FLOAT = 5, /**< Real Time Kinematic, float */
  GPS_QUALITY_ESTIMATED = 6, /**< Estimated (dead reckoning) */
  GPS_QUALITY_MANUAL = 7,    /**< Manual input mode */
  GPS_QUALITY_SIMULATION = 8 /**< Simulation mode */
} gps_fix_quality_t;

/**
 * @brief GNSS system type
 */
typedef enum {
  GNSS_GPS = 0,     /**< GPS (USA) */
  GNSS_SBAS = 1,    /**< SBAS */
  GNSS_GALILEO = 2, /**< Galileo (EU) */
  GNSS_BEIDOU = 3,  /**< BeiDou (China) */
  GNSS_IMES = 4,    /**< IMES (Japan) */
  GNSS_QZSS = 5,    /**< QZSS (Japan) */
  GNSS_GLONASS = 6  /**< GLONASS (Russia) */
} gnss_system_t;

/**
 * @brief GPS navigation mode (dynamic platform model)
 * @details Configures the receiver for optimal performance based on use case
 */
typedef enum {
  GPS_NAV_MODE_PORTABLE = 0,    /**< General purpose, low dynamics */
  GPS_NAV_MODE_STATIONARY = 2,  /**< Fixed position, timing applications */
  GPS_NAV_MODE_PEDESTRIAN = 3,  /**< Walking, running (<10 m/s) */
  GPS_NAV_MODE_AUTOMOTIVE = 4,  /**< Car (<50 m/s) */
  GPS_NAV_MODE_SEA = 5,         /**< Marine applications (<25 m/s) */
  GPS_NAV_MODE_AIRBORNE_1G = 6, /**< Airborne <1g acceleration */
  GPS_NAV_MODE_AIRBORNE_2G = 7, /**< Airborne <2g acceleration */
  GPS_NAV_MODE_AIRBORNE_4G = 8  /**< Airborne <4g acceleration */
} gps_nav_mode_t;

/**
 * @brief GPS power mode
 */
typedef enum {
  GPS_POWER_FULL = 0,  /**< Full power, continuous operation */
  GPS_POWER_SAVE = 1,  /**< Power save mode (cyclic tracking) */
  GPS_POWER_BACKUP = 2 /**< Backup mode (RTC only, wake via EXTINT) */
} gps_power_mode_t;

/**
 * @brief GPS message rate configuration
 */
typedef enum {
  GPS_MSG_DISABLE = 0, /**< Disable message */
  GPS_MSG_1HZ = 1,     /**< 1 Hz output */
  GPS_MSG_2HZ = 2,     /**< Every 2nd navigation solution */
  GPS_MSG_5HZ = 5,     /**< Every 5th navigation solution */
  GPS_MSG_10HZ = 10    /**< Every 10th navigation solution */
} gps_msg_rate_t;

/**
 * @brief SBAS mode
 */
typedef enum {
  SBAS_MODE_DISABLED = 0, /**< SBAS disabled */
  SBAS_MODE_ENABLED = 1,  /**< SBAS enabled (integrity only) */
  SBAS_MODE_TEST = 2      /**< SBAS test mode */
} sbas_mode_t;

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
 * @brief Satellite information structure
 */
typedef struct {
  uint8_t prn;          /**< Satellite PRN number */
  uint8_t elevation;    /**< Elevation angle in degrees (0-90) */
  uint16_t azimuth;     /**< Azimuth angle in degrees (0-360) */
  uint8_t snr;          /**< Signal-to-noise ratio in dB-Hz (0-99) */
  gnss_system_t system; /**< GNSS system identifier */
  bool used;            /**< True if satellite is used in fix */
} gps_satellite_t;

/**
 * @brief Dilution of Precision (DOP) structure
 */
typedef struct {
  float pdop; /**< Position DOP (3D) */
  float hdop; /**< Horizontal DOP */
  float vdop; /**< Vertical DOP */
  float tdop; /**< Time DOP */
  float gdop; /**< Geometric DOP */
} gps_dop_t;

/**
 * @brief Main GPS data structure
 */
typedef struct {
  /* Position data */
  double latitude;        /**< Latitude in degrees (positive = North) */
  double longitude;       /**< Longitude in degrees (positive = East) */
  float altitude;         /**< Altitude above MSL in meters */
  float geoid_separation; /**< Geoid separation in meters */

  /* Velocity data */
  float speed_kmh;       /**< Speed over ground in km/h */
  float speed_knots;     /**< Speed over ground in knots */
  float course;          /**< Course over ground in degrees (true) */
  float course_magnetic; /**< Course over ground in degrees (magnetic) */

  /* Time and date */
  gps_time_t utc_time;   /**< UTC time */
  gps_date_t utc_date;   /**< UTC date */
  uint32_t timestamp_ms; /**< System timestamp when data was received */

  /* Fix status */
  bool valid;                /**< True if fix is valid */
  gps_fix_type_t fix_type;   /**< Fix type (2D/3D) */
  gps_fix_quality_t quality; /**< Fix quality from GGA */

  /* Satellite information */
  uint8_t satellites_used; /**< Number of satellites used in fix */
  uint8_t satellites_view; /**< Number of satellites in view */
  gps_satellite_t satellites[GPS_MAX_SATELLITES]; /**< Satellite details */

  /* Accuracy indicators */
  gps_dop_t dop;    /**< Dilution of precision values */
  float accuracy_h; /**< Horizontal accuracy estimate (meters) */
  float accuracy_v; /**< Vertical accuracy estimate (meters) */

  /* Mode indicators */
  char mode_indicator;        /**< NMEA mode indicator (A/D/E/M/S/N) */
  gnss_system_t primary_gnss; /**< Primary GNSS system in use */
} gps_data_t;

/**
 * @brief GPS configuration structure
 */
typedef struct {
  uint8_t update_rate_hz;      /**< Navigation update rate (1-10 Hz) */
  gps_nav_mode_t nav_mode;     /**< Navigation mode */
  gps_power_mode_t power_mode; /**< Power mode */
  bool enable_glonass;         /**< Enable GLONASS (in addition to GPS) */
  bool enable_sbas;            /**< Enable SBAS augmentation */
  bool enable_qzss;            /**< Enable QZSS (Japan region) */
  bool assistnow_autonomous;   /**< Enable AssistNow Autonomous */
  uint8_t min_svs;             /**< Minimum satellites for navigation */
  uint8_t min_cno;             /**< Minimum C/N0 for navigation (dB-Hz) */
  uint16_t static_hold_thresh; /**< Static hold threshold (cm/s) */
} gps_config_t;

/**
 * @brief GPS statistics structure
 */
typedef struct {
  uint32_t total_reads;       /**< Total read attempts */
  uint32_t successful_reads;  /**< Successful data reads */
  uint32_t valid_fixes;       /**< Valid fix count */
  uint32_t checksum_errors;   /**< NMEA checksum errors */
  uint32_t parse_errors;      /**< NMEA parse errors */
  uint32_t last_fix_time_ms;  /**< Timestamp of last valid fix */
  uint32_t last_data_time_ms; /**< Timestamp of last data received */
  uint32_t ttff_ms;           /**< Time to first fix (ms) */
} gps_stats_t;

// =============================================================================
// FUNCTION PROTOTYPES - INITIALIZATION
// =============================================================================

/**
 * @brief Initialize GPS module with default configuration
 * @return true on success, false on failure
 */
bool gps_neo7m_init(void);

/**
 * @brief Initialize GPS module with custom configuration
 * @param config Pointer to configuration structure
 * @return true on success, false on failure
 */
bool gps_neo7m_init_with_config(const gps_config_t *config);

/**
 * @brief Deinitialize GPS module and release resources
 */
void gps_neo7m_deinit(void);

/**
 * @brief Check if GPS module is initialized
 * @return true if initialized
 */
bool gps_neo7m_is_initialized(void);

// =============================================================================
// FUNCTION PROTOTYPES - DATA READING
// =============================================================================

/**
 * @brief Read and parse GPS data from NMEA sentences
 * @param data Pointer to GPS data structure to populate
 * @param timeout_ms Read timeout in milliseconds
 * @return true if valid data was received
 */
bool gps_neo7m_read(gps_data_t *data, uint32_t timeout_ms);

/**
 * @brief Check if GPS has valid fix
 * @return true if valid fix is available
 */
bool gps_neo7m_has_fix(void);

/**
 * @brief Get time since last valid fix
 * @return Time in milliseconds, or UINT32_MAX if never had fix
 */
uint32_t gps_neo7m_time_since_fix(void);

/**
 * @brief Get current GPS fix type
 * @return Current fix type
 */
gps_fix_type_t gps_neo7m_get_fix_type(void);

/**
 * @brief Get GPS statistics
 * @param stats Pointer to statistics structure to populate
 */
void gps_neo7m_get_stats(gps_stats_t *stats);

// =============================================================================
// FUNCTION PROTOTYPES - CONFIGURATION
// =============================================================================

/**
 * @brief Configure GPS update rate
 * @param rate_hz Update rate in Hz (1-10, recommended 1-5)
 * @return true on success
 */
bool gps_neo7m_set_update_rate(uint8_t rate_hz);

/**
 * @brief Configure GPS navigation mode
 * @param mode Navigation mode for specific application
 * @return true on success
 */
bool gps_neo7m_set_nav_mode(gps_nav_mode_t mode);

/**
 * @brief Configure GPS power mode
 * @param mode Power mode setting
 * @return true on success
 */
bool gps_neo7m_set_power_mode(gps_power_mode_t mode);

/**
 * @brief Enable/disable GLONASS alongside GPS
 * @param enable true to enable GLONASS
 * @return true on success
 */
bool gps_neo7m_set_glonass(bool enable);

/**
 * @brief Enable/disable SBAS (WAAS/EGNOS/MSAS/GAGAN)
 * @param mode SBAS mode
 * @return true on success
 */
bool gps_neo7m_set_sbas(sbas_mode_t mode);

/**
 * @brief Enable/disable AssistNow Autonomous
 * @param enable true to enable
 * @return true on success
 */
bool gps_neo7m_set_assistnow_autonomous(bool enable);

/**
 * @brief Configure NMEA message output rate
 * @param msg_id NMEA message ID (e.g., "GGA", "RMC", "VTG")
 * @param rate Message rate setting
 * @return true on success
 */
bool gps_neo7m_set_nmea_rate(const char *msg_id, gps_msg_rate_t rate);

/**
 * @brief Save current configuration to flash (NEO-7N only)
 * @return true on success
 */
bool gps_neo7m_save_config(void);

/**
 * @brief Load default configuration
 * @return true on success
 */
bool gps_neo7m_load_defaults(void);

// =============================================================================
// FUNCTION PROTOTYPES - CONTROL
// =============================================================================

/**
 * @brief Perform hot start (use all available data)
 * @return true on success
 */
bool gps_neo7m_hot_start(void);

/**
 * @brief Perform warm start (clear ephemeris)
 * @return true on success
 */
bool gps_neo7m_warm_start(void);

/**
 * @brief Perform cold start (clear all navigation data)
 * @return true on success
 */
bool gps_neo7m_cold_start(void);

/**
 * @brief Reset GPS module (hardware reset via UBX command)
 * @return true on success
 */
bool gps_neo7m_reset(void);

// =============================================================================
// FUNCTION PROTOTYPES - LOW LEVEL
// =============================================================================

/**
 * @brief Send raw UBX command to GPS module
 * @param cmd Command buffer
 * @param cmd_len Command length
 * @return true on success
 */
bool gps_neo7m_send_ubx(const uint8_t *cmd, size_t cmd_len);

/**
 * @brief Send UBX command and wait for ACK
 * @param class_id UBX message class
 * @param msg_id UBX message ID
 * @param payload Payload data
 * @param payload_len Payload length
 * @param timeout_ms Timeout for ACK
 * @return true if ACK received
 */
bool gps_neo7m_send_ubx_with_ack(uint8_t class_id, uint8_t msg_id,
                                 const uint8_t *payload, size_t payload_len,
                                 uint32_t timeout_ms);

/**
 * @brief Send raw NMEA command to GPS module
 * @param cmd NMEA command string (without $ and checksum)
 * @return true on success
 */
bool gps_neo7m_send_nmea(const char *cmd);

// =============================================================================
// FUNCTION PROTOTYPES - DEBUG
// =============================================================================

/**
 * @brief Print GPS module status for debugging
 */
void gps_neo7m_debug_status(void);

/**
 * @brief Print satellite information
 */
void gps_neo7m_debug_satellites(void);

/**
 * @brief Get default configuration
 * @param config Pointer to configuration structure to populate
 */
void gps_neo7m_get_default_config(gps_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // GPS_NEO7M_H
