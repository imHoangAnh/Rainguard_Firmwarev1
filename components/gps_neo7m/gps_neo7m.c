/**
 * @file gps_neo7m.c
 * @brief u-blox NEO-7M GNSS Module Driver Implementation for ESP-IDF 5.5.2
 * @details Complete GNSS driver with NMEA parsing and UBX configuration
 *
 * Supported NMEA Sentences:
 * - GGA: GPS Fix Data (position, altitude, satellites, HDOP)
 * - RMC: Recommended Minimum (position, speed, course, date/time, validity)
 * - VTG: Course Over Ground and Ground Speed
 * - GSA: GNSS DOP and Active Satellites
 * - GSV: GNSS Satellites in View
 *
 * NMEA Talker ID Support:
 * - GP: GPS only
 * - GL: GLONASS only
 * - GN: Multi-GNSS (GPS+GLONASS combined)
 * - GA: Galileo
 * - BD: BeiDou
 *
 * Based on:
 * - u-blox NEO-7 Data Sheet (UBX-13003830)
 * - u-blox 7 Receiver Description (GPS.G7-SW-12001)
 *
 * @author RainGuard Team
 * @version 1.0.0
 * @date 2026-01-17
 */

#include "gps_neo7m.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pin_config.h"
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// =============================================================================
// CONSTANTS AND MACROS
// =============================================================================

static const char *TAG = "gps_neo7m";

/** UBX Protocol sync bytes */
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/** UBX Message Classes */
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_AID 0x0B
#define UBX_CLASS_TIM 0x0D

/** UBX CFG Message IDs */
#define UBX_CFG_PRT 0x00
#define UBX_CFG_MSG 0x01
#define UBX_CFG_RST 0x04
#define UBX_CFG_RATE 0x08
#define UBX_CFG_CFG 0x09
#define UBX_CFG_RXM 0x11
#define UBX_CFG_SBAS 0x16
#define UBX_CFG_NAV5 0x24
#define UBX_CFG_NAVX5 0x23
#define UBX_CFG_GNSS 0x3E
#define UBX_CFG_PM2 0x3B

/** UBX ACK Message IDs */
#define UBX_ACK_NAK 0x00
#define UBX_ACK_ACK 0x01

/** NMEA parsing constants */
#define NMEA_MAX_FIELDS 24
#define NMEA_SENTENCE_MAX_LEN 82

/** Timeout constants */
#define GPS_ACK_TIMEOUT_MS 1000
#define GPS_NO_DATA_TIMEOUT_MS 10000

/** Knot to km/h conversion factor */
#define KNOTS_TO_KMH 1.852f

// =============================================================================
// MODULE STATE
// =============================================================================

static bool gps_initialized = false;
static gps_config_t current_config;
static gps_stats_t gps_stats = {0};
static gps_data_t last_gps_data = {0};
static SemaphoreHandle_t gps_mutex = NULL;

/** Track fix state */
static bool has_valid_fix = false;
static uint32_t first_fix_time_ms = 0;
static uint32_t init_time_ms = 0;

/** Read buffer */
static uint8_t uart_buffer[GPS_BUFFER_SIZE];

// =============================================================================
// NMEA PARSING UTILITIES
// =============================================================================

/**
 * @brief Calculate NMEA checksum (XOR of all chars between '$' and '*')
 */
static uint8_t nmea_calculate_checksum(const char *sentence) {
  uint8_t checksum = 0;
  const char *p = sentence;

  if (*p == '$')
    p++;

  while (*p != '\0' && *p != '*') {
    checksum ^= *p++;
  }

  return checksum;
}

/**
 * @brief Validate NMEA sentence checksum
 */
static bool nmea_validate_checksum(const char *sentence) {
  const char *star = strchr(sentence, '*');
  if (star == NULL || strlen(star) < 3) {
    return false;
  }

  char hex[3] = {star[1], star[2], '\0'};
  unsigned int expected = 0;
  if (sscanf(hex, "%02X", &expected) != 1) {
    return false;
  }

  return (nmea_calculate_checksum(sentence) == (uint8_t)expected);
}

/**
 * @brief Convert NMEA coordinate to decimal degrees
 * @details NMEA format: DDDMM.MMMMM (longitude) or DDMM.MMMMM (latitude)
 */
static double nmea_to_degrees(const char *nmea_coord, size_t len,
                              char hemisphere) {
  if (len == 0 || nmea_coord == NULL) {
    return 0.0;
  }

  // Find decimal point
  const char *dot = NULL;
  for (size_t i = 0; i < len && nmea_coord[i] != '\0'; i++) {
    if (nmea_coord[i] == '.') {
      dot = &nmea_coord[i];
      break;
    }
  }

  if (dot == NULL)
    return 0.0;

  size_t digits_before_dot = dot - nmea_coord;
  if (digits_before_dot < 2)
    return 0.0;

  // Parse degrees (all digits except last 2 before decimal)
  int degrees = 0;
  size_t degree_digits = digits_before_dot - 2;
  for (size_t i = 0; i < degree_digits; i++) {
    if (nmea_coord[i] >= '0' && nmea_coord[i] <= '9') {
      degrees = degrees * 10 + (nmea_coord[i] - '0');
    }
  }

  // Parse minutes (last 2 digits before decimal + fractional part)
  double minutes = 0.0;
  int minutes_int = (nmea_coord[digits_before_dot - 2] - '0') * 10 +
                    (nmea_coord[digits_before_dot - 1] - '0');

  double minutes_frac = 0.0;
  double divisor = 10.0;
  for (const char *p = dot + 1;
       *p != '\0' && *p != ',' && (p - nmea_coord) < (int)len; p++) {
    if (*p >= '0' && *p <= '9') {
      minutes_frac += (*p - '0') / divisor;
      divisor *= 10.0;
    } else {
      break;
    }
  }

  minutes = (double)minutes_int + minutes_frac;

  // Convert to decimal degrees
  double decimal = (double)degrees + minutes / 60.0;

  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

/**
 * @brief Parse float from NMEA field
 */
static int parse_float(const char *str, size_t len, float *out) {
  if (str == NULL || len == 0 || *str == '\0' || *str == ',') {
    return 0;
  }

  bool negative = false;
  float result = 0.0f;
  float decimal = 0.0f;
  float decimal_div = 10.0f;
  bool in_decimal = false;

  for (size_t i = 0; i < len && str[i] != '\0' && str[i] != ','; i++) {
    if (str[i] == '-') {
      negative = true;
    } else if (str[i] == '.') {
      in_decimal = true;
    } else if (str[i] >= '0' && str[i] <= '9') {
      if (in_decimal) {
        decimal += (str[i] - '0') / decimal_div;
        decimal_div *= 10.0f;
      } else {
        result = result * 10.0f + (str[i] - '0');
      }
    }
  }

  *out = (result + decimal) * (negative ? -1.0f : 1.0f);
  return 1;
}

/**
 * @brief Parse integer from NMEA field
 */
static int parse_int(const char *str, size_t len, int *out) {
  if (str == NULL || len == 0 || *str == '\0' || *str == ',') {
    return 0;
  }

  int result = 0;
  bool negative = false;

  for (size_t i = 0; i < len && str[i] != '\0' && str[i] != ','; i++) {
    if (str[i] == '-') {
      negative = true;
    } else if (str[i] >= '0' && str[i] <= '9') {
      result = result * 10 + (str[i] - '0');
    }
  }

  *out = negative ? -result : result;
  return 1;
}

/**
 * @brief Split NMEA sentence into fields
 */
static int nmea_split_fields(const char *sentence, const char **fields,
                             size_t *field_lens, int max_fields) {
  int count = 0;
  const char *p = sentence;

  // Skip sentence type (e.g., $GNRMC,)
  while (*p != '\0' && *p != ',')
    p++;
  if (*p == ',')
    p++;

  while (*p != '\0' && *p != '*' && count < max_fields) {
    fields[count] = p;

    const char *end = p;
    while (*end != '\0' && *end != ',' && *end != '*')
      end++;

    field_lens[count] = end - p;
    count++;

    if (*end == ',') {
      p = end + 1;
    } else {
      break;
    }
  }

  return count;
}

/**
 * @brief Identify GNSS system from talker ID
 */
static gnss_system_t get_gnss_system(const char *sentence) {
  if (sentence[0] != '$' || strlen(sentence) < 3) {
    return GNSS_GPS;
  }

  char talker[3] = {sentence[1], sentence[2], '\0'};

  if (strcmp(talker, "GP") == 0)
    return GNSS_GPS;
  if (strcmp(talker, "GL") == 0)
    return GNSS_GLONASS;
  if (strcmp(talker, "GN") == 0)
    return GNSS_GPS; // Multi-GNSS
  if (strcmp(talker, "GA") == 0)
    return GNSS_GALILEO;
  if (strcmp(talker, "BD") == 0)
    return GNSS_BEIDOU;
  if (strcmp(talker, "GB") == 0)
    return GNSS_BEIDOU;
  if (strcmp(talker, "QZ") == 0)
    return GNSS_QZSS;

  return GNSS_GPS;
}

/**
 * @brief Check if sentence matches type (supports all talker IDs)
 */
static bool nmea_is_type(const char *sentence, const char *type) {
  if (sentence == NULL || type == NULL || strlen(sentence) < 6) {
    return false;
  }

  if (sentence[0] != '$')
    return false;

  // Check sentence type at position 3, 4, 5
  return (sentence[3] == type[0] && sentence[4] == type[1] &&
          sentence[5] == type[2]);
}

// =============================================================================
// NMEA SENTENCE PARSERS
// =============================================================================

/**
 * @brief Parse GGA sentence (GPS Fix Data)
 *
 * Format: $xxGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,q,nn,d.d,h.h,M,g.g,M,t,id*cs
 */
static bool parse_gga(const char *sentence, size_t len, gps_data_t *data) {
  const char *fields[NMEA_MAX_FIELDS];
  size_t field_lens[NMEA_MAX_FIELDS];

  int count = nmea_split_fields(sentence, fields, field_lens, NMEA_MAX_FIELDS);
  if (count < 10) {
    ESP_LOGD(TAG, "GGA: Insufficient fields (%d)", count);
    gps_stats.parse_errors++;
    return false;
  }

  // Field 5: Fix quality
  int quality = 0;
  if (!parse_int(fields[5], field_lens[5], &quality) || quality == 0) {
    ESP_LOGD(TAG, "GGA: No fix (quality=%d)", quality);
    return false;
  }
  data->quality = (gps_fix_quality_t)quality;

  // Field 0: UTC Time (HHMMSS.sss)
  if (field_lens[0] >= 6) {
    data->utc_time.hour = (fields[0][0] - '0') * 10 + (fields[0][1] - '0');
    data->utc_time.minute = (fields[0][2] - '0') * 10 + (fields[0][3] - '0');
    data->utc_time.second = (fields[0][4] - '0') * 10 + (fields[0][5] - '0');

    if (field_lens[0] > 7 && fields[0][6] == '.') {
      int ms = 0, div = 1;
      for (size_t i = 7;
           i < field_lens[0] && fields[0][i] >= '0' && fields[0][i] <= '9';
           i++) {
        ms = ms * 10 + (fields[0][i] - '0');
        div *= 10;
      }
      data->utc_time.millisecond = (ms * 1000) / div;
    }
  }

  // Field 1-2: Latitude + Hemisphere
  if (field_lens[1] > 0 && field_lens[2] > 0) {
    data->latitude = nmea_to_degrees(fields[1], field_lens[1], fields[2][0]);
  }

  // Field 3-4: Longitude + Hemisphere
  if (field_lens[3] > 0 && field_lens[4] > 0) {
    data->longitude = nmea_to_degrees(fields[3], field_lens[3], fields[4][0]);
  }

  // Field 6: Number of satellites
  if (field_lens[6] > 0) {
    int sats = 0;
    parse_int(fields[6], field_lens[6], &sats);
    data->satellites_used = (uint8_t)sats;
  }

  // Field 7: HDOP
  if (field_lens[7] > 0) {
    parse_float(fields[7], field_lens[7], &data->dop.hdop);
  }

  // Field 8: Altitude MSL
  if (field_lens[8] > 0) {
    parse_float(fields[8], field_lens[8], &data->altitude);
  }

  // Field 10: Geoid separation
  if (count > 10 && field_lens[10] > 0) {
    parse_float(fields[10], field_lens[10], &data->geoid_separation);
  }

  data->fix_type = GPS_FIX_3D;
  data->valid = true;
  data->primary_gnss = get_gnss_system(sentence);

  ESP_LOGD(TAG, "GGA: Lat=%.6f, Lon=%.6f, Alt=%.1f, Sats=%d, HDOP=%.1f",
           data->latitude, data->longitude, data->altitude,
           data->satellites_used, data->dop.hdop);

  return true;
}

/**
 * @brief Parse RMC sentence (Recommended Minimum)
 *
 * Format: $xxRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,s.s,c.c,ddmmyy,m.m,a,m*cs
 */
static bool parse_rmc(const char *sentence, size_t len, gps_data_t *data) {
  const char *fields[NMEA_MAX_FIELDS];
  size_t field_lens[NMEA_MAX_FIELDS];

  int count = nmea_split_fields(sentence, fields, field_lens, NMEA_MAX_FIELDS);
  if (count < 9) {
    ESP_LOGD(TAG, "RMC: Insufficient fields (%d)", count);
    gps_stats.parse_errors++;
    return false;
  }

  // Field 1: Status (A=valid, V=void)
  bool status_valid = (field_lens[1] > 0 && fields[1][0] == 'A');

  // Field 0: UTC Time
  if (field_lens[0] >= 6) {
    data->utc_time.hour = (fields[0][0] - '0') * 10 + (fields[0][1] - '0');
    data->utc_time.minute = (fields[0][2] - '0') * 10 + (fields[0][3] - '0');
    data->utc_time.second = (fields[0][4] - '0') * 10 + (fields[0][5] - '0');

    if (field_lens[0] > 7 && fields[0][6] == '.') {
      int ms = 0, div = 1;
      for (size_t i = 7;
           i < field_lens[0] && fields[0][i] >= '0' && fields[0][i] <= '9';
           i++) {
        ms = ms * 10 + (fields[0][i] - '0');
        div *= 10;
      }
      data->utc_time.millisecond = (ms * 1000) / div;
    }
  }

  // Field 2-3: Latitude + Hemisphere
  if (field_lens[2] > 0 && field_lens[3] > 0) {
    data->latitude = nmea_to_degrees(fields[2], field_lens[2], fields[3][0]);
  }

  // Field 4-5: Longitude + Hemisphere
  if (field_lens[4] > 0 && field_lens[5] > 0) {
    data->longitude = nmea_to_degrees(fields[4], field_lens[4], fields[5][0]);
  }

  // Field 6: Speed (knots)
  if (field_lens[6] > 0) {
    parse_float(fields[6], field_lens[6], &data->speed_knots);
    data->speed_kmh = data->speed_knots * KNOTS_TO_KMH;
  }

  // Field 7: Course (true)
  if (field_lens[7] > 0) {
    parse_float(fields[7], field_lens[7], &data->course);
  }

  // Field 8: Date (DDMMYY)
  if (field_lens[8] >= 6) {
    data->utc_date.day = (fields[8][0] - '0') * 10 + (fields[8][1] - '0');
    data->utc_date.month = (fields[8][2] - '0') * 10 + (fields[8][3] - '0');
    uint8_t year_2d = (fields[8][4] - '0') * 10 + (fields[8][5] - '0');
    data->utc_date.year = 2000 + year_2d;
  }

  // Field 9: Magnetic variation
  if (count > 9 && field_lens[9] > 0) {
    float mag_var = 0.0f;
    parse_float(fields[9], field_lens[9], &mag_var);
    // Apply magnetic variation direction if field 10 exists
    if (count > 10 && field_lens[10] > 0 && fields[10][0] == 'W') {
      mag_var = -mag_var;
    }
    data->course_magnetic = data->course + mag_var;
  }

  // Mode indicator (field 11 in NMEA 2.3+)
  if (count > 11 && field_lens[11] > 0) {
    data->mode_indicator = fields[11][0];
  }

  data->valid = status_valid;
  data->primary_gnss = get_gnss_system(sentence);

  if (status_valid) {
    ESP_LOGD(TAG,
             "RMC: Lat=%.6f, Lon=%.6f, Speed=%.2f km/h, Date=%02d/%02d/%04d",
             data->latitude, data->longitude, data->speed_kmh,
             data->utc_date.day, data->utc_date.month, data->utc_date.year);
  }

  return status_valid;
}

/**
 * @brief Parse VTG sentence (Course Over Ground and Ground Speed)
 *
 * Format: $xxVTG,c.c,T,m.m,M,n.n,N,k.k,K,m*cs
 */
static bool parse_vtg(const char *sentence, size_t len, gps_data_t *data) {
  const char *fields[NMEA_MAX_FIELDS];
  size_t field_lens[NMEA_MAX_FIELDS];

  int count = nmea_split_fields(sentence, fields, field_lens, NMEA_MAX_FIELDS);
  if (count < 8) {
    return false;
  }

  // Field 0: Course (true)
  if (field_lens[0] > 0 && data->course == 0.0f) {
    parse_float(fields[0], field_lens[0], &data->course);
  }

  // Field 2: Course (magnetic)
  if (field_lens[2] > 0) {
    parse_float(fields[2], field_lens[2], &data->course_magnetic);
  }

  // Field 4: Speed (knots)
  if (field_lens[4] > 0) {
    parse_float(fields[4], field_lens[4], &data->speed_knots);
  }

  // Field 6: Speed (km/h)
  if (field_lens[6] > 0) {
    parse_float(fields[6], field_lens[6], &data->speed_kmh);
  }

  // Mode indicator (field 8 in NMEA 2.3+)
  if (count > 8 && field_lens[8] > 0) {
    data->mode_indicator = fields[8][0];
  }

  ESP_LOGD(TAG, "VTG: Course=%.1f, Speed=%.2f km/h", data->course,
           data->speed_kmh);
  return true;
}

/**
 * @brief Parse GSA sentence (GNSS DOP and Active Satellites)
 *
 * Format: $xxGSA,a,f,s1,s2,...,s12,p.p,h.h,v.v*cs
 */
static bool parse_gsa(const char *sentence, size_t len, gps_data_t *data) {
  const char *fields[NMEA_MAX_FIELDS];
  size_t field_lens[NMEA_MAX_FIELDS];

  int count = nmea_split_fields(sentence, fields, field_lens, NMEA_MAX_FIELDS);
  if (count < 17) {
    return false;
  }

  // Field 1: Fix type (1=no fix, 2=2D, 3=3D)
  if (field_lens[1] > 0) {
    int fix = 0;
    parse_int(fields[1], field_lens[1], &fix);
    switch (fix) {
    case 1:
      data->fix_type = GPS_FIX_NONE;
      break;
    case 2:
      data->fix_type = GPS_FIX_2D;
      break;
    case 3:
      data->fix_type = GPS_FIX_3D;
      break;
    default:
      break;
    }
  }

  // Fields 2-13: PRN of satellites used (up to 12)
  int sats_used = 0;
  for (int i = 2; i < 14 && i < count; i++) {
    if (field_lens[i] > 0) {
      int prn = 0;
      if (parse_int(fields[i], field_lens[i], &prn) && prn > 0) {
        if (sats_used < GPS_MAX_SATELLITES) {
          data->satellites[sats_used].prn = (uint8_t)prn;
          data->satellites[sats_used].used = true;
          sats_used++;
        }
      }
    }
  }

  // Field 14: PDOP
  if (count > 14 && field_lens[14] > 0) {
    parse_float(fields[14], field_lens[14], &data->dop.pdop);
  }

  // Field 15: HDOP
  if (count > 15 && field_lens[15] > 0) {
    parse_float(fields[15], field_lens[15], &data->dop.hdop);
  }

  // Field 16: VDOP
  if (count > 16 && field_lens[16] > 0) {
    parse_float(fields[16], field_lens[16], &data->dop.vdop);
  }

  ESP_LOGD(TAG, "GSA: Fix=%d, PDOP=%.1f, HDOP=%.1f, VDOP=%.1f", data->fix_type,
           data->dop.pdop, data->dop.hdop, data->dop.vdop);

  return true;
}

/**
 * @brief Parse GSV sentence (GNSS Satellites in View)
 *
 * Format: $xxGSV,n,m,t,s1,e1,a1,c1,...*cs
 */
static bool parse_gsv(const char *sentence, size_t len, gps_data_t *data) {
  const char *fields[NMEA_MAX_FIELDS];
  size_t field_lens[NMEA_MAX_FIELDS];

  int count = nmea_split_fields(sentence, fields, field_lens, NMEA_MAX_FIELDS);
  if (count < 4) {
    return false;
  }

  // Field 2: Total satellites in view
  if (field_lens[2] > 0) {
    int total = 0;
    parse_int(fields[2], field_lens[2], &total);
    data->satellites_view = (uint8_t)total;
  }

  gnss_system_t system = get_gnss_system(sentence);

  // Parse satellite info (4 satellites per message, 4 fields each)
  for (int i = 0; i < 4; i++) {
    int base = 3 + i * 4;
    if (base + 3 >= count)
      break;

    // PRN
    int prn = 0;
    if (field_lens[base] > 0 &&
        parse_int(fields[base], field_lens[base], &prn) && prn > 0) {
      // Find or create satellite entry
      int idx = -1;
      for (int j = 0; j < GPS_MAX_SATELLITES; j++) {
        if (data->satellites[j].prn == prn) {
          idx = j;
          break;
        }
        if (data->satellites[j].prn == 0 && idx < 0) {
          idx = j;
        }
      }

      if (idx >= 0 && idx < GPS_MAX_SATELLITES) {
        data->satellites[idx].prn = (uint8_t)prn;
        data->satellites[idx].system = system;

        // Elevation
        if (field_lens[base + 1] > 0) {
          int elev = 0;
          parse_int(fields[base + 1], field_lens[base + 1], &elev);
          data->satellites[idx].elevation = (uint8_t)elev;
        }

        // Azimuth
        if (field_lens[base + 2] > 0) {
          int azim = 0;
          parse_int(fields[base + 2], field_lens[base + 2], &azim);
          data->satellites[idx].azimuth = (uint16_t)azim;
        }

        // SNR
        if (field_lens[base + 3] > 0) {
          int snr = 0;
          parse_int(fields[base + 3], field_lens[base + 3], &snr);
          data->satellites[idx].snr = (uint8_t)snr;
        }
      }
    }
  }

  return true;
}

// =============================================================================
// UBX PROTOCOL FUNCTIONS
// =============================================================================

/**
 * @brief Calculate UBX checksum (Fletcher-8)
 */
static void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a,
                         uint8_t *ck_b) {
  *ck_a = 0;
  *ck_b = 0;

  for (size_t i = 0; i < len; i++) {
    *ck_a += data[i];
    *ck_b += *ck_a;
  }
}

/**
 * @brief Build and send UBX message
 */
static bool ubx_send_message(uint8_t class_id, uint8_t msg_id,
                             const uint8_t *payload, size_t payload_len) {
  if (!gps_initialized)
    return false;

  size_t total_len = 8 + payload_len;
  uint8_t *msg = malloc(total_len);
  if (msg == NULL)
    return false;

  msg[0] = UBX_SYNC1;
  msg[1] = UBX_SYNC2;
  msg[2] = class_id;
  msg[3] = msg_id;
  msg[4] = payload_len & 0xFF;
  msg[5] = (payload_len >> 8) & 0xFF;

  if (payload && payload_len > 0) {
    memcpy(&msg[6], payload, payload_len);
  }

  uint8_t ck_a, ck_b;
  ubx_checksum(&msg[2], 4 + payload_len, &ck_a, &ck_b);
  msg[6 + payload_len] = ck_a;
  msg[7 + payload_len] = ck_b;

  int written = uart_write_bytes(GPS_UART_NUM, msg, total_len);
  uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(100));

  free(msg);
  return (written == (int)total_len);
}

/**
 * @brief Wait for UBX ACK/NAK response
 */
static bool ubx_wait_ack(uint8_t class_id, uint8_t msg_id,
                         uint32_t timeout_ms) {
  uint8_t buf[16];
  uint32_t start = esp_timer_get_time() / 1000;

  while ((esp_timer_get_time() / 1000 - start) < timeout_ms) {
    int len =
        uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (len <= 0)
      continue;

    // Search for ACK-ACK or ACK-NAK
    for (int i = 0; i < len - 7; i++) {
      if (buf[i] == UBX_SYNC1 && buf[i + 1] == UBX_SYNC2 &&
          buf[i + 2] == UBX_CLASS_ACK) {

        if (buf[i + 3] == UBX_ACK_ACK && buf[i + 6] == class_id &&
            buf[i + 7] == msg_id) {
          return true;
        }

        if (buf[i + 3] == UBX_ACK_NAK && buf[i + 6] == class_id &&
            buf[i + 7] == msg_id) {
          ESP_LOGW(TAG, "UBX NAK received for class=0x%02X msg=0x%02X",
                   class_id, msg_id);
          return false;
        }
      }
    }
  }

  ESP_LOGW(TAG, "UBX ACK timeout for class=0x%02X msg=0x%02X", class_id,
           msg_id);
  return false;
}

// =============================================================================
// PUBLIC API - INITIALIZATION
// =============================================================================

void gps_neo7m_get_default_config(gps_config_t *config) {
  if (config == NULL)
    return;

  config->update_rate_hz = 1;
  config->nav_mode = GPS_NAV_MODE_PORTABLE;
  config->power_mode = GPS_POWER_FULL;
  config->enable_glonass = true; // NEO-7M supports GPS+GLONASS
  config->enable_sbas = true;
  config->enable_qzss = false;
  config->assistnow_autonomous = true; // Enable for faster TTFF
  config->min_svs = 3;
  config->min_cno = 7;
  config->static_hold_thresh = 50; // 0.5 m/s
}

bool gps_neo7m_init(void) {
  gps_config_t config;
  gps_neo7m_get_default_config(&config);
  return gps_neo7m_init_with_config(&config);
}

bool gps_neo7m_init_with_config(const gps_config_t *config) {
  if (gps_initialized) {
    ESP_LOGW(TAG, "GPS already initialized");
    return true;
  }

  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "u-blox NEO-7M GNSS Module Initialization");
  ESP_LOGI(TAG, "========================================");

  // Create mutex
  if (gps_mutex == NULL) {
    gps_mutex = xSemaphoreCreateMutex();
    if (gps_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create mutex");
      return false;
    }
  }

  // Store configuration
  if (config != NULL) {
    current_config = *config;
  } else {
    gps_neo7m_get_default_config(&current_config);
  }

  // Configure UART (NEO-7M default: 9600 8N1)
  uart_config_t uart_config = {
      .baud_rate = GPS_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_LOGI(TAG, "UART Configuration:");
  ESP_LOGI(TAG, "  Port: UART%d", GPS_UART_NUM);
  ESP_LOGI(TAG, "  Baud: %d", GPS_BAUD_RATE);
  ESP_LOGI(TAG, "  TX (ESP->GPS): GPIO%d", GPS_TX_PIN);
  ESP_LOGI(TAG, "  RX (GPS->ESP): GPIO%d", GPS_RX_PIN);

  // Install UART driver
  esp_err_t ret = uart_driver_install(GPS_UART_NUM, GPS_BUFFER_SIZE * 2,
                                      GPS_BUFFER_SIZE, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
    return false;
  }

  ret = uart_param_config(GPS_UART_NUM, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
    uart_driver_delete(GPS_UART_NUM);
    return false;
  }

  ret = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
    uart_driver_delete(GPS_UART_NUM);
    return false;
  }

  uart_flush(GPS_UART_NUM);

  gps_initialized = true;
  init_time_ms = esp_timer_get_time() / 1000;
  gps_stats.last_data_time_ms = init_time_ms;

  // Wait for GPS module to boot (100ms)
  vTaskDelay(pdMS_TO_TICKS(100));

  // Apply configuration
  ESP_LOGI(TAG, "Applying GPS configuration...");

  // Set navigation mode
  gps_neo7m_set_nav_mode(current_config.nav_mode);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Set update rate
  if (current_config.update_rate_hz > 1) {
    gps_neo7m_set_update_rate(current_config.update_rate_hz);
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Enable GLONASS if requested
  if (current_config.enable_glonass) {
    gps_neo7m_set_glonass(true);
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Enable AssistNow Autonomous
  if (current_config.assistnow_autonomous) {
    gps_neo7m_set_assistnow_autonomous(true);
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Enable SBAS
  if (current_config.enable_sbas) {
    gps_neo7m_set_sbas(SBAS_MODE_ENABLED);
  }

  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "GPS Initialization COMPLETE");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Module: u-blox NEO-7M");
  ESP_LOGI(TAG, "GNSS: GPS%s%s",
           current_config.enable_glonass ? " + GLONASS" : "",
           current_config.enable_sbas ? " + SBAS" : "");
  ESP_LOGI(TAG, "Update Rate: %d Hz", current_config.update_rate_hz);
  ESP_LOGI(TAG, "Nav Mode: %d", current_config.nav_mode);
  ESP_LOGI(TAG, "AssistNow: %s",
           current_config.assistnow_autonomous ? "ON" : "OFF");
  ESP_LOGI(TAG, "========================================");

  return true;
}

void gps_neo7m_deinit(void) {
  if (gps_initialized) {
    uart_flush(GPS_UART_NUM);
    uart_driver_delete(GPS_UART_NUM);
    gps_initialized = false;
    has_valid_fix = false;

    if (gps_mutex != NULL) {
      vSemaphoreDelete(gps_mutex);
      gps_mutex = NULL;
    }

    memset(&gps_stats, 0, sizeof(gps_stats));
    ESP_LOGI(TAG, "GPS deinitialized");
  }
}

bool gps_neo7m_is_initialized(void) { return gps_initialized; }

// =============================================================================
// PUBLIC API - DATA READING
// =============================================================================

bool gps_neo7m_read(gps_data_t *data, uint32_t timeout_ms) {
  if (!gps_initialized || data == NULL) {
    return false;
  }

  if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }

  gps_stats.total_reads++;

  // Initialize output
  memset(data, 0, sizeof(gps_data_t));
  data->timestamp_ms = esp_timer_get_time() / 1000;

  // Read UART data
  int len = uart_read_bytes(GPS_UART_NUM, uart_buffer, GPS_BUFFER_SIZE - 1,
                            pdMS_TO_TICKS(timeout_ms));

  if (len <= 0) {
    uint32_t time_since_data = data->timestamp_ms - gps_stats.last_data_time_ms;
    if (time_since_data > GPS_NO_DATA_TIMEOUT_MS) {
      ESP_LOGW(TAG, "No GPS data for %lu ms - check wiring!",
               (unsigned long)time_since_data);
      has_valid_fix = false;
    }
    xSemaphoreGive(gps_mutex);
    return false;
  }

  gps_stats.successful_reads++;
  gps_stats.last_data_time_ms = data->timestamp_ms;
  uart_buffer[len] = '\0';

  ESP_LOGD(TAG, "Received %d bytes", len);

  // Parse NMEA sentences
  bool found_rmc = false, found_gga = false;
  gps_data_t rmc_data = {0}, gga_data = {0};

  const char *p = (const char *)uart_buffer;
  const char *end = p + len;

  while (p < end) {
    // Find '$'
    while (p < end && *p != '$')
      p++;
    if (p >= end)
      break;

    const char *start = p;

    // Find end of sentence
    const char *sent_end = start;
    while (sent_end < end && !(*sent_end == '\r' && *(sent_end + 1) == '\n') &&
           (sent_end - start) < NMEA_SENTENCE_MAX_LEN) {
      sent_end++;
    }

    if (sent_end >= end || (sent_end - start) > NMEA_SENTENCE_MAX_LEN) {
      p++;
      continue;
    }

    size_t sent_len = sent_end - start;

    // Copy sentence
    char sentence[NMEA_SENTENCE_MAX_LEN + 1];
    size_t copy_len =
        (sent_len < NMEA_SENTENCE_MAX_LEN) ? sent_len : NMEA_SENTENCE_MAX_LEN;
    memcpy(sentence, start, copy_len);
    sentence[copy_len] = '\0';

    // Validate checksum
    if (!nmea_validate_checksum(sentence)) {
      gps_stats.checksum_errors++;
      p = sent_end + 2;
      continue;
    }

    // Parse by type
    if (nmea_is_type(sentence, "RMC")) {
      if (parse_rmc(sentence, sent_len, &rmc_data)) {
        found_rmc = true;
      }
    } else if (nmea_is_type(sentence, "GGA")) {
      if (parse_gga(sentence, sent_len, &gga_data)) {
        found_gga = true;
      }
    } else if (nmea_is_type(sentence, "VTG")) {
      parse_vtg(sentence, sent_len, data);
    } else if (nmea_is_type(sentence, "GSA")) {
      parse_gsa(sentence, sent_len, data);
    } else if (nmea_is_type(sentence, "GSV")) {
      parse_gsv(sentence, sent_len, data);
    }

    p = sent_end + 2;
  }

  // Merge data (RMC priority for validity, GGA for altitude/satellites)
  if (found_rmc) {
    *data = rmc_data;

    if (found_gga) {
      data->satellites_used = gga_data.satellites_used;
      data->dop.hdop = gga_data.dop.hdop;
      data->altitude = gga_data.altitude;
      data->geoid_separation = gga_data.geoid_separation;
      data->quality = gga_data.quality;

      if (data->latitude == 0.0 && gga_data.latitude != 0.0) {
        data->latitude = gga_data.latitude;
        data->longitude = gga_data.longitude;
      }
    }
  } else if (found_gga) {
    double lat = data->latitude;
    double lon = data->longitude;
    *data = gga_data;
    if (lat != 0.0) {
      data->latitude = lat;
      data->longitude = lon;
    }
  }

  // Update fix tracking
  if (data->valid) {
    gps_stats.valid_fixes++;

    if (!has_valid_fix) {
      first_fix_time_ms = data->timestamp_ms;
      gps_stats.ttff_ms = first_fix_time_ms - init_time_ms;
      ESP_LOGI(TAG, "First fix acquired! TTFF = %lu ms",
               (unsigned long)gps_stats.ttff_ms);
    }

    has_valid_fix = true;
    gps_stats.last_fix_time_ms = data->timestamp_ms;
    last_gps_data = *data;

    ESP_LOGI(
        TAG,
        "GPS: Lat=%.6f Lon=%.6f Alt=%.1fm Speed=%.1fkm/h Sats=%d HDOP=%.1f",
        data->latitude, data->longitude, data->altitude, data->speed_kmh,
        data->satellites_used, data->dop.hdop);
  }

  xSemaphoreGive(gps_mutex);
  return found_rmc || found_gga;
}

bool gps_neo7m_has_fix(void) { return has_valid_fix; }

uint32_t gps_neo7m_time_since_fix(void) {
  if (gps_stats.last_fix_time_ms == 0) {
    return UINT32_MAX;
  }
  return (esp_timer_get_time() / 1000) - gps_stats.last_fix_time_ms;
}

gps_fix_type_t gps_neo7m_get_fix_type(void) { return last_gps_data.fix_type; }

void gps_neo7m_get_stats(gps_stats_t *stats) {
  if (stats != NULL) {
    *stats = gps_stats;
  }
}

// =============================================================================
// PUBLIC API - CONFIGURATION
// =============================================================================

bool gps_neo7m_set_update_rate(uint8_t rate_hz) {
  if (!gps_initialized || rate_hz == 0 || rate_hz > 10) {
    return false;
  }

  uint16_t meas_rate_ms = 1000 / rate_hz;

  uint8_t payload[6] = {
      meas_rate_ms & 0xFF,
      (meas_rate_ms >> 8) & 0xFF,
      0x01,
      0x00, // navRate = 1
      0x01,
      0x00 // timeRef = GPS
  };

  ESP_LOGI(TAG, "Setting update rate to %d Hz (%d ms)", rate_hz, meas_rate_ms);

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RATE, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_RATE, GPS_ACK_TIMEOUT_MS);
    if (result) {
      current_config.update_rate_hz = rate_hz;
    }
  }

  return result;
}

bool gps_neo7m_set_nav_mode(gps_nav_mode_t mode) {
  if (!gps_initialized) {
    return false;
  }

  // CFG-NAV5 payload (36 bytes)
  uint8_t payload[36] = {0};
  payload[0] = 0xFF; // mask LSB
  payload[1] = 0xFF; // mask MSB
  payload[2] = mode; // dynModel
  payload[3] = 0x03; // fixMode: Auto 2D/3D

  const char *mode_names[] = {"Portable",     "N/A",          "Stationary",
                              "Pedestrian",   "Automotive",   "Sea",
                              "Airborne <1g", "Airborne <2g", "Airborne <4g"};

  ESP_LOGI(TAG, "Setting navigation mode: %s", mode_names[mode]);

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_NAV5, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_NAV5, GPS_ACK_TIMEOUT_MS);
    if (result) {
      current_config.nav_mode = mode;
    }
  }

  return result;
}

bool gps_neo7m_set_power_mode(gps_power_mode_t mode) {
  if (!gps_initialized) {
    return false;
  }

  bool result = false;

  switch (mode) {
  case GPS_POWER_FULL: {
    // CFG-RXM: Continuous mode
    uint8_t payload[2] = {0x00, 0x00}; // reserved, lpMode=0 (continuous)
    result =
        ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RXM, payload, sizeof(payload));
    break;
  }

  case GPS_POWER_SAVE: {
    // CFG-RXM: Power Save mode
    uint8_t payload[2] = {0x00, 0x01}; // reserved, lpMode=1 (power save)
    result =
        ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RXM, payload, sizeof(payload));
    break;
  }

  case GPS_POWER_BACKUP: {
    // RXM-PMREQ: Enter backup mode
    uint8_t payload[8] = {
        0x00, 0x00, 0x00, 0x00, // duration = infinite
        0x02, 0x00, 0x00, 0x00  // flags = backup mode
    };
    result = ubx_send_message(UBX_CLASS_RXM, 0x41, payload, sizeof(payload));
    break;
  }
  }

  if (result) {
    current_config.power_mode = mode;
    ESP_LOGI(TAG, "Power mode set to %d", mode);
  }

  return result;
}

bool gps_neo7m_set_glonass(bool enable) {
  if (!gps_initialized) {
    return false;
  }

  // CFG-GNSS message for NEO-7M
  // Configure GPS and GLONASS
  uint8_t payload[20] = {
      0x00, // msgVer
      0x00, // numTrkChHw (read only)
      0xFF, // numTrkChUse
      0x02, // numConfigBlocks (GPS + GLONASS)

      // GPS config
      0x00, // gnssId = GPS
      0x08, // resTrkCh
      0x10, // maxTrkCh
      0x00, // reserved
      0x01,
      0x00,
      0x01,
      0x01, // flags: enable GPS, L1 signal

      // GLONASS config
      0x06, // gnssId = GLONASS
      0x08, // resTrkCh
      0x0E, // maxTrkCh
      0x00, // reserved
      enable ? 0x01 : 0x00,
      0x00,
      0x01,
      0x01, // flags: enable/disable GLONASS, L1 signal
  };

  ESP_LOGI(TAG, "GLONASS %s", enable ? "enabled" : "disabled");

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_GNSS, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_GNSS, GPS_ACK_TIMEOUT_MS);
    if (result) {
      current_config.enable_glonass = enable;
    }
  }

  return result;
}

bool gps_neo7m_set_sbas(sbas_mode_t mode) {
  if (!gps_initialized) {
    return false;
  }

  uint8_t payload[8] = {
      mode == SBAS_MODE_DISABLED ? 0x00 : 0x01, // mode
      0x07, // usage: range + diffCorr + integrity
      0x03, // maxSBAS = 3
      0x00, // scanmode2
      0x51,
      0x08,
      0x00,
      0x00 // scanmode1: WAAS + EGNOS + MSAS
  };

  ESP_LOGI(TAG, "SBAS mode set to %d", mode);

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_SBAS, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_SBAS, GPS_ACK_TIMEOUT_MS);
    if (result) {
      current_config.enable_sbas = (mode != SBAS_MODE_DISABLED);
    }
  }

  return result;
}

bool gps_neo7m_set_assistnow_autonomous(bool enable) {
  if (!gps_initialized) {
    return false;
  }

  // CFG-NAVX5 message
  uint8_t payload[40] = {0};
  payload[0] = 0x00;
  payload[1] = 0x00; // version
  payload[2] = 0x00;
  payload[3] = 0x40; // mask1: AOP enable bit
  payload[4] = 0x00;
  payload[5] = 0x00;
  payload[6] = 0x00;
  payload[7] = 0x00; // mask2

  // AOP cfg at offset 27
  payload[27] = enable ? 0x01 : 0x00; // aopCfg

  ESP_LOGI(TAG, "AssistNow Autonomous %s", enable ? "enabled" : "disabled");

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_NAVX5, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_NAVX5, GPS_ACK_TIMEOUT_MS);
    if (result) {
      current_config.assistnow_autonomous = enable;
    }
  }

  return result;
}

bool gps_neo7m_set_nmea_rate(const char *msg_id, gps_msg_rate_t rate) {
  if (!gps_initialized || msg_id == NULL) {
    return false;
  }

  // NMEA message class = 0xF0
  uint8_t nmea_msg_id = 0;

  if (strcmp(msg_id, "GGA") == 0)
    nmea_msg_id = 0x00;
  else if (strcmp(msg_id, "GLL") == 0)
    nmea_msg_id = 0x01;
  else if (strcmp(msg_id, "GSA") == 0)
    nmea_msg_id = 0x02;
  else if (strcmp(msg_id, "GSV") == 0)
    nmea_msg_id = 0x03;
  else if (strcmp(msg_id, "RMC") == 0)
    nmea_msg_id = 0x04;
  else if (strcmp(msg_id, "VTG") == 0)
    nmea_msg_id = 0x05;
  else {
    ESP_LOGW(TAG, "Unknown NMEA message ID: %s", msg_id);
    return false;
  }

  // CFG-MSG: Configure message rate
  uint8_t payload[3] = {0xF0, // msgClass = NMEA
                        nmea_msg_id, (uint8_t)rate};

  ESP_LOGD(TAG, "Setting %s rate to %d", msg_id, rate);

  return ubx_send_message(UBX_CLASS_CFG, UBX_CFG_MSG, payload, sizeof(payload));
}

bool gps_neo7m_save_config(void) {
  if (!gps_initialized) {
    return false;
  }

  // CFG-CFG: Save current configuration
  uint8_t payload[13] = {
      0x00, 0x00, 0x00, 0x00, // clearMask = 0
      0xFF, 0xFF, 0x00, 0x00, // saveMask = all
      0x00, 0x00, 0x00, 0x00, // loadMask = 0
      0x17                    // deviceMask: BBR + Flash + EEPROM
  };

  ESP_LOGI(TAG, "Saving configuration to flash...");

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_CFG, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_CFG, GPS_ACK_TIMEOUT_MS);
  }

  return result;
}

bool gps_neo7m_load_defaults(void) {
  if (!gps_initialized) {
    return false;
  }

  // CFG-CFG: Load default configuration
  uint8_t payload[13] = {
      0xFF, 0xFF, 0x00, 0x00, // clearMask = all
      0x00, 0x00, 0x00, 0x00, // saveMask = 0
      0xFF, 0xFF, 0x00, 0x00, // loadMask = all
      0x17                    // deviceMask
  };

  ESP_LOGI(TAG, "Loading default configuration...");

  bool result =
      ubx_send_message(UBX_CLASS_CFG, UBX_CFG_CFG, payload, sizeof(payload));
  if (result) {
    result = ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_CFG, GPS_ACK_TIMEOUT_MS);
    if (result) {
      gps_neo7m_get_default_config(&current_config);
    }
  }

  return result;
}

// =============================================================================
// PUBLIC API - CONTROL
// =============================================================================

bool gps_neo7m_hot_start(void) {
  if (!gps_initialized)
    return false;

  ESP_LOGI(TAG, "Performing hot start...");

  uint8_t payload[4] = {
      0x00, 0x00, // navBbrMask = 0 (keep all data)
      0x09,       // resetMode = controlled GPS only
      0x00        // reserved
  };

  return ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RST, payload, sizeof(payload));
}

bool gps_neo7m_warm_start(void) {
  if (!gps_initialized)
    return false;

  ESP_LOGI(TAG, "Performing warm start...");

  uint8_t payload[4] = {
      0x01, 0x00, // navBbrMask = clear ephemeris
      0x09,       // resetMode = controlled GPS only
      0x00        // reserved
  };

  has_valid_fix = false;
  return ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RST, payload, sizeof(payload));
}

bool gps_neo7m_cold_start(void) {
  if (!gps_initialized)
    return false;

  ESP_LOGI(TAG, "Performing cold start...");

  uint8_t payload[4] = {
      0xFF, 0xFF, // navBbrMask = clear all
      0x01,       // resetMode = controlled software reset
      0x00        // reserved
  };

  has_valid_fix = false;
  gps_stats.last_fix_time_ms = 0;
  first_fix_time_ms = 0;
  init_time_ms = esp_timer_get_time() / 1000;

  return ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RST, payload, sizeof(payload));
}

bool gps_neo7m_reset(void) {
  if (!gps_initialized)
    return false;

  ESP_LOGI(TAG, "Resetting GPS module...");

  uint8_t payload[4] = {
      0x00, 0x00, // navBbrMask = 0
      0x00,       // resetMode = hardware reset
      0x00        // reserved
  };

  return ubx_send_message(UBX_CLASS_CFG, UBX_CFG_RST, payload, sizeof(payload));
}

// =============================================================================
// PUBLIC API - LOW LEVEL
// =============================================================================

bool gps_neo7m_send_ubx(const uint8_t *cmd, size_t cmd_len) {
  if (!gps_initialized || cmd == NULL || cmd_len == 0) {
    return false;
  }

  int written = uart_write_bytes(GPS_UART_NUM, cmd, cmd_len);
  uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(100));

  return (written == (int)cmd_len);
}

bool gps_neo7m_send_ubx_with_ack(uint8_t class_id, uint8_t msg_id,
                                 const uint8_t *payload, size_t payload_len,
                                 uint32_t timeout_ms) {
  if (!ubx_send_message(class_id, msg_id, payload, payload_len)) {
    return false;
  }
  return ubx_wait_ack(class_id, msg_id, timeout_ms);
}

bool gps_neo7m_send_nmea(const char *cmd) {
  if (!gps_initialized || cmd == NULL) {
    return false;
  }

  // Calculate checksum
  uint8_t checksum = 0;
  for (const char *p = cmd; *p != '\0'; p++) {
    checksum ^= *p;
  }

  // Format: $cmd*XX\r\n
  char buffer[NMEA_SENTENCE_MAX_LEN];
  int len = snprintf(buffer, sizeof(buffer), "$%s*%02X\r\n", cmd, checksum);

  int written = uart_write_bytes(GPS_UART_NUM, buffer, len);
  uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(100));

  return (written == len);
}

// =============================================================================
// PUBLIC API - DEBUG
// =============================================================================

void gps_neo7m_debug_status(void) {
  ESP_LOGI(TAG, "=== GPS NEO-7M DEBUG STATUS ===");
  ESP_LOGI(TAG, "Initialized: %s", gps_initialized ? "YES" : "NO");

  if (!gps_initialized) {
    ESP_LOGE(TAG, "GPS not initialized!");
    return;
  }

  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  UART Port: UART%d", GPS_UART_NUM);
  ESP_LOGI(TAG, "  Baud Rate: %d", GPS_BAUD_RATE);
  ESP_LOGI(TAG, "  TX Pin: GPIO%d", GPS_TX_PIN);
  ESP_LOGI(TAG, "  RX Pin: GPIO%d", GPS_RX_PIN);
  ESP_LOGI(TAG, "  Update Rate: %d Hz", current_config.update_rate_hz);
  ESP_LOGI(TAG, "  Nav Mode: %d", current_config.nav_mode);
  ESP_LOGI(TAG, "  GLONASS: %s", current_config.enable_glonass ? "ON" : "OFF");
  ESP_LOGI(TAG, "  SBAS: %s", current_config.enable_sbas ? "ON" : "OFF");

  ESP_LOGI(TAG, "Statistics:");
  ESP_LOGI(TAG, "  Total Reads: %lu", (unsigned long)gps_stats.total_reads);
  ESP_LOGI(TAG, "  Successful: %lu", (unsigned long)gps_stats.successful_reads);
  ESP_LOGI(TAG, "  Valid Fixes: %lu", (unsigned long)gps_stats.valid_fixes);
  ESP_LOGI(TAG, "  Checksum Errors: %lu",
           (unsigned long)gps_stats.checksum_errors);
  ESP_LOGI(TAG, "  Parse Errors: %lu", (unsigned long)gps_stats.parse_errors);
  ESP_LOGI(TAG, "  TTFF: %lu ms", (unsigned long)gps_stats.ttff_ms);

  ESP_LOGI(TAG, "Fix Status:");
  ESP_LOGI(TAG, "  Has Valid Fix: %s", has_valid_fix ? "YES" : "NO");
  ESP_LOGI(TAG, "  Time Since Fix: %lu ms",
           (unsigned long)gps_neo7m_time_since_fix());

  if (has_valid_fix) {
    ESP_LOGI(TAG, "Last Position:");
    ESP_LOGI(TAG, "  Lat: %.6f", last_gps_data.latitude);
    ESP_LOGI(TAG, "  Lon: %.6f", last_gps_data.longitude);
    ESP_LOGI(TAG, "  Alt: %.1f m", last_gps_data.altitude);
    ESP_LOGI(TAG, "  Speed: %.1f km/h", last_gps_data.speed_kmh);
    ESP_LOGI(TAG, "  Satellites: %d", last_gps_data.satellites_used);
    ESP_LOGI(TAG, "  HDOP: %.1f", last_gps_data.dop.hdop);
  }

  // Check UART buffer
  size_t buffered = 0;
  uart_get_buffered_data_len(GPS_UART_NUM, &buffered);
  ESP_LOGI(TAG, "UART RX Buffer: %d bytes", (int)buffered);

  // Try reading some data
  uint8_t sample[128];
  int bytes = uart_read_bytes(GPS_UART_NUM, sample, sizeof(sample) - 1,
                              pdMS_TO_TICKS(100));
  if (bytes > 0) {
    sample[bytes] = '\0';
    ESP_LOGI(TAG, "Sample data (%d bytes): %.80s", bytes, (char *)sample);
  } else {
    ESP_LOGW(TAG, "No data received - check wiring!");
    ESP_LOGW(TAG, "Expected: GPS TX -> ESP GPIO%d (RX)", GPS_RX_PIN);
    ESP_LOGW(TAG, "Expected: GPS RX -> ESP GPIO%d (TX)", GPS_TX_PIN);
  }

  ESP_LOGI(TAG, "================================");
}

void gps_neo7m_debug_satellites(void) {
  if (!has_valid_fix) {
    ESP_LOGW(TAG, "No satellite data available");
    return;
  }

  ESP_LOGI(TAG, "=== SATELLITE INFORMATION ===");
  ESP_LOGI(TAG, "Satellites in view: %d", last_gps_data.satellites_view);
  ESP_LOGI(TAG, "Satellites used: %d", last_gps_data.satellites_used);
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "PRN  System    Elev  Azim  SNR  Used");
  ESP_LOGI(TAG, "---  ------    ----  ----  ---  ----");

  const char *sys_names[] = {"GPS", "SBAS", "GAL", "BD", "IMES", "QZSS", "GLO"};

  for (int i = 0; i < GPS_MAX_SATELLITES; i++) {
    if (last_gps_data.satellites[i].prn > 0) {
      ESP_LOGI(TAG, "%3d  %-6s    %3d°  %3d°  %2d   %s",
               last_gps_data.satellites[i].prn,
               sys_names[last_gps_data.satellites[i].system],
               last_gps_data.satellites[i].elevation,
               last_gps_data.satellites[i].azimuth,
               last_gps_data.satellites[i].snr,
               last_gps_data.satellites[i].used ? "YES" : "NO");
    }
  }

  ESP_LOGI(TAG, "=============================");
}
