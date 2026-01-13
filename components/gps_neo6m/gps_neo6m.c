/**
 * @file gps_neo7m.c
 * @brief NEO-7M GPS module NMEA parser implementation
 */

#include "gps_neo6m.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

static const char *TAG = "gps_neo6m";
static bool gps_initialized = false;

// Track last valid fix time (must be declared before use)
static uint32_t last_fix_time_ms = 0;
static bool has_valid_fix = false;

#define GPS_BUFFER_SIZE 512
#define NMEA_SENTENCE_MAX_LEN 82

// Optimized NMEA parsing (no dynamic allocation)
static bool parse_nmea_gga_inline(const char *sentence, size_t len,
                                  gps_data_t *data);
static bool parse_nmea_rmc_inline(const char *sentence, size_t len,
                                  gps_data_t *data);
static float nmea_to_degrees_optimized(const char *nmea_coord, size_t len,
                                       char hemisphere);
static int parse_float_field(const char *str, size_t len, float *out);
static int parse_int_field(const char *str, size_t len, int *out);

// Optimized NMEA coordinate conversion (no atof)
static float nmea_to_degrees_optimized(const char *nmea_coord, size_t len,
                                       char hemisphere) {
  if (len == 0 || nmea_coord == NULL)
    return 0.0f;

  // Find decimal point
  const char *dot = NULL;
  for (size_t i = 0; i < len && nmea_coord[i] != '\0'; i++) {
    if (nmea_coord[i] == '.') {
      dot = &nmea_coord[i];
      break;
    }
  }

  if (dot == NULL)
    return 0.0f;

  // Parse degrees (before decimal point, last 2 digits are minutes)
  int degrees = 0;
  int minutes_int = 0;
  float minutes_frac = 0.0f;

  // Count digits before decimal
  size_t before_dot = dot - nmea_coord;
  if (before_dot >= 2) {
    // Last 2 digits are minutes
    for (size_t i = 0; i < before_dot - 2; i++) {
      if (nmea_coord[i] >= '0' && nmea_coord[i] <= '9') {
        degrees = degrees * 10 + (nmea_coord[i] - '0');
      }
    }
    minutes_int = (nmea_coord[before_dot - 2] - '0') * 10 +
                  (nmea_coord[before_dot - 1] - '0');
  }

  // Parse fractional minutes (after decimal point)
  float divisor = 1.0f;
  for (const char *p = dot + 1;
       *p != '\0' && *p != ',' && (p - nmea_coord) < (int)len; p++) {
    if (*p >= '0' && *p <= '9') {
      minutes_frac = minutes_frac * 10.0f + (*p - '0');
      divisor *= 10.0f;
    }
  }
  minutes_frac /= divisor;

  float decimal_degrees = degrees + (minutes_int + minutes_frac) / 60.0f;

  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

// Parse float field from NMEA sentence (optimized)
static int parse_float_field(const char *str, size_t len, float *out) {
  if (str == NULL || len == 0 || *str == '\0' || *str == ',') {
    return 0;
  }

  bool negative = false;
  float result = 0.0f;
  float decimal = 0.0f;
  float decimal_div = 1.0f;
  bool in_decimal = false;

  for (size_t i = 0; i < len && str[i] != '\0' && str[i] != ','; i++) {
    if (str[i] == '-') {
      negative = true;
    } else if (str[i] == '.') {
      in_decimal = true;
    } else if (str[i] >= '0' && str[i] <= '9') {
      if (in_decimal) {
        decimal = decimal * 10.0f + (str[i] - '0');
        decimal_div *= 10.0f;
      } else {
        result = result * 10.0f + (str[i] - '0');
      }
    }
  }

  *out = (result + decimal / decimal_div) * (negative ? -1.0f : 1.0f);
  return 1;
}

// Parse int field from NMEA sentence (optimized)
static int parse_int_field(const char *str, size_t len, int *out) {
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

// Optimized GGA parser (no dynamic allocation)
static bool parse_nmea_gga_inline(const char *sentence, size_t len,
                                  gps_data_t *data) {
  // $GPGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*checksum
  if (len < 10 || sentence == NULL)
    return false;

  const char *p = sentence;
  int field = 0;
  bool valid = false;

  // Skip $GPGGA,
  while (*p != ',' && p < sentence + len)
    p++;
  if (*p == ',')
    p++;

  while (p < sentence + len && field < 15) {
    const char *field_start = p;
    const char *field_end = p;

    // Find field end
    while (*field_end != ',' && *field_end != '*' &&
           field_end < sentence + len) {
      field_end++;
    }
    size_t field_len = field_end - field_start;

    switch (field) {
    case 1: // Time (skip)
      break;
    case 2: { // Latitude
      if (field_len > 0) {
        p = field_end + 1;
        // Find hemisphere (next field)
        const char *hemi_start = p;
        while (*p != ',' && *p != '*' && p < sentence + len)
          p++;
        if (p > hemi_start) {
          char hemi = *hemi_start;
          data->latitude =
              nmea_to_degrees_optimized(field_start, field_len, hemi);
        }
      }
      break;
    }
    case 4: { // Longitude
      if (field_len > 0) {
        p = field_end + 1;
        const char *hemi_start = p;
        while (*p != ',' && *p != '*' && p < sentence + len)
          p++;
        if (p > hemi_start) {
          char hemi = *hemi_start;
          data->longitude =
              nmea_to_degrees_optimized(field_start, field_len, hemi);
        }
      }
      break;
    }
    case 6: { // Fix quality
      int quality = 0;
      if (parse_int_field(field_start, field_len, &quality) && quality > 0) {
        valid = true;
      }
      break;
    }
    case 7: { // Number of satellites
      parse_int_field(field_start, field_len, (int *)&data->satellites);
      break;
    }
    case 8: { // HDOP
      parse_float_field(field_start, field_len, &data->hdop);
      break;
    }
    case 9: { // Altitude
      parse_float_field(field_start, field_len, &data->altitude);
      break;
    }
    }

    if (*field_end == '*')
      break; // End of sentence
    p = field_end + 1;
    field++;
  }

  data->valid = valid;
  return valid;
}

// Optimized RMC parser (no dynamic allocation)
static bool parse_nmea_rmc_inline(const char *sentence, size_t len,
                                  gps_data_t *data) {
  // $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,mag_var,E/W*checksum
  if (len < 10 || sentence == NULL)
    return false;

  const char *p = sentence;
  int field = 0;
  bool valid = false;

  // Skip $GPRMC,
  while (*p != ',' && p < sentence + len)
    p++;
  if (*p == ',')
    p++;

  while (p < sentence + len && field < 12) {
    const char *field_start = p;
    const char *field_end = p;

    // Find field end
    while (*field_end != ',' && *field_end != '*' &&
           field_end < sentence + len) {
      field_end++;
    }
    size_t field_len = field_end - field_start;

    switch (field) {
    case 1: // Time (HHMMSS.sss format)
      if (field_len >= 6) {
        // Parse HHMMSS
        int time_val = 0;
        for (size_t i = 0; i < 6 && i < field_len; i++) {
          if (field_start[i] >= '0' && field_start[i] <= '9') {
            time_val = time_val * 10 + (field_start[i] - '0');
          }
        }
        data->fix_time = time_val;
        
        // Parse individual fields for utc_time
        data->utc_time.hour = (field_start[0] - '0') * 10 + (field_start[1] - '0');
        data->utc_time.minute = (field_start[2] - '0') * 10 + (field_start[3] - '0');
        data->utc_time.second = (field_start[4] - '0') * 10 + (field_start[5] - '0');
        
        // Parse milliseconds if present (after decimal point)
        data->utc_time.millisecond = 0;
        if (field_len > 7 && field_start[6] == '.') {
          int ms = 0;
          int divisor = 1;
          for (size_t i = 7; i < field_len && field_start[i] >= '0' && field_start[i] <= '9'; i++) {
            ms = ms * 10 + (field_start[i] - '0');
            divisor *= 10;
          }
          // Convert to milliseconds (scale to 3 decimal places)
          data->utc_time.millisecond = (ms * 1000) / divisor;
        }
      }
      break;
    case 2: // Status (A=valid, V=invalid)
      valid = (field_len > 0 && *field_start == 'A');
      break;
    case 3: { // Latitude
      if (field_len > 0) {
        p = field_end + 1;
        const char *hemi_start = p;
        while (*p != ',' && *p != '*' && p < sentence + len)
          p++;
        if (p > hemi_start) {
          char hemi = *hemi_start;
          data->latitude =
              nmea_to_degrees_optimized(field_start, field_len, hemi);
        }
      }
      break;
    }
    case 5: { // Longitude
      if (field_len > 0) {
        p = field_end + 1;
        const char *hemi_start = p;
        while (*p != ',' && *p != '*' && p < sentence + len)
          p++;
        if (p > hemi_start) {
          char hemi = *hemi_start;
          data->longitude =
              nmea_to_degrees_optimized(field_start, field_len, hemi);
        }
      }
      break;
    }
    case 7: { // Speed in knots
      float speed_knots = 0.0f;
      if (parse_float_field(field_start, field_len, &speed_knots)) {
        data->speed = speed_knots * 1.852f; // Convert to km/h
      }
      break;
    }
    case 8: { // Course
      parse_float_field(field_start, field_len, &data->course);
      break;
    }
    case 9: { // Date (DDMMYY)
      if (field_len >= 6) {
        int date_val = 0;
        for (size_t i = 0; i < 6 && i < field_len; i++) {
          if (field_start[i] >= '0' && field_start[i] <= '9') {
            date_val = date_val * 10 + (field_start[i] - '0');
          }
        }
        data->fix_date = date_val;
        
        // Parse individual fields for utc_date
        data->utc_date.day = (field_start[0] - '0') * 10 + (field_start[1] - '0');
        data->utc_date.month = (field_start[2] - '0') * 10 + (field_start[3] - '0');
        uint8_t year_2digit = (field_start[4] - '0') * 10 + (field_start[5] - '0');
        // Convert YY to YYYY (assume 2000s for values 00-99)
        data->utc_date.year = 2000 + year_2digit;
      }
      break;
    }
    }

    if (*field_end == '*')
      break; // End of sentence
    p = field_end + 1;
    field++;
  }

  data->valid = valid;
  return valid;
}

bool gps_neo6m_init(void) {
  if (gps_initialized) {
    ESP_LOGW(TAG, "GPS already initialized");
    return true;
  }

  uart_config_t uart_config = {
      .baud_rate = GPS_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret =
      uart_driver_install(GPS_UART_NUM, GPS_BUFFER_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
    return false;
  }

  ret = uart_param_config(GPS_UART_NUM, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
    uart_driver_delete(GPS_UART_NUM);
    return false;
  }

  ret = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
    uart_driver_delete(GPS_UART_NUM);
    return false;
  }

  gps_initialized = true;
  ESP_LOGI(TAG, "GPS initialized successfully (UART%d, %d baud)", GPS_UART_NUM,
           GPS_BAUD_RATE);
  return true;
}

bool gps_neo6m_read(gps_data_t *data, uint32_t timeout_ms) {
  if (!gps_initialized || data == NULL) {
    return false;
  }

  // Initialize data structure
  memset(data, 0, sizeof(gps_data_t));
  data->valid = false;

  uint8_t buffer[GPS_BUFFER_SIZE];
  int len = uart_read_bytes(GPS_UART_NUM, buffer, GPS_BUFFER_SIZE - 1,
                            pdMS_TO_TICKS(timeout_ms));

  if (len <= 0) {
    return false;
  }

  buffer[len] = '\0';

  // Parse NMEA sentences (optimized - no strstr/strncmp overhead)
  bool found_valid = false;
  const char *p = (const char *)buffer;
  const char *end = p + len;

  while (p < end) {
    // Find sentence start
    while (p < end && *p != '$')
      p++;
    if (p >= end)
      break;

    const char *sentence_start = p;

    // Find sentence end (\r\n or end of buffer)
    const char *sentence_end = sentence_start;
    while (sentence_end < end &&
           !(sentence_end[0] == '\r' && sentence_end[1] == '\n') &&
           sentence_end < sentence_start + NMEA_SENTENCE_MAX_LEN) {
      sentence_end++;
    }

    if (sentence_end >= end)
      break;

    size_t sentence_len = sentence_end - sentence_start;

    // Check sentence type (optimized - compare first 6 chars)
    if (sentence_len >= 6) {
      if (sentence_start[1] == 'G' && sentence_start[2] == 'P' &&
          sentence_start[3] == 'R' && sentence_start[4] == 'M' &&
          sentence_start[5] == 'C') {
        // GPRMC sentence
        if (parse_nmea_rmc_inline(sentence_start, sentence_len, data)) {
          found_valid = true;
        }
      } else if (sentence_start[1] == 'G' && sentence_start[2] == 'P' &&
                 sentence_start[3] == 'G' && sentence_start[4] == 'G' &&
                 sentence_start[5] == 'A') {
        // GPGGA sentence - merge data if RMC not found
        gps_data_t gga_data = {0};
        if (parse_nmea_gga_inline(sentence_start, sentence_len, &gga_data)) {
          if (!found_valid) {
            // Use GGA data if RMC not available
            *data = gga_data;
            found_valid = true;
          } else {
            // Merge additional GGA fields
            data->satellites = gga_data.satellites;
            data->hdop = gga_data.hdop;
            data->altitude = gga_data.altitude;
          }
        }
      }
    }

    // Move to next sentence
    p = sentence_end + 2;
  }

  // Update fix tracking
  if (found_valid) {
    has_valid_fix = true;
    last_fix_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  }

  return found_valid;
}

bool gps_neo6m_set_power_mode(bool enable) {
  if (!gps_initialized) {
    return false;
  }

  // Send UBX command to enable/disable GPS
  // PM2 - Power Management 2 message
  if (enable) {
    // Wake up GPS (send any command to wake it)
    const uint8_t wake_cmd[] = {0xB5, 0x62, 0x06, 0x04, 0x04,
                                0x00, 0x00, 0x00, 0x00, 0x00};
    uart_write_bytes(GPS_UART_NUM, wake_cmd, sizeof(wake_cmd));
  } else {
    // Put GPS in backup mode (low power)
    const uint8_t backup_cmd[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
                                  0x00, 0x00, 0x4D, 0x3B};
    uart_write_bytes(GPS_UART_NUM, backup_cmd, sizeof(backup_cmd));
  }

  return true;
}

bool gps_neo6m_send_ubx_command(const uint8_t *cmd, size_t cmd_len) {
  if (!gps_initialized || cmd == NULL || cmd_len == 0) {
    return false;
  }

  int written = uart_write_bytes(GPS_UART_NUM, cmd, cmd_len);
  return (written == (int)cmd_len);
}

bool gps_neo6m_set_update_rate(uint8_t rate_hz) {
  if (!gps_initialized || rate_hz == 0 || rate_hz > 10) {
    return false;
  }

  uint16_t meas_rate_ms = 1000 / rate_hz;
  
  // Build UBX CFG-RATE message
  uint8_t cmd[14];
  cmd[0] = 0xB5; cmd[1] = 0x62;  // Sync
  cmd[2] = 0x06; cmd[3] = 0x08;  // CFG-RATE
  cmd[4] = 0x06; cmd[5] = 0x00;  // Length = 6
  cmd[6] = meas_rate_ms & 0xFF;
  cmd[7] = (meas_rate_ms >> 8) & 0xFF;
  cmd[8] = 0x01; cmd[9] = 0x00;  // navRate = 1
  cmd[10] = 0x01; cmd[11] = 0x00; // timeRef = GPS
  
  // Calculate checksum
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 12; i++) {
    ck_a += cmd[i];
    ck_b += ck_a;
  }
  cmd[12] = ck_a;
  cmd[13] = ck_b;
  
  return gps_neo6m_send_ubx_command(cmd, sizeof(cmd));
}

bool gps_neo6m_set_nav_mode(gps_nav_mode_t mode) {
  if (!gps_initialized) {
    return false;
  }

  // Simplified CFG-NAV5 message for dynamic model only
  uint8_t cmd[44];
  memset(cmd, 0, sizeof(cmd));
  
  cmd[0] = 0xB5; cmd[1] = 0x62;  // Sync
  cmd[2] = 0x06; cmd[3] = 0x24;  // CFG-NAV5
  cmd[4] = 0x24; cmd[5] = 0x00;  // Length = 36
  cmd[6] = 0x01; cmd[7] = 0x00;  // Mask: dynModel only
  cmd[8] = mode;                  // dynModel
  cmd[9] = 0x03;                  // fixMode: Auto 2D/3D
  
  // Calculate checksum
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 42; i++) {
    ck_a += cmd[i];
    ck_b += ck_a;
  }
  cmd[42] = ck_a;
  cmd[43] = ck_b;
  
  return gps_neo6m_send_ubx_command(cmd, sizeof(cmd));
}

bool gps_neo6m_cold_start(void) {
  if (!gps_initialized) {
    return false;
  }

  // CFG-RST with cold start
  const uint8_t cmd[] = {
    0xB5, 0x62,             // Sync
    0x06, 0x04,             // CFG-RST
    0x04, 0x00,             // Length = 4
    0xFF, 0xFF,             // navBbrMask = 0xFFFF (clear all)
    0x01,                   // resetMode = controlled reset
    0x00,                   // reserved
    0x0E, 0x66              // Checksum
  };
  
  return gps_neo6m_send_ubx_command(cmd, sizeof(cmd));
}

bool gps_neo6m_has_fix(void) {
  return has_valid_fix;
}

uint32_t gps_neo6m_time_since_fix(void) {
  if (last_fix_time_ms == 0) {
    return UINT32_MAX;
  }
  
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return now - last_fix_time_ms;
}

void gps_neo6m_deinit(void) {
  if (gps_initialized) {
    uart_driver_delete(GPS_UART_NUM);
    gps_initialized = false;
    has_valid_fix = false;
    last_fix_time_ms = 0;
    ESP_LOGI(TAG, "GPS deinitialized");
  }
}
