/**
 * @file gps_neo7m.c
 * @brief u-blox NEO-7M GNSS Module Driver (Ultra-Lightweight) for ESP-IDF
 * @details Minimal GNSS driver for real-time tracking
 *
 * Supported NMEA Sentences:
 * - GGA: GPS Fix Data (position, altitude, satellites, HDOP)
 * - RMC: Recommended Minimum (position, speed, course, date/time)
 * - VTG: Course Over Ground and Ground Speed
 * - GSA: DOP values
 *
 * @author RainGuard Team
 * @version 3.0.0 (Ultra-Lightweight)
 * @date 2026-01-19
 */

#include "gps_neo7m.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pin_config.h"
#include <stdio.h>
#include <string.h>

// =============================================================================
// CONSTANTS
// =============================================================================

static const char *TAG = "gps";

#define NMEA_MAX_FIELDS 16
#define NMEA_MAX_LEN 82
#define KNOTS_TO_KMH 1.852f

// =============================================================================
// MODULE STATE
// =============================================================================

static bool gps_initialized = false;
static bool has_valid_fix = false;
static SemaphoreHandle_t gps_mutex = NULL;
static uint8_t uart_buffer[GPS_BUFFER_SIZE];

// =============================================================================
// NMEA PARSING UTILITIES
// =============================================================================

static bool nmea_validate_checksum(const char *sentence) {
  const char *star = strchr(sentence, '*');
  if (star == NULL || strlen(star) < 3)
    return false;

  uint8_t checksum = 0;
  const char *p = sentence;
  if (*p == '$')
    p++;
  while (*p != '\0' && *p != '*')
    checksum ^= *p++;

  unsigned int expected = 0;
  char hex[3] = {star[1], star[2], '\0'};
  if (sscanf(hex, "%02X", &expected) != 1)
    return false;

  return (checksum == (uint8_t)expected);
}

static double nmea_to_degrees(const char *nmea, size_t len, char hem) {
  if (len == 0 || nmea == NULL)
    return 0.0;

  const char *dot = NULL;
  for (size_t i = 0; i < len && nmea[i] != '\0'; i++) {
    if (nmea[i] == '.') {
      dot = &nmea[i];
      break;
    }
  }
  if (dot == NULL)
    return 0.0;

  size_t before_dot = dot - nmea;
  if (before_dot < 2)
    return 0.0;

  // Parse degrees
  int deg = 0;
  for (size_t i = 0; i < before_dot - 2; i++) {
    if (nmea[i] >= '0' && nmea[i] <= '9')
      deg = deg * 10 + (nmea[i] - '0');
  }

  // Parse minutes
  int min_int =
      (nmea[before_dot - 2] - '0') * 10 + (nmea[before_dot - 1] - '0');
  double min_frac = 0.0, div = 10.0;
  for (const char *p = dot + 1; *p && *p != ',' && (p - nmea) < (int)len; p++) {
    if (*p >= '0' && *p <= '9') {
      min_frac += (*p - '0') / div;
      div *= 10.0;
    } else
      break;
  }

  double result = (double)deg + ((double)min_int + min_frac) / 60.0;
  return (hem == 'S' || hem == 'W') ? -result : result;
}

static int parse_float(const char *str, size_t len, float *out) {
  if (!str || len == 0 || *str == '\0' || *str == ',')
    return 0;

  float result = 0.0f, frac = 0.0f, frac_div = 10.0f;
  bool neg = false, in_frac = false;

  for (size_t i = 0; i < len && str[i] && str[i] != ','; i++) {
    if (str[i] == '-')
      neg = true;
    else if (str[i] == '.')
      in_frac = true;
    else if (str[i] >= '0' && str[i] <= '9') {
      if (in_frac) {
        frac += (str[i] - '0') / frac_div;
        frac_div *= 10.0f;
      } else {
        result = result * 10.0f + (str[i] - '0');
      }
    }
  }

  *out = (result + frac) * (neg ? -1.0f : 1.0f);
  return 1;
}

static int parse_int(const char *str, size_t len, int *out) {
  if (!str || len == 0 || *str == '\0' || *str == ',')
    return 0;

  int result = 0;
  for (size_t i = 0; i < len && str[i] && str[i] != ','; i++) {
    if (str[i] >= '0' && str[i] <= '9')
      result = result * 10 + (str[i] - '0');
  }
  *out = result;
  return 1;
}

static int split_fields(const char *sentence, const char **fields, size_t *lens,
                        int max) {
  int count = 0;
  const char *p = sentence;

  // Skip header
  while (*p && *p != ',')
    p++;
  if (*p == ',')
    p++;

  while (*p && *p != '*' && count < max) {
    fields[count] = p;
    const char *end = p;
    while (*end && *end != ',' && *end != '*')
      end++;
    lens[count] = end - p;
    count++;
    p = (*end == ',') ? end + 1 : end;
  }
  return count;
}

static bool is_type(const char *sentence, const char *type) {
  return sentence && strlen(sentence) >= 6 && sentence[0] == '$' &&
         sentence[3] == type[0] && sentence[4] == type[1] &&
         sentence[5] == type[2];
}

// =============================================================================
// NMEA PARSERS
// =============================================================================

static bool parse_gga(const char *sentence, gps_data_t *data) {
  const char *f[NMEA_MAX_FIELDS];
  size_t fl[NMEA_MAX_FIELDS];

  int c = split_fields(sentence, f, fl, NMEA_MAX_FIELDS);
  if (c < 10)
    return false;

  // Quality check
  int q = 0;
  if (!parse_int(f[5], fl[5], &q) || q == 0)
    return false;

  // Time
  if (fl[0] >= 6) {
    data->utc_time.hour = (f[0][0] - '0') * 10 + (f[0][1] - '0');
    data->utc_time.minute = (f[0][2] - '0') * 10 + (f[0][3] - '0');
    data->utc_time.second = (f[0][4] - '0') * 10 + (f[0][5] - '0');
    if (fl[0] > 7 && f[0][6] == '.') {
      int ms = 0, div = 1;
      for (size_t i = 7; i < fl[0] && f[0][i] >= '0' && f[0][i] <= '9'; i++) {
        ms = ms * 10 + (f[0][i] - '0');
        div *= 10;
      }
      data->utc_time.millisecond = (ms * 1000) / div;
    }
  }

  // Position
  if (fl[1] > 0 && fl[2] > 0)
    data->latitude = nmea_to_degrees(f[1], fl[1], f[2][0]);
  if (fl[3] > 0 && fl[4] > 0)
    data->longitude = nmea_to_degrees(f[3], fl[3], f[4][0]);

  // Satellites
  if (fl[6] > 0) {
    int sats = 0;
    parse_int(f[6], fl[6], &sats);
    data->satellites_used = (uint8_t)sats;
  }

  // HDOP
  if (fl[7] > 0)
    parse_float(f[7], fl[7], &data->hdop);

  // Altitude
  if (fl[8] > 0)
    parse_float(f[8], fl[8], &data->altitude);

  data->valid = true;
  return true;
}

static bool parse_rmc(const char *sentence, gps_data_t *data) {
  const char *f[NMEA_MAX_FIELDS];
  size_t fl[NMEA_MAX_FIELDS];

  int c = split_fields(sentence, f, fl, NMEA_MAX_FIELDS);
  if (c < 9)
    return false;

  bool valid = (fl[1] > 0 && f[1][0] == 'A');

  // Time
  if (fl[0] >= 6) {
    data->utc_time.hour = (f[0][0] - '0') * 10 + (f[0][1] - '0');
    data->utc_time.minute = (f[0][2] - '0') * 10 + (f[0][3] - '0');
    data->utc_time.second = (f[0][4] - '0') * 10 + (f[0][5] - '0');
  }

  // Position
  if (fl[2] > 0 && fl[3] > 0)
    data->latitude = nmea_to_degrees(f[2], fl[2], f[3][0]);
  if (fl[4] > 0 && fl[5] > 0)
    data->longitude = nmea_to_degrees(f[4], fl[4], f[5][0]);

  // Speed & Course
  if (fl[6] > 0) {
    float knots = 0;
    parse_float(f[6], fl[6], &knots);
    data->speed_kmh = knots * KNOTS_TO_KMH;
  }
  if (fl[7] > 0)
    parse_float(f[7], fl[7], &data->course);

  // Date
  if (fl[8] >= 6) {
    data->utc_date.day = (f[8][0] - '0') * 10 + (f[8][1] - '0');
    data->utc_date.month = (f[8][2] - '0') * 10 + (f[8][3] - '0');
    data->utc_date.year = 2000 + (f[8][4] - '0') * 10 + (f[8][5] - '0');
  }

  data->valid = valid;
  return valid;
}

static void parse_vtg(const char *sentence, gps_data_t *data) {
  const char *f[NMEA_MAX_FIELDS];
  size_t fl[NMEA_MAX_FIELDS];

  int c = split_fields(sentence, f, fl, NMEA_MAX_FIELDS);
  if (c < 8)
    return;

  if (fl[0] > 0 && data->course == 0.0f)
    parse_float(f[0], fl[0], &data->course);
  if (fl[6] > 0)
    parse_float(f[6], fl[6], &data->speed_kmh);
}

static void parse_gsa(const char *sentence, gps_data_t *data) {
  const char *f[NMEA_MAX_FIELDS];
  size_t fl[NMEA_MAX_FIELDS];

  int c = split_fields(sentence, f, fl, NMEA_MAX_FIELDS);
  if (c < 16)
    return;

  if (fl[14] > 0)
    parse_float(f[14], fl[14], &data->hdop);
}

// =============================================================================
// PUBLIC API
// =============================================================================

bool gps_neo7m_init(void) {
  if (gps_initialized) {
    ESP_LOGW(TAG, "Already initialized");
    return true;
  }

  ESP_LOGI(TAG, "Initializing GPS NEO-7M...");

  if (gps_mutex == NULL) {
    gps_mutex = xSemaphoreCreateMutex();
    if (gps_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create mutex");
      return false;
    }
  }

  uart_config_t cfg = {
      .baud_rate = GPS_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret = uart_driver_install(GPS_UART_NUM, GPS_BUFFER_SIZE * 2,
                                      GPS_BUFFER_SIZE, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART install failed");
    return false;
  }

  uart_param_config(GPS_UART_NUM, &cfg);
  uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  uart_flush(GPS_UART_NUM);

  gps_initialized = true;
  ESP_LOGI(TAG, "GPS initialized (UART%d TX=%d RX=%d)", GPS_UART_NUM,
           GPS_TX_PIN, GPS_RX_PIN);

  return true;
}

void gps_neo7m_deinit(void) {
  if (gps_initialized) {
    uart_flush(GPS_UART_NUM);
    uart_driver_delete(GPS_UART_NUM);
    gps_initialized = false;
    has_valid_fix = false;

    if (gps_mutex) {
      vSemaphoreDelete(gps_mutex);
      gps_mutex = NULL;
    }

    ESP_LOGI(TAG, "GPS deinitialized");
  }
}

bool gps_neo7m_read(gps_data_t *data, uint32_t timeout_ms) {
  if (!gps_initialized || !data)
    return false;

  if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    return false;

  memset(data, 0, sizeof(gps_data_t));

  int len = uart_read_bytes(GPS_UART_NUM, uart_buffer, GPS_BUFFER_SIZE - 1,
                            pdMS_TO_TICKS(timeout_ms));
  if (len <= 0) {
    xSemaphoreGive(gps_mutex);
    return false;
  }

  uart_buffer[len] = '\0';

  bool found_rmc = false, found_gga = false;
  gps_data_t rmc = {0}, gga = {0};

  const char *p = (const char *)uart_buffer;
  const char *end = p + len;

  while (p < end) {
    // Find '$'
    while (p < end && *p != '$')
      p++;
    if (p >= end)
      break;

    const char *start = p;
    const char *sent_end = p;
    while (sent_end < end && !(*sent_end == '\r') &&
           (sent_end - start) < NMEA_MAX_LEN) {
      sent_end++;
    }

    if (sent_end >= end || (sent_end - start) > NMEA_MAX_LEN) {
      p++;
      continue;
    }

    // Copy sentence
    char sentence[NMEA_MAX_LEN + 1];
    size_t slen = sent_end - start;
    if (slen > NMEA_MAX_LEN)
      slen = NMEA_MAX_LEN;
    memcpy(sentence, start, slen);
    sentence[slen] = '\0';

    // Validate
    if (!nmea_validate_checksum(sentence)) {
      p = sent_end + 2;
      continue;
    }

    // Parse
    if (is_type(sentence, "RMC")) {
      found_rmc = parse_rmc(sentence, &rmc);
    } else if (is_type(sentence, "GGA")) {
      found_gga = parse_gga(sentence, &gga);
    } else if (is_type(sentence, "VTG")) {
      parse_vtg(sentence, data);
    } else if (is_type(sentence, "GSA")) {
      parse_gsa(sentence, data);
    }

    p = sent_end + 2;
  }

  // Merge RMC + GGA
  if (found_rmc) {
    *data = rmc;
    if (found_gga) {
      data->satellites_used = gga.satellites_used;
      data->hdop = gga.hdop;
      data->altitude = gga.altitude;
      if (data->latitude == 0.0 && gga.latitude != 0.0) {
        data->latitude = gga.latitude;
        data->longitude = gga.longitude;
      }
    }
  } else if (found_gga) {
    *data = gga;
  }

  if (data->valid) {
    has_valid_fix = true;
  }

  xSemaphoreGive(gps_mutex);
  return found_rmc || found_gga;
}

bool gps_neo7m_has_fix(void) { return has_valid_fix; }
