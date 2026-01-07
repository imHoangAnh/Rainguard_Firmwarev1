/**
 * @file gps_neo6m.c
 * @brief NEO-6M GPS module NMEA parser implementation
 */

#include "gps_neo6m.h"
#include "pin_config.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "gps_neo6m";
static bool gps_initialized = false;

#define GPS_BUFFER_SIZE 512
#define NMEA_SENTENCE_MAX_LEN 82

// NMEA sentence parsing helpers
static bool parse_nmea_gga(const char *sentence, gps_data_t *data);
static bool parse_nmea_rmc(const char *sentence, gps_data_t *data);
static float nmea_to_degrees(const char *nmea_coord, char hemisphere);

static float nmea_to_degrees(const char *nmea_coord, char hemisphere)
{
    // NMEA format: DDMM.MMMM or DDDMM.MMMM
    float coord = atof(nmea_coord);
    int degrees = (int)(coord / 100);
    float minutes = coord - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0f);
    
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    
    return decimal_degrees;
}

static bool parse_nmea_gga(const char *sentence, gps_data_t *data)
{
    // $GPGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*checksum
    char *token;
    char *sentence_copy = strdup(sentence);
    if (sentence_copy == NULL) return false;
    
    char *saveptr;
    int field = 0;
    bool valid = false;
    
    token = strtok_r(sentence_copy, ",", &saveptr);
    while (token != NULL && field < 10) {
        switch (field) {
            case 1: // Time (skip)
                break;
            case 2: // Latitude
                if (strlen(token) > 0) {
                    char *lat_hemi = strtok_r(NULL, ",", &saveptr);
                    if (lat_hemi != NULL) {
                        data->latitude = nmea_to_degrees(token, lat_hemi[0]);
                        field++; // Skip hemisphere field
                    }
                }
                break;
            case 4: // Longitude
                if (strlen(token) > 0) {
                    char *lon_hemi = strtok_r(NULL, ",", &saveptr);
                    if (lon_hemi != NULL) {
                        data->longitude = nmea_to_degrees(token, lon_hemi[0]);
                        field++; // Skip hemisphere field
                    }
                }
                break;
            case 6: // Fix quality
                if (atoi(token) > 0) {
                    valid = true;
                }
                break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }
    
    free(sentence_copy);
    data->valid = valid;
    return valid;
}

static bool parse_nmea_rmc(const char *sentence, gps_data_t *data)
{
    // $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,mag_var,E/W*checksum
    char *token;
    char *sentence_copy = strdup(sentence);
    if (sentence_copy == NULL) return false;
    
    char *saveptr;
    int field = 0;
    bool valid = false;
    
    token = strtok_r(sentence_copy, ",", &saveptr);
    while (token != NULL && field < 12) {
        switch (field) {
            case 1: // Time (skip)
                break;
            case 2: // Status (A=valid, V=invalid)
                valid = (token[0] == 'A');
                break;
            case 3: // Latitude
                if (strlen(token) > 0) {
                    char *lat_hemi = strtok_r(NULL, ",", &saveptr);
                    if (lat_hemi != NULL) {
                        data->latitude = nmea_to_degrees(token, lat_hemi[0]);
                        field++; // Skip hemisphere field
                    }
                }
                break;
            case 5: // Longitude
                if (strlen(token) > 0) {
                    char *lon_hemi = strtok_r(NULL, ",", &saveptr);
                    if (lon_hemi != NULL) {
                        data->longitude = nmea_to_degrees(token, lon_hemi[0]);
                        field++; // Skip hemisphere field
                    }
                }
                break;
            case 7: // Speed in knots
                if (strlen(token) > 0) {
                    float speed_knots = atof(token);
                    data->speed = speed_knots * 1.852f; // Convert to km/h
                }
                break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }
    
    free(sentence_copy);
    data->valid = valid;
    return valid;
}

bool gps_neo6m_init(void)
{
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

    esp_err_t ret = uart_driver_install(GPS_UART_NUM, GPS_BUFFER_SIZE * 2, 0, 0, NULL, 0);
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

    ret = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(GPS_UART_NUM);
        return false;
    }

    gps_initialized = true;
    ESP_LOGI(TAG, "GPS initialized successfully (UART%d, %d baud)", GPS_UART_NUM, GPS_BAUD_RATE);
    return true;
}

bool gps_neo6m_read(gps_data_t *data)
{
    if (!gps_initialized || data == NULL) {
        return false;
    }

    // Initialize data structure
    memset(data, 0, sizeof(gps_data_t));
    data->valid = false;

    uint8_t buffer[GPS_BUFFER_SIZE];
    int len = uart_read_bytes(GPS_UART_NUM, buffer, GPS_BUFFER_SIZE - 1, pdMS_TO_TICKS(1000));
    
    if (len <= 0) {
        return false;
    }

    buffer[len] = '\0';

    // Parse NMEA sentences
    char *line = (char *)buffer;
    char *line_end;
    bool found_valid = false;

    while ((line_end = strstr(line, "\r\n")) != NULL) {
        *line_end = '\0';
        
        // Check for GPRMC or GPGGA sentences
        if (strncmp(line, "$GPRMC", 6) == 0) {
            if (parse_nmea_rmc(line, data)) {
                found_valid = true;
            }
        } else if (strncmp(line, "$GPGGA", 6) == 0) {
            gps_data_t gga_data = {0};
            if (parse_nmea_gga(line, &gga_data)) {
                // Merge GGA data (prefer RMC for speed, but GGA for position if RMC not available)
                if (!found_valid) {
                    data->latitude = gga_data.latitude;
                    data->longitude = gga_data.longitude;
                    data->valid = gga_data.valid;
                    found_valid = gga_data.valid;
                }
            }
        }
        
        line = line_end + 2;
        if (line >= (char *)buffer + len) break;
    }

    return found_valid;
}

void gps_neo6m_deinit(void)
{
    if (gps_initialized) {
        uart_driver_delete(GPS_UART_NUM);
        gps_initialized = false;
        ESP_LOGI(TAG, "GPS deinitialized");
    }
}

