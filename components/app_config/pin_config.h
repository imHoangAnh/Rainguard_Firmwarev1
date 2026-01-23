/**
 * @file pin_config.h
 * @brief Pin definitions for RainGuard Firmware
 * @details Hardware pin assignments for ESP32-S3
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C Configuration
#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 2
#define I2C_FREQ_HZ 100000 // 100kHz

#define GPS_TX_PIN 41
#define GPS_RX_PIN 42
#define GPS_UART_NUM UART_NUM_1
#define GPS_BAUD_RATE 9600

// GPS NEO-7M Optional Configuration
#define GPS_UPDATE_RATE_HZ 1   // Default 1Hz (1-10Hz supported)
#define GPS_ENABLE_GLONASS 1   // Enable GLONASS for multi-GNSS
#define GPS_ENABLE_SBAS 1      // Enable SBAS (WAAS/EGNOS/MSAS)
#define GPS_ENABLE_ASSISTNOW 1 // Enable AssistNow Autonomous

// Camera Pin Configuration (OV2640)
#define CAM_PIN_XCLK 15 // XCLK output
#define CAM_PIN_SIOD 4  // SCCB Data (SDA)
#define CAM_PIN_SIOC 5  // SCCB Clock (SCL)
#define CAM_PIN_VSYNC 6 // VSYNC
#define CAM_PIN_HREF 7  // HREF
#define CAM_PIN_PCLK 13 // Pixel Clock

// Camera Data Pins (D0-D7)
#define CAM_PIN_Y2 11 // D0
#define CAM_PIN_Y3 9  // D1
#define CAM_PIN_Y4 8  // D2
#define CAM_PIN_Y5 10 // D3
#define CAM_PIN_Y6 12 // D4
#define CAM_PIN_Y7 18 // D5
#define CAM_PIN_Y8 17 // D6
#define CAM_PIN_Y9 16 // D7

// Camera Control Pins
#define CAM_PIN_RESET -1 // Not connected
#define CAM_PIN_PWDN -1  // Not connected

// Buzzer Configuration
#define BUZZER_GPIO_PIN 45

// Sensor I2C Addresses
#ifndef BME680_I2C_ADDR
#define BME680_I2C_ADDR 0x77
#endif
#define MPU6050_I2C_ADDR 0x68

#ifdef __cplusplus
}
#endif

#endif // PIN_CONFIG_H
