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

// GPS Configuration (UART1)
// Note: GPIO43/44 are used for Console UART (U0TXD/U0RXD)
// Using GPIO41 (MTDI) and GPIO42 (MTMS) - JTAG pins repurposed for GPS
// Warning: These are JTAG pins, JTAG debugging will not work when GPS is
// connected
#define GPS_TX_PIN 41
#define GPS_RX_PIN 42
#define GPS_UART_NUM UART_NUM_1
#define GPS_BAUD_RATE 9600

// Camera Pin Configuration (OV2640)
#define CAM_PIN_XCLK 15 // XCLK output
#define CAM_PIN_SIOD 4  // SCCB Data (SDA)
#define CAM_PIN_SIOC 5  // SCCB Clock (SCL)
#define CAM_PIN_VSYNC 6 // VSYNC
#define CAM_PIN_HREF 7  // HREF
#define CAM_PIN_PCLK 15 // Pixel Clock

// Camera Data Pins (D0-D7)
#define CAM_PIN_Y2 8  // D0
#define CAM_PIN_Y3 9  // D1
#define CAM_PIN_Y4 11 // D2
#define CAM_PIN_Y5 10 // D3
#define CAM_PIN_Y6 12 // D4
#define CAM_PIN_Y7 13 // D5
#define CAM_PIN_Y8 14 // D6
#define CAM_PIN_Y9 18 // D7

// Camera Control Pins
#define CAM_PIN_RESET 46 // Reset pin
#define CAM_PIN_PWDN 3   // Power down pin

// Sensor I2C Addresses
// BME680 can be 0x76 (SDO to GND) or 0x77 (SDO to VCC)
// Driver will auto-detect both addresses
#ifndef BME680_I2C_ADDR
#define BME680_I2C_ADDR 0x77
#endif
#define MPU6050_I2C_ADDR 0x68

#ifdef __cplusplus
}
#endif

#endif // PIN_CONFIG_H
