/**
 * @file main.c
 * @brief RainGuard Firmware - Main application entry point
 * @details Orchestrates WiFi, MQTT, sensors, GPS, and camera tasks
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

// Component includes
#include "app_network.h"
#include "cam_config.h"
#include "gps_neo6m.h"
#include "pin_config.h"
#include "sensor_bme680.h"
#include "sensor_mpu6050.h"
#include "system_i2c.h"

static const char *TAG = "main";

// Configuration from Kconfig (accessed via CONFIG_ prefix)

// Task handles
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t camera_task_handle = NULL;

/**
 * @brief Sensor reading and MQTT publishing task
 */
static void sensor_task(void *pvParameters) {
  ESP_LOGI(TAG, "Sensor task started");

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(CONFIG_SENSOR_INTERVAL_MS);

  bme680_data_t bme_data = {0};
  mpu6050_data_t mpu_data = {0};
  gps_data_t gps_data = {0};

  char json_buffer[512];

  while (1) {
    // Read BME680 (optimized with gas sensor)
    if (sensor_bme680_read(&bme_data)) {
      ESP_LOGD(
          TAG,
          "BME680: T=%.2f°C, H=%.2f%%, P=%.2fhPa, Gas=%.0fΩ, IAQ=%.1f (acc:%d)",
          bme_data.temperature, bme_data.humidity, bme_data.pressure,
          bme_data.gas_resistance, bme_data.iaq, bme_data.iaq_accuracy);
    } else {
      ESP_LOGW(TAG, "Failed to read BME680");
    }

    // Read MPU6050 (with Kalman filter and motion detection)
    if (sensor_mpu6050_read(&mpu_data)) {
      ESP_LOGD(
          TAG,
          "MPU6050: Accel(%.3f, %.3f, %.3f) g, Gyro(%.2f, %.2f, %.2f) deg/s",
          mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z, mpu_data.gyro_x,
          mpu_data.gyro_y, mpu_data.gyro_z);
      ESP_LOGD(
          TAG,
          "MPU6050: Attitude Pitch=%.2f°, Roll=%.2f°, Yaw=%.2f°, Motion=%s",
          mpu_data.pitch, mpu_data.roll, mpu_data.yaw,
          mpu_data.motion_detected ? "YES" : "NO");
    } else {
      ESP_LOGW(TAG, "Failed to read MPU6050");
    }

    // Read GPS (optimized parsing, timeout 1 second)
    if (gps_neo6m_read(&gps_data, 1000)) {
      if (gps_data.valid) {
        ESP_LOGD(TAG,
                 "GPS: Lat=%.6f, Lon=%.6f, Alt=%.1fm, Speed=%.2f km/h, "
                 "Sats=%d, HDOP=%.1f",
                 gps_data.latitude, gps_data.longitude, gps_data.altitude,
                 gps_data.speed, gps_data.satellites, gps_data.hdop);
      } else {
        ESP_LOGD(TAG, "GPS: No fix (Sats=%d)", gps_data.satellites);
      }
    } else {
      ESP_LOGD(TAG, "GPS: No data");
    }

    // Format JSON payload
    int len = snprintf(
        json_buffer, sizeof(json_buffer),
        "{"
        "\"device_id\":\"%s\","
        "\"timestamp\":%lld,"
        "\"bme680\":{"
        "\"temperature\":%.2f,"
        "\"humidity\":%.2f,"
        "\"pressure\":%.2f,"
        "\"gas_resistance\":%.0f,"
        "\"iaq\":%.1f,"
        "\"iaq_accuracy\":%d"
        "},"
        "\"mpu6050\":{"
        "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"attitude\":{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f},"
        "\"motion_detected\":%s"
        "},"
        "\"gps\":{"
        "\"latitude\":%.6f,"
        "\"longitude\":%.6f,"
        "\"altitude\":%.1f,"
        "\"speed\":%.2f,"
        "\"course\":%.2f,"
        "\"satellites\":%d,"
        "\"hdop\":%.1f,"
        "\"valid\":%s"
        "}"
        "}",
        CONFIG_DEVICE_ID,
        (long long)(esp_timer_get_time() / 1000), // milliseconds since boot
        bme_data.temperature, bme_data.humidity, bme_data.pressure,
        bme_data.gas_resistance, bme_data.iaq, bme_data.iaq_accuracy,
        mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z, mpu_data.gyro_x,
        mpu_data.gyro_y, mpu_data.gyro_z, mpu_data.pitch, mpu_data.roll,
        mpu_data.yaw, mpu_data.motion_detected ? "true" : "false",
        gps_data.latitude, gps_data.longitude, gps_data.altitude,
        gps_data.speed, gps_data.course, gps_data.satellites, gps_data.hdop,
        gps_data.valid ? "true" : "false");

    if (len > 0 && len < sizeof(json_buffer)) {
      // Publish to MQTT
      if (app_network_mqtt_publish(json_buffer)) {
        ESP_LOGI(TAG, "Published sensor data to MQTT");
      } else {
        ESP_LOGW(TAG, "Failed to publish sensor data");
      }
    } else {
      ESP_LOGE(TAG, "JSON buffer overflow");
    }

    // Wait for next interval
    vTaskDelayUntil(&last_wake_time, interval);
  }
}

/**
 * @brief Camera capture and Cloudinary upload task
 */
static void camera_task(void *pvParameters) {
  ESP_LOGI(TAG, "Camera task started");

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(CONFIG_CAMERA_INTERVAL_MS);

  while (1) {
    camera_fb_t *fb = NULL;

    // Capture frame
    if (cam_config_capture_frame(&fb)) {
      ESP_LOGI(TAG, "Captured frame: %zux%zu, %zu bytes", fb->width, fb->height,
               fb->len);

      // Upload to Cloudinary
      if (app_network_upload_image(fb)) {
        ESP_LOGI(TAG, "Image uploaded to Cloudinary successfully");
      } else {
        ESP_LOGW(TAG, "Failed to upload image to Cloudinary");
      }

      // Free frame buffer
      cam_config_free_frame(fb);
    } else {
      ESP_LOGW(TAG, "Failed to capture frame");
    }

    // Wait for next interval
    vTaskDelayUntil(&last_wake_time, interval);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "RainGuard Firmware starting...");
  ESP_LOGI(TAG, "Device ID: %s", CONFIG_DEVICE_ID);
  ESP_LOGI(TAG, "Sensor Interval: %d ms", CONFIG_SENSOR_INTERVAL_MS);
  ESP_LOGI(TAG, "Camera Interval: %d ms", CONFIG_CAMERA_INTERVAL_MS);

  // Initialize NVS (required for WiFi)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize network stack
  if (!app_network_init()) {
    ESP_LOGE(TAG, "Failed to initialize network stack");
    return;
  }

  // Wait for WiFi connection (max 10 retries)
  ESP_LOGI(TAG, "Waiting for WiFi connection...");
  if (!app_network_wait_for_wifi(10)) {
    ESP_LOGE(TAG, "Failed to connect to WiFi");
    return;
  }

  // Initialize I2C bus
  if (!system_i2c_init()) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus");
    return;
  }

  // Scan I2C bus to find connected devices
  ESP_LOGI(TAG, "Scanning I2C bus for devices...");
  system_i2c_scan();

  // Initialize sensors
  ESP_LOGI(TAG, "Initializing sensors...");
  if (!sensor_bme680_init()) {
    ESP_LOGE(TAG, "Failed to initialize BME680");
  } else {
    ESP_LOGI(TAG, "BME680 initialized");
    // Configure BME680 with optimized settings
    sensor_bme680_configure(
        BME680_OS_2X,  // Temperature oversampling
        BME680_OS_16X, // Pressure oversampling (high accuracy)
        BME680_OS_1X,  // Humidity oversampling
        3,             // Filter coefficient
        100,           // Gas wait time (ms)
        320            // Gas heater temperature (°C)
    );
  }

  if (!sensor_mpu6050_init()) {
    ESP_LOGE(TAG, "Failed to initialize MPU6050");
  } else {
    ESP_LOGI(TAG, "MPU6050 initialized");
    // Calibrate gyroscope (sensor should be stationary)
    ESP_LOGI(TAG, "Calibrating MPU6050 gyroscope (keep sensor still)...");
    sensor_mpu6050_calibrate_gyro(200);        // 200 samples (~2 seconds)
    sensor_mpu6050_set_motion_threshold(0.2f); // 0.2g threshold
  }

  // Initialize GPS
  ESP_LOGI(TAG, "Initializing GPS...");
  if (!gps_neo6m_init()) {
    ESP_LOGE(TAG, "Failed to initialize GPS");
  } else {
    ESP_LOGI(TAG, "GPS initialized");
  }

  // Initialize camera
  ESP_LOGI(TAG, "Initializing camera...");
  if (!cam_config_init()) {
    ESP_LOGE(TAG, "Failed to initialize camera");
  } else {
    ESP_LOGI(TAG, "Camera initialized");
  }

  // Create sensor task
  xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 5,
                          &sensor_task_handle,
                          1 // Core 1
  );

  // Create camera task
  xTaskCreatePinnedToCore(camera_task, "camera_task",
                          8192, // Larger stack for image processing
                          NULL, 5, &camera_task_handle,
                          0 // Core 0
  );

  ESP_LOGI(TAG, "RainGuard Firmware initialized successfully");
  ESP_LOGI(TAG, "Sensor task running on core 1");
  ESP_LOGI(TAG, "Camera task running on core 0");
}
