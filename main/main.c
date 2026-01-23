/**
 * @file main.c
 * @brief TrainGuard Firmware - Main application
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#include "app_network.h"
#include "buzzer_driver.h"
#include "cam_config.h"
#include "gps_neo7m.h"
#include "network_config.h"
#include "pin_config.h"
#include "sensor_bme680.h"
#include "sensor_mpu6050.h"
#include "system_i2c.h"

static const char *TAG = "main";

static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t camera_task_handle = NULL;

static void sensor_task(void *pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(SENSOR_INTERVAL_MS);

  bme680_data_t bme_data = {0};
  mpu6050_data_t mpu_data = {0};
  gps_data_t gps_data = {0};
  gps_data_t last_valid_gps = {0};
  bool has_last_fix = false;
  gps_data_t smoothed_gps = {0};
  bool has_smoothed = false;
  const float gps_smooth_alpha = 0.2f;
  char json_buffer[650];

  while (1) {
    sensor_bme680_read(&bme_data);
    sensor_mpu6050_read(&mpu_data);

    static uint32_t gps_fail_count = 0;
    gps_data_t current_gps = {0};
    if (gps_neo7m_read(&current_gps, 1000)) {
      gps_fail_count = 0;
      gps_data = current_gps;
      if (current_gps.valid) {
        last_valid_gps = current_gps;
        has_last_fix = true;
      }
    } else {
      gps_fail_count++;
      if (has_last_fix) {
        gps_data = last_valid_gps;
      }
    }

    if (gps_data.valid) {
      if (!has_smoothed) {
        smoothed_gps = gps_data;
        has_smoothed = true;
      } else {
        smoothed_gps.latitude +=
            gps_smooth_alpha * (gps_data.latitude - smoothed_gps.latitude);
        smoothed_gps.longitude +=
            gps_smooth_alpha * (gps_data.longitude - smoothed_gps.longitude);
        smoothed_gps.altitude +=
            gps_smooth_alpha * (gps_data.altitude - smoothed_gps.altitude);
        smoothed_gps.speed_kmh +=
            gps_smooth_alpha * (gps_data.speed_kmh - smoothed_gps.speed_kmh);
      }
      gps_data.latitude = smoothed_gps.latitude;
      gps_data.longitude = smoothed_gps.longitude;
      gps_data.altitude = smoothed_gps.altitude;
      gps_data.speed_kmh = smoothed_gps.speed_kmh;
    }

    int vn_hour = gps_data.utc_time.hour + 7;
    int vn_day = gps_data.utc_date.day;
    int vn_month = gps_data.utc_date.month;
    int vn_year = gps_data.utc_date.year;

    if (vn_hour >= 24) {
      vn_hour -= 24;
      vn_day++;
      int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      if ((vn_year % 4 == 0 && vn_year % 100 != 0) || (vn_year % 400 == 0)) {
        days_in_month[1] = 29;
      }
      if (vn_month >= 1 && vn_month <= 12 &&
          vn_day > days_in_month[vn_month - 1]) {
        vn_day = 1;
        vn_month++;
        if (vn_month > 12) {
          vn_month = 1;
          vn_year++;
        }
      }
    }

    int len = snprintf(
        json_buffer, sizeof(json_buffer),
        "{\"device_id\":\"%s\",\"timestamp\":%lld,"
        "\"bme680\":{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,"
        "\"gas_resistance\":%.0f,\"iaq\":%.1f,\"iaq_accuracy\":%d,"
        "\"gas_valid\":%s,\"heat_stable\":%s},"
        "\"mpu6050\":{\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"attitude\":{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f},"
        "\"vibration\":%.1f},"
        "\"gps\":{\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.1f,"
        "\"speed\":%.2f,\"date\":\"%04d-%02d-%02d\","
        "\"time\":\"%02d:%02d:%02d.%03d\",\"valid\":%s}}",
        DEVICE_ID, (long long)(esp_timer_get_time() / 1000),
        bme_data.temperature, bme_data.humidity, bme_data.pressure,
        bme_data.gas_resistance, bme_data.iaq, bme_data.iaq_accuracy,
        bme_data.gas_valid ? "true" : "false",
        bme_data.heat_stable ? "true" : "false", mpu_data.accel_x,
        mpu_data.accel_y, mpu_data.accel_z, mpu_data.gyro_x, mpu_data.gyro_y,
        mpu_data.gyro_z, mpu_data.pitch, mpu_data.roll, mpu_data.yaw,
        mpu_data.vibration, gps_data.latitude, gps_data.longitude,
        gps_data.altitude, gps_data.speed_kmh, vn_year, vn_month, vn_day,
        vn_hour, gps_data.utc_time.minute, gps_data.utc_time.second,
        gps_data.utc_time.millisecond, gps_data.valid ? "true" : "false");

    if (len > 0 && len < sizeof(json_buffer)) {
      if (app_network_mqtt_publish(json_buffer)) {
        ESP_LOGI(TAG, "MQTT published successfully");
      } else {
        ESP_LOGE(TAG, "MQTT published unsuccessfully");
      }
    }

    vTaskDelayUntil(&last_wake_time, interval);
  }
}

static void camera_task(void *pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(CAMERA_INTERVAL_MS);

  while (1) {
    if (!cam_config_is_initialized()) {
      vTaskDelayUntil(&last_wake_time, interval);
      continue;
    }

    camera_fb_t *fb = NULL;
    if (cam_config_capture_frame(&fb)) {
      ESP_LOGI(TAG, "Captured: %zux%zu, %zu bytes", fb->width, fb->height,
               fb->len);
      if (app_network_upload_image(fb)) {
        ESP_LOGI(TAG, "Uploaded image to Cloudinary successfully");
      }
      cam_config_free_frame(fb);
    }

    vTaskDelayUntil(&last_wake_time, interval);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "TrainGuard starting...");

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  if (!app_network_init()) {
    ESP_LOGE(TAG, "Network init failed");
    return;
  }

  if (!app_network_wait_for_wifi(10)) {
    ESP_LOGE(TAG, "WiFi failed");
    return;
  }

  if (!system_i2c_init()) {
    ESP_LOGE(TAG, "I2C init failed");
    return;
  }

  if (sensor_bme680_init()) {
    sensor_bme680_configure(BME680_OS_2X, BME680_OS_16X, BME680_OS_1X,
                            BME680_FILTER_SIZE_7, 100, 320);
  }

  if (sensor_mpu6050_init()) {
    sensor_mpu6050_calibrate_gyro(200);
  }

  gps_neo7m_init();
  cam_config_init();

  // Initialize buzzer for alert system
  if (!buzzer_driver_init()) {
    ESP_LOGW(TAG, "Buzzer init failed - alerts will be disabled");
  }

  xTaskCreatePinnedToCore(sensor_task, "sensor_task", 8192, NULL, 5,
                          &sensor_task_handle, 1);
  xTaskCreatePinnedToCore(camera_task, "camera_task", 8192, NULL, 5,
                          &camera_task_handle, 0);

  ESP_LOGI(TAG, "System ready");
}
