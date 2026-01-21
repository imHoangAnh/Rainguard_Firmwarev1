/**
 * @file cam_config.c
 * @brief OV2640 camera configuration (simplified for snapshot)
 *
 * @details Simplified camera driver for periodic snapshot capture.
 * Hardcoded to VGA resolution (640x480) with JPEG format.
 *
 * Key configurations:
 * - Resolution: VGA (640x480)
 * - Format: JPEG
 * - Quality: 12 (good balance of quality and size)
 * - Frame buffer: 2 (double buffering with PSRAM)
 */

#include "cam_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include <string.h>

static const char *TAG = "cam_config";
static bool camera_initialized = false;

/**
 * @brief Power sequence the camera module before initialization
 */
static void cam_power_sequence(void) {
  ESP_LOGI(TAG, "Power sequencing camera module...");

  // Configure PWDN pin as output (if used)
  if (CAM_PIN_PWDN >= 0) {
    gpio_config_t pwdn_conf = {
        .pin_bit_mask = (1ULL << (uint32_t)CAM_PIN_PWDN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwdn_conf);
    gpio_set_level(CAM_PIN_PWDN, 0);
    ESP_LOGI(TAG, "PWDN pin set LOW (camera enabled)");
  } else {
    ESP_LOGI(TAG, "PWDN pin not configured (disabled)");
  }

  // Configure RESET pin as output (if used)
  if (CAM_PIN_RESET >= 0) {
    gpio_config_t reset_conf = {
        .pin_bit_mask = (1ULL << (uint32_t)CAM_PIN_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&reset_conf);

    // Reset sequence: HIGH -> LOW -> HIGH
    gpio_set_level(CAM_PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CAM_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CAM_PIN_RESET, 1);
    ESP_LOGI(TAG, "Camera reset sequence completed");
  } else {
    ESP_LOGI(TAG, "RESET pin not configured (disabled)");
  }

  // Wait for camera to stabilize
  vTaskDelay(pdMS_TO_TICKS(100));
}

bool cam_config_init(void) {
  if (camera_initialized) {
    ESP_LOGW(TAG, "Camera already initialized");
    return true;
  }

  // Power sequence the camera
  cam_power_sequence();

  // Check PSRAM availability
#if CONFIG_SPIRAM
  bool psram_available = true;
  ESP_LOGI(TAG, "PSRAM enabled");
#else
  bool psram_available = false;
  ESP_LOGW(TAG, "PSRAM not enabled! Performance may be limited");
#endif

  // Camera configuration - VGA hardcoded
  camera_config_t camera_config = {
      .pin_pwdn = CAM_PIN_PWDN,
      .pin_reset = CAM_PIN_RESET,
      .pin_xclk = CAM_PIN_XCLK,
      .pin_sccb_sda = CAM_PIN_SIOD,
      .pin_sccb_scl = CAM_PIN_SIOC,
      .pin_d0 = CAM_PIN_Y2,
      .pin_d1 = CAM_PIN_Y3,
      .pin_d2 = CAM_PIN_Y4,
      .pin_d3 = CAM_PIN_Y5,
      .pin_d4 = CAM_PIN_Y6,
      .pin_d5 = CAM_PIN_Y7,
      .pin_d6 = CAM_PIN_Y8,
      .pin_d7 = CAM_PIN_Y9,
      .pin_vsync = CAM_PIN_VSYNC,
      .pin_href = CAM_PIN_HREF,
      .pin_pclk = CAM_PIN_PCLK,

      // Clock configuration - 10MHz for stable JPEG output
      .xclk_freq_hz = 10000000, // 10MHz - reduced to fix NO-SOI error
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

      // Hardcoded VGA settings
      .pixel_format = PIXFORMAT_JPEG,
      .frame_size = FRAMESIZE_VGA, // 640x480 - HARDCODED
      .jpeg_quality = 12,          // Good quality - HARDCODED
      .fb_count = psram_available ? 2 : 1,
      .fb_location = psram_available ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
      .sccb_i2c_port = -1,
  };

  // Initialize camera with configured XCLK (10MHz)
  ESP_LOGI(TAG, "Initializing camera with XCLK = %lu Hz...",
           (unsigned long)camera_config.xclk_freq_hz);

  esp_err_t err = esp_camera_init(&camera_config);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: %s (0x%x)", esp_err_to_name(err), err);

    if (err == ESP_ERR_CAMERA_NOT_DETECTED) {
      ESP_LOGE(TAG, "Camera not detected - check wiring");
    } else if (err == ESP_ERR_NO_MEM) {
      ESP_LOGE(TAG, "Out of memory - enable PSRAM");
    }
    return false;
  }

  // Log sensor info and configure for stable JPEG output
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != NULL) {
    camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
    if (info != NULL) {
      ESP_LOGI(TAG, "Sensor: %s", info->name);
    }

    // Configure OV2640 sensor settings for stable JPEG
    sensor->set_quality(sensor, 12);   // JPEG quality (10-63, lower = better)
    sensor->set_brightness(sensor, 0); // Default brightness
    sensor->set_contrast(sensor, 0);   // Default contrast
    sensor->set_saturation(sensor, 0); // Default saturation

    // Important: These settings help fix NO-SOI error
    sensor->set_whitebal(sensor, 1);      // Enable auto white balance
    sensor->set_awb_gain(sensor, 1);      // Enable AWB gain
    sensor->set_wb_mode(sensor, 0);       // Auto WB mode
    sensor->set_exposure_ctrl(sensor, 1); // Enable auto exposure
    sensor->set_aec2(sensor, 0);          // Disable AEC DSP
    sensor->set_gain_ctrl(sensor, 1);     // Enable auto gain
    sensor->set_agc_gain(sensor, 0);      // Set gain to 0
    sensor->set_gainceiling(sensor, (gainceiling_t)0); // Gain ceiling

    ESP_LOGI(TAG, "OV2640 sensor configured for JPEG output");
  }

  // Skip first few frames to let camera stabilize (fix NO-SOI)
  ESP_LOGI(TAG, "Warming up camera (skipping initial frames)...");
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb != NULL) {
      esp_camera_fb_return(fb);
      ESP_LOGD(TAG, "Warmup frame %d captured", i + 1);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI(TAG, "Camera warmup complete");

  ESP_LOGI(TAG, "Camera ready: VGA (640x480), JPEG Q=12");
  ESP_LOGI(TAG, "Free heap: %zu bytes", esp_get_free_heap_size());

  camera_initialized = true;
  return true;
}

bool cam_config_capture_frame(camera_fb_t **fb) {
  if (!camera_initialized) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  if (fb == NULL) {
    ESP_LOGE(TAG, "Invalid frame buffer pointer");
    return false;
  }

  // Retry mechanism for timeout recovery
  static int consecutive_failures = 0;
  const int MAX_RETRIES = 3;
  const int MAX_CONSECUTIVE_FAILURES = 5;

  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    *fb = esp_camera_fb_get();

    if (*fb != NULL) {
      consecutive_failures = 0; // Reset on success
      ESP_LOGD(TAG, "Captured: %zux%zu, %zu bytes", (*fb)->width, (*fb)->height,
               (*fb)->len);
      return true;
    }

    ESP_LOGW(TAG, "Capture retry %d/%d", retry + 1, MAX_RETRIES);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // All retries failed
  consecutive_failures++;
  ESP_LOGE(TAG, "Failed to capture frame after %d retries (failures: %d)",
           MAX_RETRIES, consecutive_failures);

  // Reinitialize camera if too many consecutive failures
  if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
    ESP_LOGW(TAG, "Too many failures, reinitializing camera...");
    cam_config_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));

    if (cam_config_init()) {
      ESP_LOGI(TAG, "Camera reinitialized successfully");
      consecutive_failures = 0;
    } else {
      ESP_LOGE(TAG, "Camera reinitialization failed!");
    }
  }

  return false;
}

void cam_config_free_frame(camera_fb_t *fb) {
  if (fb == NULL) {
    ESP_LOGW(TAG, "Attempted to free NULL frame buffer");
    return;
  }
  esp_camera_fb_return(fb);
}

bool cam_config_is_initialized(void) { return camera_initialized; }

void cam_config_deinit(void) {
  if (!camera_initialized) {
    ESP_LOGW(TAG, "Camera not initialized");
    return;
  }

  esp_camera_return_all();

  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Deinit failed: %s", esp_err_to_name(err));
    return;
  }

  camera_initialized = false;
  ESP_LOGI(TAG, "Camera deinitialized");
}
