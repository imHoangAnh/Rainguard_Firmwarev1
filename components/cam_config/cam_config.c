/**
 * @file cam_config.c
 * @brief OV2640 camera configuration implementation using esp32-camera library
 *
 * @details This implementation uses the esp32-camera library for OV2640 sensor.
 * Key safety considerations:
 * - Memory management: Frame buffers are managed by esp32-camera library
 * - LEDC conflicts: Uses LEDC_TIMER_0, ensure no conflicts with other
 * peripherals
 * - Power management: PWDN pin set to -1 (not used), RESET pin can be
 * configured
 * - I2C/SCCB: Camera uses separate SCCB bus (pins 4,5) from system I2C (pins
 * 1,2)
 * - Initialization: Must be called after I2C bus is initialized
 * - Frame buffers: Using fb_count=1 for memory efficiency (single buffering)
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

// Statistics tracking (must be declared before use)
static cam_stats_t cam_stats = {0};
static uint32_t last_capture_time = 0;

/**
 * @brief Map camera pin definitions to esp32-camera format
 *
 * Note: esp32-camera uses D0-D7 for data pins (LSB to MSB)
 * Standard ESP32-CAM mapping: Y2=D0, Y3=D1, Y4=D2, Y5=D3, Y6=D4, Y7=D5, Y8=D6,
 * Y9=D7
 */
/**
 * @brief Power sequence the camera module before initialization
 * @note Required for proper sensor detection on some camera modules
 */
static void cam_power_sequence(void) {
  ESP_LOGI(TAG, "Power sequencing camera module...");

  // Configure PWDN pin as output (if used)
  if (CAM_PIN_PWDN >= 0) {
    gpio_config_t pwdn_conf = {
        .pin_bit_mask = (1ULL << CAM_PIN_PWDN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwdn_conf);

    // Set PWDN LOW to enable camera (active high = power down)
    gpio_set_level(CAM_PIN_PWDN, 0);
    ESP_LOGI(TAG, "PWDN pin set LOW (camera enabled)");
  }

  // Configure RESET pin as output (if used)
  if (CAM_PIN_RESET >= 0) {
    gpio_config_t reset_conf = {
        .pin_bit_mask = (1ULL << CAM_PIN_RESET),
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
  }

  // Wait for camera to stabilize after power on/reset
  vTaskDelay(pdMS_TO_TICKS(100));
}

bool cam_config_init(void) {
  if (camera_initialized) {
    ESP_LOGW(TAG, "Camera already initialized");
    return true;
  }

  // Power sequence the camera before initialization
  cam_power_sequence();

  // Check if PSRAM is available
#if CONFIG_SPIRAM
  bool psram_available = true;
  ESP_LOGI(TAG, "PSRAM enabled, using PSRAM for frame buffer");
#else
  bool psram_available = false;
  ESP_LOGW(TAG, "PSRAM not enabled! Using DRAM with reduced resolution");
  ESP_LOGW(TAG, "Enable PSRAM in menuconfig for better performance");
#endif

  // Camera configuration for OV2640 sensor
  // Resolution and buffer location depend on PSRAM availability
  camera_config_t camera_config = {
      // Power and reset pins - let esp_camera handle them after we've done
      // power sequencing
      // Set to -1 since we've already done the power sequencing manually
      .pin_pwdn = -1,  // Already handled in cam_power_sequence()
      .pin_reset = -1, // Already handled in cam_power_sequence()

      // Clock pin
      .pin_xclk = CAM_PIN_XCLK, // XCLK pin (GPIO 15)

      // SCCB/I2C pins for sensor control
      .pin_sccb_sda = CAM_PIN_SIOD, // SCCB SDA (GPIO 4)
      .pin_sccb_scl = CAM_PIN_SIOC, // SCCB SCL (GPIO 5)

      // Data pins (Y2=D0 to Y9=D7)
      .pin_d0 = CAM_PIN_Y2, // D0 (LSB) - GPIO 8
      .pin_d1 = CAM_PIN_Y3, // D1 - GPIO 9
      .pin_d2 = CAM_PIN_Y4, // D2 - GPIO 11
      .pin_d3 = CAM_PIN_Y5, // D3 - GPIO 10
      .pin_d4 = CAM_PIN_Y6, // D4 - GPIO 12
      .pin_d5 = CAM_PIN_Y7, // D5 - GPIO 13
      .pin_d6 = CAM_PIN_Y8, // D6 - GPIO 14
      .pin_d7 = CAM_PIN_Y9, // D7 (MSB) - GPIO 18

      // Control pins
      .pin_vsync = CAM_PIN_VSYNC, // VSYNC - GPIO 6
      .pin_href = CAM_PIN_HREF,   // HREF - GPIO 7
      .pin_pclk = CAM_PIN_PCLK,   // PCLK - GPIO 15

      // Clock configuration - Reduced to 10MHz for better stability
      .xclk_freq_hz = 10000000,       // 10MHz XCLK
      .ledc_timer = LEDC_TIMER_0,     // LEDC timer for XCLK generation
      .ledc_channel = LEDC_CHANNEL_0, // LEDC channel for XCLK

      // Image format and quality
      .pixel_format = PIXFORMAT_JPEG, // JPEG format for memory efficiency
      // Use smaller resolution if no PSRAM available
      .frame_size = psram_available ? FRAMESIZE_VGA : FRAMESIZE_QVGA,
      .jpeg_quality = psram_available ? 12 : 15, // Lower quality if no PSRAM

      // Frame buffer configuration
      .fb_count = psram_available ? 2 : 1, // Double buffering with PSRAM
      .fb_location = psram_available ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,

      // Frame grab mode
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY, // Grab frames when buffer is empty

      // SCCB/I2C port (use dedicated SCCB pins, not shared I2C bus)
      .sccb_i2c_port = -1, // Use dedicated SCCB pins (not shared I2C)
  };

  // Try initialization with different XCLK frequencies if first attempt fails
  static const uint32_t xclk_frequencies[] = {20000000, 10000000, 8000000,
                                              16000000};
  static const int num_frequencies =
      sizeof(xclk_frequencies) / sizeof(xclk_frequencies[0]);

  esp_err_t err = ESP_FAIL;

  for (int i = 0; i < num_frequencies; i++) {
    camera_config.xclk_freq_hz = xclk_frequencies[i];
    ESP_LOGI(TAG, "Trying camera init with XCLK = %lu Hz...",
             (unsigned long)xclk_frequencies[i]);

    err = esp_camera_init(&camera_config);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Camera initialized successfully with XCLK = %lu Hz",
               (unsigned long)xclk_frequencies[i]);
      break;
    }

    ESP_LOGW(TAG, "Camera init failed with XCLK = %lu Hz: %s (0x%x)",
             (unsigned long)xclk_frequencies[i], esp_err_to_name(err), err);

    // Small delay before retry
    vTaskDelay(pdMS_TO_TICKS(100));

    // Deinit before retry (might be needed for some errors)
    esp_camera_deinit();
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera initialization failed after all attempts: %s (0x%x)",
             esp_err_to_name(err), err);

    // Provide helpful error messages
    if (err == ESP_ERR_CAMERA_NOT_DETECTED) {
      ESP_LOGE(TAG, "Camera sensor not detected. Check:");
      ESP_LOGE(TAG, "  1. SCCB/I2C connections (SDA=GPIO%d, SCL=GPIO%d)",
               CAM_PIN_SIOD, CAM_PIN_SIOC);
      ESP_LOGE(TAG, "  2. Power supply to camera module (3.3V)");
      ESP_LOGE(TAG, "  3. PWDN pin connection (GPIO%d should be LOW)",
               CAM_PIN_PWDN);
      ESP_LOGE(TAG, "  4. RESET pin connection (GPIO%d)", CAM_PIN_RESET);
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
      ESP_LOGE(TAG, "Camera sensor detected but NOT SUPPORTED!");
      ESP_LOGE(TAG,
               "  The camera module may not be OV2640. Supported sensors:");
      ESP_LOGE(TAG, "  OV2640, OV5640, OV7670, OV7725, GC0308, GC2145, etc.");
      ESP_LOGE(TAG, "  Please check your camera module model.");
      ESP_LOGE(
          TAG,
          "  If using a clone/variant, it may have incompatible sensor ID.");
    } else if (err == ESP_ERR_NO_MEM) {
      ESP_LOGE(TAG, "Insufficient memory for camera buffers. Consider:");
      ESP_LOGE(TAG, "  1. Enabling PSRAM if available");
      ESP_LOGE(TAG, "  2. Reducing frame size or jpeg_quality");
      ESP_LOGE(TAG, "  3. Reducing fb_count");
    } else if (err == ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "Camera already initialized or system not ready");
    }

    return false;
  }

  // Get sensor handle for potential future configuration
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != NULL) {
    // Get sensor info to retrieve sensor name
    camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
    if (info != NULL) {
      ESP_LOGI(TAG, "Camera sensor detected: %s", info->name);
    } else {
      ESP_LOGI(TAG, "Camera sensor detected: PID=0x%02x", sensor->id.PID);
    }

    // Optional: Configure sensor settings
    // sensor->set_framesize(sensor, FRAMESIZE_VGA);
    // sensor->set_quality(sensor, 12);
  } else {
    ESP_LOGW(TAG, "Camera sensor handle not available");
  }

  // Log memory usage
  size_t free_heap = esp_get_free_heap_size();
  ESP_LOGI(TAG, "Camera initialized successfully");
  ESP_LOGI(TAG, "Free heap after camera init: %zu bytes", free_heap);
  ESP_LOGI(TAG,
           "Configuration: %s (%dx%d), JPEG quality=%d, fb_count=%d, "
           "fb_location=%s, xclk=%dMHz",
           psram_available ? "VGA" : "QVGA", psram_available ? 640 : 320,
           psram_available ? 480 : 240, camera_config.jpeg_quality,
           camera_config.fb_count, psram_available ? "PSRAM" : "DRAM",
           (int)(camera_config.xclk_freq_hz / 1000000));

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

  // Track capture time for statistics
  uint32_t start_time = (uint32_t)(esp_timer_get_time() / 1000);

  // Capture frame from camera
  // Note: This function blocks until a frame is available
  // With fb_count=1, this will wait for the previous frame to be returned
  *fb = esp_camera_fb_get();

  if (*fb == NULL) {
    ESP_LOGE(TAG, "Failed to capture frame from camera");
    cam_stats.frames_dropped++;
    return false;
  }

  // Update statistics
  uint32_t capture_time = (uint32_t)(esp_timer_get_time() / 1000) - start_time;
  cam_stats.frames_captured++;
  cam_stats.last_frame_size = (*fb)->len;

  // Running average of frame time
  if (cam_stats.avg_frame_time_ms == 0) {
    cam_stats.avg_frame_time_ms = (float)capture_time;
  } else {
    cam_stats.avg_frame_time_ms =
        cam_stats.avg_frame_time_ms * 0.9f + (float)capture_time * 0.1f;
  }

  // Calculate FPS based on time since last capture
  if (last_capture_time > 0) {
    uint32_t delta =
        (uint32_t)(esp_timer_get_time() / 1000) - last_capture_time;
    if (delta > 0) {
      cam_stats.current_fps = 1000.0f / (float)delta;
    }
  }
  last_capture_time = (uint32_t)(esp_timer_get_time() / 1000);

  ESP_LOGD(TAG, "Frame captured: %zux%zu, %zu bytes, format=%d", (*fb)->width,
           (*fb)->height, (*fb)->len, (*fb)->format);

  return true;
}

void cam_config_free_frame(camera_fb_t *fb) {
  if (fb == NULL) {
    ESP_LOGW(TAG, "Attempted to free NULL frame buffer");
    return;
  }

  // Return frame buffer to camera driver for reuse
  // This is critical - failing to return buffers will cause memory leaks
  // and eventually camera capture failures
  esp_camera_fb_return(fb);
}

bool cam_config_set_resolution(cam_resolution_t resolution) {
  if (!camera_initialized) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return false;
  }

  int ret = sensor->set_framesize(sensor, (framesize_t)resolution);
  if (ret == 0) {
    ESP_LOGI(TAG, "Resolution set to %d", resolution);
    return true;
  }

  ESP_LOGE(TAG, "Failed to set resolution");
  return false;
}

bool cam_config_set_quality(uint8_t quality) {
  if (!camera_initialized) {
    return false;
  }

  // Clamp quality to valid range
  if (quality < 10)
    quality = 10;
  if (quality > 63)
    quality = 63;

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return false;
  }

  int ret = sensor->set_quality(sensor, quality);
  if (ret == 0) {
    ESP_LOGI(TAG, "JPEG quality set to %d", quality);
    return true;
  }

  return false;
}

bool cam_config_set_image_settings(int8_t brightness, int8_t contrast,
                                   int8_t saturation) {
  if (!camera_initialized) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return false;
  }

  // Clamp values to valid range (-2 to 2)
  brightness = (brightness < -2) ? -2 : (brightness > 2) ? 2 : brightness;
  contrast = (contrast < -2) ? -2 : (contrast > 2) ? 2 : contrast;
  saturation = (saturation < -2) ? -2 : (saturation > 2) ? 2 : saturation;

  sensor->set_brightness(sensor, brightness);
  sensor->set_contrast(sensor, contrast);
  sensor->set_saturation(sensor, saturation);

  ESP_LOGI(TAG, "Image settings: brightness=%d, contrast=%d, saturation=%d",
           brightness, contrast, saturation);
  return true;
}

bool cam_config_set_hmirror(bool enable) {
  if (!camera_initialized) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return false;
  }

  return sensor->set_hmirror(sensor, enable ? 1 : 0) == 0;
}

bool cam_config_set_vflip(bool enable) {
  if (!camera_initialized) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return false;
  }

  return sensor->set_vflip(sensor, enable ? 1 : 0) == 0;
}

bool cam_config_get_stats(cam_stats_t *stats) {
  if (stats == NULL) {
    return false;
  }

  *stats = cam_stats;
  return true;
}

size_t cam_config_get_free_memory(void) {
#if CONFIG_SPIRAM
  return heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#else
  return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#endif
}

bool cam_config_is_initialized(void) { return camera_initialized; }

void cam_config_deinit(void) {
  if (!camera_initialized) {
    ESP_LOGW(TAG, "Camera not initialized, nothing to deinit");
    return;
  }

  // Return all frame buffers before deinit
  esp_camera_return_all();

  // Deinitialize camera
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera deinitialization failed: %s", esp_err_to_name(err));
    return;
  }

  camera_initialized = false;

  // Reset statistics
  memset(&cam_stats, 0, sizeof(cam_stats));

  ESP_LOGI(TAG, "Camera deinitialized successfully");
}
