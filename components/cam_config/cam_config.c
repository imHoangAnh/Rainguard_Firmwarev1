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
#include "driver/ledc.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "pin_config.h"
#include <string.h>

static const char *TAG = "cam_config";
static bool camera_initialized = false;

/**
 * @brief Map camera pin definitions to esp32-camera format
 *
 * Note: esp32-camera uses D0-D7 for data pins (LSB to MSB)
 * Current pin config uses Y2-Y9, we need to map them correctly:
 * Y2 = D7 (MSB), Y3 = D6, Y4 = D5, Y5 = D4, Y6 = D3, Y7 = D2, Y8 = D1, Y9 = D0
 * (LSB)
 */
bool cam_config_init(void) {
  if (camera_initialized) {
    ESP_LOGW(TAG, "Camera already initialized");
    return true;
  }

  // Camera configuration for OV2640 sensor
  // Using VGA (640x480) resolution, JPEG format for memory efficiency
  camera_config_t camera_config = {
      // Power and reset pins (set to -1 if not used in hardware)
      .pin_pwdn = -1,  // Power down pin (not used, set to -1)
      .pin_reset = -1, // Reset pin (not used, set to -1)

      // Clock pin
      .pin_xclk = CAM_PIN_XCLK, // XCLK pin (GPIO 15)

      // SCCB/I2C pins for sensor control
      .pin_sccb_sda = CAM_PIN_SIOD, // SCCB SDA (GPIO 4)
      .pin_sccb_scl = CAM_PIN_SIOC, // SCCB SCL (GPIO 5)

      // Data pins (D7 to D0, MSB to LSB)
      .pin_d7 = CAM_PIN_Y2, // D7 (MSB) - GPIO 11
      .pin_d6 = CAM_PIN_Y3, // D6 - GPIO 9
      .pin_d5 = CAM_PIN_Y4, // D5 - GPIO 8
      .pin_d4 = CAM_PIN_Y5, // D4 - GPIO 10
      .pin_d3 = CAM_PIN_Y6, // D3 - GPIO 12
      .pin_d2 = CAM_PIN_Y7, // D2 - GPIO 18
      .pin_d1 = CAM_PIN_Y8, // D1 - GPIO 17
      .pin_d0 = CAM_PIN_Y9, // D0 (LSB) - GPIO 16

      // Control pins
      .pin_vsync = CAM_PIN_VSYNC, // VSYNC - GPIO 6
      .pin_href = CAM_PIN_HREF,   // HREF - GPIO 7
      .pin_pclk = CAM_PIN_PCLK,   // PCLK - GPIO 13

      // Clock configuration
      .xclk_freq_hz = 20000000,       // 20MHz XCLK frequency
      .ledc_timer = LEDC_TIMER_0,     // LEDC timer for XCLK generation
      .ledc_channel = LEDC_CHANNEL_0, // LEDC channel for XCLK

      // Image format and quality
      .pixel_format = PIXFORMAT_JPEG, // JPEG format for memory efficiency
      .frame_size = FRAMESIZE_VGA,    // VGA: 640x480
      .jpeg_quality = 12, // JPEG quality (0-63, lower = higher quality)

      // Frame buffer configuration
      // Using fb_count=1 for memory efficiency (single buffering)
      // Note: With fb_count=1, frames might be skipped if processing is slow
      // For better performance, use fb_count=2 (double buffering) if memory
      // allows
      .fb_count = 1, // Number of frame buffers
      .fb_location =
          CAMERA_FB_IN_PSRAM, // Prefer PSRAM if available, fallback to DRAM

      // Frame grab mode
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY, // Grab frames when buffer is empty

      // SCCB/I2C port (use dedicated SCCB pins, not shared I2C bus)
      .sccb_i2c_port = -1, // Use dedicated SCCB pins (not shared I2C)
  };

  // Initialize camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera initialization failed: %s (0x%x)",
             esp_err_to_name(err), err);

    // Provide helpful error messages
    if (err == ESP_ERR_CAMERA_NOT_DETECTED) {
      ESP_LOGE(TAG, "Camera sensor not detected. Check:");
      ESP_LOGE(TAG, "  1. SCCB/I2C connections (SDA/SCL pins)");
      ESP_LOGE(TAG, "  2. Power supply to camera module");
      ESP_LOGE(TAG, "  3. Camera module is OV2640 compatible");
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
  ESP_LOGI(TAG, "Configuration: VGA (640x480), JPEG quality=%d, fb_count=%zu",
           camera_config.jpeg_quality, camera_config.fb_count);

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

  // Capture frame from camera
  // Note: This function blocks until a frame is available
  // With fb_count=1, this will wait for the previous frame to be returned
  *fb = esp_camera_fb_get();

  if (*fb == NULL) {
    ESP_LOGE(TAG, "Failed to capture frame from camera");
    return false;
  }

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
  ESP_LOGI(TAG, "Camera deinitialized successfully");
}
