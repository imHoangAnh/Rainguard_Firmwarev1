/**
 * @file cam_config.h
 * @brief OV2640 camera configuration wrapper using esp32-camera library
 * 
 * @details This header provides a wrapper interface for the esp32-camera library.
 * Optimized for ESP32-S3 with PSRAM support.
 * 
 * Features:
 * - Automatic PSRAM detection and configuration
 * - Multiple resolution support
 * - JPEG quality control
 * - Frame rate statistics
 * - Memory usage monitoring
 */

#ifndef CAM_CONFIG_H
#define CAM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Camera resolution presets
 */
typedef enum {
  CAM_RES_QQVGA = FRAMESIZE_QQVGA,  // 160x120
  CAM_RES_QVGA = FRAMESIZE_QVGA,    // 320x240
  CAM_RES_VGA = FRAMESIZE_VGA,      // 640x480
  CAM_RES_SVGA = FRAMESIZE_SVGA,    // 800x600
  CAM_RES_XGA = FRAMESIZE_XGA,      // 1024x768
  CAM_RES_HD = FRAMESIZE_HD,        // 1280x720
  CAM_RES_SXGA = FRAMESIZE_SXGA,    // 1280x1024
  CAM_RES_UXGA = FRAMESIZE_UXGA     // 1600x1200
} cam_resolution_t;

/**
 * @brief Camera statistics
 */
typedef struct {
  uint32_t frames_captured;  // Total frames captured
  uint32_t frames_dropped;   // Frames dropped due to buffer full
  float avg_frame_time_ms;   // Average frame capture time
  float current_fps;         // Current frames per second
  size_t last_frame_size;    // Last frame size in bytes
} cam_stats_t;

/**
 * @brief Initialize OV2640 camera with default settings
 * @return true on success, false on failure
 */
bool cam_config_init(void);

/**
 * @brief Capture a frame from the camera
 * @param fb Pointer to camera_fb_t pointer (will be set to frame buffer)
 * @return true on success, false on failure
 */
bool cam_config_capture_frame(camera_fb_t **fb);

/**
 * @brief Free a captured frame buffer
 * @param fb Frame buffer to return to camera driver
 */
void cam_config_free_frame(camera_fb_t *fb);

/**
 * @brief Set camera resolution
 * @param resolution Desired resolution
 * @return true on success, false on failure
 */
bool cam_config_set_resolution(cam_resolution_t resolution);

/**
 * @brief Set JPEG quality
 * @param quality JPEG quality (10-63, lower = better quality, more data)
 * @return true on success, false on failure
 */
bool cam_config_set_quality(uint8_t quality);

/**
 * @brief Configure camera image settings
 * @param brightness Brightness (-2 to 2)
 * @param contrast Contrast (-2 to 2)
 * @param saturation Saturation (-2 to 2)
 * @return true on success, false on failure
 */
bool cam_config_set_image_settings(int8_t brightness, int8_t contrast, 
                                    int8_t saturation);

/**
 * @brief Enable/disable horizontal mirror
 * @param enable true to enable mirror
 * @return true on success
 */
bool cam_config_set_hmirror(bool enable);

/**
 * @brief Enable/disable vertical flip
 * @param enable true to enable flip
 * @return true on success
 */
bool cam_config_set_vflip(bool enable);

/**
 * @brief Get camera statistics
 * @param stats Pointer to statistics structure
 * @return true on success
 */
bool cam_config_get_stats(cam_stats_t *stats);

/**
 * @brief Get free memory available for camera buffers
 * @return Free bytes in PSRAM (if available) or DRAM
 */
size_t cam_config_get_free_memory(void);

/**
 * @brief Check if camera is initialized
 * @return true if initialized
 */
bool cam_config_is_initialized(void);

/**
 * @brief Deinitialize camera
 */
void cam_config_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CAM_CONFIG_H
