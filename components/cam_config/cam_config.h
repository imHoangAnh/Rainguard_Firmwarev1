/**
 * @file cam_config.h
 * @brief OV2640 camera configuration wrapper (simplified for snapshot)
 *
 * @details Simplified camera driver for periodic snapshot capture.
 * Hardcoded to VGA resolution (640x480) with JPEG format.
 * Optimized for ESP32-S3 with PSRAM support.
 */

#ifndef CAM_CONFIG_H
#define CAM_CONFIG_H

#include "esp_camera.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize OV2640 camera with VGA resolution
 * @note Hardcoded settings: VGA (640x480), JPEG quality 12
 * @return true on success, false on failure
 */
bool cam_config_init(void);

/**
 * @brief Capture a JPEG frame from the camera
 * @param fb Pointer to camera_fb_t pointer (will be set to frame buffer)
 * @return true on success, false on failure
 */
bool cam_config_capture_frame(camera_fb_t **fb);

/**
 * @brief Free a captured frame buffer
 * @param fb Frame buffer to return to camera driver
 * @note MUST be called after each successful capture to prevent memory leak
 */
void cam_config_free_frame(camera_fb_t *fb);

/**

 * @brief Check if camera is initialized
 * @return true if initialized
 */
bool cam_config_is_initialized(void);

/**
 * @brief Deinitialize camera and free resources
 */
void cam_config_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CAM_CONFIG_H
