/**
 * @file cam_config.h
 * @brief OV2640 camera configuration wrapper using esp32-camera library
 * 
 * @details This header provides a wrapper interface for the esp32-camera library.
 * The underlying camera_fb_t structure is from esp32-camera library.
 */

#ifndef CAM_CONFIG_H
#define CAM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_camera.h"

// Use camera_fb_t directly from esp_camera.h
// No need to redefine it - the esp32-camera library provides the correct structure

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize OV2640 camera
 * 
 * @details Initializes the camera with VGA (640x480) resolution, JPEG format.
 * Must be called after system I2C is initialized (for sensor detection).
 * 
 * @return true on success, false on failure
 * 
 * @note Error messages are logged to help diagnose initialization failures:
 *       - ESP_ERR_CAMERA_NOT_DETECTED: Check SCCB connections and power
 *       - ESP_ERR_NO_MEM: Insufficient memory, enable PSRAM or reduce settings
 *       - ESP_ERR_INVALID_STATE: Camera already initialized
 */
bool cam_config_init(void);

/**
 * @brief Capture a frame from the camera
 * 
 * @param fb Pointer to camera_fb_t pointer (will be set to frame buffer)
 * @return true on success, false on failure
 * 
 * @note This function blocks until a frame is available.
 * @note The caller MUST call cam_config_free_frame() after using the frame.
 * @note Frame buffer is managed by esp32-camera library, do not free() it.
 */
bool cam_config_capture_frame(camera_fb_t **fb);

/**
 * @brief Free a captured frame buffer
 * 
 * @param fb Frame buffer to return to camera driver
 * 
 * @note This MUST be called after using a captured frame to prevent memory leaks.
 * @note Failing to return buffers will cause camera capture to eventually fail.
 * @note Safe to call with NULL pointer (no-op).
 */
void cam_config_free_frame(camera_fb_t *fb);

/**
 * @brief Deinitialize camera
 * 
 * @details Returns all frame buffers and deinitializes camera driver.
 * Should be called during system shutdown or when camera is no longer needed.
 */
void cam_config_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CAM_CONFIG_H
