/**
 * @file app_network.h
 * @brief Network stack: WiFi, MQTT, and Cloudinary HTTP client
 */

#ifndef APP_NETWORK_H
#define APP_NETWORK_H

#include <stdbool.h>
#include <stdint.h>
#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize network stack (WiFi, MQTT)
 * @return true on success, false on failure
 */
bool app_network_init(void);

/**
 * @brief Wait for WiFi connection (with retry logic)
 * @param max_retries Maximum number of connection retries (default: 10)
 * @return true on success, false on failure
 */
bool app_network_wait_for_wifi(uint8_t max_retries);

/**
 * @brief Publish JSON data to MQTT topic
 * @param json_data JSON string to publish
 * @return true on success, false on failure
 */
bool app_network_mqtt_publish(const char *json_data);

/**
 * @brief Upload image to Cloudinary
 * @param fb Camera frame buffer containing JPEG image
 * @return true on success, false on failure
 */
bool app_network_upload_image(camera_fb_t *fb);

/**
 * @brief Deinitialize network stack
 */
void app_network_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // APP_NETWORK_H
