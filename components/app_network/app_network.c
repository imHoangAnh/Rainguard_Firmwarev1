/**
 * @file app_network.c
 * @brief WiFi, MQTT, Cloudinary upload, and Alert subscription
 */

#include "app_network.h"
#include "buzzer_driver.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include "network_config.h"
#include "nvs_flash.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "net";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAX_RETRY 10

static EventGroupHandle_t wifi_event_group;
static int retry_num = 0;
static esp_mqtt_client_handle_t mqtt = NULL;

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id,
                         void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    if (retry_num < MAX_RETRY) {
      esp_wifi_connect();
      retry_num++;
    } else {
      xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    retry_num = 0;
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

/**
 * @brief Parse alert JSON and trigger buzzer
 * @param json_str JSON string like {"alert_code": 2}
 */
static void handle_alert_message(const char *json_str, int len) {
  // Create null-terminated copy
  char *json_copy = malloc(len + 1);
  if (!json_copy) {
    ESP_LOGE(TAG, "Failed to allocate memory for alert JSON");
    return;
  }
  memcpy(json_copy, json_str, len);
  json_copy[len] = '\0';

  cJSON *root = cJSON_Parse(json_copy);
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to parse alert JSON: %s", json_copy);
    free(json_copy);
    return;
  }

  cJSON *alert_code = cJSON_GetObjectItem(root, "alert_code");
  if (cJSON_IsNumber(alert_code)) {
    int code = alert_code->valueint;
    ESP_LOGI(TAG, "Received alert_code: %d", code);
    buzzer_driver_set_alert_code(code);
  } else {
    ESP_LOGW(TAG, "alert_code not found or invalid in JSON");
  }

  cJSON_Delete(root);
  free(json_copy);
}

static void mqtt_handler(void *args, esp_event_base_t base, int32_t id,
                         void *data) {
  (void)args;
  (void)base;
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;

  switch (id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT connected");
    // Subscribe to alert topic
    int msg_id = esp_mqtt_client_subscribe(mqtt, MQTT_ALERT_TOPIC, 1);
    ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", MQTT_ALERT_TOPIC, msg_id);
    break;

  case MQTT_EVENT_DATA:
    ESP_LOGD(TAG, "MQTT data received on topic: %.*s", event->topic_len,
             event->topic);
    // Check if this is alert topic
    if (event->topic_len > 0 &&
        strncmp(event->topic, MQTT_ALERT_TOPIC, event->topic_len) == 0) {
      handle_alert_message(event->data, event->data_len);
    }
    break;

  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGW(TAG, "MQTT disconnected");
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "MQTT error");
    break;

  default:
    break;
  }
}

bool app_network_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_event_group = xEventGroupCreate();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t inst1, inst2;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL, &inst1));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL, &inst2));

  wifi_config_t wc = {.sta = {.threshold.authmode = WIFI_AUTH_WPA2_PSK}};
  strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
  strncpy((char *)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password) - 1);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_mqtt_client_config_t mc = {.broker.address.uri = MQTT_BROKER_URI};
  mqtt = esp_mqtt_client_init(&mc);
  esp_mqtt_client_register_event(mqtt, ESP_EVENT_ANY_ID, mqtt_handler, NULL);
  esp_mqtt_client_start(mqtt);

  return true;
}

bool app_network_wait_for_wifi(uint8_t max_retries) {
  (void)max_retries;
  EventBits_t bits =
      xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                          pdFALSE, pdFALSE, portMAX_DELAY);
  return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool app_network_mqtt_publish(const char *json) {
  if (!mqtt)
    return false;
  char topic[128];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, DEVICE_ID);
  return esp_mqtt_client_publish(mqtt, topic, json, 0, 1, 0) >= 0;
}

// Maximum size for Cloudinary JSON response
#define CLOUDINARY_RESPONSE_MAX_SIZE 2048

/**
 * @brief Publish Cloudinary response to MQTT wrapped in "image" object
 * @param json_response JSON response string from Cloudinary
 * @return true on success
 */
static bool publish_cloudinary_response(const char *json_response) {
  if (!mqtt || !json_response)
    return false;

  // Allocate buffer for wrapped JSON:
  // {"device_id":"XX","timestamp":XXXX,"image":{...}}
  size_t response_len = strlen(json_response);
  size_t wrapped_size = response_len + 256; // Extra space for wrapper
  char *wrapped_json = malloc(wrapped_size);
  if (!wrapped_json) {
    ESP_LOGE(TAG, "Failed to allocate memory for wrapped JSON");
    return false;
  }

  // Create wrapped JSON with device_id, timestamp, and image object
  int len = snprintf(wrapped_json, wrapped_size,
                     "{\"device_id\":\"%s\",\"timestamp\":%lld,\"image\":%s}",
                     DEVICE_ID, (long long)(esp_timer_get_time() / 1000),
                     json_response);

  if (len < 0 || len >= wrapped_size) {
    ESP_LOGE(TAG, "Failed to format wrapped JSON");
    free(wrapped_json);
    return false;
  }

  char topic[128];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, DEVICE_ID);

  int msg_id = esp_mqtt_client_publish(mqtt, topic, wrapped_json, 0, 1, 0);
  free(wrapped_json);

  if (msg_id >= 0) {
    ESP_LOGI(TAG, "Published Cloudinary response to %s", topic);
    return true;
  }
  ESP_LOGE(TAG, "Failed to publish Cloudinary response");
  return false;
}

bool app_network_upload_image(camera_fb_t *fb) {
  if (!fb || !fb->buf)
    return false;

  const char *boundary = "----ESP32Boundary";
  char ct[128];
  snprintf(ct, sizeof(ct), "multipart/form-data; boundary=%s", boundary);

  char p1[256], p2[256];
  snprintf(p1, sizeof(p1),
           "--%s\r\nContent-Disposition: form-data; name=\"file\"; "
           "filename=\"img.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n",
           boundary);
  snprintf(p2, sizeof(p2),
           "\r\n--%s\r\nContent-Disposition: form-data; "
           "name=\"upload_preset\"\r\n\r\n%s\r\n--%s--\r\n",
           boundary, CLOUDINARY_UPLOAD_PRESET, boundary);

  size_t len = strlen(p1) + fb->len + strlen(p2);
  uint8_t *body = malloc(len);
  if (!body)
    return false;

  memcpy(body, p1, strlen(p1));
  memcpy(body + strlen(p1), fb->buf, fb->len);
  memcpy(body + strlen(p1) + fb->len, p2, strlen(p2));

  // Allocate buffer for response
  char *response_buffer = malloc(CLOUDINARY_RESPONSE_MAX_SIZE);
  if (!response_buffer) {
    free(body);
    return false;
  }
  memset(response_buffer, 0, CLOUDINARY_RESPONSE_MAX_SIZE);

  esp_http_client_config_t cfg = {
      .url = CLOUDINARY_UPLOAD_URL,
      .method = HTTP_METHOD_POST,
      .crt_bundle_attach = esp_crt_bundle_attach,
      .timeout_ms = 30000,
      .buffer_size = CLOUDINARY_RESPONSE_MAX_SIZE,
  };

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) {
    free(body);
    free(response_buffer);
    return false;
  }

  esp_http_client_set_header(client, "Content-Type", ct);
  esp_http_client_set_post_field(client, (char *)body, len);

  // Use open/write/fetch_headers/read for response capture
  esp_err_t err = esp_http_client_open(client, len);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    free(body);
    free(response_buffer);
    return false;
  }

  // Write POST body
  int wlen = esp_http_client_write(client, (char *)body, len);
  free(body); // Free body after writing

  if (wlen < 0) {
    ESP_LOGE(TAG, "Failed to write HTTP body");
    esp_http_client_cleanup(client);
    free(response_buffer);
    return false;
  }

  // Fetch response headers
  int content_length = esp_http_client_fetch_headers(client);
  (void)content_length; // Suppress unused variable warning
  int status_code = esp_http_client_get_status_code(client);

  bool ok = false;

  if (status_code == 200) {
    // Read response body
    int read_len = esp_http_client_read_response(
        client, response_buffer, CLOUDINARY_RESPONSE_MAX_SIZE - 1);
    if (read_len > 0) {
      response_buffer[read_len] = '\0';
      ESP_LOGI(TAG, "Cloudinary response (%d bytes): %.100s...", read_len,
               response_buffer);

      // Publish JSON response to MQTT
      ok = publish_cloudinary_response(response_buffer);
    } else {
      ESP_LOGW(TAG, "No response body received from Cloudinary");
    }
  } else {
    ESP_LOGE(TAG, "Cloudinary upload failed with status: %d", status_code);
  }

  esp_http_client_cleanup(client);
  free(response_buffer);
  return ok;
}

void app_network_deinit(void) {
  if (mqtt) {
    esp_mqtt_client_stop(mqtt);
    esp_mqtt_client_destroy(mqtt);
    mqtt = NULL;
  }
  esp_wifi_stop();
  esp_wifi_deinit();
  if (wifi_event_group) {
    vEventGroupDelete(wifi_event_group);
    wifi_event_group = NULL;
  }
}
