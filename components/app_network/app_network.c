/**
 * @file app_network.c
 * @brief WiFi, MQTT and Cloudinary upload
 */

#include "app_network.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include "network_config.h"
#include "nvs_flash.h"
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

static void mqtt_handler(void *args, esp_event_base_t base, int32_t id,
                         void *data) {
  (void)args;
  (void)base;
  (void)data;
  if (id == MQTT_EVENT_CONNECTED)
    ESP_LOGI(TAG, "MQTT connected");
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

  esp_http_client_config_t cfg = {
      .url = CLOUDINARY_UPLOAD_URL,
      .method = HTTP_METHOD_POST,
      .crt_bundle_attach = esp_crt_bundle_attach,
      .timeout_ms = 30000,
  };

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) {
    free(body);
    return false;
  }

  esp_http_client_set_header(client, "Content-Type", ct);
  esp_http_client_set_post_field(client, (char *)body, len);

  esp_err_t err = esp_http_client_perform(client);
  bool ok = (err == ESP_OK && esp_http_client_get_status_code(client) == 200);

  esp_http_client_cleanup(client);
  free(body);
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
