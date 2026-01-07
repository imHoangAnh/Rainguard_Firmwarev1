/**
 * @file app_network.c
 * @brief Network stack implementation: WiFi, MQTT, Cloudinary
 */

#include "app_network.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "app_network";

// WiFi event group
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define MAX_RETRY 10

// MQTT client handle
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Configuration from Kconfig (accessed via CONFIG_ prefix)
// These are defined in main/Kconfig.projbuild

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (attempt %d/%d)", s_retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "MQTT published, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        if (event->error_handle) {
            ESP_LOGE(TAG, "MQTT error: type=%d, code=%d, esp_err=0x%x",
                     event->error_handle->error_type,
                     event->error_handle->connect_return_code,
                     event->error_handle->esp_tls_last_esp_err);
        } else {
            ESP_LOGE(TAG, "MQTT error: error_handle is NULL");
        }
        break;
    default:
        break;
    }
}

bool app_network_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create WiFi event group
    s_wifi_event_group = xEventGroupCreate();

    // Initialize WiFi
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char*)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished. SSID: %s", CONFIG_WIFI_SSID);

    // Initialize MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "MQTT client initialized. Broker: %s", CONFIG_MQTT_BROKER_URI);

    return true;
}

bool app_network_wait_for_wifi(uint8_t max_retries)
{
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi AP");
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi AP after %d attempts", max_retries);
        return false;
    } else {
        ESP_LOGE(TAG, "Unexpected WiFi event");
        return false;
    }
}

bool app_network_mqtt_publish(const char *json_data)
{
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return false;
    }

    // Construct topic: train/data/{DEVICE_ID}
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/%s", CONFIG_MQTT_TOPIC_PREFIX, CONFIG_DEVICE_ID);

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, json_data, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT publish failed");
        return false;
    }

    ESP_LOGD(TAG, "MQTT published to %s: %s", topic, json_data);
    return true;
}

bool app_network_upload_image(camera_fb_t *fb)
{
    if (fb == NULL || fb->buf == NULL) {
        ESP_LOGE(TAG, "Invalid frame buffer");
        return false;
    }

    // Generate multipart/form-data boundary
    const char *boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    char content_type[256];
    snprintf(content_type, sizeof(content_type), 
             "multipart/form-data; boundary=%s", boundary);

    // Calculate total body size
    size_t body_size = 0;
    char part1[512];
    snprintf(part1, sizeof(part1),
             "--%s\r\n"
             "Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n"
             "Content-Type: image/jpeg\r\n\r\n",
             boundary);
    body_size += strlen(part1);
    body_size += fb->len;
    char part2[256];  // Increased from 128 to 256 to avoid format-truncation warning
    snprintf(part2, sizeof(part2),
             "\r\n--%s\r\n"
             "Content-Disposition: form-data; name=\"upload_preset\"\r\n\r\n"
             "%s\r\n"
             "--%s--\r\n",
             boundary, CONFIG_CLOUDINARY_UPLOAD_PRESET, boundary);
    body_size += strlen(part2);

    // Allocate buffer for multipart body
    uint8_t *body = malloc(body_size);
    if (body == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for HTTP body");
        return false;
    }

    // Construct multipart body
    size_t offset = 0;
    memcpy(body + offset, part1, strlen(part1));
    offset += strlen(part1);
    memcpy(body + offset, fb->buf, fb->len);
    offset += fb->len;
    memcpy(body + offset, part2, strlen(part2));

    // HTTP client configuration
    esp_http_client_config_t config = {
        .url = CONFIG_CLOUDINARY_UPLOAD_URL,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        free(body);
        return false;
    }

    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_post_field(client, (char *)body, body_size);

    esp_err_t err = esp_http_client_perform(client);
    bool success = false;

    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        int content_length = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", status_code, content_length);
        if (status_code == 200) {
            success = true;
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(body);

    return success;
}

void app_network_deinit(void)
{
    if (mqtt_client != NULL) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }

    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_wifi_event_group != NULL) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    ESP_LOGI(TAG, "Network stack deinitialized");
}

