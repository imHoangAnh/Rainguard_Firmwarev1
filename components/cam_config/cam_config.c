/**
 * @file cam_config.c
 * @brief OV2640 camera driver for VGA JPEG snapshots
 */

#include "cam_config.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "camera";
static bool camera_initialized = false;

static void cam_power_sequence(void) {
  if (CAM_PIN_PWDN >= 0) {
    gpio_config_t conf = {
        .pin_bit_mask =
            (CAM_PIN_PWDN >= 0 ? (1ULL << (uint32_t)CAM_PIN_PWDN) : 0),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&conf);
    gpio_set_level(CAM_PIN_PWDN, 0);
  }

  if (CAM_PIN_RESET >= 0) {
    gpio_config_t conf = {
        .pin_bit_mask =
            (CAM_PIN_RESET >= 0 ? (1ULL << (uint32_t)CAM_PIN_RESET) : 0),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&conf);
    gpio_set_level(CAM_PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CAM_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CAM_PIN_RESET, 1);
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}

bool cam_config_init(void) {
  if (camera_initialized)
    return true;

  cam_power_sequence();

#if CONFIG_SPIRAM
  bool psram = true;
#else
  bool psram = false;
#endif

  camera_config_t config = {
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
      .xclk_freq_hz = 10000000,
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,
      .pixel_format = PIXFORMAT_JPEG,
      .frame_size = FRAMESIZE_VGA,
      .jpeg_quality = 10,
      .fb_count = psram ? 2 : 1,
      .fb_location = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
      .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
      .sccb_i2c_port = -1,
  };

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(err));
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_quality(sensor, 10);
    sensor->set_whitebal(sensor, 1);
    sensor->set_awb_gain(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
  }

  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb)
      esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  camera_initialized = true;
  ESP_LOGI(TAG, "Camera ready");
  return true;
}

bool cam_config_capture_frame(camera_fb_t **fb) {
  if (!camera_initialized || !fb)
    return false;

  static int failures = 0;

  for (int i = 0; i < 3; i++) {
    *fb = esp_camera_fb_get();
    if (*fb) {
      failures = 0;
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  failures++;
  if (failures >= 5) {
    cam_config_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    cam_config_init();
    failures = 0;
  }

  return false;
}

void cam_config_free_frame(camera_fb_t *fb) {
  if (fb)
    esp_camera_fb_return(fb);
}

bool cam_config_is_initialized(void) { return camera_initialized; }

void cam_config_deinit(void) {
  if (!camera_initialized)
    return;
  esp_camera_return_all();
  esp_camera_deinit();
  camera_initialized = false;
}
