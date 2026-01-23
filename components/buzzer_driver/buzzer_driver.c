/**
 * @file buzzer_driver.c
 * @brief Simple buzzer driver for active buzzer
 */

#include "buzzer_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "buzzer";

static bool initialized = false;
static TaskHandle_t buzzer_task_handle = NULL;
static volatile int current_alert_code = 0;

// Timing configuration (milliseconds)
#define BEEP_WARNING_ON 200
#define BEEP_WARNING_OFF 300
#define BEEP_ALARM_ON 500
#define BEEP_ALARM_OFF 200
#define WARNING_DURATION_MS 10000 // 10 seconds
#define ALARM_DURATION_MS 30000   // 30 seconds

/**
 * @brief Buzzer task - handles alert patterns
 */
static void buzzer_task(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    int code = current_alert_code;

    if (code == 0) {
      // NORMAL - turn off
      gpio_set_level(BUZZER_GPIO_PIN, 0);
      ESP_LOGI(TAG, "Alert OFF");
    } else if (code == 1) {
      // DISTRACTED - beep for 10 seconds
      ESP_LOGI(TAG, "DISTRACTED - beeping 10s");
      uint32_t elapsed = 0;
      while (elapsed < WARNING_DURATION_MS && current_alert_code == 1) {
        gpio_set_level(BUZZER_GPIO_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(BEEP_WARNING_ON));
        gpio_set_level(BUZZER_GPIO_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(BEEP_WARNING_OFF));
        elapsed += BEEP_WARNING_ON + BEEP_WARNING_OFF;
      }
      gpio_set_level(BUZZER_GPIO_PIN, 0);
    } else if (code == 2) {
      // DROWSY - urgent beep for 30 seconds
      ESP_LOGI(TAG, "DROWSY - urgent beeping 30s!");
      uint32_t elapsed = 0;
      while (elapsed < ALARM_DURATION_MS && current_alert_code == 2) {
        gpio_set_level(BUZZER_GPIO_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(BEEP_ALARM_ON));
        gpio_set_level(BUZZER_GPIO_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(BEEP_ALARM_OFF));
        elapsed += BEEP_ALARM_ON + BEEP_ALARM_OFF;
      }
      gpio_set_level(BUZZER_GPIO_PIN, 0);
    }
  }
}

bool buzzer_driver_init(void) {
  if (initialized) {
    return true;
  }

  // Configure GPIO
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << BUZZER_GPIO_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  if (gpio_config(&io_conf) != ESP_OK) {
    ESP_LOGE(TAG, "GPIO config failed");
    return false;
  }

  gpio_set_level(BUZZER_GPIO_PIN, 0);

  // Create task
  if (xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 4, &buzzer_task_handle) !=
      pdPASS) {
    ESP_LOGE(TAG, "Task create failed");
    return false;
  }

  initialized = true;
  ESP_LOGI(TAG, "Buzzer ready on GPIO %d", BUZZER_GPIO_PIN);
  return true;
}

void buzzer_driver_set_alert_code(int alert_code) {
  if (!initialized) {
    return;
  }

  if (alert_code < 0)
    alert_code = 0;
  if (alert_code > 2)
    alert_code = 2;

  current_alert_code = alert_code;

  // If alert_code is 0, turn off buzzer IMMEDIATELY
  if (alert_code == 0) {
    gpio_set_level(BUZZER_GPIO_PIN, 0);
    ESP_LOGI(TAG, "Buzzer OFF immediately");
  }

  if (buzzer_task_handle != NULL) {
    xTaskNotifyGive(buzzer_task_handle);
  }
}
