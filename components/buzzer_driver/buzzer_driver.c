/**
 * @file buzzer_driver.c
 * @brief Passive buzzer driver using LEDC PWM
 */

#include "buzzer_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "buzzer";

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define MAX_DUTY 1023

static bool initialized = false;
static TaskHandle_t task_handle = NULL;
static volatile int alert_code = 0;

static void buzzer_on(uint32_t freq) {
  ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq);
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (MAX_DUTY * BUZZER_VOLUME) / 100);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void buzzer_off(void) {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void buzzer_task(void *arg) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int code = alert_code;

    if (code == 1) {
      ESP_LOGI(TAG, "Warning 10s");
      for (uint32_t t = 0; t < BUZZER_WARNING_DURATION_MS && alert_code == 1;
           t += BUZZER_WARNING_ON_MS + BUZZER_WARNING_OFF_MS) {
        buzzer_on(BUZZER_FREQ_WARNING);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_WARNING_ON_MS));
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(BUZZER_WARNING_OFF_MS));
      }
    } else if (code == 2) {
      ESP_LOGI(TAG, "Alarm 30s!");
      for (uint32_t t = 0; t < BUZZER_ALARM_DURATION_MS && alert_code == 2;
           t += BUZZER_ALARM_ON_MS + BUZZER_ALARM_OFF_MS) {
        buzzer_on(BUZZER_FREQ_ALARM);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_ALARM_ON_MS));
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(BUZZER_ALARM_OFF_MS));
      }
    }
    buzzer_off();
  }
}

bool buzzer_driver_init(void) {
  if (initialized)
    return true;

  ledc_timer_config_t timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,
      .freq_hz = BUZZER_FREQ_WARNING,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer));

  ledc_channel_config_t channel = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL,
      .timer_sel = LEDC_TIMER,
      .gpio_num = BUZZER_GPIO_PIN,
      .duty = 0,
      .hpoint = 0,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&channel));

  xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 4, &task_handle);
  initialized = true;
  ESP_LOGI(TAG, "Ready GPIO%d", BUZZER_GPIO_PIN);
  return true;
}

void buzzer_driver_set_alert_code(int code) {
  if (!initialized)
    return;
  alert_code = (code < 0) ? 0 : (code > 2) ? 2 : code;
  if (alert_code == 0)
    buzzer_off();
  if (task_handle)
    xTaskNotifyGive(task_handle);
}
