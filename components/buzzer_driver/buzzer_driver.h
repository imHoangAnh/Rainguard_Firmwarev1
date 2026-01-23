/**
 * @file buzzer_driver.h
 * @brief Passive buzzer driver with PWM control (Hardcoded Config)
 *
 * Alert Levels:
 * - 0 (NORMAL): Buzzer off
 * - 1 (DISTRACTED): Warning tone
 * - 2 (DROWSY): Urgent alarm
 */

#ifndef BUZZER_DRIVER_H
#define BUZZER_DRIVER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Volume level (0-100%)
 */
#define BUZZER_VOLUME 100

/**
 * @brief Warning tone frequency (Hz) - Level 1
 */
#define BUZZER_FREQ_WARNING 5000

/**
 * @brief Alarm tone frequency (Hz) - Level 2
 */
#define BUZZER_FREQ_ALARM 5000

/**
 * @brief Alert durations (milliseconds)
 */
#define BUZZER_WARNING_DURATION_MS 10000
#define BUZZER_ALARM_DURATION_MS 30000

/**
 * @brief Beep patterns (milliseconds)
 */
#define BUZZER_WARNING_ON_MS 300
#define BUZZER_WARNING_OFF_MS 400

#define BUZZER_ALARM_ON_MS 150
#define BUZZER_ALARM_OFF_MS 100

/**
 * @brief Initialize passive buzzer (PWM)
 * @return true on success
 */
bool buzzer_driver_init(void);

/**
 * @brief Set buzzer alert from MQTT alert_code
 * @param alert_code 0=off, 1=warning, 2=alarm
 */
void buzzer_driver_set_alert_code(int alert_code);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_DRIVER_H
