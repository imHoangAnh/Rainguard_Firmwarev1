/**
 * @file buzzer_driver.h
 * @brief Simple buzzer driver for driver drowsiness alert
 *
 * Alert Levels:
 * - 0 (NORMAL): Buzzer off
 * - 1 (DISTRACTED): Beeping for 10 seconds
 * - 2 (DROWSY): Urgent beeping for 30 seconds
 */

#ifndef BUZZER_DRIVER_H
#define BUZZER_DRIVER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize buzzer driver
 * @return true on success
 */
bool buzzer_driver_init(void);

/**
 * @brief Set buzzer alert from MQTT alert_code
 * @param alert_code 0=off, 1=warning(10s), 2=alarm(30s)
 */
void buzzer_driver_set_alert_code(int alert_code);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_DRIVER_H
