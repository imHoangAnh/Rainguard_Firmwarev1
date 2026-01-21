/**
 * @file sensor_bme680.c
 * @brief BME680 driver using Bosch BME68x API
 */

#include "sensor_bme680.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "system_i2c.h"
#include <string.h>

static const char *TAG = "bme680";

static i2c_master_dev_handle_t bme680_handle = NULL;
static struct bme68x_dev bme68x_sensor;
static bool initialized = false;

static struct {
  bme680_oversampling_t temp_os;
  bme680_oversampling_t press_os;
  bme680_oversampling_t hum_os;
  uint8_t filter;
  uint16_t gas_wait_ms;
  uint16_t gas_heater_temp;
  bool gas_enabled;
} config = {.temp_os = BME680_OS_2X,
            .press_os = BME680_OS_16X,
            .hum_os = BME680_OS_1X,
            .filter = BME68X_FILTER_SIZE_3,
            .gas_wait_ms = 100,
            .gas_heater_temp = 320,
            .gas_enabled = true};

#define IAQ_BASELINE_DEFAULT 50000.0f
#define IAQ_BURN_IN_SAMPLES 50
#define IAQ_CALIBRATION_RATE 0.005f
#define IAQ_HISTORY_SIZE 10
#define IAQ_TEMP_COMP 0.003f
#define IAQ_HUM_COMP 0.015f

static struct {
  float baseline;
  float sum;
  float min;
  float max;
  float history[IAQ_HISTORY_SIZE];
  uint8_t idx;
  uint32_t count;
  bool valid;
} iaq = {0};

static BME68X_INTF_RET_TYPE bme68x_read(uint8_t reg, uint8_t *data,
                                        uint32_t len, void *intf) {
  (void)intf;
  if (!bme680_handle)
    return BME68X_E_COM_FAIL;
  esp_err_t ret = i2c_master_transmit_receive(bme680_handle, &reg, 1, data, len,
                                              pdMS_TO_TICKS(100));
  return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static BME68X_INTF_RET_TYPE bme68x_write(uint8_t reg, const uint8_t *data,
                                         uint32_t len, void *intf) {
  (void)intf;
  if (!bme680_handle)
    return BME68X_E_COM_FAIL;
  uint8_t *buf = malloc(len + 1);
  if (!buf)
    return BME68X_E_COM_FAIL;
  buf[0] = reg;
  memcpy(buf + 1, data, len);
  esp_err_t ret =
      i2c_master_transmit(bme680_handle, buf, len + 1, pdMS_TO_TICKS(100));
  free(buf);
  return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void bme68x_delay(uint32_t period, void *intf) {
  (void)intf;
  if (period >= 1000)
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
  else
    esp_rom_delay_us(period);
}

static float compensate_gas(float gas, float temp, float hum) {
  float tf = 1.0f + IAQ_TEMP_COMP * (temp - 25.0f);
  float hf = 1.0f + IAQ_HUM_COMP * (hum - 40.0f);
  return gas * tf / hf;
}

static void update_baseline(float gas) {
  iaq.history[iaq.idx] = gas;
  iaq.idx = (iaq.idx + 1) % IAQ_HISTORY_SIZE;
  iaq.sum += gas;
  if (gas > iaq.max)
    iaq.max = gas;
  if (gas < iaq.min || iaq.min == 0)
    iaq.min = gas;
  iaq.count++;

  if (iaq.count <= IAQ_BURN_IN_SAMPLES) {
    iaq.baseline = iaq.sum / iaq.count;
    iaq.valid = false;
  } else {
    iaq.valid = true;
    if (gas > iaq.baseline) {
      iaq.baseline = iaq.baseline * (1.0f - IAQ_CALIBRATION_RATE) +
                     gas * IAQ_CALIBRATION_RATE;
    }
  }
}

static float calc_iaq_score(float gas) {
  float baseline = iaq.baseline > 0 ? iaq.baseline : IAQ_BASELINE_DEFAULT;
  float ratio = gas / baseline;
  float score;

  if (ratio >= 1.0f) {
    score = 50.0f * (2.0f - (ratio > 2.0f ? 2.0f : ratio));
  } else if (ratio >= 0.5f) {
    score = 50.0f + 100.0f * (1.0f - ratio) * 2.0f;
  } else if (ratio >= 0.2f) {
    score = 150.0f + 100.0f * ((0.5f - ratio) / 0.3f);
  } else if (ratio >= 0.1f) {
    score = 250.0f + 100.0f * ((0.2f - ratio) / 0.1f);
  } else {
    float s = (0.1f - ratio) / 0.1f;
    score = 350.0f + 150.0f * (s > 1.0f ? 1.0f : s);
  }

  return score < 0 ? 0 : (score > 500 ? 500 : score);
}

static uint8_t get_accuracy(void) {
  if (iaq.count < IAQ_BURN_IN_SAMPLES / 4)
    return 0;
  if (iaq.count < IAQ_BURN_IN_SAMPLES / 2)
    return 1;
  if (iaq.count < IAQ_BURN_IN_SAMPLES)
    return 2;
  return 3;
}

static float calculate_iaq(float temp, float hum, float gas, uint8_t *acc) {
  if (gas <= 0) {
    *acc = 0;
    return 0.0f;
  }
  float comp = compensate_gas(gas, temp, hum);
  update_baseline(comp);
  *acc = get_accuracy();
  return calc_iaq_score(comp);
}

bool sensor_bme680_init(void) {
  if (initialized)
    return true;

  i2c_master_bus_handle_t bus = system_i2c_get_bus_handle();
  if (!bus)
    return false;

  uint8_t addrs[] = {BME680_I2C_ADDR, BME680_I2C_ADDR == 0x77 ? 0x76 : 0x77};
  bool found = false;
  uint8_t addr = 0;

  for (int i = 0; i < 2; i++) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addrs[i],
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &cfg, &bme680_handle) != ESP_OK)
      continue;

    uint8_t id = 0, reg = BME68X_REG_CHIP_ID;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (i2c_master_transmit_receive(bme680_handle, &reg, 1, &id, 1,
                                    pdMS_TO_TICKS(100)) == ESP_OK &&
        id == BME68X_CHIP_ID) {
      found = true;
      addr = addrs[i];
      break;
    }
    i2c_master_bus_rm_device(bme680_handle);
    bme680_handle = NULL;
  }

  if (!found) {
    ESP_LOGE(TAG, "BME680 not found");
    return false;
  }

  memset(&bme68x_sensor, 0, sizeof(bme68x_sensor));
  bme68x_sensor.intf = BME68X_I2C_INTF;
  bme68x_sensor.intf_ptr = &addr;
  bme68x_sensor.read = bme68x_read;
  bme68x_sensor.write = bme68x_write;
  bme68x_sensor.delay_us = bme68x_delay;
  bme68x_sensor.amb_temp = 25;

  if (bme68x_init(&bme68x_sensor) != BME68X_OK) {
    i2c_master_bus_rm_device(bme680_handle);
    bme680_handle = NULL;
    return false;
  }

  sensor_bme680_configure(config.temp_os, config.press_os, config.hum_os,
                          config.filter, config.gas_wait_ms,
                          config.gas_heater_temp);

  initialized = true;
  ESP_LOGI(TAG, "BME680 ready");
  return true;
}

bool sensor_bme680_configure(bme680_oversampling_t t, bme680_oversampling_t p,
                             bme680_oversampling_t h, uint8_t f, uint16_t wait,
                             uint16_t temp) {
  if (!bme680_handle)
    return false;

  config.temp_os = t;
  config.press_os = p;
  config.hum_os = h;
  config.filter = f & 0x07;
  config.gas_wait_ms = wait;
  config.gas_heater_temp = temp < 200 ? 200 : (temp > 400 ? 400 : temp);
  config.gas_enabled = wait > 0;

  struct bme68x_conf conf;
  if (bme68x_get_conf(&conf, &bme68x_sensor) != BME68X_OK)
    return false;
  conf.os_temp = t;
  conf.os_pres = p;
  conf.os_hum = h;
  conf.filter = f;
  conf.odr = BME68X_ODR_NONE;
  if (bme68x_set_conf(&conf, &bme68x_sensor) != BME68X_OK)
    return false;

  if (config.gas_enabled) {
    struct bme68x_heatr_conf hc = {
        .enable = BME68X_ENABLE,
        .heatr_temp = config.gas_heater_temp,
        .heatr_dur = config.gas_wait_ms,
    };
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hc, &bme68x_sensor) !=
        BME68X_OK)
      return false;
  }

  return true;
}

bool sensor_bme680_set_power_mode(bme680_power_mode_t mode) {
  if (!bme680_handle)
    return false;
  return bme68x_set_op_mode((uint8_t)mode, &bme68x_sensor) == BME68X_OK;
}

bool sensor_bme680_read(bme680_data_t *data) {
  if (!bme680_handle || !data || !initialized)
    return false;

  struct bme68x_conf conf = {
      .os_temp = config.temp_os,
      .os_pres = config.press_os,
      .os_hum = config.hum_os,
      .filter = config.filter,
      .odr = BME68X_ODR_NONE,
  };
  if (bme68x_set_conf(&conf, &bme68x_sensor) != BME68X_OK)
    return false;

  if (config.gas_enabled) {
    struct bme68x_heatr_conf hc = {
        .enable = BME68X_ENABLE,
        .heatr_temp = config.gas_heater_temp,
        .heatr_dur = config.gas_wait_ms,
    };
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hc, &bme68x_sensor) !=
        BME68X_OK)
      return false;
  }

  if (bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_sensor) != BME68X_OK)
    return false;

  uint32_t dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme68x_sensor);
  if (config.gas_enabled)
    dur += config.gas_wait_ms * 1000;
  bme68x_sensor.delay_us(dur + 20000, bme68x_sensor.intf_ptr);

  struct bme68x_data d[3];
  uint8_t n = 0;
  for (int i = 0; i < 10 && n == 0; i++) {
    int8_t r = bme68x_get_data(BME68X_FORCED_MODE, d, &n, &bme68x_sensor);
    if (r == BME68X_W_NO_NEW_DATA) {
      bme68x_sensor.delay_us(10000, bme68x_sensor.intf_ptr);
    } else if (r != BME68X_OK) {
      return false;
    }
  }

  if (n == 0)
    return false;

#ifdef BME68X_USE_FPU
  data->temperature = d[0].temperature;
  data->humidity = d[0].humidity;
  data->pressure = d[0].pressure / 100.0f;
  data->gas_resistance = d[0].gas_resistance;
#else
  data->temperature = d[0].temperature / 100.0f;
  data->humidity = d[0].humidity / 1000.0f;
  data->pressure = d[0].pressure / 100.0f;
  data->gas_resistance = (float)d[0].gas_resistance;
#endif

  data->gas_valid = (d[0].status & BME68X_GASM_VALID_MSK) != 0;
  data->heat_stable = (d[0].status & BME68X_HEAT_STAB_MSK) != 0;
  data->iaq = calculate_iaq(data->temperature, data->humidity,
                            data->gas_resistance, &data->iaq_accuracy);

  return true;
}

bool sensor_bme680_self_test(void) {
  if (!bme680_handle || !initialized)
    return false;
  return bme68x_selftest_check(&bme68x_sensor) == BME68X_OK;
}

void sensor_bme680_reset_iaq_baseline(void) { memset(&iaq, 0, sizeof(iaq)); }

bool sensor_bme680_get_status(bool *gas_valid, bool *heat_stable) {
  if (!bme680_handle)
    return false;
  uint8_t status = 0;
  if (bme68x_get_regs(BME68X_REG_FIELD0 + 14, &status, 1, &bme68x_sensor) !=
      BME68X_OK)
    return false;
  if (gas_valid)
    *gas_valid = (status & BME68X_GASM_VALID_MSK) != 0;
  if (heat_stable)
    *heat_stable = (status & BME68X_HEAT_STAB_MSK) != 0;
  return true;
}

struct bme68x_dev *sensor_bme680_get_dev(void) {
  return initialized ? &bme68x_sensor : NULL;
}

void sensor_bme680_deinit(void) {
  if (bme680_handle) {
    bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme68x_sensor);
    i2c_master_bus_rm_device(bme680_handle);
    bme680_handle = NULL;
    initialized = false;
    sensor_bme680_reset_iaq_baseline();
  }
}
