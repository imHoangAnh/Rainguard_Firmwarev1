idf_component_register(
    SRCS "sensor_bme680.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c freertos
    PRIV_REQUIRES system_i2c app_config esp_timer
)

