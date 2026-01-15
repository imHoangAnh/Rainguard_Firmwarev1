idf_component_register(
    SRCS "sensor_bme680.c" "driver/bme68x.c"
    INCLUDE_DIRS "." "driver"
    REQUIRES esp_driver_i2c freertos
    PRIV_REQUIRES system_i2c app_config esp_timer
)

