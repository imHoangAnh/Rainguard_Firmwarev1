idf_component_register(
    SRCS "system_i2c.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c freertos
    PRIV_REQUIRES app_config
)
