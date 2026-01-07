idf_component_register(
    SRCS "sensor_bme680.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos system_i2c app_config
)

