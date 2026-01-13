idf_component_register(
    SRCS "sensor_mpu6050.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c driver freertos esp_timer
    PRIV_REQUIRES system_i2c app_config
)

