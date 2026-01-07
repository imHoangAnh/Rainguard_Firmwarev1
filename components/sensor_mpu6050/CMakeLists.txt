idf_component_register(
    SRCS "sensor_mpu6050.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos system_i2c app_config esp_timer
)

