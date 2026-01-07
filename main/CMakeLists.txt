idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_wifi mqtt esp_http_client nvs_flash freertos
                 app_network system_i2c sensor_bme680 sensor_mpu6050 gps_neo6m cam_config app_config
)

