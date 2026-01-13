idf_component_register(
    SRCS "cam_config.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp32-camera esp_driver_ledc
    PRIV_REQUIRES app_config esp_timer
)

