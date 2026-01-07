idf_component_register(
    SRCS "cam_config.c"
    INCLUDE_DIRS "include"
    REQUIRES driver app_config esp32-camera esp_driver_ledc
)

