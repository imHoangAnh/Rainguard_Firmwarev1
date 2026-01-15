idf_component_register(
    SRCS "gps_neo6m.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos
    PRIV_REQUIRES app_config
)

