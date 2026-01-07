idf_component_register(
    SRCS "gps_neo6m.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos app_config
)

