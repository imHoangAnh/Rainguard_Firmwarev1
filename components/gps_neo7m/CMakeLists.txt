# GPS NEO-7M Component CMakeLists.txt
# u-blox NEO-7M GNSS Module Driver for ESP-IDF 5.5.2

idf_component_register(
    SRCS "gps_neo7m.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer
    PRIV_REQUIRES app_config
)

# Optional: Enable strict warnings
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
