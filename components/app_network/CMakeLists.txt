idf_component_register(
    SRCS "app_network.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi mqtt esp_http_client nvs_flash esp_netif cam_config
)

