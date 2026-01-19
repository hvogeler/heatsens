idf_component_register(
    SRCS "webprov.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_server nvs_flash esp_netif esp_event log json
)
