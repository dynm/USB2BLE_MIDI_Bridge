idf_component_register(
    SRCS "main.c" "ble_to_usb_midi.c" "usb_to_ble_midi.c" "remote_debug.c"
    INCLUDE_DIRS "."
    REQUIRES usb_midi ble_midi nvs_flash driver esp_wifi esp_netif esp_event
             esp_http_server app_update json esp_timer mbedtls
    EMBED_TXTFILES "debug.html"
)
