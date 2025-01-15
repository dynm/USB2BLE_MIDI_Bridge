idf_component_register(
    SRCS "ble_midi.c"
    INCLUDE_DIRS "include"
    REQUIRES bt log esp_common esp_system driver led_strip
) 