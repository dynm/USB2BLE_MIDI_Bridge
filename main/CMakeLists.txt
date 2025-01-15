idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES usb_midi ble_midi nvs_flash driver
) 