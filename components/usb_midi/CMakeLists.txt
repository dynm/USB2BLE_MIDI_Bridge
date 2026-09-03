idf_component_register(
    SRCS "usb_midi.c" "usb_midi_descriptor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_hw_support usb
)
