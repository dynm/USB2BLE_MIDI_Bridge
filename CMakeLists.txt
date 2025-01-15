cmake_minimum_required(VERSION 3.16)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/managed_components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/managed_components)

project(usb2ble_midi_bridge) 