# MicroPython CSI user C module
# Include via: cmake ... -DUSER_C_MODULES=/path/to/csi_module/micropython.cmake

add_library(usermod_csi INTERFACE)

target_sources(usermod_csi INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/csi.c
)

target_include_directories(usermod_csi INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link against esp_wifi component (already available in ESP-IDF build)
target_link_libraries(usermod_csi INTERFACE
    idf::esp_wifi
    idf::esp_event
    idf::freertos
)

target_link_libraries(usermod INTERFACE usermod_csi)
