# Collect top-level app sources and all .c files under drivers, utils, server, db
set(SRCS
    "main.c"
    "measurement.c"
    "dynamic_load.c"
    "ui.c"
)
file(GLOB_RECURSE LIB_SRCS  "${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.c"
                            "${CMAKE_CURRENT_SOURCE_DIR}/utils/*.c"
                            "${CMAKE_CURRENT_SOURCE_DIR}/server/*.c"
                            "${CMAKE_CURRENT_SOURCE_DIR}/db/*.c"
                            "${CMAKE_CURRENT_SOURCE_DIR}/app/*.c"
                            )
list(APPEND SRCS ${LIB_SRCS})

# Register component with include dirs.
# sweep_plan and json_builder are picked up by ESP-IDF's automatic
# component requirement detection (they're found via #include in
# measurement.c and server/server.h). Listing them explicitly in
# REQUIRES/PRIV_REQUIRES turns off that auto-detection for the whole
# component, which then fails to find driver/esp_timer/etc, so they're
# deliberately left out here.
idf_component_register(SRCS ${SRCS}
                        INCLUDE_DIRS "." "drivers" "utils" "server" "db")

# From: https://github.com/espressif/esp-idf/blob/master/examples/storage/spiffsgen/CMakeLists.txt
# Create a SPIFFS image from the contents of the 'spiffs' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'.
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)
