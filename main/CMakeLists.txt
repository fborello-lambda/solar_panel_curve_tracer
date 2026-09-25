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
# sweep_plan and json_builder are NOT listed in REQUIRES/PRIV_REQUIRES below.
# There is no #include scanning: when a component's idf_component_register
# call omits REQUIRES/PRIV_REQUIRES entirely, ESP-IDF implicitly requires
# every other component in the build for it (main gets driver, esp_timer,
# sweep_plan, json_builder, all of it, for free). Listing even one component
# explicitly switches that off and makes the list exhaustive instead of
# additive, so main would then need driver/esp_timer/etc spelled out by hand
# too. Left unset here on purpose so main keeps its (undeclared) access to
# every project component.
idf_component_register(SRCS ${SRCS}
                        INCLUDE_DIRS "." "drivers" "utils" "server" "db")

# From: https://github.com/espressif/esp-idf/blob/master/examples/storage/spiffsgen/CMakeLists.txt
# Create a SPIFFS image from the contents of the 'spiffs' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'.
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)
