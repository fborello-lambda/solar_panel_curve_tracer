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

# --- Embedded web UI ---------------------------------------------------
# Each file under web/ is gzip-compressed at build time (mtime pinned to 0
# for reproducible output) and the resulting .gz is embedded directly into
# the firmware image as binary data. This replaces the old SPIFFS image, so
# an OTA update is a single app binary instead of app + storage.
set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/web")
# guide.pdf is deliberately NOT embedded: it barely gzips (already-compressed
# PDF content) and its ~80 KB pushed the secure-boot variant over the app
# partition size gate. /guide now serves a compact embedded guide.html
# instead (see server.c), with a link to the hosted docs/quick_guide.md.
set(WEB_FILES
    "index.html"
    "script.js"
    "chart.umd.min.js"
    "ota.html"
    "i18n.js"
    "guide.html"
)

set(WEB_GZ_FILES "")
foreach(WEB_FILE ${WEB_FILES})
    set(SRC_PATH "${WEB_SRC_DIR}/${WEB_FILE}")
    set(GZ_PATH "${CMAKE_CURRENT_BINARY_DIR}/${WEB_FILE}.gz")
    add_custom_command(
        OUTPUT "${GZ_PATH}"
        COMMAND ${CMAKE_COMMAND} -E env python3 -c
                "import gzip,shutil,sys; f_in=open(sys.argv[1],'rb'); data=f_in.read(); f_in.close(); f_out=gzip.GzipFile(filename='', mode='wb', fileobj=open(sys.argv[2],'wb'), mtime=0); f_out.write(data); f_out.close()"
                "${SRC_PATH}" "${GZ_PATH}"
        DEPENDS "${SRC_PATH}"
        COMMENT "Gzipping ${WEB_FILE} for embedding"
        VERBATIM
    )
    list(APPEND WEB_GZ_FILES "${GZ_PATH}")
endforeach()

add_custom_target(web_gz_files DEPENDS ${WEB_GZ_FILES})
add_dependencies(${COMPONENT_LIB} web_gz_files)

foreach(GZ_PATH ${WEB_GZ_FILES})
    target_add_binary_data(${COMPONENT_LIB} "${GZ_PATH}" BINARY)
endforeach()
