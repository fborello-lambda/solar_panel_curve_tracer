# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Firmware version string, reported over the web UI/OTA. `git describe` was
# picking up stale secure-boot tags and producing strings over the 31-char
# limit (e.g. "rev1.8177289-secure-boot-1-g5bd"), so set PROJECT_VER
# explicitly instead:
#   - default: short commit hash of HEAD, or "dev" if git is unavailable
#   - override: -DPROJECT_VER_OVERRIDE=<value> on the idf.py/cmake command
#     line, or a PROJECT_VER_OVERRIDE environment variable. CI passes
#     "<shortsha>-standard" / "<shortsha>-secure".
if(NOT DEFINED PROJECT_VER_OVERRIDE AND DEFINED ENV{PROJECT_VER_OVERRIDE})
  set(PROJECT_VER_OVERRIDE "$ENV{PROJECT_VER_OVERRIDE}")
endif()

if(DEFINED PROJECT_VER_OVERRIDE AND NOT "${PROJECT_VER_OVERRIDE}" STREQUAL "")
  set(PROJECT_VER "${PROJECT_VER_OVERRIDE}")
else()
  execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE PROJECT_VER
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE PROJECT_VER_GIT_RESULT
  )
  if(NOT PROJECT_VER_GIT_RESULT EQUAL 0 OR "${PROJECT_VER}" STREQUAL "")
    set(PROJECT_VER "dev")
  endif()
endif()

string(LENGTH "${PROJECT_VER}" PROJECT_VER_LEN)
if(PROJECT_VER_LEN GREATER 31)
  string(SUBSTRING "${PROJECT_VER}" 0 31 PROJECT_VER)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(web_server)