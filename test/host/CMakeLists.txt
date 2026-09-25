# Host unit test app for the pure sweep_plan / json_builder logic.
# Builds and runs on the ESP-IDF `linux` target, with no ESP32 hardware.
#
#   idf.py --preview set-target linux
#   idf.py build
#   ./build/host_tests.elf
#
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(host_tests)
