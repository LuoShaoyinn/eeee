cmake_minimum_required(VERSION 3.16)

# This repository starts without a commit, so provide a stable app version
# instead of relying on ESP-IDF's git-describe fallback.
set(PROJECT_VER "0.1.0")
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/motor-driver/JCA25-2430-CE")
list(APPEND EXTRA_COMPONENT_DIRS
     "${CMAKE_CURRENT_LIST_DIR}/servo-driver/standard-3wire")
list(APPEND EXTRA_COMPONENT_DIRS
     "${CMAKE_CURRENT_LIST_DIR}/imu-driver/JY61P-UART")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_hello_world)
