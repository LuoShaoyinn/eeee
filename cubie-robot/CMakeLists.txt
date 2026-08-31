cmake_minimum_required(VERSION 3.16)
project(cubie_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_transport
    src/serial_port.cpp
    src/esp32_client.cpp)
target_include_directories(robot_transport PUBLIC include)
target_compile_options(robot_transport PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotd src/robotd.cpp)
target_link_libraries(robotd PRIVATE robot_transport)
target_compile_options(robotd PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotctl src/robotctl.cpp)
target_link_libraries(robotctl PRIVATE robot_transport)
target_compile_options(robotctl PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS robotd robotctl RUNTIME DESTINATION bin)
