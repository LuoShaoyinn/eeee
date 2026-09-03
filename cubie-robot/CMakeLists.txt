cmake_minimum_required(VERSION 3.16)
project(cubie_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio
             OPTIONAL_COMPONENTS calib calib3d features)
if(NOT OpenCV_calib_FOUND AND NOT OpenCV_calib3d_FOUND)
    message(FATAL_ERROR "OpenCV camera-calibration module not found (calib or calib3d)")
endif()
if(OpenCV_VERSION_MAJOR GREATER_EQUAL 5 AND NOT OpenCV_features_FOUND)
    message(FATAL_ERROR "OpenCV 5 features module not found")
endif()

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

add_library(robot_location src/location.cpp)
target_include_directories(robot_location PUBLIC include)
target_link_libraries(robot_location PUBLIC ${OpenCV_LIBS})
target_compile_options(robot_location PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotloc src/robotloc.cpp)
target_link_libraries(robotloc PRIVATE robot_location)
target_compile_options(robotloc PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS robotd robotctl robotloc RUNTIME DESTINATION bin)
