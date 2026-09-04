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

add_executable(robotd apps/robotd.cpp)
target_link_libraries(robotd PRIVATE robot_transport)
target_compile_options(robotd PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotctl apps/robotctl.cpp)
target_link_libraries(robotctl PRIVATE robot_transport)
target_compile_options(robotctl PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(robot_location src/location.cpp)
target_include_directories(robot_location PUBLIC include)
target_link_libraries(robot_location PUBLIC ${OpenCV_LIBS})
target_compile_options(robot_location PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(robot_mission src/mission.cpp)
target_include_directories(robot_mission PUBLIC include)
target_compile_options(robot_mission PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotloc apps/robotloc.cpp)
target_link_libraries(robotloc PRIVATE robot_location)
target_compile_options(robotloc PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(mission_sim apps/mission_sim.cpp)
target_link_libraries(mission_sim PRIVATE robot_mission)
target_compile_options(mission_sim PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotbrain apps/robotbrain.cpp)
target_link_libraries(robotbrain PRIVATE robot_mission)
target_compile_options(robotbrain PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(CTest)
if(BUILD_TESTING)
    add_executable(mission_test tests/mission_test.cpp)
    target_link_libraries(mission_test PRIVATE robot_mission)
    target_compile_options(mission_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME mission_test COMMAND mission_test)
endif()

install(TARGETS robotd robotctl robotloc mission_sim robotbrain RUNTIME DESTINATION bin)
install(FILES
    config/camera_fisheye_1280x720.yaml
    config/mission_v1.yaml
    DESTINATION config)
