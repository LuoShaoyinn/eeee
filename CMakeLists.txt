cmake_minimum_required(VERSION 3.16)
project(cubie_robot LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio
             OPTIONAL_COMPONENTS calib calib3d features)
find_package(Threads REQUIRED)
if(NOT OpenCV_calib_FOUND AND NOT OpenCV_calib3d_FOUND)
    message(FATAL_ERROR "OpenCV camera-calibration module not found (calib or calib3d)")
endif()
if(OpenCV_VERSION_MAJOR GREATER_EQUAL 5 AND NOT OpenCV_features_FOUND)
    message(FATAL_ERROR "OpenCV 5 features module not found")
endif()

add_library(robot_transport
    src/hardware/serial_port.cpp
    src/hardware/esp32_client.cpp)
target_include_directories(robot_transport PUBLIC include)
target_compile_options(robot_transport PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotd apps/robotd.cpp)
target_link_libraries(robotd PRIVATE robot_transport)
target_compile_options(robotd PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotctl apps/robotctl.cpp)
target_link_libraries(robotctl PRIVATE robot_transport)
target_compile_options(robotctl PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(robot_location src/localization/location.cpp)
target_include_directories(robot_location PUBLIC include)
target_link_libraries(robot_location PUBLIC ${OpenCV_LIBS})
target_compile_options(robot_location PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(robot_config src/config/runtime_config.cpp)
target_include_directories(robot_config PUBLIC include)
target_link_libraries(robot_config PUBLIC ${OpenCV_LIBS})
target_compile_options(robot_config PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(robot_autonomy
    src/control/safety_supervisor.cpp
    src/perception/object_projection.cpp
    src/planning/approach_controller.cpp
    src/planning/mission.cpp
    src/planning/world_model.cpp)
target_include_directories(robot_autonomy PUBLIC include)
target_link_libraries(robot_autonomy PUBLIC robot_location)
target_compile_options(robot_autonomy PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robotloc apps/robotloc.cpp)
target_link_libraries(robotloc PRIVATE robot_location robot_config robot_autonomy Threads::Threads)
target_compile_options(robotloc PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(robot-runtime apps/robotloc.cpp)
target_link_libraries(robot-runtime PRIVATE robot_location robot_config robot_autonomy Threads::Threads)
target_compile_options(robot-runtime PRIVATE -Wall -Wextra -Wpedantic -Werror)

set(ROBOT_A733_MODEL_ZOO "" CACHE PATH "Path to the A733 model-zoo root")
if(ROBOT_A733_MODEL_ZOO)
    set(A733_NPU_RUNTIME "${ROBOT_A733_MODEL_ZOO}/common/npuruntime")
    file(GLOB A733_NPU_LIBS "${A733_NPU_RUNTIME}/lib_linux_aarch64/A733/*.so*")
    add_executable(robot-runtime-a733
        apps/robotloc.cpp
        src/perception/a733_detector.cpp
        src/a733-yolo26/yolo26_pre.cpp
        src/a733-yolo26/yolo26_raw_post.cpp
        ${A733_NPU_RUNTIME}/npulib.cpp
        ${A733_NPU_RUNTIME}/npu_util.cpp)
    target_compile_definitions(robot-runtime-a733 PRIVATE ROBOT_A733_NPU=1)
    target_include_directories(robot-runtime-a733 PRIVATE
        src/a733-yolo26 ${A733_NPU_RUNTIME} ${A733_NPU_RUNTIME}/include)
    target_link_libraries(robot-runtime-a733 PRIVATE robot_location robot_config
        robot_autonomy Threads::Threads ${A733_NPU_LIBS} dl m)
    target_compile_options(robot-runtime-a733 PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS robot-runtime-a733 RUNTIME DESTINATION bin)
endif()

install(TARGETS robotd robotctl robotloc robot-runtime RUNTIME DESTINATION bin)
install(FILES config/camera_fisheye_1280x720.yaml config/robot.yaml DESTINATION config)
install(PROGRAMS tools/uart_ota_update.py DESTINATION tools)

if(BUILD_TESTING)
    add_executable(robot_core_tests tests/core_tests.cpp)
    target_link_libraries(robot_core_tests PRIVATE robot_autonomy)
    target_compile_options(robot_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME robot_core_tests COMMAND robot_core_tests)

    add_executable(robot_config_tests tests/config_tests.cpp)
    target_link_libraries(robot_config_tests PRIVATE robot_config)
    target_compile_definitions(robot_config_tests PRIVATE ROBOT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_compile_options(robot_config_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME robot_config_tests COMMAND robot_config_tests)

    add_executable(robot_location_tests tests/location_tests.cpp)
    target_link_libraries(robot_location_tests PRIVATE robot_location)
    target_compile_options(robot_location_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME robot_location_tests COMMAND robot_location_tests)
endif()
