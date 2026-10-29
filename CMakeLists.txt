cmake_minimum_required(VERSION 3.16.3)
project(try_sp_vision)

# 强制使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release) # ?
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# 寻找依赖库
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED)


include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/omniperception)
# add_subdirectory(tasks/auto_buff)

####################src###################
file(GLOB OVGIMBAL_SOURCES CONFIGURE_DEPENDS src/ovgimbal*.cpp) # 编译ovgimbal通信的常规程序
foreach(OVGIMBAL_SOURCE ${OVGIMBAL_SOURCES})
    get_filename_component(OVGIMBAL_TARGET ${OVGIMBAL_SOURCE} NAME_WE)
    add_executable(${OVGIMBAL_TARGET} ${OVGIMBAL_SOURCE})
    # target_link_libraries(${OVGIMBAL_TARGET} ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    target_link_libraries(${OVGIMBAL_TARGET} 
        ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
endforeach()

if(TARGET io_ros2_gimbal)
    find_package(fastcdr REQUIRED)
    file(GLOB OVSENTRY_SOURCES CONFIGURE_DEPENDS src/ovsentry*.cpp) # 编译ovsentry程序，它们使用ROS2通信
    foreach(OVSENTRY_SOURCE ${OVSENTRY_SOURCES})
        get_filename_component(OVSENTRY_TARGET ${OVSENTRY_SOURCE} NAME_WE)
        add_executable(${OVSENTRY_TARGET} ${OVSENTRY_SOURCE})
        target_link_libraries(${OVSENTRY_TARGET} 
            ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io_ros2_gimbal)
    endforeach()
else()
    message(STATUS "io_ros2_gimbal target unavailable, skipping ovsentry_* targets.")
endif()

##################calibration################
file(GLOB CALIBRATION_SOURCES CONFIGURE_DEPENDS calibration/*.cpp)
foreach(CALIBRATION_SOURCE ${CALIBRATION_SOURCES})
    get_filename_component(CALIBRATION_TARGET ${CALIBRATION_SOURCE} NAME_WE)
    add_executable(${CALIBRATION_TARGET} ${CALIBRATION_SOURCE})
    target_link_libraries(${CALIBRATION_TARGET} ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
endforeach()

if(TARGET capture_chessboard)
    if(TARGET io_ros2_gimbal)
        target_link_libraries(capture_chessboard io_ros2_gimbal)
    else()
        message(FATAL_ERROR "capture_chessboard reads quaternion from navigation and requires io_ros2_gimbal.")
    endif()
endif()

##################test################
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/*.cpp)
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_TARGET ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_TARGET} ${TEST_SOURCE})
    target_link_libraries(${TEST_TARGET} ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
endforeach()