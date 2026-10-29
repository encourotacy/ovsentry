cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp
    camera.cpp
    ovgimbal.cpp
    usbcamera/usbcamera.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  	target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  	target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  	message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  	target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  	target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  	message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(fastcdr QUIET)
find_package(rm_interfaces QUIET)
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)

set(_sentry_special_missing_deps "")
if(NOT fastcdr_FOUND)
  	list(APPEND _sentry_special_missing_deps "fastcdr")
endif()
if(NOT rm_interfaces_FOUND)
  	list(APPEND _sentry_special_missing_deps "rm_interfaces")
endif()
if(NOT ament_cmake_FOUND)
  	list(APPEND _sentry_special_missing_deps "ament_cmake")
endif()
if(NOT rclcpp_FOUND)
  	list(APPEND _sentry_special_missing_deps "rclcpp")
endif()
if(NOT std_msgs_FOUND)
  	list(APPEND _sentry_special_missing_deps "std_msgs")
endif()
if(NOT rosidl_typesupport_cpp_FOUND)
  	list(APPEND _sentry_special_missing_deps "rosidl_typesupport_cpp")
endif()

list(JOIN _sentry_special_missing_deps ", " _sentry_special_missing_deps_str)

if(_sentry_special_missing_deps)
  	message(STATUS "[warn] skipping io_ros2_gimbal target because dependencies are missing: ${_sentry_special_missing_deps_str}")
else()
	add_library(io_ros2_gimbal STATIC
		ros2/ros2_gimbal.cpp
		ros2/subscribe.cpp
	)
	ament_target_dependencies(io_ros2_gimbal rclcpp std_msgs rm_interfaces)
	target_link_libraries(io_ros2_gimbal io)
	target_link_libraries(io_ros2_gimbal fastcdr)
endif()