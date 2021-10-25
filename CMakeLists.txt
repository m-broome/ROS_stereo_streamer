cmake_minimum_required(VERSION 3.5)
project(stereo_streamer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)

#======================
# Stereo publisher node
#======================
add_library(stereo_publisher_lib SHARED
  src/stereo_publisher.cpp
  src/stereo_publisher_node.cpp
)
ament_target_dependencies(stereo_publisher_lib camera_calibration_parsers OpenCV rclcpp sensor_msgs)

#================
# Stereo streamer
#================
add_executable(stereo_streamer 
  src/stereo_publisher_main.cpp
)
target_link_libraries(stereo_streamer stereo_publisher_lib)
ament_target_dependencies(stereo_streamer rclcpp)

#=============
# Install
#=============

install(
  TARGETS stereo_publisher_lib
  EXPORT export_stereo_publisher_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
stereo_streamer
DESTINATION lib/${PROJECT_NAME})

ament_package()
