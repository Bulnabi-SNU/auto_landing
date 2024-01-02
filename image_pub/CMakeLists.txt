cmake_minimum_required(VERSION 3.5)
project(image_pub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mono_image_pub src/mono_image_pub.cpp)
ament_target_dependencies(mono_image_pub rclcpp OpenCV sensor_msgs)

add_executable(stereo_image_pub src/stereo_image_pub.cpp)
ament_target_dependencies(stereo_image_pub rclcpp OpenCV sensor_msgs)

add_executable(camera_calibration_pub src/camera_calibration_pub.cpp)
ament_target_dependencies(camera_calibration_pub rclcpp sensor_msgs)

install(TARGETS
  mono_image_pub
  stereo_image_pub
  camera_calibration_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
