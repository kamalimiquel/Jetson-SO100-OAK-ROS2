cmake_minimum_required(VERSION 3.16)
project(f710_teleop CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 Humble binary install: call its setup.bat first (build.cmd does this) so
# CMAKE_PREFIX_PATH sees rclcpp. SDL2 dev: unzip SDL2-devel-2.x.x-VC to
# C:\dev\SDL2 and pass -DSDL2_DIR=C:/dev/SDL2/cmake.
find_package(rclcpp REQUIRED)
find_package(sdl2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(f710_teleop f710_teleop.cpp)
target_link_libraries(f710_teleop rclcpp::rclcpp sensor_msgs::__rosidl_generator_cpp std_msgs::__rosidl_generator_cpp SDL2::SDL2)

install(TARGETS f710_teleop RUNTIME DESTINATION bin)
