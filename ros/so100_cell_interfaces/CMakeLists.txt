cmake_minimum_required(VERSION 3.8)
project(so100_cell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SolveRequest.msg"
  "msg/SolveResult.msg"
  "srv/ClaimArm.srv"
  "srv/SkillRequest.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
