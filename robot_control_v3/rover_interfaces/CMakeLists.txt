cmake_minimum_required(VERSION 3.8)
project(rover_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/WheelCommand.msg"
  "msg/WheelState.msg"
  "msg/RoverWheelsState.msg"
  "msg/RoverStatus.msg"
  "msg/TrajectoryStatus.msg"
  "srv/ResetOdometry.srv"
  "srv/StartStraightTrajectory.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
