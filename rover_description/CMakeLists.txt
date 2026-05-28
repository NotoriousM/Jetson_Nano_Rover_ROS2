cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)          # ← добавьте если нет

install(DIRECTORY
  launch
  meshes
  urdf
  worlds
  rviz
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Регистрируем figure_eight.py как исполняемый узел ROS 2
install(PROGRAMS
  scripts/figure_lemniscate_rviz.py
  scripts/figure_eight.py
  scripts/figure_eight_rviz.py 
  scripts/path_publisher.py
  scripts/ackermann_gazebo_node.py   # ← добавить
  scripts/ackermann_sim_node.py
  scripts/robot_keyboard_controller.py
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
