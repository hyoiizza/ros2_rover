cmake_minimum_required(VERSION 3.5)
project(rover_localization)

# find dependencies
find_package(ament_cmake REQUIRED)

# install the launch directory
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/scan_self_filter.py
  scripts/odom_stationary_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
