cmake_minimum_required(VERSION 3.5)
project(rover_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# install the launch directory
install(DIRECTORY
  launch config udev
  DESTINATION share/${PROJECT_NAME}/
)

# helper scripts, installed executable
install(PROGRAMS
  scripts/list_usb_serial.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
