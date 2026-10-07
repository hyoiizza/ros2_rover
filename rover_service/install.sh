# dependencies
echo "--------INSTALLING DEPENDENCIES"
# Manual driving uses rover_teleop's keyboard node (pure rclpy), so there is
# nothing extra to apt-install for teleop.
# sllidar_ros2 (RPLIDAR C1), orbbec_camera (Gemini 335) and bno08x_driver
# (BNO085) are not in the ROS 2 apt repositories: clone them into the
# workspace src/ and build them with colcon.

# udev rules: the RPLIDAR C1 and the LX-16A BusLinker are both USB-serial,
# so /dev/ttyUSB* numbering is not stable. These rules give them the fixed
# names /dev/rplidar and /dev/lx16a that the configs expect. Verify the
# VID:PID of your own boards first with:
#   ros2 run rover_bringup list_usb_serial.sh
echo "--------INSTALLING UDEV RULES"
cp ../rover_bringup/udev/99-rover-sensors.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

# copy rover project
echo "--------COPYING SH FILE"
cp rover.sh /usr/local/bin/rover.sh >>/dev/null

# copy rover service
echo "--------COPYING SERVICE FILE"
cp rover.service /etc/systemd/system/rover.service

# add permissions
echo "--------ADDING PERMISSIONS"
chmod 744 /usr/local/bin/rover.sh
chmod 664 /etc/systemd/system/rover.service

# enable service
echo "--------ENABLING SERVICE"
systemctl daemon-reload
systemctl enable rover.service

echo "--------STARTING SERVICE"
# start service
systemctl start rover
