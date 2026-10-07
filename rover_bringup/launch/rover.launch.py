# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import os
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    rover_bringup_shared_dir = get_package_share_directory("rover_bringup")
    rover_motor_controller_shared_dir = get_package_share_directory(
        "rover_motor_controller_cpp"
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    #
    # LAUNCHES
    #

    # Slamtec RPLIDAR C1 (2D lidar)
    rplidar_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_shared_dir, "launch", "rplidar.launch.py")
        ),
        launch_arguments={
            "config_filepath": os.path.join(
                rover_bringup_shared_dir, "config", "rplidar_c1.yaml"
            )
        }.items(),
    )

    # Orbbec Gemini 335 (RGB-D camera)
    gemini_335_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_shared_dir, "launch", "gemini_335.launch.py")
        )
    )

    # Adafruit BNO085 (9-DOF IMU).
    #
    # Optional: the driving stack works without it. Odometry comes from
    # rf2o_laser_odometry (scan matching) and the EKF simply gets no imu0 data,
    # which robot_localization tolerates. Expect more yaw drift, especially
    # while turning in place or in open, feature-poor areas - see
    # rover_localization/config/ekf.yaml.
    use_imu = LaunchConfiguration("use_imu")
    use_imu_cmd = DeclareLaunchArgument(
        "use_imu",
        default_value="True",
        description="Run the BNO085 driver. Set False if the IMU is not wired up yet.",
    )

    bno085_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_shared_dir, "launch", "bno085.launch.py")
        ),
        launch_arguments={
            "config_filepath": os.path.join(
                rover_bringup_shared_dir, "config", "bno085.yaml"
            )
        }.items(),
        condition=IfCondition(use_imu),
    )

    rover_motor_controller_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_motor_controller_shared_dir, "launch", "motor_controller.launch.py"
            )
        )
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(use_imu_cmd)

    ld.add_action(rplidar_action_cmd)
    ld.add_action(gemini_335_action_cmd)
    ld.add_action(bno085_action_cmd)
    ld.add_action(rover_motor_controller_action_cmd)

    # Manual driving is done from the keyboard, which is NOT started here:
    # teleop_keyboard_node reads raw keypresses from its terminal and ros2
    # launch does not hand its children a usable stdin. Run it yourself in a
    # second terminal:
    #
    #   ros2 run rover_teleop teleop_keyboard_node
    #
    # It publishes geometry_msgs/Twist on /cmd_vel, which vel_parser_node
    # (started above) turns into /motors_command.

    return ld
