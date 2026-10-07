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


from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


def generate_launch_description():

    pkg_name = "rover_motor_controller_cpp"
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    #
    # ARGS
    #
    hardware_distances = LaunchConfiguration("hardware_distances")
    declare_hardware_distances_cmd = DeclareLaunchArgument(
        "hardware_distances",
        default_value="[23.0, 25.5, 28.5, 26.0]",
        description="Rover hardware distances",
    )

    enc_min = LaunchConfiguration("enc_min")
    declare_enc_min_cmd = DeclareLaunchArgument(
        "enc_min",
        default_value="250",
        description="enc_min",
    )

    speed_factor = LaunchConfiguration("speed_factor")
    declare_speed_factor_cmd = DeclareLaunchArgument(
        "speed_factor",
        default_value="10",
        description="Speed [-100, +100] * 6 = [-1000, +1000]",
    )

    enc_max = LaunchConfiguration("enc_max")
    declare_enc_max_cmd = DeclareLaunchArgument(
        "enc_max",
        default_value="750",
        description="enc_max",
    )

    motor_controller_device = LaunchConfiguration("motor_controller_device")
    declare_motor_controller_device_cmd = DeclareLaunchArgument(
        "motor_controller_device",
        # Stable symlink from rover_bringup/udev/99-rover-sensors.rules.
        # A bare /dev/ttyUSB0 would race the RPLIDAR C1, which is also USB-serial.
        default_value="/dev/lx16a",  # /dev/ttyTHS1
        description="Motor controller device",
    )

    disabled_servo_ids = LaunchConfiguration("disabled_servo_ids")
    declare_disabled_servo_ids_cmd = DeclareLaunchArgument(
        "disabled_servo_ids",
        default_value="",
        description="Comma-separated LX-16A ids that get no commands (broken servo), e.g. '6' on robot2",
    )

    baud_rate = LaunchConfiguration("baud_rate")
    declare_baud_rate_cmd = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="baud rate",
    )

    #
    # NODES
    #

    vel_parser_node_cmd = Node(
        package=pkg_name,
        executable="vel_parser_node",
        name="vel_parser_node",
        parameters=[
            {
                "hardware_distances": hardware_distances,
                "enc_min": enc_min,
                "enc_max": enc_max,
                "speed_factor": speed_factor,
            }
        ],
    )

    controller_node_cmd = Node(
        package=pkg_name,
        executable="controller_node",
        name="controller_node",
        parameters=[
            {"motor_controller_device": motor_controller_device, "baud_rate": baud_rate,
             "disabled_servo_ids": ParameterValue(disabled_servo_ids, value_type=str)}
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_hardware_distances_cmd)
    ld.add_action(declare_enc_min_cmd)
    ld.add_action(declare_enc_max_cmd)
    ld.add_action(declare_speed_factor_cmd)
    ld.add_action(declare_motor_controller_device_cmd)
    ld.add_action(declare_baud_rate_cmd)
    ld.add_action(declare_disabled_servo_ids_cmd)

    ld.add_action(vel_parser_node_cmd)
    ld.add_action(controller_node_cmd)

    return ld
