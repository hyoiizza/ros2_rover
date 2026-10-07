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

"""Rover state estimation.

Driving stack (2D lidar only, no camera in the loop):

    /scan --> rf2o_laser_odometry --> /odom_rf2o --+
                                                   +--> EKF --> /odom
    /imu  (BNO085) --------------------------------+           TF odom->base_link

    /scan --> slam_toolbox --> /map, TF map->odom            (slam:=mapping)

Perception stack (optional, off by default):

    Gemini 335 --> rtabmap --> vision / 3D map               (use_vision_map:=True)

rtabmap runs with publish_tf disabled and its occupancy grid turned off, so it
never competes with slam_toolbox for map -> odom or for /map.

Arguments:
    slam            mapping | off   (off = a map_server + AMCL run is expected
                                     instead, see rover_navigation)
    use_vision_map  also run rtabmap for the RGB-D vision map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    pkg_rover_localization = get_package_share_directory("rover_localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    slam = LaunchConfiguration("slam")
    slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="mapping",
        choices=["mapping", "off"],
        description="Run slam_toolbox mapping, or nothing (when navigating a saved map)",
    )

    use_vision_map = LaunchConfiguration("use_vision_map")
    use_vision_map_cmd = DeclareLaunchArgument(
        "use_vision_map",
        default_value="False",
        description="Also run rtabmap on the Gemini 335 to build a vision map",
    )

    # --- driving stack ---------------------------------------------------
    rf2o_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch", "rf2o.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch", "slam_toolbox.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", slam, "' == 'mapping'"])),
    )

    # --- perception stack (opt-in) ---------------------------------------
    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(use_vision_map),
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(use_vision_map_cmd)

    ld.add_action(rf2o_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(slam_toolbox_cmd)
    ld.add_action(rtabmap_cmd)

    return ld
