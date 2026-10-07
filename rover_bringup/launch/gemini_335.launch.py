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

"""Launch the Orbbec Gemini 335 RGB-D camera.

The Gemini 335 is part of the Gemini 330 series, so it is driven by the
gemini_330_series.launch.py file shipped with the OrbbecSDK_ROS2 wrapper
(package orbbec_camera).

That launch file takes ~180 arguments; rather than spelling them out here, the
rover's settings live in config/gemini_335.yaml and are passed through the
wrapper's own ``config_file_path`` argument, which overrides the launch
defaults. See that file for what is set and why.

``camera_name`` stays a launch argument because the wrapper uses it for the
node namespace (and therefore the topic and TF prefixes), not only as a
parameter.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    orbbec_camera_shared_dir = get_package_share_directory("orbbec_camera")
    config_directory = os.path.join(
        get_package_share_directory("rover_bringup"), "config"
    )

    camera_name = LaunchConfiguration("camera_name")
    camera_name_cmd = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="Namespace of the camera node, used as topic and TF prefix",
    )

    config_filepath = LaunchConfiguration("config_filepath")
    config_filepath_cmd = DeclareLaunchArgument(
        "config_filepath",
        default_value=os.path.join(config_directory, "gemini_335.yaml"),
        description="Gemini 335 parameters file",
    )

    gemini_335_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                orbbec_camera_shared_dir, "launch", "gemini_330_series.launch.py"
            )
        ),
        launch_arguments={
            "camera_name": camera_name,
            "config_file_path": config_filepath,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(camera_name_cmd)
    ld.add_action(config_filepath_cmd)

    ld.add_action(gemini_335_cmd)

    return ld
