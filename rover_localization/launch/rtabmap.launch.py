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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        # rtabmap_viz is a GUI; the rover is normally driven headless over SSH.
        default_value="False",
        description="Wheather to launch rtabmapviz",
    )

    # rtabmap's "-d" wipes its database at startup. Keep that as the default so
    # each run starts a clean vision map, but allow resuming an existing one.
    delete_db = LaunchConfiguration("delete_db")
    delete_db_cmd = DeclareLaunchArgument(
        "delete_db",
        default_value="True",
        description="Delete the rtabmap database on start instead of resuming it",
    )

    database_path = LaunchConfiguration("database_path")
    database_path_cmd = DeclareLaunchArgument(
        "database_path",
        default_value=os.path.expanduser("~/.ros/rtabmap.db"),
        description="RTAB-Map database path",
    )

    subscribe_scan = LaunchConfiguration("subscribe_scan")
    subscribe_scan_cmd = DeclareLaunchArgument(
        "subscribe_scan",
        default_value="True",
        description="Subscribe to the 2D laser scan",
    )

    scan_topic = LaunchConfiguration("scan_topic")
    scan_topic_cmd = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="2D laser scan topic",
    )

    imu_topic = LaunchConfiguration("imu_topic")
    imu_topic_cmd = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu",
        description="IMU topic",
    )

    # Frame / topic names. The defaults are the single-robot setup; the
    # multi-robot bringup (rover_multi) passes "<robot>/base_link" etc.
    base_frame = LaunchConfiguration("base_frame")
    base_frame_cmd = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame",
    )

    map_frame = LaunchConfiguration("map_frame")
    map_frame_cmd = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Map frame published by RTAB-Map",
    )

    odom_frame = LaunchConfiguration("odom_frame")
    odom_frame_cmd = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="Odometry frame (EKF odom -> base_link), looked up on TF",
    )

    map_topic = LaunchConfiguration("map_topic")
    map_topic_cmd = DeclareLaunchArgument(
        "map_topic",
        default_value="/map",
        description="Occupancy grid topic",
    )

    parameters = [
        {
            "frame_id": base_frame,
            "map_frame_id": map_frame,
            # Odometry from TF instead of the odom topic: the depth stream
            # arrives ~2 s late on the Jetson, and a 30 Hz odom topic in the
            # sync queue has already dropped the matching message by then.
            # The TF buffer keeps ~10 s, so the pose at the image stamp is
            # still there.
            "odom_frame_id": odom_frame,
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": subscribe_scan,
            "approx_sync": True,
            # Must cover the depth delay for the fastest synced topic
            # (camera_info ~30 Hz): 100 msgs ~ 3 s.
            "sync_queue_size": 100,
            "approx_sync_max_interval": 0.1,
            "wait_for_transform": 0.5,
            # RTAB-Map provides the navigation map and map -> odom transform.
            "publish_tf": True,
            # Republish /map on every update, not only when a node is added. The frontier
            # explorer calibrates its rate from the first /map messages and never starts
            # while the rover stands still and the map is published only once, so the
            # rover never got its first goal.
            "map_always_update": True,
            "use_sim_time": use_sim_time,
            "database_path": database_path,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_imu": 2,
            # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            "RGBD/Enabled": "true",
            # rtabmap default 3.0 (was 0.5). ICP-refined links have tiny stddev, so at
            # 0.5 even 0.7 deg / 3 cm loop closures were rejected as "wrong".
            "RGBD/OptimizeMaxError": "3.0",
            "RGBD/OptimizeFromGraphEnd": "false",
            # Publish the RGB-D occupancy grid on /map for Nav2.
            "RGBD/CreateOccupancyGrid": "true",
            "RGBD/LoopClosureIdentityGuess": "false",
            "RGBD/LocalBundleOnLoopClosure": "false",
            "VhEp/Enabled": "false",
            "Rtabmap/CreateIntermediateNodes": "false",
            # Use the RPLIDAR, not only the camera, for registration: every new
            # node's odometry link is refined by scan ICP, and proximity / loop
            # closures are verified with ICP too. Without this the map was plain
            # EKF odometry and the walls smeared.
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "true",
            "RGBD/NeighborLinkRefining": "true",
            "Icp/VoxelSize": "0.05",
            "Icp/MaxCorrespondenceDistance": "0.1",
            # A single 2D scan has no normals unless rtabmap is asked to compute them.
            "Icp/PointToPlane": "false",
            "GFTT/MinDistance": "7.0",
            "GFTT/QualityLevel": "0.001",
            "GFTT/BlockSize": "3",
            "GFTT/UseHarrisDetector": "true",
            "GFTT/K": "0.04",
            "BRIEF/Bytes": "64",
            # Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
            "Vis/EstimationType": "1",
            "Vis/ForwardEstOnly": "true",
            # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector
            "Vis/FeatureType": "8",
            "Vis/DepthAsMask": "true",
            "Vis/CorGuessWinSize": "40",
            "Vis/MaxFeatures": "0",
            "Vis/MinDepth": "0.0",
            "Vis/MaxDepth": "0.0",
            # 0=Features Matching, 1=Optical Flow
            "Vis/CorType": "0",
            # kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
            "Vis/CorNNType": "1",
            # 0=laser scan (was 1=depth camera): 360 deg / 12 m lidar gives thin,
            # sharp walls; the camera saw ~86 deg / 5 m. 2 = both, once the
            # camera timestamps are verified against odometry.
            "Grid/Sensor": "0",
            "Grid/DepthDecimation": "1",
            "Grid/RangeMin": "0.0",
            "Grid/RangeMax": "8.0",
            "Grid/MinClusterSize": "10",
            "Grid/MaxGroundAngle": "45",
            "Grid/NormalK": "20",
            "Grid/ClusterRadius": "0.2",
            "Grid/CellSize": "0.1",
            "Grid/FlatObstacleDetected": "false",
            "Grid/RayTracing": "true",
            # 2D grid only: the OctoMap path (Grid/3D true) aborts rtabmap with
            # "OcTreeDataNode ... Assertion `children == NULL' failed".
            "Grid/3D": "false",
            "Grid/MapFrameProjection": "true",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "100.0",
            "GridGlobal/Eroded": "true",
            "GridGlobal/FloodFillDepth": "16",
        }
    ]

    remappings = [
        ("rgb/image", "camera/color/image_raw"),
        ("rgb/camera_info", "camera/color/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", imu_topic),
        ("scan", scan_topic),
        # Odometry comes from the EKF's odom -> base_link TF (odom_frame_id
        # above) instead of running rgbd_odometry; that keeps the camera out
        # of the driving loop and saves a node's worth of CPU on the Jetson.
        # Keep RTAB-Map's occupancy grid on Nav2's canonical map topic.
        ("map", map_topic),
    ]

    return LaunchDescription(
        [
            use_sim_time_cmd,
            launch_rtabmapviz_cmd,
            delete_db_cmd,
            database_path_cmd,
            subscribe_scan_cmd,
            scan_topic_cmd,
            imu_topic_cmd,
            base_frame_cmd,
            map_frame_cmd,
            odom_frame_cmd,
            map_topic_cmd,
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="log",
                parameters=parameters,
                remappings=remappings,
                arguments=[
                    PythonExpression(
                        ["'-d' if '", delete_db, "'.lower() in ['true','1'] else ''"]
                    ),
                    "--ros-args",
                    "--log-level",
                    "Warn",
                ],
            ),
            Node(
                condition=IfCondition(launch_rtabmapviz),
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
        ]
    )
