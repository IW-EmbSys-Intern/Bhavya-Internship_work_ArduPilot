"""
Combined launch file for:
  - MAVROS (connected to ArduPilot SITL over UDP)
  - Gazebo -> ROS 2 camera_info bridge
  - Gazebo -> ROS 2 image bridge (remapped to /camera/image_raw)

Usage:
    ros2 launch sim_bringup.launch.py

Optional: override the FCU URL from the command line, e.g.
    ros2 launch sim_bringup.launch.py fcu_url:=udp://:14550@127.0.0.1:14550
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://:14550@",
        description="FCU connection URL for MAVROS",
    )

    # Include mavros' own apm.launch (works whether it's XML or Python under the hood)
    mavros_launch_path = os.path.join(
        get_package_share_directory("mavros"), "launch", "apm.launch"
    )
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[
            "/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/"
            "camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="image_bridge",
        arguments=[
            "/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/"
            "image@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        remappings=[
            (
                "/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/image",
                "/camera/image_raw",
            )
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            fcu_url_arg,
            mavros_launch,
            camera_info_bridge,
            image_bridge,
        ]
    )
