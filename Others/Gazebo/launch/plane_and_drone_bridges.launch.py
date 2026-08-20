"""
Combined launch file for dual-vehicle simulation:
  - MAVROS instance for the plane  (namespace: /plane,  fcu instance 0 -> udp 14550)
  - MAVROS instance for the copter (namespace: /copter, fcu instance 1 -> udp 14560)
  - Gazebo -> ROS 2 camera_info bridge
  - Gazebo -> ROS 2 image bridge (remapped to /camera/image_raw)

Usage:
    ros2 launch sim_bringup_dual.launch.py

Override either FCU URL if your ports differ, e.g.:
    ros2 launch sim_bringup_dual.launch.py plane_fcu_url:=udp://:14550@ copter_fcu_url:=udp://:14560@

Each mission node you run afterwards must be told which vehicle to talk to via
ROS2 namespace remapping, e.g.:
    ros2 run <pkg> mavros_mission_node.py --ros-args -r __ns:=/copter -p target_lat:=... ...
    ros2 run <pkg> plane_mission_node.py  --ros-args -r __ns:=/plane  ...
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    plane_fcu_url_arg = DeclareLaunchArgument(
        "plane_fcu_url",
        default_value="udp://:14550@",
        description="FCU connection URL for the plane's MAVROS instance (SITL instance 0)",
    )
    copter_fcu_url_arg = DeclareLaunchArgument(
        "copter_fcu_url",
        default_value="udp://:14560@",
        description="FCU connection URL for the copter's MAVROS instance (SITL instance 1)",
    )

    mavros_launch_path = os.path.join(
        get_package_share_directory("mavros"), "launch", "apm.launch"
    )

    plane_mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            "fcu_url": LaunchConfiguration("plane_fcu_url"),
            "namespace": "plane/mavros",
            "tgt_system": "1",
        }.items(),
    )

    copter_mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            "fcu_url": LaunchConfiguration("copter_fcu_url"),
            "namespace": "copter/mavros",
            "tgt_system": "1",
        }.items(),
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
    
    copter_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="copter_camera_info_bridge",
        arguments=[
            "/world/runway/model/iris_I1/model/gimbal/link/pitch_link/"
            "sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
    )


    copter_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="copter_image_bridge",
        arguments=[
            "/world/runway/model/iris_I1/model/gimbal/link/pitch_link/"
            "sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        remappings=[
            (
                "/world/runway/model/iris_I1/model/gimbal/link/pitch_link/"
                "sensor/camera/image",
                "/copter/camera/image_raw",
            )
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            plane_fcu_url_arg,
            copter_fcu_url_arg,
            plane_mavros,
            copter_mavros,
            camera_info_bridge,
            image_bridge,
            copter_camera_info_bridge,
            copter_image_bridge,
        ]
    )
