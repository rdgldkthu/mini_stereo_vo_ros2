# mini_stereo_vo_ros2 — stereo_vo_gazebo.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mini_stereo_vo_ros2")
    tb4_gz_share = get_package_share_directory("turtlebot4_gz_bringup")

    params_file = os.path.join(pkg_share, "config", "stereo_vo_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "stereo_vo.rviz")

    # 1. TurtleBot4 + Gazebo Harmonic
    tb4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_gz_share, "launch", "turtlebot4_gz.launch.py")
        ),
        launch_arguments={"world": "maze"}.items(),
    )

    # 2. ros_gz_bridge: bridge stereo image topics from Gazebo → ROS2
    #
    # NOTE: After launching TurtleBot4 sim run:
    #   gz topic -l | grep -i camera
    # to verify Gazebo topic names and update the bridge config entries below.
    # Typical OAK-D topics in turtlebot4_gz_bringup are:
    #   /oakd/left/image_raw   and   /oakd/right/image_raw
    # We remap them to /left/image_raw and /right/image_raw for the VO node.
    bridge_left_img = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_left_img",
        arguments=[
            "/oakd/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        remappings=[("/oakd/left/image_raw", "/left/image_raw")],
    )

    bridge_right_img = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_right_img",
        arguments=[
            "/oakd/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        remappings=[("/oakd/right/image_raw", "/right/image_raw")],
    )

    bridge_left_info = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_left_info",
        arguments=[
            "/oakd/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[("/oakd/left/camera_info", "/left/camera_info")],
    )

    bridge_right_info = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_right_info",
        arguments=[
            "/oakd/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[("/oakd/right/camera_info", "/right/camera_info")],
    )

    # 3. StereoVONode
    stereo_vo_node = Node(
        package="mini_stereo_vo_ros2",
        executable="stereo_vo_node",
        name="stereo_vo_node",
        parameters=[params_file],
        output="screen",
    )

    # 4. RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        tb4_launch,
        bridge_left_img,
        bridge_right_img,
        bridge_left_info,
        bridge_right_info,
        stereo_vo_node,
        rviz_node,
    ])
