# mini_stereo_vo_ros2 — stereo_vo_gazebo.launch.py
#
# Single command brings up everything:
#   1. Gazebo Harmonic (maze world) via turtlebot4_gz_bringup/launch/sim.launch.py
#   2. robot_state_publisher with a custom URDF that adds stereo cameras to
#      the OAK-D left/right frames (turtlebot4_stereo.urdf.xacro)
#   3. Spawn TurtleBot4 from robot_description topic
#   4. ros_gz_bridge: stereo images + camera_info → /left/ and /right/ topics
#   5. stereo_vo_node
#   6. rviz2
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORLD = "warehouse"
ROBOT = "turtlebot4"


def generate_launch_description():
    pkg_share = get_package_share_directory("mini_stereo_vo_ros2")
    pkg_tb4_gz = get_package_share_directory("turtlebot4_gz_bringup")

    params_file = os.path.join(pkg_share, "config", "stereo_vo_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "stereo_vo.rviz")
    stereo_xacro = os.path.join(pkg_share, "urdf", "turtlebot4_stereo.urdf.xacro")

    # ── 1. Gazebo Harmonic + clock bridge ─────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz, "launch", "sim.launch.py")
        ),
        launch_arguments={"world": WORLD, "model": "standard"}.items(),
    )

    # ── 2. robot_state_publisher with stereo-extended URDF ────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", stereo_xacro, " gazebo:=ignition namespace:="]),
                    value_type=str,
                )
            },
        ],
    )

    # ── 3. Spawn robot (delayed 30 s to let Gazebo finish loading the world) ──
    spawn_robot = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", ROBOT,
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.01",
                    "-topic", "robot_description",
                ],
                output="screen",
            )
        ],
    )

    # ── 4. Stereo camera bridges ───────────────────────────────────────────────
    # Gazebo topic pattern for sensor data:
    #   /world/<world>/model/<robot>/link/<link>/sensor/<sensor>/<data>
    gz_left_img   = (
        f"/world/{WORLD}/model/{ROBOT}"
        f"/link/oakd_left_camera_frame/sensor/left_camera/image"
    )
    gz_right_img  = (
        f"/world/{WORLD}/model/{ROBOT}"
        f"/link/oakd_right_camera_frame/sensor/right_camera/image"
    )
    gz_left_info  = (
        f"/world/{WORLD}/model/{ROBOT}"
        f"/link/oakd_left_camera_frame/sensor/left_camera/camera_info"
    )
    gz_right_info = (
        f"/world/{WORLD}/model/{ROBOT}"
        f"/link/oakd_right_camera_frame/sensor/right_camera/camera_info"
    )

    bridge_stereo = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_stereo_cameras",
        arguments=[
            gz_left_img   + "@sensor_msgs/msg/Image[gz.msgs.Image",
            gz_right_img  + "@sensor_msgs/msg/Image[gz.msgs.Image",
            gz_left_info  + "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            gz_right_info + "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            (gz_left_img,   "/left/image_raw"),
            (gz_right_img,  "/right/image_raw"),
            (gz_left_info,  "/left/camera_info"),
            (gz_right_info, "/right/camera_info"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── 5. cmd_vel bridge: ROS /cmd_vel (Twist) → Gazebo /cmd_vel (Twist) ────
    #    The Gazebo-native DiffDrive plugin in our custom URDF subscribes to
    #    Gazebo /cmd_vel directly, bypassing the ros2_control chain entirely.
    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_cmd_vel",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
        output="screen",
    )

    # ── 7. StereoVONode ────────────────────────────────────────────────────────
    stereo_vo_node = Node(
        package="mini_stereo_vo_ros2",
        executable="stereo_vo_node",
        name="stereo_vo_node",
        parameters=[params_file, {"use_sim_time": True}],
        output="screen",
    )

    # ── 8. RViz2 ──────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge_stereo,
        bridge_cmd_vel,
        stereo_vo_node,
        rviz_node,
    ])
