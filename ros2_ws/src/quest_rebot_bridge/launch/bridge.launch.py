"""Milestone 2 launch: static calibration TF + quest_rebot_bridge bridge_node.

Does NOT launch controller_raw_publisher (needs the isaacteleop venv, a
different Python interpreter -- run it separately, see docs/architecture.md)
and does NOT launch anything that talks to the real reBot arm.

The base_link -> quest_world static transform is a PLACEHOLDER identity by
default. It must be replaced with the real measured calibration between the
Quest tracking origin and the robot base before Milestone 4/5 -- this is the
single transform that also carries the OpenXR-RUB -> robot-frame axis
convention change (see docs/architecture.md).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arm_side = LaunchConfiguration("arm_side")
    base_frame = LaunchConfiguration("base_frame")
    quest_world_frame = LaunchConfiguration("quest_world_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_side", default_value="right"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("quest_world_frame", default_value="quest_world"),
            DeclareLaunchArgument("calib_x", default_value="0.0"),
            DeclareLaunchArgument("calib_y", default_value="0.0"),
            DeclareLaunchArgument("calib_z", default_value="0.0"),
            DeclareLaunchArgument("calib_qx", default_value="0.0"),
            DeclareLaunchArgument("calib_qy", default_value="0.0"),
            DeclareLaunchArgument("calib_qz", default_value="0.0"),
            DeclareLaunchArgument("calib_qw", default_value="1.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="quest_world_calibration",
                arguments=[
                    "--x", LaunchConfiguration("calib_x"),
                    "--y", LaunchConfiguration("calib_y"),
                    "--z", LaunchConfiguration("calib_z"),
                    "--qx", LaunchConfiguration("calib_qx"),
                    "--qy", LaunchConfiguration("calib_qy"),
                    "--qz", LaunchConfiguration("calib_qz"),
                    "--qw", LaunchConfiguration("calib_qw"),
                    "--frame-id", base_frame,
                    "--child-frame-id", quest_world_frame,
                ],
            ),
            Node(
                package="quest_rebot_bridge",
                executable="bridge_node",
                name="quest_rebot_bridge_node",
                output="screen",
                parameters=[
                    {
                        "arm_side": arm_side,
                        "base_frame": base_frame,
                        "quest_world_frame": quest_world_frame,
                    }
                ],
            ),
        ]
    )
