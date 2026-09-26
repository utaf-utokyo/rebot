"""Milestone 2 launch: static calibration TF + quest_rebot_bridge bridge_node.

Does NOT launch controller_raw_publisher (needs the isaacteleop venv, a
different Python interpreter -- run it separately, see docs/architecture.md)
and does NOT launch anything that talks to the real reBot arm.

The base_link -> quest_world static transform's rotation defaults below are
the values measured on 2026-09-16 via quest_world_calibrate.py's rewritten
push-based procedure (a 30.6cm horizontal push toward the robot; see that
file's module docstring for why it measures a push direction rather than a
held orientation -- an earlier orientation-hold version of that tool produced
a rotation that never remapped OpenXR's RUB axis order onto base_link's FLU
convention, verified wrong from a [motion]-tagged bridge_node log where a
pure vertical hand lift produced a mostly-sideways-and-down commanded
target). This new value was verified separately: applying its rotation to
quest_world's Y (up) axis lands exactly on base_link's Z (up) axis with zero
leakage into X/Y. calib_x/y/z stay 0.0 -- translation never affects teleop
direction/scale, see quest_world_calibrate.py. Re-run quest_world_calibrate
and override calib_qx/qy/qz/qw as launch arguments only if the robot base
physically moves or the Quest's Guardian/room origin gets redefined.
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
            DeclareLaunchArgument("calib_qx", default_value="0.558549"),
            DeclareLaunchArgument("calib_qy", default_value="-0.433616"),
            DeclareLaunchArgument("calib_qz", default_value="-0.433616"),
            DeclareLaunchArgument("calib_qw", default_value="0.558549"),
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
