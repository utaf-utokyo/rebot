"""M4/M5: robot_state_publisher wired to the REAL rebotarmcontroller joint_states.

This does not open any CAN/serial connection and does not send any command --
it only subscribes to /rebotarm/joint_states (already being published by the
real rebotarmcontroller driver) and republishes TF, so that bridge_node's
base_link -> gripper_link lookup reflects the arm's true current pose instead
of falling back to the home_position placeholder.

Run this alongside (not instead of) rebotarm_bringup's driver.launch.py.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    urdf_path = (
        get_package_share_directory("rebotarm_bringup")
        + "/description/urdf/reBot_B601_DM_with_gripper.urdf"
    )
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/rebotarm/joint_states"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", joint_states_topic)],
            ),
        ]
    )
