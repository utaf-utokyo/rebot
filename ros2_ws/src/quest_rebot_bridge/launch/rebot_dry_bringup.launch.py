"""M3 dry validation: robot_state_publisher + joint_state_publisher_gui only.

Uses the B601-DM *with gripper* URDF directly (rebotarm_bringup's own
bringup.launch.py model-selection logic only branches on 'rs' vs everything
else, and picks reBot-DevArm_fixend.urdf -- a fixed-end, no-gripper model --
for 'dm', which is NOT the robot we have; that appears to be an upstream gap).

Does NOT launch rebotarmcontroller, so no serial/CAN connection is opened and
no motor command path exists at all in this process.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    urdf_path = (
        get_package_share_directory("rebotarm_bringup")
        + "/description/urdf/reBot_B601_DM_with_gripper.urdf"
    )
    robot_description = ParameterValue(
        Command(["cat ", urdf_path]), value_type=str
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )
