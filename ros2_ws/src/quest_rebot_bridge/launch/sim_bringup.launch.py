"""M5 dry-run: fake_ik_driver + robot_state_publisher, fully simulated.

No CAN/serial is opened anywhere in this launch file. fake_ik_driver serves
the same move_to_pose_ik service contract as the real rebotarmcontroller
(same URDF, same IK solver) under the /rebotarm_sim namespace, so
bridge_node can be pointed at it (arm_namespace:=rebotarm_sim) to rehearse
the full Quest -> clutch -> safety_limits -> IK path before ever touching
the real /rebotarm namespace.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arm_namespace = LaunchConfiguration("arm_namespace")

    urdf_path = (
        get_package_share_directory("rebotarm_bringup")
        + "/description/urdf/reBot_B601_DM_with_gripper.urdf"
    )
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm_sim"),
            Node(
                package="quest_rebot_bridge",
                executable="fake_ik_driver",
                name="fake_ik_driver",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="sim_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
        ]
    )
