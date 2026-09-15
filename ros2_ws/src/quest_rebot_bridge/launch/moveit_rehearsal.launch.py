"""M5 dry-run, take 2: rebotarm_moveit_config's MoveIt demo (RViz + ros2_control
mock hardware) instead of fake_ik_driver's headless stand-in.

No CAN/serial is opened anywhere in this launch file -- rebotarm_moveit_config
demo.launch.py uses mock_components/GenericSystem, and moveit_ik_bridge only
solves IK and publishes a JointTrajectory to that mock hardware.

Run this, then separately run bridge.launch.py (bridge_node defaults to
arm_namespace:=rebotarm, matching moveit_ik_bridge's default service name, so
no extra args are needed) and controller_raw_publisher (in the isaacteleop
venv) to rehearse the full Quest -> clutch -> safety_limits -> IK path with
the arm visible in RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    arm_namespace = LaunchConfiguration("arm_namespace")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("rebotarm_moveit_config")
            + "/launch/demo.launch.py"
        ),
        launch_arguments={"model": "dm", "use_rviz": "true"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            moveit_demo,
            Node(
                package="quest_rebot_bridge",
                executable="moveit_ik_bridge",
                name="moveit_ik_bridge",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace}],
            ),
        ]
    )
