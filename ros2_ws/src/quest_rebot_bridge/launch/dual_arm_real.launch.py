"""M6 real hardware: Quest 3 left/right controllers -> two real B601-DM arms.

Same per-side TF / bridge_node layout as dual_arm_rehearsal.launch.py (see its
docstring for the frame naming and quest_world design), but each side's
move_to_pose_ik backend is a real rebotarmcontroller on its own USB-CAN
adapter instead of fake_ik_driver:

  left  -> /dev/rebot_left  (tools/udev/99-rebot-arms.rules, USB port NVDA8000:01)
  right -> /dev/rebot_right (USB port NVDA8000:02)

Both arms use the same motor IDs (CAN 0x01-0x07 / Master 0x11-0x17); they
are on separate CAN buses, so that doesn't collide.

SAFETY: each reBotArmController enables its motors and starts holding the
current pose the instant it connects, each bridge_node opens its gripper
~1s after start, and (home_on_startup, default true) each arm then moves on
its own from the folded pose to the home pose -- no A/X press needed.
On shutdown (Ctrl-C) each controller runs safe_home (back to
the folded zero pose) and disables.

The real controller solves IK for ReBot_Arm_DM.urdf's end_link, which is
gripper_link rotated 180deg about local X, so bridge_node gets
ik_orientation_offset_xyzw=[1,0,0,0] here (see bridge_node.py).

fake_hardware:=true swaps each real controller for fake_ik_driver running
the SAME model the real controller uses (ReBot_Arm_DM.urdf / end_link,
started at the folded zero pose), so this exact launch -- including the
orientation offset and the first A/X home reset from the folded pose -- can
be rehearsed without touching hardware.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_SIDES = ("left", "right")
# Rx(pi): gripper_link -> end_link (see module docstring).
_END_LINK_OFFSET_XYZW = [1.0, 0.0, 0.0, 0.0]


def generate_launch_description() -> LaunchDescription:
    urdf_path = (
        get_package_share_directory("rebotarm_bringup")
        + "/description/urdf/reBot_B601_DM_with_gripper.urdf"
    )
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    calib = {
        side: [LaunchConfiguration(f"{side}_calib_q{a}") for a in "xyzw"] for side in _SIDES
    }
    fake_hardware = LaunchConfiguration("fake_hardware")
    world_y_offsets = {
        "left": LaunchConfiguration("left_world_y_offset_m"),
        "right": LaunchConfiguration("right_world_y_offset_m"),
    }
    # The SDK only treats a channel as a USB-serial adapter when it starts
    # with "/dev/tty" (reBotArm_control_py actuator/rebotarm.py); anything
    # else is opened as a SocketCAN interface name. So the udev symlinks
    # (/dev/rebot_left etc.) must be resolved to their /dev/ttyACM* target
    # here, at launch time, rather than passed through as-is.
    channels = {
        side: PythonExpression(
            ["__import__('os').path.realpath('", LaunchConfiguration(f"{side}_channel"), "')"]
        )
        for side in _SIDES
    }

    declares = [
        # quest_world -> {side}/base_link rotation, measured per side on the
        # real rig with quest_world_calibrate arm_side:=left/right (2026-09-27),
        # pushing toward each arm's own forward (+X) direction. The two
        # differ by ~27deg. Redo both if the Quest's tracking origin is reset
        # (Guardian/recenter) -- the 2026-09-16 value used by
        # dual_arm_rehearsal was ~65deg off from today's by then.
        *[
            DeclareLaunchArgument(f"{side}_calib_q{a}", default_value=v)
            for side, values in (
                ("left", ("0.385817", "-0.592575", "-0.592575", "0.385817")),
                ("right", ("0.237646", "-0.665976", "-0.665976", "0.237646")),
            )
            for a, v in zip("xyzw", values)
        ],
        DeclareLaunchArgument("left_world_y_offset_m", default_value="0.4"),
        DeclareLaunchArgument("right_world_y_offset_m", default_value="-0.4"),
        DeclareLaunchArgument("left_channel", default_value="/dev/rebot_left"),
        DeclareLaunchArgument("right_channel", default_value="/dev/rebot_right"),
        DeclareLaunchArgument("translation_scale", default_value="1.0"),
        DeclareLaunchArgument("fake_hardware", default_value="false"),
        # Move both arms from the folded pose to home as soon as each is up,
        # so the operator doesn't have to press A/X before the first squeeze.
        DeclareLaunchArgument("home_on_startup", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                get_package_share_directory("quest_rebot_bridge")
                + "/rviz/dual_arm.rviz",
            ],
        ),
    ]

    nodes = []
    for side in _SIDES:
        arm_namespace = f"rebotarm_{side}"
        frame_prefix = f"{side}/"
        base_link = f"{side}/base_link"
        quest_world = f"{side}/quest_world"

        nodes += [
            Node(
                condition=UnlessCondition(fake_hardware),
                package="rebotarmcontroller",
                executable="reBotArmController",
                name=f"reBotArmController_{side}",
                output="screen",
                parameters=[{"channel": channels[side], "arm_namespace": arm_namespace}],
            ),
            Node(
                condition=IfCondition(fake_hardware),
                package="quest_rebot_bridge",
                executable="fake_ik_driver",
                name=f"fake_ik_driver_{side}",
                output="screen",
                # No urdf_path/ee_frame/initial_joints: its defaults are the
                # real controller's own model (ReBot_Arm_DM.urdf, end_link)
                # at the folded zero pose, which is the point of this mode.
                parameters=[{"arm_namespace": arm_namespace}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name=f"robot_state_publisher_{side}",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "frame_prefix": frame_prefix}
                ],
                remappings=[
                    ("joint_states", f"/{arm_namespace}/joint_states"),
                    ("robot_description", f"/{side}/robot_description"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"world_to_{side}_base_link",
                arguments=[
                    "--y", world_y_offsets[side],
                    "--frame-id", "world",
                    "--child-frame-id", base_link,
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{side}_quest_world_calibration",
                arguments=[
                    "--qx", calib[side][0],
                    "--qy", calib[side][1],
                    "--qz", calib[side][2],
                    "--qw", calib[side][3],
                    "--frame-id", base_link,
                    "--child-frame-id", quest_world,
                ],
            ),
            Node(
                package="quest_rebot_bridge",
                executable="bridge_node",
                name=f"quest_rebot_bridge_node_{side}",
                output="screen",
                parameters=[
                    {
                        "arm_side": side,
                        "arm_namespace": arm_namespace,
                        "base_frame": base_link,
                        "ee_frame": frame_prefix + "gripper_link",
                        "ik_orientation_offset_xyzw": _END_LINK_OFFSET_XYZW,
                        "home_on_startup": ParameterValue(
                            LaunchConfiguration("home_on_startup"), value_type=bool
                        ),
                        "translation_scale": ParameterValue(
                            LaunchConfiguration("translation_scale"), value_type=float
                        ),
                        "home_orientation": [-0.7071067811865475, 0.0, 0.7071067811865475, 0.0],
                        "orientation_source": "aim",
                        "quest_world_frame": quest_world,
                    }
                ],
            ),
        ]

    return LaunchDescription(declares + nodes)
