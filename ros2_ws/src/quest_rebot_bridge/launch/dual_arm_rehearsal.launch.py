"""M6 dry-run: two independent fake_ik_driver + robot_state_publisher pairs
("left"/"right"), each driven by its own bridge_node reading the matching
Quest controller side, visualized side by side in one RViz window.

Position uses a relative clutch; orientation is fixed straight down
regardless of controller rotation (plus thumbstick-driven roll) -- see
bridge_node.py's module docstring. Both IK and TF use the same gripper URDF.
The simulation starts at the reachable home pose, not the folded joint
limits.

Design notes (why this differs from moveit_rehearsal.launch.py):

  - Uses fake_ik_driver + robot_state_publisher (like sim_bringup.launch.py),
    not rebotarm_moveit_config's MoveIt demo. Namespacing that demo's
    move_group/controller_manager for two robots would require reworking the
    vendored rebotarm_moveit_config package itself; fake_ik_driver already
    just publishes JointState per-namespace, which two independent
    robot_state_publisher instances can each consume directly.

  - Each side gets its OWN quest_world frame ({side}/quest_world) as a direct
    TF child of {side}/base_link -- exactly the same parent/child direction
    as the single-arm bridge.launch.py's calibration transform, just
    duplicated per side, rather than one shared "quest_world" frame that
    both base_links would need to attach to. A single frame can only have one
    TF parent, so making quest_world shared would require it to be the
    *parent* of both base_links instead -- which would mean publishing the
    *inverse* of the measured calib_q* rotation (a real, easy-to-get-backwards
    footgun this project has already been bitten by once with the ee_frame
    mismatch and the RUB/FLU calibration bug -- see README). Two independent
    {side}/quest_world frames sidestep that risk entirely: both controllers'
    raw grip_position/orientation numbers already live in the same physical
    Quest tracking origin, so the exact same measured calib_q* rotation is
    reused for both sides below (only valid if both arms are mounted at the
    same orientation -- see world_to_*_base_link below if that's ever not
    true).

  - Frame names use a "/" separator (e.g. "left/base_link"), not "left_base_link".
    robot_state_publisher's frame_prefix parameter does plain string
    concatenation with no separator of its own, but RViz2's RobotModel
    display's "TF Prefix" property *always* inserts its own "/" when looking
    up each link's transform (tf_prefix + "/" + link_name) -- confirmed via
    an RViz screenshot showing it querying "left_/base_link" (extra slash)
    when frame_prefix was "left_", which never matched the actually-published
    "left_base_link" and left both RobotModel displays permanently in Error
    status (TF/Grid worked fine; only the mesh display silently failed). Using
    "left/" as frame_prefix here and "left" as the RViz TF Prefix makes both
    sides agree on "left/base_link".

  - "world" is a third, purely cosmetic root frame that only exists so RViz
    can render both (otherwise-unrelated) {side}_base_link trees in one
    scene, side by side. Its left/right offset (world_y_offset_m) is an
    arbitrary visual spacing, not a physically measured distance -- no
    bridge_node math ever references "world", so getting this number wrong
    cannot affect teleop direction/scale (same reasoning as calib_x/y/z in
    quest_world_calibrate.py).
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_SIDES = ("left", "right")


def generate_launch_description() -> LaunchDescription:
    urdf_path = (
        get_package_share_directory("rebotarm_bringup")
        + "/description/urdf/reBot_B601_DM_with_gripper.urdf"
    )
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    calib_qx = LaunchConfiguration("calib_qx")
    calib_qy = LaunchConfiguration("calib_qy")
    calib_qz = LaunchConfiguration("calib_qz")
    calib_qw = LaunchConfiguration("calib_qw")
    world_y_offsets = {
        "left": LaunchConfiguration("left_world_y_offset_m"),
        "right": LaunchConfiguration("right_world_y_offset_m"),
    }

    declares = [
        # 2026-09-16 measured value (see README/bridge.launch.py); reused for
        # both sides -- only valid while both arms share the same mounting
        # orientation, see module docstring above.
        DeclareLaunchArgument("calib_qx", default_value="0.558549"),
        DeclareLaunchArgument("calib_qy", default_value="-0.433616"),
        DeclareLaunchArgument("calib_qz", default_value="-0.433616"),
        DeclareLaunchArgument("calib_qw", default_value="0.558549"),
        DeclareLaunchArgument("left_world_y_offset_m", default_value="0.4"),
        DeclareLaunchArgument("right_world_y_offset_m", default_value="-0.4"),
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
        arm_namespace = f"rebotarm_{side}_sim"
        frame_prefix = f"{side}/"
        base_link = f"{side}/base_link"
        quest_world = f"{side}/quest_world"

        nodes += [
            Node(
                package="quest_rebot_bridge",
                executable="fake_ik_driver",
                name=f"fake_ik_driver_{side}",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace, "urdf_path": urdf_path,
                             "ee_frame": "gripper_link",
                             # Solved via IK (2026-09-26) for bridge_node's new
                             # home_position=(0.3,0,0.10) + straight-down
                             # home_orientation below, so the sim boots already
                             # at home instead of needing an initial ~120deg
                             # reorientation on first engage.
                             "initial_joints": [0.0, -1.7120388, -1.3319939, 1.1906555, 0.0, 0.0]}],
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
                    "--qx", calib_qx,
                    "--qy", calib_qy,
                    "--qz", calib_qz,
                    "--qw", calib_qw,
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
                        # Matches bridge_node.py's home_orientation default exactly
                        # (straight down -- see _DOWN_POINTING_QUAT there) rather
                        # than the old horizontal-ish [-0.707107,0,0,0.707107]: IK
                        # probing (2026-09-26) found that mismatch forced a ~120deg
                        # reorientation on every clutch engage, which the sim's
                        # damped-least-squares IK couldn't always complete in
                        # time, leaving the arm stuck rejecting every further
                        # target ("IK did not converge") -- not a bug in the
                        # fixed-orientation teleop code itself, just two
                        # different "down-ish" defaults that didn't actually
                        # agree. Overriding it here is no longer necessary now
                        # that it matches the default, but kept explicit so a
                        # future default change doesn't silently reintroduce
                        # the mismatch for this launch file.
                        "home_orientation": [-0.7071067811865475, 0.0, 0.7071067811865475, 0.0],
                        # aim: only affects the raw controller-orientation values
                        # in the [motion] diagnostic log now (orientation_mode and
                        # controller_to_tool_xyzw, which used to make this actually
                        # drive the commanded EE orientation, were deleted as dead
                        # code once the fixed-orientation policy made them
                        # permanently unreachable -- see bridge_node.py).
                        "orientation_source": "aim",
                        "quest_world_frame": quest_world,
                    }
                ],
            ),
        ]

    return LaunchDescription(declares + nodes)
