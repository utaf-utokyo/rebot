"""quest_rebot_bridge ROS 2 node -- Milestone 4/5 scope.

Subscribes to xr_teleop/controller_data (published by
controller_raw_publisher), applies a squeeze-to-engage clutch and
relative-pose scaling/limits, publishes a target end-effector pose, and
calls rebotarmcontroller's move_to_pose_ik service with that same target.

move_to_pose_ik is non-blocking on the rebotarmcontroller side (it just
updates the internal target that an already-running 500Hz control loop
tracks), so it is safe to call every frame. If IK fails, rebotarmcontroller
itself leaves its internal target unchanged (the arm holds its last good
pose) -- this node does not need to replicate that logic, only log it.

Frames:
  quest_world -- raw OpenXR/CloudXR controller tracking origin, as decoded
                 from controller_data (untransformed).
  base_link   -- reBot Arm base frame. A static transform
                 base_link -> quest_world (physical calibration between the
                 Quest tracking origin and the robot base) must be published
                 separately -- see launch/bridge.launch.py. Also folds in
                 the OpenXR RUB -> robot-frame axis convention change, since
                 that's just part of the same fixed SE(3) transform.
  gripper_link -- reBot Arm B601-DM+gripper end-effector frame (confirmed via
                 M3: robot_state_publisher + joint_state_publisher_gui on the
                 real reBot_B601_DM_with_gripper.urdf, no hardware attached).
                 Once rebotarmcontroller + robot_state_publisher are running,
                 this is looked up via TF; until then, the `home_position` /
                 `home_orientation` parameters are used as a placeholder
                 clutch-engage reference so this node is testable standalone.
"""

from __future__ import annotations

import rclpy
import tf2_geometry_msgs  # noqa: F401 -- registers PoseStamped for Buffer.transform()
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ByteMultiArray
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rebotarm_msgs.srv import MoveToPoseIK

from quest_rebot_bridge.clutch import Clutch, Pose
from quest_rebot_bridge.controller_data_codec import controller_sample, decode_controller_data
from quest_rebot_bridge.safety_limits import SafetyLimits, SafetyLimitsConfig

_STALE_CHECK_PERIOD_S = 0.1


class QuestRebotBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("quest_rebot_bridge_node")

        self._arm_side = self._declare_str("arm_side", "right")
        controller_data_topic = self._declare_str(
            "controller_data_topic", "/xr_teleop/controller_data"
        )
        target_pose_topic = self._declare_str(
            "target_pose_topic", f"/{self._arm_side}_arm/target_pose"
        )
        self._base_frame = self._declare_str("base_frame", "base_link")
        self._ee_frame = self._declare_str("ee_frame", "gripper_link")
        self._quest_world_frame = self._declare_str("quest_world_frame", "quest_world")

        self._squeeze_engage = self._declare_float("squeeze_engage_threshold", 0.5)
        self._squeeze_release = self._declare_float("squeeze_release_threshold", 0.4)
        self._translation_scale = self._declare_float("translation_scale", 1.0)
        self._rotation_scale = self._declare_float("rotation_scale", 1.0)
        self._stale_timeout_s = self._declare_float("stale_input_timeout_s", 0.5)

        home_position = self._declare_double_array("home_position", [0.3, 0.0, 0.3])
        home_orientation = self._declare_double_array(
            "home_orientation", [0.0, 0.0, 0.0, 1.0]
        )
        self._home_pose = Pose(
            position=tuple(home_position), orientation=tuple(home_orientation)
        )

        workspace_min = self._declare_double_array("workspace_min", [-0.4, -0.4, 0.0])
        workspace_max = self._declare_double_array("workspace_max", [0.4, 0.4, 0.6])
        self._safety_limits = SafetyLimits(
            SafetyLimitsConfig(
                max_translation_step_m=self._declare_float("max_translation_step_m", 0.01),
                max_rotation_step_rad=self._declare_float("max_rotation_step_rad", 0.05),
                max_linear_velocity_mps=self._declare_float("max_linear_velocity_mps", 0.15),
                max_angular_velocity_radps=self._declare_float(
                    "max_angular_velocity_radps", 1.0
                ),
                workspace_min=tuple(workspace_min),
                workspace_max=tuple(workspace_max),
            )
        )
        self.get_logger().warn(
            "home_position/workspace_min/workspace_max are placeholder defaults "
            "-- tune them to the real B601-DM reachable workspace before "
            "Milestone 4/5, and stop relying on home_position once "
            f"{self._base_frame} -> {self._ee_frame} TF is available from M3."
        )

        self._clutch = Clutch()
        self._last_target: Pose | None = None
        self._last_target_time: Time | None = None
        self._last_msg_time: Time | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._target_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.create_subscription(
            ByteMultiArray, controller_data_topic, self._on_controller_data, 10
        )
        self.create_timer(_STALE_CHECK_PERIOD_S, self._check_stale)
        arm_namespace = self._declare_str("arm_namespace", "rebotarm")
        self._move_ik_client = self.create_client(
            MoveToPoseIK, f"/{arm_namespace}/move_to_pose_ik"
        )
        self.get_logger().info(
            f"quest_rebot_bridge_node up: side={self._arm_side} "
            f"controller_data={controller_data_topic} target_pose={target_pose_topic} "
            "(Milestone 2 scope: publish-only, no hardware commands)"
        )

    # -- parameter helpers ----------------------------------------------

    def _declare_str(self, name: str, default: str) -> str:
        self.declare_parameter(name, default)
        return str(self.get_parameter(name).value)

    def _declare_float(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        return float(self.get_parameter(name).value)

    def _declare_double_array(self, name: str, default: list) -> list:
        self.declare_parameter(name, [float(v) for v in default])
        return [float(v) for v in self.get_parameter(name).value]

    # -- controller_data handling -----------------------------------------

    def _on_controller_data(self, msg: ByteMultiArray) -> None:
        now = self.get_clock().now()
        try:
            data = decode_controller_data(msg)
            sample = controller_sample(data, self._arm_side)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f"failed to decode controller_data: {exc}")
            return

        self._last_msg_time = now

        if not sample.is_active:
            self._force_release("controller inactive")
            return

        controller_pose_quest = Pose(
            position=sample.grip_position, orientation=sample.grip_orientation
        )
        controller_pose_base = self._transform_to_base(controller_pose_quest)
        if controller_pose_base is None:
            self._force_release(
                f"{self._quest_world_frame} -> {self._base_frame} transform unavailable"
            )
            return

        if sample.squeeze >= self._squeeze_engage and not self._clutch.engaged:
            ee_pose = self._lookup_ee_pose()
            self._clutch.engage(controller_pose_base, ee_pose)
            self._last_target = ee_pose
            self._last_target_time = now
            self.get_logger().info("clutch engaged")
        elif sample.squeeze < self._squeeze_release and self._clutch.engaged:
            self.get_logger().info("clutch released -- arm holds")
            self._clutch.release()

        if not self._clutch.engaged or self._last_target is None:
            return

        proposed = self._clutch.compute_target(
            controller_pose_base, self._translation_scale, self._rotation_scale
        )
        dt_s = 0.0
        if self._last_target_time is not None:
            dt_s = (now - self._last_target_time).nanoseconds * 1e-9
        target = self._safety_limits.clamp(self._last_target, proposed, dt_s)
        self._last_target_time = now
        self._publish_target(target)
        self._call_move_to_pose_ik(target)

    def _call_move_to_pose_ik(self, pose: Pose) -> None:
        if not self._move_ik_client.service_is_ready():
            self.get_logger().warn(
                "move_to_pose_ik service not available -- target not sent to hardware",
                throttle_duration_sec=2.0,
            )
            return
        req = MoveToPoseIK.Request()
        req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z = (
            pose.position
        )
        qx, qy, qz, qw = pose.orientation
        req.target_pose.orientation.x = qx
        req.target_pose.orientation.y = qy
        req.target_pose.orientation.z = qz
        req.target_pose.orientation.w = qw
        future = self._move_ik_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_move_ik_response(f, pose)
        )

    def _on_move_ik_response(self, future, pose: Pose) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"move_to_pose_ik call failed: {exc}", throttle_duration_sec=2.0)
            return
        if not response.success:
            self.get_logger().warn(
                f"move_to_pose_ik rejected: {response.message}", throttle_duration_sec=1.0
            )
            return
        # Only commit the new reference once the arm has actually accepted
        # it -- otherwise a run of IK failures would let _last_target drift
        # further and further from where the arm truly is, since each
        # subsequent frame's delta is computed against it regardless.
        self._last_target = pose

    def _check_stale(self) -> None:
        if self._last_msg_time is None or not self._clutch.engaged:
            return
        elapsed_s = (self.get_clock().now() - self._last_msg_time).nanoseconds * 1e-9
        if elapsed_s > self._stale_timeout_s:
            self._force_release(f"stale controller_data ({elapsed_s:.2f}s)")

    def _force_release(self, reason: str) -> None:
        if self._clutch.engaged:
            self.get_logger().warn(f"clutch force-released: {reason}")
        self._clutch.release()

    # -- tf2 helpers --------------------------------------------------------

    def _transform_to_base(self, pose: Pose) -> Pose | None:
        stamped = PoseStamped()
        stamped.header.frame_id = self._quest_world_frame
        stamped.pose.position.x, stamped.pose.position.y, stamped.pose.position.z = (
            pose.position
        )
        qx, qy, qz, qw = pose.orientation
        stamped.pose.orientation.x = qx
        stamped.pose.orientation.y = qy
        stamped.pose.orientation.z = qz
        stamped.pose.orientation.w = qw
        try:
            transformed = self._tf_buffer.transform(stamped, self._base_frame)
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as exc:
            self.get_logger().warn(f"tf2 transform to {self._base_frame} failed: {exc}")
            return None
        return _pose_from_msg(transformed.pose)

    def _lookup_ee_pose(self) -> Pose:
        try:
            t = self._tf_buffer.lookup_transform(self._base_frame, self._ee_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as exc:
            self.get_logger().warn(
                f"tf2 lookup {self._base_frame} -> {self._ee_frame} failed ({exc}); "
                "using home_position/home_orientation placeholder"
            )
            return self._home_pose
        return Pose(
            position=(
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ),
            orientation=(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ),
        )

    # -- output ---------------------------------------------------------

    def _publish_target(self, pose: Pose) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose.position
        qx, qy, qz, qw = pose.orientation
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._target_pub.publish(msg)


def _pose_from_msg(pose) -> Pose:
    return Pose(
        position=(pose.position.x, pose.position.y, pose.position.z),
        orientation=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuestRebotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
