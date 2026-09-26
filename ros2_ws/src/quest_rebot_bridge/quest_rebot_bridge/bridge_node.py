"""quest_rebot_bridge ROS 2 node -- Milestone 4/5 scope.

Subscribes to xr_teleop/controller_data (published by
controller_raw_publisher), applies a squeeze-to-engage clutch and
relative-pose scaling/limits, publishes a target end-effector pose, and
calls rebotarmcontroller's move_to_pose_ik service with that same target.

The controller's primary button (A on the right controller, X on the left)
releases the clutch and drives the arm back to home_position/home_orientation,
one safety_limits-clamped step at a time via the same _HOMING_STEP_PERIOD_S
timer used for stale-input checks -- so the return respects the same
translation/rotation/velocity limits as teleop, rather than jumping. Squeezing
again (clutch re-engage) cancels an in-progress home reset.

move_to_pose_ik is non-blocking on the rebotarmcontroller side (it just
updates the internal target that an already-running 500Hz control loop
tracks), so it is safe to call every frame. If IK fails, rebotarmcontroller
itself leaves its internal target unchanged (the arm holds its last good
pose) -- this node does not need to replicate that logic, only log it.

The index trigger drives the gripper via a binary open/close (not
proportional): unlike move_to_pose_ik, rebotarmcontroller's gripper/open and
gripper/close services block server-side until the target is reached or a
timeout (see hardware_manager.py's set_gripper_position), so this node calls
them once per threshold crossing (same hysteresis pattern as the squeeze
clutch), not every frame. "Open" targets 1/3 of the real B601-DM gripper's
full travel by default (gripper_open_position param) rather than fully open,
since the real gripper opens very wide at its actual full-open position.

Orientation is now fixed rather than tracked from the controller at all: the
target's forward/pointing direction is always straight down (base_link -Z,
see _DOWN_POINTING_QUAT), regardless of how the Quest controller is held.
Only position (3 DOF, from compute_target()) and roll about that fixed-down
axis (1 DOF -- confirmed 2026-09-17 via FK to be exactly and only joint6's
angle, independent of arm configuration) are teleoperated; roll comes from
the right thumbstick's X axis as a rate control: tilt right/left to spin the
gripper clockwise/counter-clockwise, released at any angle it holds
(persists across clutch engage/release). _pointing_pose() builds this by
discarding compute_target()'s orientation output entirely (both its forward
direction and its roll) and composing _DOWN_POINTING_QUAT with the
thumbstick-integrated roll offset. Clutch itself is now position-only; it no
longer has any orientation-tracking mode (see clutch.py) since nothing
consumes its orientation output. orientation_source (grip/aim, still
declared below) only affects the raw controller-orientation values that
appear in the [motion] diagnostic log, not anything commanded to the arm.

(Earlier milestones tracked the controller's own pointing direction here,
via Clutch's now-removed absolute/relative orientation modes and a
controller_to_tool_xyzw axis mapping -- see git history around 2026-09-17
for that design and the roll-leak bug it had -- before the policy changed to
a fixed downward orientation and that machinery was deleted as dead code
(2026-09-26).)

Frames:
  quest_world -- raw OpenXR/CloudXR controller tracking origin, as decoded
                 from controller_data (untransformed).
  base_link   -- reBot Arm base frame. A static transform
                 base_link -> quest_world (physical calibration between the
                 Quest tracking origin and the robot base) must be published
                 separately -- see launch/bridge.launch.py. Also folds in
                 the OpenXR RUB -> robot-frame axis convention change, since
                 that's just part of the same fixed SE(3) transform.
  ee_frame    -- must match the IK backend and the published TF model. The
                 dual-arm simulation uses {side}/gripper_link on the same URDF
                 for FK/IK and visualization. Missing TF prevents engagement.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 -- registers PoseStamped for Buffer.transform()
import tf_transformations as tft
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

from rebotarm_msgs.srv import GripperCommand, MoveToPoseIK

from quest_rebot_bridge.clutch import Clutch, Pose
from quest_rebot_bridge.controller_data_codec import controller_sample, decode_controller_data
from quest_rebot_bridge.node_diagnostics import warn_if_duplicate_node
from quest_rebot_bridge.safety_limits import SafetyLimits, SafetyLimitsConfig

_STALE_CHECK_PERIOD_S = 0.1
_HOMING_STEP_PERIOD_S = 0.05
_HOMING_POSITION_TOLERANCE_M = 0.005
_HOMING_ROTATION_TOLERANCE_RAD = 0.02
_PRIMARY_CLICK_THRESHOLD = 0.5
_HEARTBEAT_PERIOD_S = 1.0
_GRIPPER_INIT_DELAY_S = 1.0
# rebot_bringup/config/rebotarm_hardware.yaml (model=dm): gripper.position_limits
# open=-5.0 close=0.0 rad. The real B601-DM gripper opens very wide at that
# extreme, so "open" here targets 1/3 of that travel rather than the full
# open position.
_GRIPPER_FULL_OPEN_POSITION_RAD = -5.0
_GRIPPER_OPEN_FRACTION = 1.0 / 3.0
# joint6 is confirmed (via FK perturbation test, 2026-09-17) to control exactly
# and only the EE's rotation about its own local X axis -- zero effect on
# position or the other two rotation DOF, at any arm configuration. So a
# thumbstick-driven "roll" offset, applied as a further local-X rotation on
# top of the fixed straight-down orientation (see _pointing_pose), corresponds
# directly to joint6's angle without needing any dedicated joint-space control
# path.


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
        self._ee_frame = self._declare_str("ee_frame", "end_link")
        self._quest_world_frame = self._declare_str("quest_world_frame", "quest_world")

        self._squeeze_engage = self._declare_float("squeeze_engage_threshold", 0.5)
        self._squeeze_release = self._declare_float("squeeze_release_threshold", 0.4)
        self._trigger_close_threshold = self._declare_float("trigger_close_threshold", 0.5)
        self._trigger_open_threshold = self._declare_float("trigger_open_threshold", 0.4)
        self._gripper_open_position = self._declare_float(
            "gripper_open_position",
            _GRIPPER_FULL_OPEN_POSITION_RAD * _GRIPPER_OPEN_FRACTION,
        )
        self._translation_scale = self._declare_float("translation_scale", 1.0)
        self._stale_timeout_s = self._declare_float("stale_input_timeout_s", 0.5)

        # Thumbstick-driven roll (see joint6 note above). 60 deg/s at full
        # deflection: fast enough to feel responsive, but under
        # max_angular_velocity_radps's default ~57 deg/s ceiling so
        # safety_limits.clamp() doesn't visibly choke full-deflection input.
        self._roll_rate_rad_s = math.radians(self._declare_float("roll_rate_deg_s", 60.0))
        self._roll_deadzone = self._declare_float("roll_thumbstick_deadzone", 0.15)
        # Roll offset, in the EE's own local-X frame, composed with the fixed
        # straight-down orientation by _pointing_pose() -- independent of
        # home_orientation (only used while actively teleoperating, never
        # during the A/X home reset). Starts at 0: home_orientation and this
        # offset are deliberately independent, not two expressions of the
        # same "baseline" -- a nonzero default here would permanently bias
        # every live target regardless of home_orientation's own value
        # (caught historically, back when orientation still tracked the
        # controller via Clutch's now-removed absolute mode, by
        # tools/check_dual_arm_tracking.py failing with a ~90deg error on
        # every axis once this was set to 90deg). Persists across clutch
        # engage/release (per user request); only resets to 0 on node
        # restart.
        self._roll_offset_rad = 0.0
        self._last_roll_update_time: Time | None = None

        # home_position/home_orientation default to a pose where straight-down
        # pointing is actually reachable (verified 2026-09-26 via direct IK
        # probing of the B601-DM/gripper model: at (0.3, 0, z), IK for a
        # straight-down orientation only converges for z <~ 0.15-0.20; it
        # fails with a growing residual above that, up to z=0.6, regardless of
        # IK seed or random restarts -- a real kinematic limit, not a solver
        # quality issue). home_orientation now matches _DOWN_POINTING_QUAT
        # exactly so engaging the clutch at the home pose requires zero
        # reorientation, instead of the ~120deg jump the old mismatched
        # defaults (home pointing roughly horizontal) forced on every engage.
        home_position = self._declare_double_array("home_position", [0.3, 0.0, 0.10])
        home_orientation = self._declare_double_array(
            "home_orientation", list(_DOWN_POINTING_QUAT)
        )
        self._home_pose = Pose(
            position=tuple(home_position), orientation=tuple(home_orientation)
        )

        # workspace_max z lowered to match: above ~0.15-0.2, straight-down
        # orientation isn't reachable at all (see home_position note above),
        # so move_to_pose_ik would just reject targets up there anyway.
        workspace_min = self._declare_double_array("workspace_min", [-0.4, -0.4, 0.0])
        workspace_max = self._declare_double_array("workspace_max", [0.4, 0.4, 0.15])
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
            "home_position/workspace_min/workspace_max were chosen from IK "
            "probing of the SIMULATED B601-DM/gripper model only (2026-09-26) "
            "-- re-verify against the real arm before relying on them there, "
            "and stop relying on home_position once "
            f"{self._base_frame} -> {self._ee_frame} TF is available from M3."
        )

        # grip/aim: which raw controller-orientation value is decoded into
        # controller_pose_quest/_base below for the [motion] diagnostic log
        # (see _log_motion_diagnostic) -- this no longer feeds anything
        # commanded to the arm (see module docstring).
        self._orientation_source = self._declare_str("orientation_source", "grip")
        if self._orientation_source not in ("grip", "aim"):
            raise ValueError("orientation_source must be grip or aim")
        self._clutch = Clutch()
        self._ik_future = None
        self._command_epoch = 0
        self._last_target: Pose | None = None
        self._last_target_time: Time | None = None
        self._last_msg_time: Time | None = None
        self._homing = False
        self._prev_primary_click = False
        self._last_sample_squeeze: float | None = None
        self._last_sample_is_active: bool | None = None
        self._last_pipeline_latency_ms: float | None = None
        self._last_ik_call_success: bool | None = None
        self._ref_controller_quest: Pose | None = None
        self._ref_controller_base: Pose | None = None
        self._gripper_closed = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._target_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.create_subscription(
            ByteMultiArray, controller_data_topic, self._on_controller_data, 10
        )
        self.create_timer(_STALE_CHECK_PERIOD_S, self._check_stale)
        self.create_timer(_HOMING_STEP_PERIOD_S, self._homing_step)
        self.create_timer(_HEARTBEAT_PERIOD_S, self._log_heartbeat)
        arm_namespace = self._declare_str("arm_namespace", "rebotarm")
        self._move_ik_client = self.create_client(
            MoveToPoseIK, f"/{arm_namespace}/move_to_pose_ik"
        )
        self._gripper_open_client = self.create_client(
            GripperCommand, f"/{arm_namespace}/gripper/open"
        )
        self._gripper_close_client = self.create_client(
            GripperCommand, f"/{arm_namespace}/gripper/close"
        )
        self._gripper_init_timer = self.create_timer(
            _GRIPPER_INIT_DELAY_S, self._init_gripper_once
        )
        self.get_logger().info(
            f"quest_rebot_bridge_node up: side={self._arm_side} "
            f"controller_data={controller_data_topic} target_pose={target_pose_topic} "
            "orientation=fixed-downward(+thumbstick roll) "
            f"ee_frame={self._ee_frame}"
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
            sample = controller_sample(data, self._arm_side, self._orientation_source)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f"failed to decode controller_data: {exc}")
            return

        self._last_msg_time = now
        self._last_sample_squeeze = sample.squeeze
        self._last_sample_is_active = sample.is_active
        if sample.sample_timestamp_ns > 0:
            self._last_pipeline_latency_ms = (
                now.nanoseconds - sample.sample_timestamp_ns
            ) * 1e-6

        if not sample.is_active:
            self._force_release("controller inactive")
            self._prev_primary_click = False
            return

        primary_pressed = sample.primary_click >= _PRIMARY_CLICK_THRESHOLD
        if primary_pressed and not self._prev_primary_click:
            self._start_homing()
        self._prev_primary_click = primary_pressed

        if sample.trigger >= self._trigger_close_threshold and not self._gripper_closed:
            self._set_gripper(closed=True)
        elif sample.trigger < self._trigger_open_threshold and self._gripper_closed:
            self._set_gripper(closed=False)

        if self._last_roll_update_time is not None:
            dt_roll = (now - self._last_roll_update_time).nanoseconds * 1e-9
            if abs(sample.thumbstick_x) > self._roll_deadzone:
                self._roll_offset_rad += self._roll_rate_rad_s * sample.thumbstick_x * dt_roll
        self._last_roll_update_time = now

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
            self._homing = False
            ee_pose = self._lookup_ee_pose()
            if ee_pose is None:
                return
            self._command_epoch += 1
            self._clutch.engage(controller_pose_base, ee_pose)
            self._ref_controller_quest = controller_pose_quest
            self._ref_controller_base = controller_pose_base
            self._last_target = ee_pose
            self._last_target_time = now
            self.get_logger().info("clutch engaged")
        elif sample.squeeze < self._squeeze_release and self._clutch.engaged:
            self.get_logger().info("clutch released -- arm holds")
            self._clutch.release()

        if not self._clutch.engaged or self._last_target is None:
            return

        if self._ik_future is not None:
            return
        proposed = self._clutch.compute_target(controller_pose_base, self._translation_scale)
        # Only proposed.position survives past here -- _pointing_pose replaces
        # proposed.orientation with the fixed straight-down orientation plus
        # thumbstick roll (see module docstring).
        proposed = _pointing_pose(proposed, self._roll_offset_rad)
        self._log_motion_diagnostic(controller_pose_quest, controller_pose_base, proposed)
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
        if self._ik_future is not None:
            return
        epoch = self._command_epoch
        future = self._move_ik_client.call_async(req)
        self._ik_future = future
        future.add_done_callback(
            lambda f: self._on_move_ik_response(f, pose, epoch)
        )

    def _on_move_ik_response(self, future, pose: Pose, epoch: int) -> None:
        self._ik_future = None
        if epoch != self._command_epoch:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._last_ik_call_success = False
            self.get_logger().warn(f"move_to_pose_ik call failed: {exc}", throttle_duration_sec=2.0)
            return
        if not response.success:
            self._last_ik_call_success = False
            self.get_logger().warn(
                f"move_to_pose_ik rejected: {response.message}", throttle_duration_sec=1.0
            )
            return
        self._last_ik_call_success = True
        # Only commit the new reference once the arm has actually accepted
        # it -- otherwise a run of IK failures would let _last_target drift
        # further and further from where the arm truly is, since each
        # subsequent frame's delta is computed against it regardless.
        self._last_target = pose

    # -- gripper (index trigger) -----------------------------------------

    def _init_gripper_once(self) -> None:
        self._gripper_init_timer.cancel()
        self._set_gripper(closed=False)

    def _set_gripper(self, closed: bool) -> None:
        client = self._gripper_close_client if closed else self._gripper_open_client
        label = "close" if closed else "open"
        if not client.service_is_ready():
            self.get_logger().warn(
                f"gripper/{label} service not available -- gripper not commanded",
                throttle_duration_sec=2.0,
            )
            return
        req = GripperCommand.Request()
        if not closed:
            req.position = self._gripper_open_position
        # Optimistic: flip state before the response arrives so a trigger
        # held past the threshold for several frames (this callback runs at
        # controller_data rate, the gripper service call is blocking on the
        # server side) doesn't fire the same open/close call repeatedly.
        self._gripper_closed = closed
        future = client.call_async(req)
        future.add_done_callback(lambda f: self._on_gripper_response(f, label))
        self.get_logger().info(f"gripper {label} requested")

    def _on_gripper_response(self, future, label: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"gripper/{label} call failed: {exc}", throttle_duration_sec=2.0)
            return
        if not response.success:
            self.get_logger().warn(
                f"gripper/{label} did not reach target: {response.message}",
                throttle_duration_sec=1.0,
            )
            return
        self.get_logger().info(f"gripper {label} complete: {response.message}")

    # -- home reset (A/X button) -----------------------------------------

    def _start_homing(self) -> None:
        if self._clutch.engaged:
            self.get_logger().info("clutch released -- home reset requested")
        self._clutch.release()
        self._command_epoch += 1
        self._last_target = None
        self._homing = True
        self.get_logger().info("home reset requested (primary button)")

    def _homing_step(self) -> None:
        if not self._homing or self._ik_future is not None:
            return
        now = self.get_clock().now()
        dt_s = 0.0
        if self._last_target_time is not None:
            dt_s = (now - self._last_target_time).nanoseconds * 1e-9
        current = self._last_target if self._last_target is not None else self._lookup_ee_pose()
        if current is None:
            return
        if _pose_close(current, self._home_pose,
                       _HOMING_POSITION_TOLERANCE_M, _HOMING_ROTATION_TOLERANCE_RAD):
            self._homing = False
            self.get_logger().info("home reset complete")
            return
        target = self._safety_limits.clamp(current, self._home_pose, dt_s)
        self._last_target_time = now
        self._publish_target(target)
        self._call_move_to_pose_ik(target)

    def _log_motion_diagnostic(
        self, controller_pose_quest: Pose, controller_pose_base: Pose, proposed: Pose
    ) -> None:
        """Throttled position-delta trace for tracking down direction-mapping bugs.

        delta_quest is the raw controller displacement since engage, as reported
        by the headset (quest_world frame, untransformed). delta_base is the same
        displacement after the quest_world -> base_link calibration transform --
        this is exactly what gets added to the reference EE pose, unscaled by
        anything else, to become the commanded target position (see clutch.py).
        If the arm visibly moves in the wrong direction, compare a known
        controller displacement (e.g. "moved +10cm along quest's X") against
        delta_base here: if delta_base doesn't look like that displacement
        rotated by the calibration quaternion, the bug is in quest_world
        calibration / the static transform, not in IK.

        rot_deg_*/axis_* are the rotation *angle* (degrees) and *axis* (unit
        xyz, in the quest/base frame respectively) of the same before/after-
        calibration deltas, computed the same way clutch.compute_target()
        computes its rotation delta. Added 2026-09-17 to debug a
        left-arm-only orientation bug in the dual-arm rehearsal (right arm's
        orientation was fine with the same calibration rotation reused for
        both sides).

        NOTE: the angle alone (an earlier version of this log only showed
        that) is USELESS for this -- rotation angle magnitude is preserved
        by any pure rotation, so rot_deg_quest and rot_deg_base are
        mathematically guaranteed to be equal regardless of whether anything
        is wrong; that's exactly what the first left-arm test showed (all
        equal) and it proved nothing. The *axis* is what can reveal a
        direction/handedness bug: axis_base should equal the calibration
        rotation applied to axis_quest (same check as delta_base vs
        delta_quest for position). target_rpy_deg is the resulting commanded
        target orientation itself (base_link frame, degrees) for a direct
        read of "what orientation is being commanded right now".
        """
        if self._ref_controller_quest is None or self._ref_controller_base is None:
            return
        delta_quest = tuple(
            c - r for c, r in zip(controller_pose_quest.position, self._ref_controller_quest.position)
        )
        delta_base = tuple(
            c - r for c, r in zip(controller_pose_base.position, self._ref_controller_base.position)
        )
        rot_deg_quest, axis_quest = _rotation_angle_axis(
            self._ref_controller_quest.orientation, controller_pose_quest.orientation
        )
        rot_deg_base, axis_base = _rotation_angle_axis(
            self._ref_controller_base.orientation, controller_pose_base.orientation
        )
        target_rpy_deg = tuple(
            math.degrees(v) for v in tft.euler_from_quaternion(proposed.orientation)
        )
        self.get_logger().info(
            f"[motion] delta_quest={tuple(round(v, 3) for v in delta_quest)} "
            f"delta_base={tuple(round(v, 3) for v in delta_base)} "
            f"rot_deg_quest={rot_deg_quest:.1f} axis_quest={tuple(round(v, 2) for v in axis_quest)} "
            f"rot_deg_base={rot_deg_base:.1f} axis_base={tuple(round(v, 2) for v in axis_base)} "
            f"target_pos={tuple(round(v, 3) for v in proposed.position)} "
            f"target_rpy_deg={tuple(round(v, 1) for v in target_rpy_deg)}",
            throttle_duration_sec=0.5,
        )

    def _log_heartbeat(self) -> None:
        age_s = None
        if self._last_msg_time is not None:
            age_s = (self.get_clock().now() - self._last_msg_time).nanoseconds * 1e-9
        self.get_logger().info(
            "[heartbeat] "
            f"controller_data_age={'never' if age_s is None else f'{age_s:.2f}s'} "
            f"is_active={self._last_sample_is_active} "
            f"squeeze={self._last_sample_squeeze} "
            f"clutch_engaged={self._clutch.engaged} "
            f"homing={self._homing} "
            f"gripper_closed={self._gripper_closed} "
            f"roll_offset_deg={math.degrees(self._roll_offset_rad):.1f} "
            f"last_ik_call_success={self._last_ik_call_success} "
            "pipeline_latency_ms="
            f"{'n/a' if self._last_pipeline_latency_ms is None else f'{self._last_pipeline_latency_ms:.1f}'}"
        )
        warn_if_duplicate_node(self, "quest_rebot_bridge_node")

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

    def _lookup_ee_pose(self) -> Pose | None:
        try:
            t = self._tf_buffer.lookup_transform(self._base_frame, self._ee_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as exc:
            self.get_logger().warn(
                f"tf2 lookup {self._base_frame} -> {self._ee_frame} failed ({exc}); "
                "waiting for actual EE pose; motion is not started", throttle_duration_sec=2.0
            )
            return None
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


def _local_roll_quat(angle_rad: float) -> tuple:
    """Quaternion for a rotation of angle_rad about the local +X axis (xyzw)."""
    return (math.sin(angle_rad / 2.0), 0.0, 0.0, math.cos(angle_rad / 2.0))


def _basis_quaternion(forward: np.ndarray, up_hint: np.ndarray) -> tuple:
    """Right-handed orientation quaternion whose local +X axis is forward,
    constructed the same way _pointing_pose always has: up_hint is a fixed
    reference (not a measured value, same reasoning as
    quest_world_calibrate.py's up axis) used only to pick a canonical roll=0
    baseline, falling back to a different reference axis when forward is
    within ~2.5deg of up_hint (otherwise degenerate).
    """
    forward = forward / np.linalg.norm(forward)
    up = up_hint
    if abs(float(np.dot(forward, up))) > 0.999:
        up = np.array([1.0, 0.0, 0.0]) if abs(forward[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    z_neg = up - forward * np.dot(up, forward)
    z_neg = z_neg / np.linalg.norm(z_neg)
    z_axis = -z_neg
    y_axis = np.cross(z_axis, forward)

    basis = np.eye(4)
    basis[:3, :3] = np.column_stack([forward, y_axis, z_axis])
    return tuple(tft.quaternion_from_matrix(basis))


# The EE's forward (local +X) direction is now a fixed constant -- straight
# down in base_link (-Z), never derived from the controller (see module
# docstring). up_hint=(0,0,1) is the same "just need a consistent roll=0
# reference axis" choice this always used; it's degenerate against a
# straight-down forward (anti-parallel), so this always falls through to the
# +X fallback inside _basis_quaternion -- computed once here rather than
# every call, since forward no longer varies.
_DOWN_POINTING_QUAT = _basis_quaternion(
    np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, 1.0])
)


def _pointing_pose(pose: Pose, roll_rad: float) -> Pose:
    """Return pose with orientation replaced by the fixed straight-down
    pointing direction (_DOWN_POINTING_QUAT) plus roll_rad about that same
    axis -- pose.orientation is ignored entirely (both its forward direction
    and whatever roll it happened to carry); only pose.position passes
    through unchanged. roll_rad is thumbstick-integrated, see
    _on_controller_data.
    """
    final_quat = tft.quaternion_multiply(_DOWN_POINTING_QUAT, _local_roll_quat(roll_rad))
    return Pose(position=pose.position, orientation=tuple(final_quat))


def _rotation_angle_axis(ref_quat: tuple, cur_quat: tuple) -> tuple[float, tuple]:
    """Angle (degrees) and spatial rotation axis in the common parent frame."""
    delta_quat = tft.quaternion_multiply(cur_quat, tft.quaternion_inverse(ref_quat))
    x, y, z, w = delta_quat
    w_clamped = max(-1.0, min(1.0, abs(w)))
    angle_deg = math.degrees(2.0 * math.acos(w_clamped))
    # Negate xyz along with w<0 (same rotation, other quaternion sign) so the
    # axis always corresponds to the angle above via the right-hand rule.
    if w < 0:
        x, y, z = -x, -y, -z
    axis_norm = math.sqrt(x * x + y * y + z * z)
    if axis_norm < 1e-9:
        return angle_deg, (0.0, 0.0, 0.0)
    return angle_deg, (x / axis_norm, y / axis_norm, z / axis_norm)


def _pose_close(a: Pose, b: Pose, pos_tol_m: float, rot_tol_rad: float) -> bool:
    dist = math.sqrt(sum((x - y) ** 2 for x, y in zip(a.position, b.position)))
    if dist > pos_tol_m:
        return False
    angle_deg, _ = _rotation_angle_axis(a.orientation, b.orientation)
    return math.radians(angle_deg) <= rot_tol_rad


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
