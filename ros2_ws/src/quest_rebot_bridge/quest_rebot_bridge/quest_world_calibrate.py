"""Interactive quest_world -> base_link ROTATION calibration helper.

Why only rotation: Clutch.compute_target() only ever uses *differences*
between the current and reference controller pose (see clutch.py). For a
rigid transform T(p) = R*p + t, T(c) - T(r) = R*(c - r) -- the translation t
cancels exactly. So the base_link -> quest_world static transform's
translation (calib_x/y/z in bridge.launch.py) never affects teleop direction
or scale; only its rotation (calib_qx/qy/qz/qw) does. This tool solves for
that rotation and leaves translation at 0.

Why this measures a *movement* (position), not a *held orientation*:
An earlier version of this tool asked the user to hold the controller
pointing "forward, level" and used that instant's raw orientation quaternion
directly as the quest_world -> base_link rotation. That silently assumed the
controller's own local axis convention (whatever isaacteleop's
GRIP_ORIENTATION uses) lines up with base_link's ROS REP103 convention
(X=forward, Y=left, Z=up) whenever a human holds it "level, pointing
forward" -- which is not guaranteed and, when actually measured on this rig
(2026-09-16 M5 session), was wrong: it correctly fixed the forward/yaw
direction but never remapped OpenXR's RUB axis order (X=right, Y=up, Z=back)
onto FLU's (X=forward, Y=left, Z=up), so "up" in Quest space ended up mostly
on base_link's Y (sideways) instead of Z (up) -- diagnosed from a
[motion]-tagged bridge_node log showing a pure vertical hand lift producing a
mostly-sideways-and-down commanded target.

This version avoids the whole controller-local-axis-convention question by
using only two things that don't depend on it:
  - quest_world's Y axis is *always* vertical/up (a guarantee of OpenXR
    room-scale tracking, independent of which way the headset faced during
    Guardian/room setup) -- so it always maps to base_link's Z, no
    measurement needed.
  - a single measured direction: the horizontal direction the user physically
    pushes the controller (grip_position, not orientation) while pointing it
    at the robot -- this maps to base_link's X (forward).
"left" (base_link's Y) then follows from the right-hand rule so the whole
thing is a proper rotation.

Procedure:
  1. Run this node with the real Quest 3 connected (controller_raw_publisher
     already running, reBot/rebotarmcontroller NOT required -- this only
     reads xr_teleop/controller_data, it never touches the arm).
  2. Stand where the robot is in front of you. Press and HOLD the primary
     button (A on the right controller, X on the left), then push the
     controller in a straight horizontal line away from you, toward the
     robot -- at least ~15cm of travel -- then release the button.
  3. The tool takes the straight-line displacement between the start and end
     of that push, flattens it onto the horizontal plane (drops whatever
     vertical component crept in -- small unintentional tilt in the push
     doesn't matter, only its horizontal direction does), and prints the
     resulting calib_qx/qy/qz/qw to paste into bridge.launch.py
     (calib_x/y/z stay 0.0 -- see above).

This only needs to be redone if you physically move the Quest tracking
origin (e.g. re-run Guardian/room setup) relative to the robot base -- not
every session, as long as the Quest's room-scale origin stays put.
"""

from __future__ import annotations

import numpy as np
import rclpy
import tf_transformations as tft
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

from quest_rebot_bridge.controller_data_codec import controller_sample, decode_controller_data

_MIN_PUSH_DISTANCE_M = 0.15
_UP_QUEST = np.array([0.0, 1.0, 0.0])  # OpenXR/Guardian: quest_world's Y is always up.


class QuestWorldCalibrate(Node):
    def __init__(self) -> None:
        super().__init__("quest_world_calibrate")

        self._arm_side = self._declare_str("arm_side", "right")
        controller_data_topic = self._declare_str(
            "controller_data_topic", "/xr_teleop/controller_data"
        )

        self._positions: list[tuple] = []
        self._prev_primary_click = False
        self._done = False

        self.create_subscription(
            ByteMultiArray, controller_data_topic, self._on_controller_data, 10
        )
        button_name = "A" if self._arm_side == "right" else "X"
        self.get_logger().info(
            f"quest_world_calibrate up: side={self._arm_side} "
            f"controller_data={controller_data_topic}\n"
            f"Press and HOLD {button_name}, push the controller in a straight "
            f"horizontal line toward the robot (>= {_MIN_PUSH_DISTANCE_M*100:.0f}cm), "
            "then release."
        )

    def _declare_str(self, name: str, default: str) -> str:
        self.declare_parameter(name, default)
        return str(self.get_parameter(name).value)

    def _on_controller_data(self, msg: ByteMultiArray) -> None:
        if self._done:
            return
        try:
            data = decode_controller_data(msg)
            sample = controller_sample(data, self._arm_side)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f"failed to decode controller_data: {exc}")
            return

        if not sample.is_active:
            self._positions = []
            self._prev_primary_click = False
            return

        pressed = sample.primary_click >= 0.5

        if pressed and not self._prev_primary_click:
            self._positions = []
            self.get_logger().info("tracking... push toward the robot now")
        if pressed:
            self._positions.append(sample.grip_position)
        elif self._prev_primary_click and self._positions:
            self._finish()

        self._prev_primary_click = pressed

    def _finish(self) -> None:
        start = np.array(self._positions[0])
        end = np.array(self._positions[-1])
        forward_raw = end - start
        distance = float(np.linalg.norm(forward_raw))
        if distance < _MIN_PUSH_DISTANCE_M:
            self.get_logger().warn(
                f"only moved {distance*100:.1f}cm (< {_MIN_PUSH_DISTANCE_M*100:.0f}cm) -- "
                "press and push further, release again"
            )
            self._positions = []
            return

        # Flatten onto the horizontal plane: only the push's horizontal
        # direction matters, per this file's module docstring.
        forward_horizontal = forward_raw - _UP_QUEST * float(np.dot(forward_raw, _UP_QUEST))
        f = forward_horizontal / np.linalg.norm(forward_horizontal)
        u = _UP_QUEST / np.linalg.norm(_UP_QUEST)
        left = np.cross(u, f)
        left /= np.linalg.norm(left)

        # Columns = quest_world-frame images of base_link's (X=fwd, Y=left,
        # Z=up); this matrix is orthonormal, so its transpose is its inverse
        # and maps quest_world vectors into base_link coordinates directly.
        quest_basis = np.column_stack([f, left, u])
        rot_matrix = quest_basis.T

        homogeneous = np.eye(4)
        homogeneous[:3, :3] = rot_matrix
        qx, qy, qz, qw = tft.quaternion_from_matrix(homogeneous)
        self.get_logger().info(
            f"pushed {distance*100:.1f}cm -- quest_world -> base_link rotation:\n"
            f"  calib_qx: {qx:.6f}\n"
            f"  calib_qy: {qy:.6f}\n"
            f"  calib_qz: {qz:.6f}\n"
            f"  calib_qw: {qw:.6f}\n"
            "Paste these as launch arguments to bridge.launch.py "
            "(calib_x/y/z stay 0.0 -- translation doesn't affect teleop, "
            "see this file's module docstring), e.g.:\n"
            "  ros2 launch quest_rebot_bridge bridge.launch.py "
            f"calib_qx:={qx:.6f} calib_qy:={qy:.6f} "
            f"calib_qz:={qz:.6f} calib_qw:={qw:.6f}"
        )
        self._done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuestWorldCalibrate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
