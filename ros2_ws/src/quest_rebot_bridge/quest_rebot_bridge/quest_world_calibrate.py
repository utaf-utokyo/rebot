"""Interactive quest_world -> base_link ROTATION calibration helper.

Why only rotation: Clutch.compute_target() only ever uses *differences*
between the current and reference controller pose (see clutch.py). For a
rigid transform T(p) = R*p + t, T(c) - T(r) = R*(c - r) -- the translation t
cancels exactly. So the base_link -> quest_world static transform's
translation (calib_x/y/z in bridge.launch.py) never affects teleop direction
or scale; only its rotation (calib_qx/qy/qz/qw) does. This tool solves for
that rotation and leaves translation at 0.

Procedure:
  1. Run this node with the real Quest 3 connected (controller_raw_publisher
     already running, reBot/rebotarmcontroller NOT required -- this only
     reads xr_teleop/controller_data, it never touches the arm).
  2. Hold the controller the way you want "neutral, facing forward" to mean:
     pointing away from you, roughly level, the same way you'd want the
     gripper facing when you haven't rotated your wrist at all. In
     particular, "away from you" should be the direction you consider the
     robot's forward (+X, base_link) -- e.g. stand where the robot is in
     front of you and point the controller the same way the arm reaches out.
  3. Press and hold the primary button (A on the right controller, X on the
     left) for ~2 seconds while holding as still as possible, then release.
  4. The tool averages the sampled orientation and prints the
     calib_qx/qy/qz/qw values to paste into bridge.launch.py (calib_x/y/z
     stay 0.0 -- see above).

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

_HOLD_DURATION_S = 2.0
_PRIMARY_CLICK_THRESHOLD = 0.5


class QuestWorldCalibrate(Node):
    def __init__(self) -> None:
        super().__init__("quest_world_calibrate")

        self._arm_side = self._declare_str("arm_side", "right")
        controller_data_topic = self._declare_str(
            "controller_data_topic", "/xr_teleop/controller_data"
        )

        self._held_since: float | None = None
        self._samples: list[tuple] = []
        self._prev_primary_click = False
        self._done = False

        self.create_subscription(
            ByteMultiArray, controller_data_topic, self._on_controller_data, 10
        )
        button_name = "A" if self._arm_side == "right" else "X"
        self.get_logger().info(
            f"quest_world_calibrate up: side={self._arm_side} "
            f"controller_data={controller_data_topic}\n"
            "Hold the controller pointing the way you want base_link +X "
            f"(robot forward) to mean, then press and hold {button_name} "
            f"for {_HOLD_DURATION_S:.0f}s and release."
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
            self._held_since = None
            self._samples.clear()
            self._prev_primary_click = False
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        pressed = sample.primary_click >= _PRIMARY_CLICK_THRESHOLD

        if pressed and not self._prev_primary_click:
            self._held_since = now
            self._samples = []
            self.get_logger().info("sampling... hold still")
        if pressed:
            self._samples.append(sample.grip_orientation)
        elif self._prev_primary_click and self._held_since is not None:
            held_for = now - self._held_since
            if held_for < _HOLD_DURATION_S:
                self.get_logger().warn(
                    f"held only {held_for:.1f}s (< {_HOLD_DURATION_S:.0f}s) -- "
                    "press and hold again"
                )
            else:
                self._finish()
            self._held_since = None

        self._prev_primary_click = pressed

    def _finish(self) -> None:
        q_sample = _average_quaternion(self._samples)
        calib_quat = tft.quaternion_inverse(q_sample)
        qx, qy, qz, qw = calib_quat
        self.get_logger().info(
            f"averaged {len(self._samples)} samples -- "
            f"quest_world -> base_link rotation:\n"
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


def _average_quaternion(samples: list[tuple]) -> tuple:
    """Component-wise mean + renormalize.

    Only valid for samples clustered closely together (as expected here --
    the user is asked to hold still), where this approximates the true
    rotation average well; it is not a general-purpose quaternion averaging
    method (no antipodal handling, no eigenvector solve).
    """
    arr = np.asarray(samples, dtype=np.float64)
    ref = arr[0]
    signs = np.sign(arr @ ref)
    signs[signs == 0.0] = 1.0
    arr = arr * signs[:, None]
    mean = arr.mean(axis=0)
    return tuple(mean / np.linalg.norm(mean))


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
