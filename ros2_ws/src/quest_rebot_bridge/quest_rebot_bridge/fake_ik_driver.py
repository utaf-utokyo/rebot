"""Pure-software stand-in for rebotarmcontroller's move_to_pose_ik, for M5 dry runs.

This node NEVER touches motorbridge/CAN/serial -- it has no hardware
dependency at all. It exists so quest_rebot_bridge (clutch, safety limits,
calibration, IK direction/sign) can be exercised end-to-end against the real
Quest 3 controller before any of that traffic is pointed at the real arm.

By default it uses the hardware SDK model and end_link. The dual-arm launch
explicitly supplies the visualization URDF and gripper_link, so simulated IK,
TF, and the displayed gripper all use the same kinematic model. This dual-arm
configuration validates simulation tracking, not hardware model equivalence.

Runs in a separate ROS namespace (default: rebotarm_sim) so it can never be
confused with, or collide with, the real /rebotarm namespace.

Also provides gripper/open and gripper/close (matching bridge_node's
trigger-driven gripper control), publishing straight into the same
periodic JointState message rather than a separate controller/topic --
see moveit_ik_bridge.py's module docstring for why request.position is
ignored here (same rough visual approximation, different plumbing).
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# quest_rebot_bridge and rebotarmcontroller are sibling ament_python packages
# under ros2_ws/src -- third_party/reBotArm_control_py lives under
# rebotarmcontroller's own package tree, not this one, so reuse its resolver
# instead of independently walking up from this file's own location.
from rebotarmcontroller.hardware_config import _ensure_rebot_sdk_in_syspath

_SDK_ROOT = _ensure_rebot_sdk_in_syspath()

from reBotArm_control_py.kinematics.forward_kinematics import compute_fk  # noqa: E402
from reBotArm_control_py.kinematics.inverse_kinematics import (  # noqa: E402
    IKParams,
    pos_rot_to_se3,
    solve_ik,
)
from reBotArm_control_py.kinematics.robot_model import (  # noqa: E402
    load_robot_model,
    pad_q_for_model,
)

from rebotarm_msgs.srv import GripperCommand, MoveToPoseIK  # noqa: E402

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_GRIPPER_JOINT_NAMES = ["gripper_joint1", "gripper_joint2"]
# reBot_B601_DM_with_gripper.urdf: gripper_joint1/2 are prismatic, range [0, 0.0715]m.
# Same rough visual approximation as moveit_ik_bridge -- see its module docstring
# for why request.position (the real hardware's single-motor radian value) isn't
# reused here directly.
_GRIPPER_SIM_CLOSE_M = 0.0
_GRIPPER_SIM_FULL_OPEN_M = 0.0715


class FakeIKDriver(Node):
    def __init__(self) -> None:
        super().__init__("fake_ik_driver")

        self._namespace = self._declare_str("arm_namespace", "rebotarm_sim")
        rate = self._declare_float("joint_state_rate", 100.0)
        self._gripper_open_fraction = self._declare_float("gripper_open_fraction", 1.0 / 3.0)
        self._gripper_q = [_GRIPPER_SIM_CLOSE_M, _GRIPPER_SIM_CLOSE_M]

        urdf_path = str(_SDK_ROOT / "urdf" / "DM" / "urdf" / "ReBot_Arm_DM.urdf")
        urdf_path = self._declare_str("urdf_path", urdf_path)
        self._ee_frame = self._declare_str("ee_frame", "end_link")
        self._model = load_robot_model(urdf_path=urdf_path)
        self._data = self._model.createData()
        self._end_frame_id = self._model.getFrameId(self._ee_frame)
        if self._end_frame_id >= self._model.nframes:
            raise ValueError(f"EE frame {self._ee_frame} absent from {urdf_path}")
        self._ik_params = IKParams()

        self.declare_parameter("initial_joints", [0.0] * 6)
        initial = np.array(self.get_parameter("initial_joints").value)
        if len(initial) != 6:
            raise ValueError("initial_joints requires six values")
        self._q = pad_q_for_model(self._model, initial, 6)
        if np.any(self._q < self._model.lowerPositionLimit) or np.any(self._q > self._model.upperPositionLimit):
            raise ValueError("initial_joints outside model limits")

        self.create_service(
            MoveToPoseIK,
            f"/{self._namespace}/move_to_pose_ik",
            self._on_move_to_pose_ik,
        )
        self.create_service(
            GripperCommand,
            f"/{self._namespace}/gripper/open",
            self._make_gripper_handler(_GRIPPER_SIM_FULL_OPEN_M * self._gripper_open_fraction, "open"),
        )
        self.create_service(
            GripperCommand,
            f"/{self._namespace}/gripper/close",
            self._make_gripper_handler(_GRIPPER_SIM_CLOSE_M, "close"),
        )
        self._joint_state_pub = self.create_publisher(
            JointState, f"/{self._namespace}/joint_states", 10
        )
        self.create_timer(1.0 / rate, self._publish_joint_states)

        pos, rot, _ = compute_fk(self._model, self._q, frame_name=self._ee_frame)
        self.get_logger().info(
            f"fake_ik_driver up: namespace={self._namespace} "
            f"(SIMULATION ONLY -- no CAN/serial, no real motion) "
            f"ee_frame={self._ee_frame} start pos={pos.tolist()}"
        )

    def _declare_str(self, name: str, default: str) -> str:
        self.declare_parameter(name, default)
        return str(self.get_parameter(name).value)

    def _declare_float(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        return float(self.get_parameter(name).value)

    def _on_move_to_pose_ik(self, request, response):
        p = request.target_pose.position
        o = request.target_pose.orientation
        target_pos = np.array([p.x, p.y, p.z])
        target_rot = _quat_to_matrix(o.x, o.y, o.z, o.w)
        target = pos_rot_to_se3(target_pos, target_rot)

        result = solve_ik(
            self._model, self._data, self._end_frame_id, target,
            self._q.copy(), self._ik_params, controlled_joints=6,
        )
        if not result.success:
            response.success = False
            response.message = f"IK did not converge (err={result.error:.4g})"
            response.q_solution = []
            return response

        self._q = pad_q_for_model(self._model, result.q[:6], 6)
        response.success = True
        response.message = "IK target accepted (SIMULATED)"
        response.q_solution = [float(v) for v in result.q[:6]]
        return response

    def _make_gripper_handler(self, target_m: float, label: str):
        def handler(request, response):
            self._gripper_q = [target_m, target_m]
            response.success = True
            response.reached_position = target_m
            response.message = f"gripper {label} SIMULATED (target={target_m:.4f}m)"
            return response
        return handler

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = _JOINT_NAMES + _GRIPPER_JOINT_NAMES
        msg.position = [float(v) for v in self._q[:6]] + list(self._gripper_q)
        self._joint_state_pub.publish(msg)


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeIKDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
