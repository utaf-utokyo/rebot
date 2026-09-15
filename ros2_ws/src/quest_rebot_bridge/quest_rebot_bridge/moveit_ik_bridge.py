"""move_to_pose_ik adapter in front of rebotarm_moveit_config's demo.launch.py.

bridge_node (M2/M4/M5) only knows how to call the MoveToPoseIK service; it
has no idea whether the thing on the other end is the real
rebotarmcontroller, fake_ik_driver, or this node. This node exists so an
M5 Quest rehearsal can target the standard MoveIt demo (RViz + ros2_control
mock hardware from rebotarm_moveit_config) instead of fake_ik_driver's
headless, no-visualization stand-in -- so a human can actually watch the
arm follow the Quest controller in RViz while debugging quest_world
calibration / IK edge cases, instead of only trusting joint_states numbers.

Like fake_ik_driver, this node never opens CAN/serial: it solves IK with the
same reBotArm_control_py solver/URDF/joint limits as the real driver, then
publishes a single-point JointTrajectory to rebotarm_controller (the
ros2_control mock_components/GenericSystem hardware spawned by
demo.launch.py). No real motion, ever.
"""

from __future__ import annotations

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarmcontroller.hardware_config import _ensure_rebot_sdk_in_syspath

_SDK_ROOT = _ensure_rebot_sdk_in_syspath()

from reBotArm_control_py.kinematics.inverse_kinematics import (  # noqa: E402
    IKParams,
    pos_rot_to_se3,
    solve_ik,
)
from reBotArm_control_py.kinematics.robot_model import (  # noqa: E402
    load_robot_model,
    pad_q_for_model,
)

from rebotarm_msgs.srv import MoveToPoseIK  # noqa: E402

_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class MoveItIKBridge(Node):
    def __init__(self) -> None:
        super().__init__("moveit_ik_bridge")

        arm_namespace = self._declare_str("arm_namespace", "rebotarm")
        trajectory_topic = self._declare_str(
            "joint_trajectory_topic", "/rebotarm_controller/joint_trajectory"
        )
        self._time_from_start_s = self._declare_float("time_from_start_s", 0.15)

        urdf_path = str(_SDK_ROOT / "urdf" / "DM" / "urdf" / "ReBot_Arm_DM.urdf")
        self._model = load_robot_model(urdf_path=urdf_path)
        self._data = self._model.createData()
        self._end_frame_id = self._model.getFrameId("end_link")
        self._ik_params = IKParams()
        self._q = pad_q_for_model(self._model, np.zeros(6), 6)

        self._trajectory_pub = self.create_publisher(
            JointTrajectory, trajectory_topic, 10
        )
        self.create_service(
            MoveToPoseIK,
            f"/{arm_namespace}/move_to_pose_ik",
            self._on_move_to_pose_ik,
        )
        self.get_logger().info(
            f"moveit_ik_bridge up: service=/{arm_namespace}/move_to_pose_ik "
            f"-> trajectory_topic={trajectory_topic} "
            "(SIMULATION ONLY -- no CAN/serial, targets rebotarm_moveit_config demo)"
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
        self._publish_trajectory(self._q[:6])
        response.success = True
        response.message = "IK target accepted (SIMULATED via rebotarm_moveit_config)"
        response.q_solution = [float(v) for v in result.q[:6]]
        return response

    def _publish_trajectory(self, q: np.ndarray) -> None:
        msg = JointTrajectory()
        msg.joint_names = _JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        seconds = int(self._time_from_start_s)
        point.time_from_start = Duration(
            sec=seconds,
            nanosec=int((self._time_from_start_s - seconds) * 1e9),
        )
        msg.points = [point]
        self._trajectory_pub.publish(msg)


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
    node = MoveItIKBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
