#!/usr/bin/env python3
"""Publishes Meta Quest 3 controller state to /xr_teleop/controller_data.

Milestone 1 scope only: reads controller pose/button state from the
isaacteleop/CloudXR SDK and publishes it as a ROS 2 topic. Never opens a
connection to the reBot arm and never sends anything that could move real
hardware.

Replaces NVIDIA IsaacTeleop's own examples/teleop_ros2/python/teleop_ros2_node.py
(mode=controller_raw) rather than running it directly: that reference script
calls `session.step()` with no `execution_events`, which was found (by direct
reproduction on this GX10) to leave `deviceio_session` as `None` under
isaacteleop==1.3.131, raising `AttributeError` on every step and destabilizing
the CloudXR connection. This node uses the same `ControllersSource` +
`TeleopSession` primitives, but calls `step(execution_events=...)` explicitly
(the pattern already verified working end-to-end against the physical Quest 3
in tools/xr_pose_smoke_test.py), and publishes the same wire format
(std_msgs/ByteMultiArray, msgpack-encoded dict, one byte per array element --
see IsaacTeleop's teleop_ros2 README "Controller Data Decoding") so any
tooling written against the upstream schema still works.
"""

from __future__ import annotations

import time

import msgpack
import msgpack_numpy as mnp
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray


def _build_controller_payload(left_ctrl, right_ctrl) -> dict:
    from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex

    def as_list(ctrl, index, default):
        if ctrl.is_none:
            return list(default)
        return [float(x) for x in ctrl[index]]

    def as_float(ctrl, index):
        if ctrl.is_none:
            return 0.0
        return float(ctrl[index])

    payload = {"timestamp": time.time_ns()}
    for side, ctrl in (("left", left_ctrl), ("right", right_ctrl)):
        payload[f"{side}_thumbstick"] = [
            as_float(ctrl, ControllerInputIndex.THUMBSTICK_X),
            as_float(ctrl, ControllerInputIndex.THUMBSTICK_Y),
        ]
        payload[f"{side}_trigger_value"] = as_float(ctrl, ControllerInputIndex.TRIGGER_VALUE)
        payload[f"{side}_squeeze_value"] = as_float(ctrl, ControllerInputIndex.SQUEEZE_VALUE)
        payload[f"{side}_aim_position"] = as_list(ctrl, ControllerInputIndex.AIM_POSITION, (0.0, 0.0, 0.0))
        payload[f"{side}_grip_position"] = as_list(ctrl, ControllerInputIndex.GRIP_POSITION, (0.0, 0.0, 0.0))
        payload[f"{side}_aim_orientation"] = as_list(ctrl, ControllerInputIndex.AIM_ORIENTATION, (0.0, 0.0, 0.0, 1.0))
        payload[f"{side}_grip_orientation"] = as_list(ctrl, ControllerInputIndex.GRIP_ORIENTATION, (0.0, 0.0, 0.0, 1.0))
        payload[f"{side}_primary_click"] = as_float(ctrl, ControllerInputIndex.PRIMARY_CLICK)
        payload[f"{side}_secondary_click"] = as_float(ctrl, ControllerInputIndex.SECONDARY_CLICK)
        payload[f"{side}_thumbstick_click"] = as_float(ctrl, ControllerInputIndex.THUMBSTICK_CLICK)
        payload[f"{side}_menu_click"] = as_float(ctrl, ControllerInputIndex.MENU_CLICK)
        payload[f"{side}_is_active"] = not ctrl.is_none
    return payload


class ControllerRawPublisher(Node):
    def __init__(self) -> None:
        super().__init__("controller_raw_publisher")

        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("controller_data_topic", "xr_teleop/controller_data")
        self.declare_parameter("cloudxr_accept_eula", True)

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        topic = str(self.get_parameter("controller_data_topic").value)
        self._accept_eula = bool(self.get_parameter("cloudxr_accept_eula").value)

        self._pub = self.create_publisher(ByteMultiArray, topic, 10)
        self.get_logger().info(
            f"controller_raw_publisher up: topic={topic} rate_hz={self._rate_hz} "
            "(Milestone 1 scope: publish-only, no hardware commands)"
        )

    def _publish(self, left_ctrl, right_ctrl) -> None:
        if left_ctrl.is_none and right_ctrl.is_none:
            return
        payload = _build_controller_payload(left_ctrl, right_ctrl)
        packed = msgpack.packb(payload, default=mnp.encode)
        msg = ByteMultiArray()
        msg.data = tuple(bytes([b]) for b in packed)
        self._pub.publish(msg)

    def run(self) -> int:
        from isaacteleop.cloudxr import CloudXRLauncher
        from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
        from isaacteleop.retargeting_engine.interface import (
            ExecutionEvents,
            ExecutionState,
            OutputCombiner,
        )
        from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig

        controllers = ControllersSource(name="controllers")
        pipeline = OutputCombiner(
            {
                "left": controllers.output(ControllersSource.LEFT),
                "right": controllers.output(ControllersSource.RIGHT),
            }
        )
        session_config = TeleopSessionConfig(
            app_name="QuestRebotBridgeControllerRaw", pipeline=pipeline
        )
        running_events = ExecutionEvents(execution_state=ExecutionState.RUNNING, reset=False)
        dt = 1.0 / self._rate_hz

        launcher = CloudXRLauncher(accept_eula=self._accept_eula)
        self.get_logger().info("CloudXR runtime launching (first run may take ~30s)...")
        try:
            while rclpy.ok():
                launcher.health_check()
                session = None
                try:
                    session = TeleopSession(session_config)
                    session.__enter__()
                    self.get_logger().info("TeleopSession started successfully")
                    while rclpy.ok():
                        launcher.health_check()
                        t0 = time.monotonic()
                        result = session.step(execution_events=running_events)
                        info = session.last_step_info
                        if info is not None and info.worker_exception is not None:
                            raise RuntimeError("retargeting worker raised") from info.worker_exception
                        self._publish(result["left"], result["right"])
                        rclpy.spin_once(self, timeout_sec=0.0)
                        time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                except RuntimeError as e:
                    self.get_logger().warn(f"XR session issue ({e}); retrying in 2s...")
                    time.sleep(2.0)
                finally:
                    if session is not None:
                        try:
                            session.__exit__(None, None, None)
                        except Exception:
                            pass
        finally:
            launcher.stop()
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerRawPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
