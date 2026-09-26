"""Decodes xr_teleop/controller_data (as published by controller_raw_publisher)."""

from __future__ import annotations

from dataclasses import dataclass

import msgpack
from std_msgs.msg import ByteMultiArray


@dataclass
class ControllerSample:
    grip_position: tuple
    grip_orientation: tuple  # xyzw
    squeeze: float
    trigger: float
    is_active: bool
    primary_click: float  # A (right controller) / X (left controller)
    thumbstick_x: float  # -1 (left) .. +1 (right)
    sample_timestamp_ns: int  # time.time_ns() when controller_raw_publisher's
    # CloudXR SDK call returned this sample (PC-local clock, not the Quest's
    # own clock) -- 0 for older/synthetic payloads that predate this field
    # (e.g. tools/check_dual_arm_tracking.py), same fallback pattern as
    # thumbstick_x above. Lets bridge_node measure how much of any perceived
    # teleop lag is inside the ROS pipeline (this PC) versus upstream of it
    # (CloudXR/Wi-Fi, which this timestamp can't see since it's captured
    # after the SDK call already returned).


def decode_controller_data(msg: ByteMultiArray) -> dict:
    raw = bytes(b[0] for b in msg.data)
    return msgpack.unpackb(raw, raw=False)


def controller_sample(data: dict, side: str, orientation_source: str = "grip") -> ControllerSample:
    return ControllerSample(
        grip_position=tuple(data[f"{side}_grip_position"]),
        grip_orientation=tuple(data[f"{side}_{orientation_source}_orientation"]),
        squeeze=float(data[f"{side}_squeeze_value"]),
        trigger=float(data[f"{side}_trigger_value"]),
        is_active=bool(data[f"{side}_is_active"]),
        primary_click=float(data[f"{side}_primary_click"]),
        # Optional: absent from synthetic/older payloads (e.g.
        # tools/check_dual_arm_tracking.py) that predate roll-via-thumbstick.
        thumbstick_x=float(data.get(f"{side}_thumbstick", (0.0, 0.0))[0]),
        sample_timestamp_ns=int(data.get("timestamp", 0)),
    )
