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


def decode_controller_data(msg: ByteMultiArray) -> dict:
    raw = bytes(b[0] for b in msg.data)
    return msgpack.unpackb(raw, raw=False)


def controller_sample(data: dict, side: str) -> ControllerSample:
    return ControllerSample(
        grip_position=tuple(data[f"{side}_grip_position"]),
        grip_orientation=tuple(data[f"{side}_grip_orientation"]),
        squeeze=float(data[f"{side}_squeeze_value"]),
        trigger=float(data[f"{side}_trigger_value"]),
        is_active=bool(data[f"{side}_is_active"]),
        primary_click=float(data[f"{side}_primary_click"]),
    )
