"""Per-step safety limiting for the teleop target pose.

Applied to every proposed target pose before it is published, regardless of
clutch state:
  - workspace limits (position clamped to a bounding box in base_link frame)
  - max translation step / max rotation step (jump-safety, per call)
  - max linear/angular velocity (step limit tightened further using dt)

IK failure / stale input / tracking loss / clutch-OFF handling live in
bridge_node.py, which decides when *not* to call clamp() at all (holding the
last published target instead).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import tf_transformations as tft

from quest_rebot_bridge.clutch import Pose


@dataclass
class SafetyLimitsConfig:
    max_translation_step_m: float
    max_rotation_step_rad: float
    max_linear_velocity_mps: float
    max_angular_velocity_radps: float
    workspace_min: tuple  # (x, y, z)
    workspace_max: tuple  # (x, y, z)


class SafetyLimits:
    def __init__(self, config: SafetyLimitsConfig) -> None:
        self._cfg = config

    def clamp(self, last_target: Pose, proposed: Pose, dt_s: float) -> Pose:
        pos = tuple(
            min(max(p, lo), hi)
            for p, lo, hi in zip(
                proposed.position, self._cfg.workspace_min, self._cfg.workspace_max
            )
        )
        pos = self._clamp_translation_step(last_target.position, pos, dt_s)
        quat = self._clamp_rotation_step(
            last_target.orientation, proposed.orientation, dt_s
        )
        return Pose(position=pos, orientation=quat)

    def _clamp_translation_step(
        self, last_pos: tuple, proposed_pos: tuple, dt_s: float
    ) -> tuple:
        delta = tuple(p - l for p, l in zip(proposed_pos, last_pos))
        dist = math.sqrt(sum(d * d for d in delta))
        max_step = self._cfg.max_translation_step_m
        if dt_s > 0.0:
            max_step = min(max_step, self._cfg.max_linear_velocity_mps * dt_s)
        if dist <= max_step or dist == 0.0:
            return proposed_pos
        scale = max_step / dist
        return tuple(l + d * scale for l, d in zip(last_pos, delta))

    def _clamp_rotation_step(
        self, last_quat: tuple, proposed_quat: tuple, dt_s: float
    ) -> tuple:
        delta_quat = tft.quaternion_multiply(
            tft.quaternion_inverse(last_quat), proposed_quat
        )
        # Angle of the delta rotation; clamp w for acos() numerical safety.
        w = max(-1.0, min(1.0, abs(delta_quat[3])))
        angle = 2.0 * math.acos(w)
        max_angle = self._cfg.max_rotation_step_rad
        if dt_s > 0.0:
            max_angle = min(max_angle, self._cfg.max_angular_velocity_radps * dt_s)
        if angle <= max_angle or angle == 0.0:
            return proposed_quat
        fraction = max_angle / angle
        return tuple(tft.quaternion_slerp(last_quat, proposed_quat, fraction))
