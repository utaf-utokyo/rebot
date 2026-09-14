"""Squeeze-to-engage relative pose clutch.

squeeze OFF: reBot does not move.
squeeze ON (rising edge): stores the current Quest controller pose (in
base_link frame) and the current reBot EE pose as the reference pair.
squeeze held: reBot EE tracks the *relative* displacement of the controller
since engage, applied on top of the reference EE pose (scaled).
squeeze OFF: clutch releases; the last commanded target pose is held.
"""

from __future__ import annotations

from dataclasses import dataclass

import tf_transformations as tft


@dataclass
class Pose:
    position: tuple  # (x, y, z)
    orientation: tuple  # (x, y, z, w)


class Clutch:
    def __init__(self) -> None:
        self.engaged = False
        self._ref_controller: Pose | None = None
        self._ref_ee: Pose | None = None

    def engage(self, controller_pose: Pose, ee_pose: Pose) -> None:
        self._ref_controller = controller_pose
        self._ref_ee = ee_pose
        self.engaged = True

    def release(self) -> None:
        self.engaged = False

    def compute_target(
        self,
        controller_pose: Pose,
        translation_scale: float,
        rotation_scale: float,
    ) -> Pose:
        """Return the target EE pose for the current controller pose.

        Must only be called while engaged.
        """
        assert self.engaged and self._ref_controller is not None and self._ref_ee is not None

        delta_pos = tuple(
            (c - r) * translation_scale
            for c, r in zip(controller_pose.position, self._ref_controller.position)
        )
        target_pos = tuple(
            e + d for e, d in zip(self._ref_ee.position, delta_pos)
        )

        # Rotation delta of the controller since engage, expressed as a
        # quaternion; slerp from identity toward it scales the angle by
        # `rotation_scale` while keeping the same axis.
        delta_quat = tft.quaternion_multiply(
            tft.quaternion_inverse(self._ref_controller.orientation),
            controller_pose.orientation,
        )
        identity = (0.0, 0.0, 0.0, 1.0)
        scaled_delta_quat = tft.quaternion_slerp(identity, delta_quat, rotation_scale)
        target_quat = tft.quaternion_multiply(self._ref_ee.orientation, scaled_delta_quat)

        return Pose(position=target_pos, orientation=tuple(target_quat))
