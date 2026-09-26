"""Squeeze-to-engage position clutch.

squeeze OFF: reBot does not move.
squeeze ON (rising edge): stores the current Quest controller pose (in
base_link frame) and the current reBot EE pose as the reference pair.
squeeze held: reBot EE tracks the *relative* displacement of the controller
since engage, applied on top of the reference EE position (scaled).
squeeze OFF: clutch releases; the last commanded target pose is held.

Orientation is not tracked here at all -- compute_target() carries the
reference EE orientation through unchanged as a placeholder; bridge_node.py's
_pointing_pose() always overrides it with a fixed pose (see that module's
docstring). Clutch used to also support absolute/relative controller-tracked
orientation modes with a configurable controller-to-tool axis mapping, but
that machinery was deleted (2026-09-26) once the fixed-orientation policy
made it permanently dead code -- see git history around 2026-09-17 for the
removed design.
"""

from __future__ import annotations

from dataclasses import dataclass


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

    def compute_target(self, controller_pose: Pose, translation_scale: float) -> Pose:
        """Return the target EE pose for the current controller pose.

        Must only be called while engaged. orientation is the reference EE
        orientation, unchanged -- callers that need a real commanded
        orientation must compute it themselves (see bridge_node.py's
        _pointing_pose()).
        """
        assert self.engaged and self._ref_controller is not None and self._ref_ee is not None

        delta_pos = tuple(
            (c - r) * translation_scale
            for c, r in zip(controller_pose.position, self._ref_controller.position)
        )
        target_pos = tuple(
            e + d for e, d in zip(self._ref_ee.position, delta_pos)
        )
        return Pose(position=target_pos, orientation=self._ref_ee.orientation)
