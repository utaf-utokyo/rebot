"""Regression tests for the position clutch and the fixed-down pointing pose.

Clutch used to also carry absolute/relative controller-tracked orientation
modes (tested here as test_relative_rotation_uses_world_axis,
test_absolute_orientation_survives_regrip_and_translation_is_relative,
test_forward_and_up_axis_mapping) -- removed 2026-09-26 along with that dead
code once the fixed-orientation policy (see bridge_node.py) made it
permanently unreachable; see git history for the old tests/design.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from quest_rebot_bridge.clutch import Clutch, Pose


def pose(rotation, position=(0., 0., 0.)):
    return Pose(position, tuple(rotation.as_quat()))


def test_compute_target_tracks_relative_translation_only():
    ref_ee = pose(R.from_euler('x', 30, degrees=True), (0.2, 0., 0.3))
    clutch = Clutch()
    clutch.engage(pose(R.identity(), (1., 2., 3.)), ref_ee)
    actual = clutch.compute_target(pose(R.from_euler('z', 90, degrees=True), (1.5, 2., 3.)), 1.)
    np.testing.assert_allclose(actual.position, (0.7, 0., 0.3))
    # orientation is untouched (a placeholder -- bridge_node's _pointing_pose
    # is what actually decides the commanded orientation)
    assert actual.orientation == ref_ee.orientation


def test_missing_tf_does_not_substitute_home():
    from unittest.mock import Mock
    from tf2_ros import LookupException
    from quest_rebot_bridge.bridge_node import QuestRebotBridgeNode
    node = Mock()
    node._base_frame = 'left/base_link'
    node._ee_frame = 'left/gripper_link'
    node._tf_buffer.lookup_transform.side_effect = LookupException('missing')
    assert QuestRebotBridgeNode._lookup_ee_pose(node) is None


def test_old_ik_response_cannot_overwrite_new_clutch_reference():
    from unittest.mock import Mock
    from quest_rebot_bridge.bridge_node import QuestRebotBridgeNode
    node = Mock()
    node._command_epoch = 2
    node._last_target = 'new reference'
    future = Mock()
    QuestRebotBridgeNode._on_move_ik_response(node, future, 'old target', 1)
    assert node._last_target == 'new reference'
    future.result.assert_not_called()


def test_pointing_pose_ignores_controllers_orientation_entirely():
    from quest_rebot_bridge.bridge_node import _pointing_pose
    # Orientation is now fixed straight down regardless of what
    # compute_target() produced -- unlike the earlier "extract forward,
    # discard roll" design, even completely different forward directions
    # must produce the IDENTICAL _pointing_pose() result at the same
    # roll_rad, since pose.orientation is no longer consulted at all.
    arbitrary_1 = R.from_euler('xyz', [10, -20, 140], degrees=True)
    arbitrary_2 = R.from_euler('xyz', [-88, 5, 33], degrees=True)
    p1 = pose(arbitrary_1, (1., 2., 3.))
    p2 = pose(arbitrary_2, (1., 2., 3.))
    r1 = _pointing_pose(p1, 0.0)
    r2 = _pointing_pose(p2, 0.0)
    np.testing.assert_allclose(R.from_quat(r1.orientation).as_matrix(), R.from_quat(r2.orientation).as_matrix(), atol=1e-9)
    # and the forward (local +X) direction is always straight down (base_link -Z)
    np.testing.assert_allclose(R.from_quat(r1.orientation).apply([1, 0, 0]), [0., 0., -1.], atol=1e-9)
    np.testing.assert_allclose(r1.position, p1.position)


def test_pointing_pose_roll_rad_controls_roll_about_fixed_down_axis():
    from quest_rebot_bridge.bridge_node import _pointing_pose
    p = pose(R.from_euler('xyz', [10, -20, 140], degrees=True))
    down = np.array([0., 0., -1.])
    r0 = _pointing_pose(p, 0.0)
    r90 = _pointing_pose(p, np.radians(90))
    delta = R.from_quat(r90.orientation) * R.from_quat(r0.orientation).inv()
    angle = np.degrees(delta.magnitude())
    np.testing.assert_allclose(angle, 90.0, atol=1e-6)
    np.testing.assert_allclose(delta.apply(down), down, atol=1e-9)  # rotation axis is the fixed forward itself
