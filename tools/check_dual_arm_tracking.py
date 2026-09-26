"""Simulation-only synthetic controller -> bridge -> IK -> TF verification.

Run dual_arm_rehearsal.launch.py rviz:=false on a separate ROS_DOMAIN_ID,
then run this script on that same domain, with the ROS overlay sourced.
Never run alongside a Quest publisher. Only *_sim joint topics are inspected.
"""
import time
import msgpack
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import ByteMultiArray
from tf2_ros import Buffer, TransformListener

from quest_rebot_bridge.bridge_node import _DOWN_POINTING_QUAT

rclpy.init()
node = Node('dual_arm_tracking_check')
buffer = Buffer()
listener = TransformListener(buffer, node)
pub = node.create_publisher(ByteMultiArray, '/xr_teleop/controller_data', 10)
calib = R.from_quat([.558549, -.433616, -.433616, .558549])
offset = R.from_quat([.5, .5, -.5, .5])
# Live teleop orientation is fixed straight down regardless of controller
# rotation (see bridge_node.py's module docstring / _DOWN_POINTING_QUAT) --
# imported rather than hand-derived so this test can't drift from the
# implementation. No thumbstick data is sent below, so roll_offset_rad stays
# at its 0 default throughout and this is the exact orientation expected
# while engaged.
fixed = R.from_quat(_DOWN_POINTING_QUAT)
# home_orientation (dual_arm_rehearsal.launch.py) now matches _DOWN_POINTING_QUAT
# exactly (2026-09-26) -- home and fixed are the same rotation by design, so
# engaging the clutch at the home pose needs zero reorientation. Kept as a
# separate name since it represents a different concept (the home-reset
# target, independent of live teleop) that happens to coincide numerically.
home = fixed
# Must match bridge_node.py's home_position default (2026-09-26: lowered from
# (0.3,0,0.3) to (0.3,0,0.10) -- IK probing found straight-down orientation
# only reachable below z~0.15-0.20 for this arm, see its module comment).
_HOME_POSITION = np.array([0.3, 0.0, 0.10])
rotations = {s: home for s in ('left', 'right')}
positions = {s: np.zeros(3) for s in rotations}


def tick(seconds, squeeze=1., primary=0., active=True):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        data = {}
        for side in rotations:
            data.update({f'{side}_grip_position': (calib.inv().apply(positions[side])).tolist(),
                         f'{side}_aim_orientation': (calib.inv() * rotations[side] * offset.inv()).as_quat().tolist(),
                         f'{side}_grip_orientation': [0., 0., 0., 1.],
                         f'{side}_squeeze_value': squeeze, f'{side}_trigger_value': 0.,
                         f'{side}_is_active': active, f'{side}_primary_click': primary})
        msg = ByteMultiArray()
        msg.data = [bytes([v]) for v in msgpack.packb(data)]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.01)
        time.sleep(.01)


def actual(side):
    t = buffer.lookup_transform(f'{side}/base_link', f'{side}/gripper_link', Time()).transform
    return np.array([t.translation.x, t.translation.y, t.translation.z]), R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])


def wait_until(predicate, desc, timeout_s=15., squeeze=1., primary=0.):
    """Tick (with the given squeeze/primary) until predicate() is true, or
    raise after timeout_s.

    home_orientation and the fixed-down teleop orientation are ~120deg apart
    (not the small few-degree gap the old controller-tracking design
    assumed both at clutch-engage and at home-reset), so both convergences
    take longer -- and more variably, since each safety_limits-clamped step
    only advances once its move_to_pose_ik round-trip completes -- than a
    flat tick() duration can reliably cover.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tick(.1, squeeze, primary)
        if predicate():
            return
    raise AssertionError(f'{desc} did not converge within {timeout_s}s')


def _at_fixed_orientation():
    return all((actual(s)[1].inv() * fixed).magnitude() < np.deg2rad(.5) for s in rotations)


try:
    tick(2., 0.)
    origins = {s: actual(s)[0] for s in rotations}
    wait_until(_at_fixed_orientation, 'engage -> fixed-down orientation')
    worst = 0.
    for axis in 'xyz':
        for sign in (1, -1):
            for side, direction in [('left', sign), ('right', -sign)]:
                # Rotating the synthetic controller here must NOT rotate the
                # EE -- this exercises that orientation stays fixed straight
                # down no matter how the controller is held.
                rotations[side] = R.from_euler(axis, direction * 12, degrees=True) * home
                positions[side] = np.array([0., direction * .01, .01])
            tick(2.)
            for side in rotations:
                p, r = actual(side)
                deg = (r.inv() * fixed).magnitude() * 180 / np.pi
                mm = np.linalg.norm(p - origins[side] - positions[side]) * 1000
                worst = max(worst, deg)
                print(f'{side} {axis} {sign:+}: orientation_dev_from_fixed_down={deg:.4f} deg position={mm:.4f} mm', flush=True)
                assert deg < .5 and mm < 1., (side, axis, sign, deg, mm)
    # Release must hold; changing controller pose while released must not move EE.
    held = {s: actual(s) for s in rotations}
    tick(.2, 0.)
    for s in rotations:
        rotations[s] = home
    tick(.5, 0.)
    for s in rotations:
        p, r = actual(s)
        assert np.linalg.norm(p-held[s][0]) < 1e-5
        assert (r.inv()*held[s][1]).magnitude() < 1e-5
    # Regrip must still show the fixed straight-down orientation (unaffected
    # by the controller's rotation) -- already converged since the previous
    # per-axis loop ended there, so this should resolve on the first poll.
    wait_until(_at_fixed_orientation, 'regrip -> fixed-down orientation')
    for s in rotations:
        p, r = actual(s)
        assert np.linalg.norm(p-held[s][0]) < .001
    # Input loss must hold as well.
    held = {s: actual(s) for s in rotations}
    until = time.monotonic()+.8
    while time.monotonic()<until:
        rclpy.spin_once(node, timeout_sec=.02)
    for s in rotations:
        p,r=actual(s)
        assert np.linalg.norm(p-held[s][0]) < 1e-5
        assert (r.inv()*held[s][1]).magnitude() < 1e-5
    # Home must finish at the configured gripper-frame pose. Orientation is
    # already fixed-down from the per-axis loop above, so only position needs
    # to move (home_orientation matches fixed exactly -- see top of file).
    tick(.1, 0., primary=1.)  # rising-edge press to trigger the reset

    def _at_home():
        return all(
            np.linalg.norm(actual(s)[0] - _HOME_POSITION) < .0051
            and (actual(s)[1].inv() * home).magnitude() < .0201
            for s in rotations
        )

    wait_until(_at_home, 'home reset', squeeze=0., primary=0.)
    for s in rotations:
        p, r = actual(s)
        # Match the bridge's existing home completion tolerances (5 mm, .02 rad).
        print(f'{s} home: position={np.linalg.norm(p-_HOME_POSITION)*1000:.3f} mm', flush=True)
    print(f'PASS: both arms, +/- XYZ rotations, translation, release, regrip, stale input, home; worst={worst:.4f} deg')
finally:
    node.destroy_node()
    rclpy.shutdown()
