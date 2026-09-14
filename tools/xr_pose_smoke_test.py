#!/usr/bin/env python
# SPDX-License-Identifier: Apache-2.0
"""Minimal Quest 3 controller pose/button reader -- NO robot code involved.

Ported from sim2real-rebot-devarm/scripts/xr_pose_print_test.py. Kept
byte-for-byte in spirit: this is the original, independently-verified
Quest3 -> GX10 -> NVIDIA CloudXR -> Isaac Teleop smoke test. It is
deliberately NOT part of the ROS 2 build (ros2_ws/) -- it predates and is
independent of quest_rebot_bridge, and exists here only as a standalone
connectivity check. It never imports lerobot, never opens a serial/CAN
port, and never sends anything to a motor -- it only reads controller state
from `isaacteleop` and prints it.

Data path exercised:
    Quest 3 (WebXR browser) --CloudXR (WiFi or --adb-open)--> CloudXR runtime
        (this process, native aarch64 libs bundled in the `isaacteleop` wheel)
        -> OpenXR session -> isaacteleop retargeting engine (ControllersSource)
        -> this script (prints grip/aim pose + buttons for both controllers)

Install (aarch64 wheels exist for this exact combo -- verified on the GX10,
Python 3.12, manylinux_2_35_aarch64):

    python3 -m venv .venv-xrtest && source .venv-xrtest/bin/activate
    pip install "isaacteleop[cloudxr]~=1.3.131"
    python -m isaacteleop.cloudxr --accept-eula   # one-time EULA accept

Usage:
    python tools/xr_pose_smoke_test.py
    python tools/xr_pose_smoke_test.py --adb-open     # auto-open the CloudXR
                                                       # client on the headset via adb
    python tools/xr_pose_smoke_test.py --hz 10

Ctrl+C to stop (cleanly disconnects the session and stops the CloudXR runtime).
"""

from __future__ import annotations

import argparse
import shutil
import socket
import subprocess
import sys
import time

import numpy as np

CLOUDXR_WEB_CLIENT_URL = "https://nvidia.github.io/IsaacTeleop/client"
CLOUDXR_WSS_PORT = 48322


def _primary_ipv4() -> str | None:
    """Best-guess outbound IPv4 via the UDP-socket trick (no packets actually sent)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return None


def print_connect_instructions() -> None:
    ip = _primary_ipv4() or "<check `hostname -I` / `ip addr`>"
    print("\n" + "=" * 76)
    print("Connect the Quest 3 to this workstation over NVIDIA CloudXR:")
    print(f"  1. In the headset browser, open:  {CLOUDXR_WEB_CLIENT_URL}")
    print(f"  2. Enter this workstation's IP:   {ip}")
    print(f"  3. Accept the self-signed cert at https://{ip}:{CLOUDXR_WSS_PORT}/ , then Connect.")
    print("=" * 76 + "\n")


def adb_open_on_headset() -> None:
    """Best-effort: launch the CloudXR client URL on the headset via `adb shell am start`."""
    if not shutil.which("adb"):
        print("[xr-test] --adb-open requested but `adb` is not on PATH; skipping.")
        return
    ip = _primary_ipv4()
    if ip is None:
        print("[xr-test] --adb-open: could not determine this host's IP; open the URL manually.")
        return
    url = f"{CLOUDXR_WEB_CLIENT_URL}?serverIP={ip}&port={CLOUDXR_WSS_PORT}"
    print(f"[xr-test] adb: launching {url} on the headset...")
    proc = subprocess.run(
        ["adb", "shell", "am", "start", "-a", "android.intent.action.VIEW", "-d", url],
        capture_output=True,
        text=True,
        timeout=15,
    )
    if proc.returncode != 0:
        print(f"[xr-test] adb am start failed (rc={proc.returncode}): "
              f"{(proc.stderr or proc.stdout).strip()}")
        print("[xr-test] Open the URL on the headset manually instead.")
    else:
        print("[xr-test] adb: launch command sent. Accept the cert warning on the headset "
              "if this is the first connection, then tap Connect.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hz", type=float, default=10.0, help="Print rate")
    parser.add_argument(
        "--adb-open", action="store_true",
        help="Auto-launch the CloudXR client URL on the headset via `adb shell am start`",
    )
    parser.add_argument(
        "--accept-eula", action="store_true",
        help="Accept the NVIDIA CloudXR EULA non-interactively (only needed once ever; "
             "same effect as `python -m isaacteleop.cloudxr --accept-eula`)",
    )
    args = parser.parse_args()

    try:
        from isaacteleop.cloudxr import CloudXRLauncher
        from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
        from isaacteleop.retargeting_engine.interface import (
            ExecutionEvents,
            ExecutionState,
            OutputCombiner,
        )
        from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex
        from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig
    except ImportError as e:
        sys.exit(
            "FAILED: `isaacteleop` is not importable.\n"
            f"  {e}\n\n"
            "Install it first (aarch64 wheels exist for this GX10/Python combo):\n"
            '  pip install "isaacteleop[cloudxr]~=1.3.131"\n'
            "  python -m isaacteleop.cloudxr --accept-eula   # one-time\n"
        )

    print("[xr-test] launching CloudXR runtime (first run may take ~30s)...")
    launcher = CloudXRLauncher(accept_eula=args.accept_eula)

    controllers = ControllersSource(name="controllers")
    pipeline = OutputCombiner(
        {
            "left": controllers.output(ControllersSource.LEFT),
            "right": controllers.output(ControllersSource.RIGHT),
        }
    )
    session_config = TeleopSessionConfig(app_name="RebotXRPoseTest", pipeline=pipeline)

    dt = 1.0 / args.hz
    running_events = ExecutionEvents(execution_state=ExecutionState.RUNNING, reset=False)

    def fmt_hand(label: str, controller) -> str:
        if getattr(controller, "is_none", False):
            return f"{label}: not tracked"
        try:
            pos = np.asarray(controller[ControllerInputIndex.GRIP_POSITION], dtype=float)
            quat = np.asarray(controller[ControllerInputIndex.GRIP_ORIENTATION], dtype=float)
            squeeze = float(controller[ControllerInputIndex.SQUEEZE_VALUE])
            trigger = float(controller[ControllerInputIndex.TRIGGER_VALUE])
            primary = bool(controller[ControllerInputIndex.PRIMARY_CLICK])
            secondary = bool(controller[ControllerInputIndex.SECONDARY_CLICK])
            menu = bool(controller[ControllerInputIndex.MENU_CLICK])
            thumb_click = bool(controller[ControllerInputIndex.THUMBSTICK_CLICK])
            thumb_x = float(controller[ControllerInputIndex.THUMBSTICK_X])
            thumb_y = float(controller[ControllerInputIndex.THUMBSTICK_Y])
        except (IndexError, KeyError, TypeError, ValueError) as e:
            return f"{label}: read error ({e})"
        return (
            f"{label}: pos={np.round(pos, 3).tolist()} quat_xyzw={np.round(quat, 3).tolist()} "
            f"squeeze={squeeze:.2f} trigger={trigger:.2f} "
            f"A/X={primary} B/Y={secondary} menu={menu} "
            f"stick=({thumb_x:+.2f},{thumb_y:+.2f}) stick_click={thumb_click}"
        )

    session = None
    try:
        print_connect_instructions()
        if args.adb_open:
            adb_open_on_headset()

        # xrGetSystem() (opened inside TeleopSession.__enter__) fails with
        # XR_ERROR_FORM_FACTOR_UNAVAILABLE (-35) until the headset has actually
        # started connecting to this CloudXR runtime -- verified empirically on this
        # GX10: entering the session immediately after CloudXR launch, with no
        # headset attempting a connection, raises this every time. So open the
        # session lazily and retry while the operator connects, instead of failing
        # once at startup like the reference examples assume.
        print("[xr-test] opening OpenXR session (retries until the headset starts "
              "connecting)... Ctrl+C to abort")
        last_msg = 0.0
        while session is None:
            try:
                session = TeleopSession(session_config)
                session.__enter__()
            except RuntimeError as e:
                session = None
                now = time.monotonic()
                if now - last_msg >= 5.0:
                    print(f"[xr-test] not ready yet ({e}); retrying...")
                    last_msg = now
                time.sleep(2.0)
        print("[xr-test] OpenXR session open.")

        print(f"[xr-test] streaming at {args.hz:.0f} Hz -- move the controllers. Ctrl+C to stop.")
        while True:
            t0 = time.monotonic()
            result = session.step(execution_events=running_events)
            info = session.last_step_info
            if info is not None and info.worker_exception is not None:
                raise RuntimeError("retargeting worker raised") from info.worker_exception
            print(fmt_hand("L", result["left"]) + "  |  " + fmt_hand("R", result["right"]))
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\n[xr-test] stopping.")
    finally:
        if session is not None:
            try:
                session.__exit__(None, None, None)
            except Exception:
                pass
        launcher.stop()
        print("[xr-test] CloudXR runtime stopped.")


if __name__ == "__main__":
    main()
