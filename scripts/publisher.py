#!/usr/bin/env python3
"""
Read pose lines from PrecisionLander and send MAVLink LANDING_TARGET messages.

Expected stdin format from main.cpp:
    x y z distance angle_x angle_y

The C++ estimator writes raw camera-frame pose values. This script converts those
values into ArduPilot BODY_FRD coordinates and sends position-based LANDING_TARGET
messages to Pixhawk through MAVLink.
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time
from pathlib import Path

from pymavlink import mavutil


DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 57600

SOURCE_SYSTEM = 2
SOURCE_COMPONENT = 191

FRAME_BODY_FRD = getattr(mavutil.mavlink, "MAV_FRAME_BODY_FRD", 12)
TARGET_TYPE_FIDUCIAL = getattr(
    mavutil.mavlink,
    "MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL",
    2,
)


def map_camera_to_body(raw_x: float, raw_y: float, raw_z: float) -> tuple[float, float, float]:
    """
    Convert mounted-camera pose output into ArduPilot BODY_FRD coordinates.

    ArduPilot BODY_FRD expects:
        +X = drone forward
        +Y = drone right
        +Z = drone down

    Mounted-camera polarity test:
        drone front = raw -X
        drone right = raw -Y
        drone down  = raw -Z

    Therefore all three raw axes are inverted before sending LANDING_TARGET.
    """

    return -raw_x, -raw_y, -raw_z


def parse_args() -> tuple[str, int]:
    port = DEFAULT_PORT
    baud = DEFAULT_BAUD

    if len(sys.argv) >= 2:
        port = sys.argv[1]

    if len(sys.argv) >= 3:
        baud = int(sys.argv[2])

    return port, baud


def open_mavlink_connection(port: str, baud: int):
    print(f"Connecting to Pixhawk on {port} at {baud} baud...", file=sys.stderr)

    master = mavutil.mavlink_connection(
        port,
        baud=baud,
        source_system=SOURCE_SYSTEM,
        source_component=SOURCE_COMPONENT,
    )

    print("Waiting for Pixhawk heartbeat...", file=sys.stderr)
    heartbeat = master.wait_heartbeat(timeout=15)

    if heartbeat is None:
        print("No heartbeat received. Exiting.", file=sys.stderr)
        sys.exit(1)

    print("Heartbeat received!", file=sys.stderr)
    print(f"Target system: {master.target_system}", file=sys.stderr)
    print(f"Target component: {master.target_component}", file=sys.stderr)

    return master


def create_log_writer():
    project_root = Path(__file__).resolve().parents[1]
    log_dir = project_root / "logs"
    os.makedirs(log_dir, exist_ok=True)

    log_filename = time.strftime("pose_mavlink_log_%Y%m%d_%H%M%S.csv")
    log_path = log_dir / log_filename

    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)

    log_writer.writerow(
        [
            "time_usec",
            "raw_x",
            "raw_y",
            "raw_z",
            "distance",
            "angle_x",
            "angle_y",
            "body_x",
            "body_y",
            "body_z",
            "position_valid",
            "frame",
        ]
    )

    print(f"Logging sent values to: {log_path}", file=sys.stderr)
    return log_file, log_writer


def parse_pose_line(line: str):
    parts = line.strip().split()

    if len(parts) != 6:
        return None

    try:
        values = tuple(map(float, parts))
    except ValueError:
        return None

    if any(math.isnan(value) or math.isinf(value) for value in values):
        return None

    return values


def send_landing_target(
    master,
    time_usec: int,
    body_x: float,
    body_y: float,
    body_z: float,
    distance: float,
    angle_x: float,
    angle_y: float,
) -> None:
    master.mav.landing_target_send(
        time_usec,             # time_usec
        0,                     # target_num
        FRAME_BODY_FRD,        # frame
        angle_x,               # angle_x
        angle_y,               # angle_y
        distance,              # distance
        0.0,                   # size_x
        0.0,                   # size_y
        body_x,                # x
        body_y,                # y
        body_z,                # z
        [1.0, 0.0, 0.0, 0.0],  # q quaternion, unknown orientation
        TARGET_TYPE_FIDUCIAL,  # type
        1,                     # position_valid
    )


def main() -> int:
    port, baud = parse_args()
    master = open_mavlink_connection(port, baud)
    log_file, log_writer = create_log_writer()

    print("Ready. Waiting for pose lines from PrecisionLander...", file=sys.stderr)

    sent_count = 0

    try:
        for line in sys.stdin:
            pose = parse_pose_line(line)
            if pose is None:
                continue

            raw_x, raw_y, raw_z, distance, angle_x, angle_y = pose
            body_x, body_y, body_z = map_camera_to_body(raw_x, raw_y, raw_z)
            time_usec = int(time.time() * 1_000_000)

            send_landing_target(
                master,
                time_usec,
                body_x,
                body_y,
                body_z,
                distance,
                angle_x,
                angle_y,
            )

            log_writer.writerow(
                [
                    time_usec,
                    raw_x,
                    raw_y,
                    raw_z,
                    distance,
                    angle_x,
                    angle_y,
                    body_x,
                    body_y,
                    body_z,
                    1,
                    FRAME_BODY_FRD,
                ]
            )
            log_file.flush()

            sent_count += 1

            if sent_count % 10 == 0:
                print(
                    f"Sent LANDING_TARGET #{sent_count}: "
                    f"body_x={body_x:.3f}, body_y={body_y:.3f}, body_z={body_z:.3f}, "
                    f"distance={distance:.3f}, "
                    f"angle_x={angle_x:.3f}, angle_y={angle_y:.3f}",
                    file=sys.stderr,
                )

    finally:
        log_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
