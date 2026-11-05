from pymavlink import mavutil
import sys
import time
import math
import csv
import os

# PORT, BAUD
PORT = "/dev/serial0"
BAUD = 57600

if len(sys.argv) >= 2:
    PORT = sys.argv[1]

if len(sys.argv) >= 3:
    BAUD = int(sys.argv[2])

print(f"Connecting to Pixhawk on {PORT} at {BAUD} baud...", file=sys.stderr)

master = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=2,
    source_component=191
)

print("Waiting for Pixhawk heartbeat...", file=sys.stderr)
heartbeat = master.wait_heartbeat(timeout=15)

if heartbeat is None:
    print("No heartbeat received. Exiting.", file=sys.stderr)
    sys.exit(1)

print("Heartbeat received!", file=sys.stderr)
print(f"Target system: {master.target_system}", file=sys.stderr)
print(f"Target component: {master.target_component}", file=sys.stderr)

FRAME_BODY_FRD = getattr(mavutil.mavlink, "MAV_FRAME_BODY_FRD", 12)
TARGET_TYPE_FIDUCIAL = getattr(
    mavutil.mavlink,
    "MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL",
    2
)

sent_count = 0

# Adding Coordinate Log for debugging

log_dir = "../logs"
os.makedirs(log_dir, exist_ok=True)

log_filename = time.strftime("pose_mavlink_log_%Y%m%d_%H%M%S.csv")
log_path = os.path.join(log_dir, log_filename)

log_file = open(log_path, "w", newline="")
log_writer = csv.writer(log_file)

log_writer.writerow([
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
    "frame"
])

print("Ready. Waiting for pose lines from PrecisionLander...", file=sys.stderr)
print(f"Logging sent values to: {log_path}", file=sys.stderr)

# main loop
for line in sys.stdin:
    line = line.strip()

    if not line:
        continue

    parts = line.split()

    if len(parts) != 6:
        # Ignore non-pose lines like "No ArUco in FOV" or camera logs.
        continue

    try:
        x, y, z, distance, angle_x, angle_y = map(float, parts)
    except ValueError:
        continue

    # Skip invalid values.
    values = [x, y, z, distance, angle_x, angle_y]
    if any(math.isnan(v) or math.isinf(v) for v in values):
        continue

    time_usec = int(time.time() * 1_000_000)

    body_x = x
    body_y = y
    body_z = z

    # Send x/y/z position-based LANDING_TARGET.
    # position_valid = 1 means the x, y, z fields are being used.
    #
    # Still raw coordinates, need to be convert axis for the body.
    master.mav.landing_target_send(
        time_usec,             # time_usec
        0,                     # target_num
        FRAME_BODY_FRD,        # frame
        angle_x,               # angle_x
        angle_y,               # angle_y
        distance,              # distance
        0.0,                   # size_x
        0.0,                   # size_y
        body_x,                     # x
        body_y,                     # y
        body_z,                     # z
        [1.0, 0.0, 0.0, 0.0],  # q quaternion, unknown orientation
        TARGET_TYPE_FIDUCIAL,  # type
        1                      # position_valid
    )
    log_writer.writerow([
        time_usec,
        x,
        y,
        z,
        distance,
        angle_x,
        angle_y,
        body_x,
        body_y,
        body_z,
        1,
        FRAME_BODY_FRD
    ])

    log_file.flush()

    sent_count += 1

    if sent_count % 10 == 0:
        print(
            f"Sent LANDING_TARGET #{sent_count}: "
            f"body_x={body_x:.3f}, body_y={body_y:.3f}, body_z={body_z:.3f}, "
            f"distance={distance:.3f}, "
            f"angle_x={angle_x:.3f}, angle_y={angle_y:.3f}",
            file=sys.stderr
        )
