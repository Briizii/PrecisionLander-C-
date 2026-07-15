"""
@file publisher.py
@project PrecisionLander
@brief Python script to consume pose data from stdin and send MAVLink LANDING_TARGET messages.
@author [ADD AUTHOR NAME]
@date 2026-01-09
@license [ADD LICENSE TYPE, e.g., MIT, Apache 2.0]

Start in a pipeline with PrecisionLander. Consumes Pose and sends MAVLink LANDING_TARGET.
"""

import sys
from pymavlink import mavutil
import time

# # MAVLink connection
# print("\033[31m"+"Starting mavlink communication..."+"\033[0m")
# m = mavutil.mavlink_connection("/dev/ttyAMA0", 57600)
# try:
#     m.wait_heartbeat(timeout=5)
# except Exception:
#     pass
# print(f"\x1b[31mHeartbeat received from system: {m.target_system} and component: {m.target_component}\033[0m")

rate_hz = 50.0 # pick the hertz
period = 1.0 / rate_hz
next_t = time.time()

# main loop
for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    x_str, y_str, z_str, dist_str, angx_str, angy_str = line.split()
    x, y, z, distance, angle_x, angle_y = map(float, (x_str, y_str, z_str, dist_str, angx_str, angy_str))
    print(f"Got: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m, Distance={distance:.3f}m")
    tnow = time.time()
    if tnow >= next_t:
        next_t += period
        time_usec = int(time.time()*1e6)
        # m.mav.landing_target_send(
        #     time_usec,        # time_usec
        #     0,                      # target_num
        #     mavutil.mavlink.MAV_FRAME_BODY_FRD,
        #     float(angle_x), float(angle_y),
        #     float(distance),                    # distance (set 0 if using rangefinder)
        #     0.61, 0.61,               # size_x, size_y
        #     x,
        #     y,
        #     z,
        #     # q=[1.0, 0.0, 0.0, 0.0],   # orientation (unused here)
        #     type=mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        #     position_valid=1
        # )