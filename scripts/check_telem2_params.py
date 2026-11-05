from pymavlink import mavutil
import sys
import time

PORT = "/dev/serial0"
BAUD = 57600

if len(sys.argv) >= 2:
    PORT = sys.argv[1]

if len(sys.argv) >= 3:
    BAUD = int(sys.argv[2])

params_to_check = [
    "SERIAL2_PROTOCOL",
    "SERIAL2_BAUD",
    "BRD_SER2_RTSCTS",
]

print(f"Connecting to {PORT} at {BAUD}...")
master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("Waiting for heartbeat...")
heartbeat = master.wait_heartbeat(timeout=15)

if heartbeat is None:
    print("No heartbeat received.")
    sys.exit(1)

print("Heartbeat received.")
print("Requesting TELEM2-related parameters...\n")

for param_name in params_to_check:
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode("utf-8"),
        -1
    )

    start = time.time()
    found = False

    while time.time() - start < 5:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)

        if msg is None:
            continue

        received_name = msg.param_id
        if isinstance(received_name, bytes):
            received_name = received_name.decode("utf-8")

        received_name = received_name.strip("\x00")

        if received_name == param_name:
            print(f"{param_name} = {msg.param_value}")
            found = True
            break

    if not found:
        print(f"{param_name} = not found / no response")
