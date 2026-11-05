from pymavlink import mavutil
import sys

PORT = "/dev/serial0"
BAUD = 57600

if len(sys.argv) >= 2:
    PORT = sys.argv[1]

if len(sys.argv) >= 3:
    BAUD = int(sys.argv[2])

print(f"Connecting to Pixhawk on {PORT} at {BAUD} baud...")

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("Waiting for heartbeat...")
heartbeat = master.wait_heartbeat(timeout=15)

if heartbeat is None:
    print("No heartbeat received.")
    print("Try another baud rate, or check TELEM2 settings / TX-RX wiring.")
    sys.exit(1)

print("Heartbeat received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")
print(f"Autopilot type: {heartbeat.autopilot}")
print(f"Vehicle type: {heartbeat.type}")
