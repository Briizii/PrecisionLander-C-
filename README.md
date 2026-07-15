[README_PrecisionLander(1).md](https://github.com/user-attachments/files/30040094/README_PrecisionLander.1.md)
# PrecisionLander

PrecisionLander is a Raspberry Pi + Pixhawk precision-landing system that detects a visual landing marker and sends MAVLink `LANDING_TARGET` messages to ArduPilot. The system uses a C++ multi-stage fractal/ArUco marker estimator on the Raspberry Pi and a Python MAVLink publisher that forwards the estimated target position to the flight controller.

The implementation is designed for a mounted downward-facing camera, a Pixhawk running ArduPilot/ArduCopter, and a Raspberry Pi connected to the Pixhawk through a MAVLink serial link.

> **Safety note**  
> This project interacts with real flight-control hardware. Always test with propellers removed first. Do not perform flight tests unless the aircraft is supervised, in a safe open area, and a pilot is ready to take over manually.

---

## System Overview

Runtime data flow:

```text
OV9281 camera on Raspberry Pi
→ C++ fractal/ArUco marker detector
→ pose output: x y z distance angle_x angle_y
→ Python MAVLink publisher
→ Pixhawk / ArduPilot
→ LANDING_TARGET precision landing
```

Main runtime command on the Raspberry Pi:

```bash
cd ~/dev/PrecisionLander/build
./PrecisionLander | ../.venv/bin/python -u ../scripts/publisher.py /dev/serial0 57600
```

The C++ program writes numeric pose values to `stdout`. The Python publisher reads those values from `stdin`, maps the camera coordinates into ArduPilot BODY_FRD coordinates, and sends `LANDING_TARGET` through MAVLink.

---

## Hardware Requirements

This repository is intended for the following hardware setup:

- Raspberry Pi running Raspberry Pi OS, preferably 64-bit.
- OV9281 monochrome/global-shutter CSI camera.
- Pixhawk V6X, or similar Pixhawk-compatible flight controller.
- ArduPilot / ArduCopter firmware.
- Raspberry Pi UART connected to Pixhawk TELEM2.
- Windows laptop running Mission Planner.
- Fractal/ArUco landing marker.

### Camera note

This project uses an **OV9281 camera**, not an official Raspberry Pi camera module. Because of that, normal OpenCV camera access such as:

```cpp
cv::VideoCapture camera(0);
```

may not work reliably.

The project uses a GStreamer + libcamera pipeline instead:

```cpp
"libcamerasrc ! "
"video/x-raw,width=1280,height=800,framerate=30/1 ! "
"videoconvert ! "
"video/x-raw,format=BGR ! "
"appsink drop=true sync=false"
```

This is expected for the OV9281 + Raspberry Pi setup used in this project.

---

## Software Stack

### Raspberry Pi

- Raspberry Pi OS.
- CMake.
- C++ compiler, such as `g++`.
- OpenCV C++.
- ArUco C++ library.
- GStreamer.
- libcamera / rpicam stack.
- Python 3.
- Python virtual environment.
- `pymavlink`.
- `pyserial`.

### Local computer

- Windows laptop.
- Mission Planner.
- SSH client for accessing the Raspberry Pi.
- Optional C++ IDE, such as CLion or VS Code, for development.

### Flight controller

- Pixhawk-compatible board.
- ArduPilot / ArduCopter.
- MAVLink serial connection through TELEM2 or another configured telemetry port.

---

## Repository Layout

Typical important files:

```text
PrecisionLander/
├── assets/
│   └── camera_result.xml          # camera calibration file
├── build/                         # CMake build folder, usually not committed
├── logs/                          # publisher logs, usually not committed
├── scripts/
│   └── publisher.py               # reads pose from stdin and sends LANDING_TARGET
├── src/
│   └── main.cpp                   # C++ camera + marker detector
├── CMakeLists.txt
└── README.md
```

---

## Raspberry Pi Setup

### Update system packages

```bash
sudo apt update
sudo apt upgrade -y
```

### Install build tools and Python tools

```bash
sudo apt install -y \
  git \
  cmake \
  build-essential \
  pkg-config \
  python3 \
  python3-pip \
  python3-venv
```

### Install OpenCV and GStreamer dependencies

```bash
sudo apt install -y \
  libopencv-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libcamera
```

If `gstreamer1.0-libcamera` is unavailable on your Raspberry Pi OS version, check the OS release and the installed libcamera/rpicam stack. Package names can vary between Raspberry Pi OS releases.

### Install libcamera/rpicam tools

Many recent Raspberry Pi OS images already include the camera stack. If not, install:

```bash
sudo apt install -y libcamera-tools rpicam-apps
```

Depending on the OS version, the camera command may be called `rpicam-hello` or `libcamera-hello`.

Check camera detection:

```bash
rpicam-hello --list-cameras
```

or:

```bash
libcamera-hello --list-cameras
```

### Test the camera through GStreamer

```bash
gst-launch-1.0 libcamerasrc ! video/x-raw,width=1280,height=800,framerate=30/1 ! videoconvert ! fakesink
```

Fix camera, ribbon cable, or libcamera issues before building the project if this command fails.

---

## ArUco C++ Library

This project was developed with ArUco `3.1.12`.

Example source installation:

```bash
cd ~
git clone https://github.com/PlusToolkit/aruco.git
cd aruco
git checkout 3.1.12
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
sudo cmake --install .
sudo ldconfig
```

Use an ArUco version compatible with the project source code. If your system uses a different ArUco package or repository, adjust the installation command accordingly.

---

## Clone and Build on Raspberry Pi

Clone the repository:

```bash
mkdir -p ~/dev
cd ~/dev
git clone https://github.com/<your-username>/<your-repo>.git PrecisionLander
cd PrecisionLander
```

Create the Python virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install pymavlink pyserial
```

If the repository includes a `requirements.txt`, install from it instead:

```bash
pip install -r requirements.txt
```

Build the C++ detector:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j$(nproc)
```

Run the detector only:

```bash
cd ~/dev/PrecisionLander/build
./PrecisionLander
```

Expected numeric pose output when a marker is detected:

```text
x y z distance angle_x angle_y
```

Example:

```text
0.040 0.003 0.295 0.297 0.137 0.009
```

Keep C++ `stdout` numeric-only because `publisher.py` parses it. Send debug messages to `stderr`.

---

## Camera Calibration

The project expects a camera calibration file such as:

```text
assets/camera_result.xml
```

The provided calibration workflow should match the camera and resolution used by the runtime pipeline. The tested camera stream resolution is:

```text
1280 x 800
```

Do not reuse calibration from another resolution unless you understand the effect. Incorrect calibration can cause incorrect target position and distance estimates.

---

## Marker Size Configuration

The C++ estimator depends on the physical marker size. Update the marker constants in the C++ source to match the printed landing marker.

Example marker configuration:

```cpp
MARKER_SIZE = 0.138f;
```

Example multi-stage marker size ratios:

```cpp
id 0 = MARKER_SIZE
id 1 = MARKER_SIZE * 0.431f
id 2 = MARKER_SIZE * 0.143f
```

Measure the printed marker physically before flight testing. Incorrect marker size will scale the estimated `x`, `y`, and `z` values incorrectly.

---

## Pixhawk and ArduPilot Setup

### Physical connection

Typical connection:

```text
Raspberry Pi GPIO UART  →  Pixhawk TELEM2
```

The default runtime connection used by the publisher is:

```text
serial device: /dev/serial0
baud rate: 57600
```

### Pixhawk serial parameters

In Mission Planner, open:

```text
Config/Tuning → Full Parameter Tree
```

For TELEM2, check:

```text
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD     = 57
```

Meaning:

```text
2  = MAVLink2
57 = 57600 baud
```

If a different telemetry port is used, configure the corresponding serial parameter group, such as `SERIAL1`, `SERIAL3`, or another available port.

### Precision landing parameters

Common starting parameters:

```text
PLND_ENABLED     = 1
PLND_TYPE        = 1
PLND_XY_DIST_MAX = 0
```

Parameters commonly reviewed during setup and tuning:

```text
PLND_STRICT
PLND_ALT_MIN
PLND_ALT_MAX
PLND_RET_MAX
PLND_RET_BEHAVE
PLND_OPTIONS
LAND_SPD_MS
LAND_SPD_HIGH_MS
PLND_CAM_POS_X
PLND_CAM_POS_Y
PLND_CAM_POS_Z
PLND_YAW_ALIGN
PLND_LAND_OFS_X
PLND_LAND_OFS_Y
```

Save a parameter backup in Mission Planner before changing real Pixhawk parameters.

---

## Mission Planner Connection

Connect the Pixhawk to the laptop through USB.

In Mission Planner:

```text
Select Pixhawk COM port, for example COM5
Baud rate: 115200
Click CONNECT
```

After connecting, check that Mission Planner is reading the real vehicle. The attitude display should move when the drone body is gently tilted.

---

## Run with Real Pixhawk

On the Raspberry Pi:

```bash
cd ~/dev/PrecisionLander/build
./PrecisionLander | ../.venv/bin/python -u ../scripts/publisher.py /dev/serial0 57600
```

In Mission Planner, open:

```text
Flight Data → MAVLink Inspector
```

Expected MAVLink Inspector path:

```text
Vehicle 2
→ Comp 191
→ LANDING_TARGET
```

Expected `LANDING_TARGET` fields:

```text
frame = 12
position_valid = 1
type = 2
x/y/z change when the marker or mounted camera moves
```

---

## Coordinate Frame and Mapping

ArduPilot expects position-based `LANDING_TARGET` in BODY_FRD:

```text
+X = drone forward
+Y = drone right
+Z = drone down
```

The mounted camera mapping used by this repository is:

```python
def map_camera_to_body(raw_x, raw_y, raw_z):
    body_x = -raw_x
    body_y = -raw_y
    body_z = -raw_z
    return body_x, body_y, body_z
```

If the camera is mounted differently, repeat the polarity test and adjust only `map_camera_to_body()` inside:

```text
scripts/publisher.py
```

### Polarity test

Test with propellers removed and the drone disarmed.

| Marker position relative to drone | Expected `LANDING_TARGET` |
|---|---:|
| Toward drone front | `x` positive |
| Toward drone back | `x` negative |
| Toward drone right | `y` positive |
| Toward drone left | `y` negative |
| Farther below camera / drone higher | `z` positive |
| Closer to camera / drone lower | `z` smaller |

Do not flight test until the polarity is correct.

---

## Camera Position Offset

After the camera is mounted, measure its position relative to the drone body center.

Use the measurements to configure the ArduPilot precision-landing camera offset parameters:

```text
PLND_CAM_POS_X
PLND_CAM_POS_Y
PLND_CAM_POS_Z
```

Use the ArduPilot body-frame convention:

```text
+X = forward
+Y = right
+Z = down
```

The exact values depend on the final camera mounting location.

---

## Debug Video Stream

The project can stream a debug video over TCP from the Raspberry Pi.

Example stream pipeline concept:

```cpp
"appsrc ! videoconvert ! video/x-raw,format=I420 ! "
"x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 key-int-max=15 bframes=0 ! "
"h264parse config-interval=1 ! mpegtsmux ! "
"tcpserversink host=0.0.0.0 port=8554 sync=false"
```

On the laptop, open VLC and connect to:

```text
tcp://<raspberry-pi-ip>:8554
```

---

## Troubleshooting

### Camera cannot open

Stop old processes:

```bash
pkill -f PrecisionLander
pkill -f gst-launch
pkill -f rpicam
pkill -f libcamera
```

Check camera detection:

```bash
rpicam-hello --list-cameras
```

or:

```bash
libcamera-hello --list-cameras
```

Test GStreamer:

```bash
gst-launch-1.0 libcamerasrc ! video/x-raw,width=1280,height=800,framerate=30/1 ! videoconvert ! fakesink
```

If the camera was recently mounted, check the CSI ribbon cable.

### Vehicle 2 does not appear in Mission Planner

Check:

- The full pipeline is running, not only `./PrecisionLander`.
- The TELEM2 cable is connected.
- `publisher.py` is running.
- Mission Planner is connected to the real Pixhawk.
- `SERIAL2_PROTOCOL` and `SERIAL2_BAUD` are correct for the Pixhawk port used.
- The Raspberry Pi is using the correct serial device, usually `/dev/serial0`.

Fake constant target test:

```bash
cd ~/dev/PrecisionLander
while true; do
  echo "0 0 1 1 0 0"
  sleep 0.2
done | .venv/bin/python -u scripts/publisher.py /dev/serial0 57600
```

If Vehicle 2 appears with this test, the MAVLink serial connection works.

### LANDING_TARGET appears but does not change

Check whether the C++ estimator output changes:

```bash
cd ~/dev/PrecisionLander/build
./PrecisionLander
```

If C++ values change but Mission Planner does not, use `tee` to confirm the pipe:

```bash
cd ~/dev/PrecisionLander/build
./PrecisionLander | tee /tmp/pl_pipe_values.txt | ../.venv/bin/python -u ../scripts/publisher.py /dev/serial0 57600
```

In another terminal:

```bash
tail -f /tmp/pl_pipe_values.txt
```

### Wrong x/y/z direction

Edit only:

```text
scripts/publisher.py → map_camera_to_body()
```

Do not change the C++ estimator to fix body-frame polarity.

---

## Flight Test Guidance

Before any real flight:

- Remove propellers for all bench tests.
- Confirm `LANDING_TARGET` in Mission Planner.
- Confirm BODY_FRD polarity.
- Confirm marker size.
- Confirm camera offset.
- Backup Pixhawk parameters.
- Test with a pilot ready to take over.

Recommended first landing offsets:

```text
0.3 m → 0.5 m → 1.0 m
```

Avoid starting with a large offset even if software tests are successful.

---

## Development Notes

- Keep production `stdout` numeric-only from the C++ program.
- Send debug messages to `stderr`.
- Do not leave temporary marker diagnostics enabled in production unless needed.
- Prefer small, reversible changes.
- Backup working checkpoints before major changes.
- Tune anti-overcorrection behavior based on logged behavior, not guesses.

Suggested local checkpoint names:

```text
main_MULTISTAGE_MARKER_WORKING_CLEAN.cpp
main_MULTISTAGE_MARKER_MOUNTED_BODY_FRD_CONFIRMED.cpp
publisher_MOUNTED_CAMERA_BODY_FRD_CONFIRMED.py
```

---

## License

Add the chosen license before publishing the repository.

Common options:

- MIT License
- Apache License 2.0
- GPLv3

---

## Disclaimer

This repository is for research and development. It is not a certified autonomous landing system. Use carefully, test gradually, and follow local drone regulations and safety procedures.
