import cv2
import os
import time


OUTPUT_DIR = "data/calibration_images"

# OpenCV uses INNER corners, not number of squares.
# Example: 10 squares x 7 squares = 9 x 6 inner corners.
CHECKERBOARD = (8, 5)

TARGET_IMAGE_COUNT = 40
MIN_SECONDS_BETWEEN_SAVES = 1.0

os.makedirs(OUTPUT_DIR, exist_ok=True)

pipeline = (
    "libcamerasrc ! "
    "video/x-raw,width=1280,height=800,framerate=30/1 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=true sync=false"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    raise SystemExit(1)

print("Camera opened.")
print("Move the checkerboard around the image.")
print("The script will save images only when checkerboard corners are detected.")
print(f"Target image count: {TARGET_IMAGE_COUNT}")
print("Press Ctrl+C to stop.")

saved_count = len([
    name for name in os.listdir(OUTPUT_DIR)
    if name.lower().endswith((".jpg", ".png"))
])

last_save_time = 0.0

try:
    while saved_count < TARGET_IMAGE_COUNT:
        ret, frame = cap.read()

        if not ret:
            print("WARNING: Failed to read frame.")
            time.sleep(0.2)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        now = time.time()

        if found and (now - last_save_time) >= MIN_SECONDS_BETWEEN_SAVES:
            filename = f"calib_{saved_count:03d}.jpg"
            path = os.path.join(OUTPUT_DIR, filename)

            cv2.imwrite(path, frame)
            saved_count += 1
            last_save_time = now

            print(f"Saved {saved_count}/{TARGET_IMAGE_COUNT}: {path}")

        elif found:
            print("Checkerboard found, waiting before next save...", end="\r")

        else:
            print("Checkerboard not found. Adjust board position/angle.", end="\r")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    cap.release()
    print(f"\nTotal saved images: {saved_count}")
