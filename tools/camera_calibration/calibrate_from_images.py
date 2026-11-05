import cv2
import numpy as np
import glob
import os


IMAGE_DIR = "data/calibration_images"
OUTPUT_DIR = "data/calibration_output"

# Checkerboard size
# 9 squares x 6 squares = 8 x 5 inner corners
CHECKERBOARD = (8, 5)

# One printed square size
# Example:
# 30 mm = 0.030
SQUARE_SIZE = 0.021

os.makedirs(OUTPUT_DIR, exist_ok=True)

criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)

object_points_template = np.zeros(
    (CHECKERBOARD[0] * CHECKERBOARD[1], 3),
    np.float32
)

object_points_template[:, :2] = np.mgrid[
    0:CHECKERBOARD[0],
    0:CHECKERBOARD[1]
].T.reshape(-1, 2)

object_points_template *= SQUARE_SIZE

object_points = []
image_points = []

image_paths = sorted(glob.glob(os.path.join(IMAGE_DIR, "*.jpg")))

if not image_paths:
    print(f"ERROR: No calibration images found in {IMAGE_DIR}")
    raise SystemExit(1)

print(f"Found {len(image_paths)} images.")

image_size = None
used_images = 0
failed_images = 0

for image_path in image_paths:
    image = cv2.imread(image_path)

    if image is None:
        print(f"WARNING: Could not read {image_path}")
        failed_images += 1
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if not found and hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD)

    if found:
        if corners is not None and corners.dtype != np.float32:
            corners = corners.astype(np.float32)

        # Refine corners only for standard corner output.
        if corners is not None:
            corners_refined = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria
            )
        else:
            corners_refined = corners

        object_points.append(object_points_template)
        image_points.append(corners_refined)

        used_images += 1

        debug_image = image.copy()
        cv2.drawChessboardCorners(debug_image, CHECKERBOARD, corners_refined, True)

        debug_path = os.path.join(
            OUTPUT_DIR,
            "debug_" + os.path.basename(image_path)
        )
        cv2.imwrite(debug_path, debug_image)

        print(f"Used: {image_path}")
    else:
        failed_images += 1
        print(f"Failed to find corners: {image_path}")

print()
print(f"Used images: {used_images}")
print(f"Failed images: {failed_images}")

if used_images < 10:
    print("ERROR: Too few usable images. Capture more images.")
    raise SystemExit(1)

ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(
    object_points,
    image_points,
    image_size,
    None,
    None
)

print()
print("Calibration RMS reprojection error:")
print(ret)

print()
print("Camera matrix:")
print(camera_matrix)

print()
print("Distortion coefficients:")
print(distortion_coefficients)

# Calculate mean reprojection error.
total_error = 0

for i in range(len(object_points)):
    projected_points, _ = cv2.projectPoints(
        object_points[i],
        rvecs[i],
        tvecs[i],
        camera_matrix,
        distortion_coefficients
    )

    error = cv2.norm(
        image_points[i],
        projected_points,
        cv2.NORM_L2
    ) / len(projected_points)

    total_error += error

mean_error = total_error / len(object_points)

print()
print("Mean reprojection error:")
print(mean_error)

output_xml = os.path.join(OUTPUT_DIR, "camera_result_1280x800.xml")

fs = cv2.FileStorage(output_xml, cv2.FILE_STORAGE_WRITE)

fs.write("image_width", image_size[0])
fs.write("image_height", image_size[1])
fs.write("camera_matrix", camera_matrix)
fs.write("distortion_coefficients", distortion_coefficients)

fs.release()

print()
print(f"Saved calibration file to: {output_xml}")
print()
print("Debug images with detected corners were saved to:")
print(OUTPUT_DIR)
