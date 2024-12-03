import cv2
import numpy as np
import glob
import os

# Define the checkerboard dimensions (number of inner corners per row and column)
checkerboard_dims = (7, 9)

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points
object_points = []  # 3D points in real world space
image_points = []   # 2D points in image plane

# Prepare the object points (0,0,0), (1,0,0), ..., based on checkerboard size
objp = np.zeros((checkerboard_dims[0] * checkerboard_dims[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_dims[0], 0:checkerboard_dims[1]].T.reshape(-1, 2)

# Load all calibration images
image_folder = "calibration_images"  # Path to your images folder
image_pattern = os.path.join(image_folder, "*.jpg")
images = glob.glob(image_pattern)

# Check if any images were found
if not images:
    print(f"No images found in the directory {image_folder}. Please check the path and file extension.")
    exit()

# Initialize a variable for the size of images (use the first image to get shape)
img = cv2.imread(images[0])
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Loop through the images to find corners and store points
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_dims, None)

    if ret:
        # Refine corner locations
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        image_points.append(corners_refined)
        object_points.append(objp)

        # Draw and display corners for visualization
        cv2.drawChessboardCorners(img, checkerboard_dims, corners_refined, ret)
        cv2.imshow("Corners", img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None
)

# Save calibration results
np.savez("calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)
