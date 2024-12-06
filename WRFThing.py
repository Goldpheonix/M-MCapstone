import cv2
import numpy as np

# Camera intrinsic parameters (example values, use calibration data)
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype="double")
dist_coeffs = np.zeros((4, 1))  # Assume no lens distortion

# 3D coordinates of key points on the object
object_points = np.array([
    [x1, y1, z1],
    [x2, y2, z2],
    [x3, y3, z3],
    ...
], dtype="double")

# 2D coordinates of the corresponding points in the image
image_points = np.array([
    [u1, v1],
    [u2, v2],
    [u3, v3],
    ...
], dtype="double")

# Estimate pose
success, rotation_vector, translation_vector = cv2.solvePnP(
    object_points, image_points, camera_matrix, dist_coeffs
)

# Convert rotation vector to a rotation matrix
rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

# Pose: translation (x, y, z) and rotation matrix (3x3)
print("Translation Vector:\n", translation_vector)
print("Rotation Matrix:\n", rotation_matrix)

# If needed, extract Euler angles (rx, ry, rz)
def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])

euler_angles = rotation_matrix_to_euler_angles(rotation_matrix)
print("Euler Angles (rx, ry, rz):\n", euler_angles)