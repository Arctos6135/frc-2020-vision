import cv2
import numpy as np
import math
from get_target import CAM_WIDTH, CAM_HEIGHT, CAM_HORIZ_FOV, CAM_VERT_FOV

# Calculate focal lengths
# This is used later to calculate an object's angle
CAM_HORIZ_FOCAL_LEN = CAM_WIDTH / 2 / math.tan(math.radians(CAM_HORIZ_FOV / 2))
CAM_VERT_FOCAL_LEN = CAM_HEIGHT / 2 / math.tan(math.radians(CAM_VERT_FOV / 2))

# The real coordinates of the target's corner points in 3d space
# Note the order of points and down is positive
# Start at the top left point and go clockwise
OBJECT_POINTS = np.array([
    (-19.625, 0, 0),
    (-17.31560, 0, 0),
    (-8.65780, 15, 0),
    (8.65780, 15, 0),
    (17.31560, 0, 0),
    (19.625, 0, 0),
    (9.8125, 17, 0),
    (-9.8125, 17, 0)
])

# Intrinsic camera matrix
# [ fx   0  cx ]
# [  0  fy  cy ]
# [  0   0   1 ]
CAMERA_MATRIX = np.array([
    [CAM_HORIZ_FOCAL_LEN, 0, CAM_WIDTH / 2],
    [0, CAM_VERT_FOCAL_LEN, CAM_HEIGHT / 2],
    [0, 0, 1]
], dtype="double")

# Distortion coefficients
# Can be obtained with calibration
# Currently assumes no distortion
CAMERA_DIST_COEFFS = np.zeros((4, 1))


def get_corner_points(contour):
    """
    Get all corner points of the hexagonal target.

    Returns a list of 8 points, starting with the top left and going clockwise.
    """
    # First actually convert to points
    points = [point[0] for point in contour]
    # Find the top left point
    # This should be the leftmost point
    top_left_idx = 0
    for i in range(1, len(points)):
        if points[i][0] < points[top_left_idx][0]:
            top_left_idx = i
    # Find the next and previous points
    n = points[(top_left_idx + 1) % len(points)]
    p = points[top_left_idx - 1]
    # Get the one with the lower Y value and keep going in that direction
    if n[1] < p[1]:
        return [points[(top_left_idx + i) % len(points)] for i in range(len(points))]
    else:
        return [points[(top_left_idx - i) % len(points)] for i in range(len(points))]


def get_target_angles(vec):
    """
    Converts a rotation vector as outputted by solvePnP to (yaw, pitch, roll).
    """
    R, jacobian = cv2.Rodrigues(vec)
    # Roll, pitch and yaw of the target IRL is different from in the math
    # X axis - left to right
    # Y axis - top to bottom
    # Z axis - in to out
    # Therefore the format we want is actually YXZ
    # Ask Tyler about the math here
    yaw = math.atan2(-R[2, 0], R[2, 2])
    pitch = math.asin(R[2, 1])
    roll = math.atan2(-R[0, 1], R[1, 1])
    return (yaw, pitch, roll)


def get_target_info(target):
    """
    Gets target info.

    Returns a tuple of (retval, rotation_vector, translation_vector). For more
    information, see the documentation for cv2.solvePnP().

    Note: To get roll, pitch and yaw from the rotation vector returned, use
    rotvec_to_euler().
    """
    corners = get_corner_points(target[0])
    return cv2.solvePnP(OBJECT_POINTS, np.array(corners, dtype="double"),
                        CAMERA_MATRIX, CAMERA_DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)


def draw_target_info(img, rv, tv):
    """
    Draws target info returned by get_target_info.
    """
    yaw, pitch, roll = [math.degrees(angle) for angle in get_target_angles(rv)]
    cv2.putText(img, f"Yaw: {yaw:.2f}, Pitch: {pitch:.2f}, Roll: {roll:.2f}",
                (0, img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
    cv2.putText(img, f"Translation: ({tv[0][0]:.2f}, {tv[1][0]:.2f}, {tv[2][0]:.2f})",
                (0, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

    # Re-project to visualize
    l = 20
    points, jacobian = cv2.projectPoints(np.array([
        [0, 0, 0],  # Center
        [l, 0, 0],  # X axis
        [0, l, 0],  # Y axis
        [0, 0, l],  # Z axis
    ], dtype="double"), rv, tv, CAMERA_MATRIX, CAMERA_DIST_COEFFS)
    points = [(int(point[0][0]), int(point[0][1])) for point in points]
    cv2.arrowedLine(img, points[0], points[1], (0, 0, 255))
    cv2.arrowedLine(img, points[0], points[2], (255, 0, 0))
    cv2.arrowedLine(img, points[0], points[3], (0, 255, 0))
