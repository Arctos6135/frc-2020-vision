import cv2
import numpy as np

# HSV thresholding values
# Note: OpenCV uses H 0-360, S 0-255 and V 0-255
THRESH_HIGH = (130, 255, 255)
THRESH_LOW = (80, 80, 80)

MORPH_KERNEL_SIZE = 5

MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))

CAM_HORIZ_FOV = 61
CAM_VERT_FOV = 37
CAM_WIDTH = 1280
CAM_HEIGHT = 720

FILTER_FULLNESS_HIGH = 0.25
FILTER_FULLNESS_LOW = 0.10

def preprocess(img):
    """
    Pre-process an image.

    This applies thresholding and morphological operations.
    """
    # Convert image to hsv
    # HSV_FULL has a hue of 0-360 instead of 0-180
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    # Apply thresholding to get a monochrome image
    img = cv2.inRange(img, THRESH_LOW, THRESH_HIGH)
    # Apply morphological operations
    # Opening: Erosion followed by dilation
    # Used to get rid of noise
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, MORPH_KERNEL)
    # Closing: Dilation followed by erosion
    # Used to get rid of small holes
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, MORPH_KERNEL)
    return img


def filter_targets(targets):
    """
    Filter a list of possible targets to find the correct one.

    If the input list is empty, or no target meets the requirement, this
    function will return None.
    """
    if not targets:
        return None
    valid_targets = []
    for contour, rect in targets:
        # RotatedRect format: 
        rect_area = rect[1][0] * rect[1][1]
        if rect_area == 0:
            continue
        area = cv2.contourArea(contour)
        # Filter by fullness
        # Fullness is the percentage of the bounding box filled by the contour
        fullness = area / rect_area
        if FILTER_FULLNESS_LOW <= fullness <= FILTER_FULLNESS_HIGH:
            print(f"Contour meets fullness requirement: area={area}, rect_area={rect_area}")
            valid_targets.append(((contour, rect), area))
        else:
            print(f"Contour does not meet fullness requirement: area={area}, rect_area={rect_area}")
    # If there are multiple left, return the one with the biggest area
    target = max(valid_targets, key=lambda v: v[1], default=None)
    return target[0] if target else None


def get_target(img):
    """
    Find the target from a pre-processed image.
    
    Returns a tuple of (contour, rect) where contour is the OpenCV contour of
    the target, and rect is the minimum bounding rect (rotated rect).

    Returns None if no target is found.
    """
    # Find the contours
    # We only care about external contours
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Get the minimum bounding box of each target and filter
    target = filter_targets([(contour, cv2.minAreaRect(contour)) for contour in contours])
    return target


def draw_target(img, target):
    """
    Draw a target's contour and bonding box onto an image.
    
    Source: https://stackoverflow.com/a/18208020/5745575
    """
    box = np.int0(cv2.boxPoints(target[1]))
    cv2.drawContours(img, [target[0]], -1, (0, 0, 255), 3)
    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)
