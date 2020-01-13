import cv2
import numpy as np

# HSV thresholding values
# Note: OpenCV uses H 0-360, S 0-255 and V 0-255
THRESH_HIGH = (130, 255, 255)
THRESH_LOW = (80, 80, 80)

MORPH_KERNEL_SIZE = 5

MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))

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

