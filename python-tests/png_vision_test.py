"""
Vision tests using PNG images.
"""
import cv2
import numpy as np
import time
from glob import glob
from get_target import *

paths = glob("*.png")
for path in paths:
    img = cv2.imread(path)

    out = preprocess(img)
    target = get_target(out)
    if target is not None:
        print(f"Target found in image {path}")
        draw_target(img, target)
        # Convert the pre-processed image to BGR so the contour can be shown
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        draw_target(out, target)
    else:
        print(f"No target found in image {path}")

    cv2.imshow("Raw Image", img)
    cv2.imshow("Pre-processed Image", out)
    
    cv2.waitKey(0)
