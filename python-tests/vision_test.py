"""
Live vision tests.
"""
import cv2
import numpy as np
import time
from get_target import *

cam = cv2.VideoCapture(1)

# Configure camera
print("Configuring Camera")
cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
# 1 means off for some reason
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
time.sleep(2)
cam.set(cv2.CAP_PROP_EXPOSURE, 10)
time.sleep(0.5)

while True:
    ret, frame = cam.read()
    if not ret:
        print("Capture failed!")
        break

    out = preprocess(frame)
    target = get_target(out)
    if target is not None:
        draw_target(frame, target)
        # Convert the pre-processed image to BGR so the contour can be shown
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        draw_target(out, target)

    cv2.imshow("Raw Image", frame)
    cv2.imshow("Pre-processed Image", out)
    
    code = cv2.waitKey(50) & 0xFF
    if code == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()

