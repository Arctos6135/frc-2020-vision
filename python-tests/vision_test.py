"""
Live vision tests.
"""
import cv2
import numpy as np
import time
from get_target import *
from get_distance import *

def process_img(img):
    """
    Run tests on an image.
    """
    out = preprocess(img)
    target = get_target(out)
    if target is not None:
        print("Target found in image")
        retval, rv, tv = get_target_info(target)
        if retval:
            draw_target_info(img, rv, tv)
        else:
            print("solvePnP failed!")
        # Convert the pre-processed image to BGR so the contour can be shown
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        corners = get_corner_points(target[0])
        print("Corner points are:")
        for point in corners:
            cv2.circle(out, tuple(point), 5, (0, 255, 0), -1)
            print(point)
    else:
        print("Target not found in image")

    cv2.imshow("Raw Image", img)
    cv2.imshow("Pre-processed Image", out)


if __name__ == "__main__":
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

        process_img(frame)
        
        code = cv2.waitKey(50) & 0xFF
        if code == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()

