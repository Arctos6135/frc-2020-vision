"""
Live vision tests.
"""
import cv2
import numpy as np
from get_target import *

cam = cv2.VideoCapture(1)

while True:
    ret, frame = cam.read()
    if not ret:
        print("Capture failed!")
        break
    img = preprocess(img)

    cv2.imshow("Image Output", img)
    
    code = cv2.waitKey(50) & 0xFF
    if code == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()

