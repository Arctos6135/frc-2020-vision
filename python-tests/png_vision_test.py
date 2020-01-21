"""
Vision tests using PNG images.
"""
import cv2
import numpy as np
from glob import glob
from vision_test import *

paths = glob("*.png")

if __name__ == "__main__":
    i = 0
    while i < len(paths):
        path = paths[i]
        print(f"Processing image #{i}")
        img = cv2.imread(path)

        process_img(img)
        
        code = cv2.waitKey(0)
        if code == ord('q'):
            break
        elif code == ord('p'):
            i = (i - 1) % len(paths)
        else:
            i = (i + 1) % len(paths)

    for path in paths:
        img = cv2.imread(path)

        process_img(img)
        
        code = cv2.waitKey(0)
        

        print(code)
        print(chr(code))
        if code == ord('q'):
            break
