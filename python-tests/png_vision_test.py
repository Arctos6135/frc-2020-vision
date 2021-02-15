"""
Vision tests using PNG images.
"""
import cv2
import numpy as np
import sys
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
        
        while True:
            code = cv2.waitKey(0)
            if code == ord('q'):
                sys.exit(0)
            elif code == ord('p'):
                i = (i - 1) % len(paths)
                break
            elif code == ord('n') or code == ord(' '):
                i = (i + 1) % len(paths)
                break
