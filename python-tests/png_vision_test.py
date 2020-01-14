"""
Vision tests using PNG images.
"""
import cv2
import numpy as np
from glob import glob
from vision_test import *

paths = glob("*.png")
for path in paths:
    img = cv2.imread(path)

    process_img(img)
    
    cv2.waitKey(0)
