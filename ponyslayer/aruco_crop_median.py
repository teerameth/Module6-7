import cv2
import numpy as np
import math
import random
from transform import four_point_transform, order_points, aruco_crop

cap = cv2.VideoCapture("X:/M.mp4")
N = 1000
for i in range(N):
    cap.set(1, int(random.randrange(N)))
    frame = cap.read()[1]
    try:
        warped, valid_mask = aruco_crop(frame)
        cv2.imshow("Warped", warped)
        cv2.waitKey(1)
    except: pass