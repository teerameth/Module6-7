from transform import *
import cv2
import numpy as np
cap = cv2.VideoCapture("../Q.mp4")
vdo_warped, vdo_mask = [], []
ret = True
while ret:
    ret, frame = cap.read()
    try:
        aruco = aruco_crop(frame)
        if aruco != 0:
            warped, valid_mask = aruco
            vdo_warped.append(warped)
            vdo_mask.append(valid_mask)
    except:
        continue
np.save("X:/warped", vdo_warped)
np.save("X:/mask", vdo_mask)