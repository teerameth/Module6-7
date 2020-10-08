from ponyslayer.transform import four_point_transform, order_points
import numpy as np
import imutils
import argparse
import cv2

image_path = "img/Perspective_raw3.png"
n = 0
refPt = []

def click(event, x, y, flags, param):
    global refPt, n, img
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt.append((x, y))
        n += 1
    if event == cv2.EVENT_LBUTTONUP:
        img = cv2.circle(img, refPt[-1], 3, (0, 0, 255), -1)
        cv2.imshow("image", img)

image = cv2.imread(image_path)
img = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click)

cv2.imshow("image", image)
cv2.waitKey(0)

pts = np.array(refPt, dtype = "float32")

warped = four_point_transform(image, pts)
warped = cv2.resize(warped, (500, 500))
cv2.imshow("Original", img)
cv2.imshow("Warped", warped)
cv2.imwrite("./img/Perspective_transformed3.png", warped)
cv2.waitKey(0)