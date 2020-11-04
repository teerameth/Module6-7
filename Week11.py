import cv2
import numpy as np
from ponyslayer.utils import imshow, imshows, imdraw, imdraws, imdraws_color, immask, immasks, implot, imsave
img = cv2.imread("./img/week11.png", 0)
# imshow(img)
sample_o = img[443:484, 1057:1098].copy()
# imshow(sample_o)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
sample_o = cv2.erode(sample_o, kernel, iterations=1)
# imshow(sample_o)

kernel = np.array(sample_o, np.float32)
erode = cv2.filter2D(img, -1, kernel/160000)
cv2.imshow("Erode", erode)

(_, thresh) = cv2.threshold(erode, 140, 255, cv2.THRESH_BINARY)
cv2.imshow("Thresh", thresh)
filled = cv2.filter2D(thresh, -1, sample_o)
cv2.imshow("Filled", filled)
cv2.waitKey(0)
