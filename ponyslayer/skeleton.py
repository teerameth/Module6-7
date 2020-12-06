import cv2
import numpy as np
from skimage.morphology import skeletonize
from skimage import data
from skimage.util import invert
from scipy.ndimage import generic_filter
img1 = cv2.imread("X:/pathMask1.png", 0)
img2 = cv2.imread("X:/pathMask2.png", 0)
for mask in [img1, img2]:
    cnt = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
    skeleton = skeletonize(mask, method='lee')
    cv2.drawContours(skeleton, cnt, -1, 100, 1)
    cv2.imshow("Skeleton", skeleton)
    cv2.waitKey(0)
cv2.destroyAllWindows()