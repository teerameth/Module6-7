import numpy as np
import cv2
from math import sqrt
import imutils
from .transform import four_point_transform

def euclidean(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
def medianCanny(img, thresh1, thresh2):
    median = np.median(img)
    img = cv2.Canny(img, int(thresh1 * median), int(thresh2 * median))
    return img
def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result
def padding_image(img, color):
    s = max(img.shape[0:2])
    f = np.ones((s, s, 3),np.uint8) * color
    ax,ay = (s - img.shape[1])//2,(s - img.shape[0])//2 # Getting the centering position
    f[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img
    return f
def padding_image_gray(img, color):
    s = max(img.shape[0:2])
    f = np.ones((s, s),np.uint8) * color
    ax,ay = (s - img.shape[1])//2,(s - img.shape[0])//2 # Getting the centering position
    f[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img
    return f
