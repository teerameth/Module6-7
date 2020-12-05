import numpy as np
import cv2
from math import sqrt
import imutils
from transform import four_point_transform
from skimage.morphology import skeletonize
from skimage import data
from skimage.util import invert
from scipy.ndimage import generic_filter
def immask(c, frame):
    mask = np.ones(frame.shape[:2], dtype="uint8") * 255
    cv2.drawContours(mask, [c], 0, 0, -1)
    return 255 - mask
# Line ends filter
def lineEnds(P):
    """Central pixel and just one other must be set to be a line end"""
    return 255 * ((P[4]==255) and np.sum(P)==510)
path_mask_opening = None
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
def getPathMask(mask, startMask, chessMask):
    path_mask = cv2.bitwise_or(mask, chessMask)
    path_mask = cv2.bitwise_or(path_mask, startMask)
    path_mask = cv2.bitwise_not(path_mask)
    path_mask_opening = cv2.morphologyEx(path_mask, cv2.MORPH_OPEN, np.ones((9, 9)))
    return path_mask_opening
def getPath(path_mask_opening, visualize=False):
    contours, hierarchy = cv2.findContours(path_mask_opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    path_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        poly_points = cv2.approxPolyDP(cnt, 0.02 * path_mask_opening.shape[0], False)  # approximate polygon
        m = immask(poly_points, np.zeros_like(path_mask_opening))
        # imshow(m)
        if area > 20000: path_contours.append(poly_points)
    print("Path count: " + str(len(path_contours)))

    path_masks = [immask(cnt, path_mask_opening) for cnt in path_contours]
    path_skeletons = []
    for i in range(len(path_masks)):
        mask_erode = cv2.erode(path_masks[i], np.ones((15, 15)), iterations=1)
        skeleton = skeletonize(mask_erode, method='lee')
        path_skeletons.append(skeleton)
        if visualize:
            cv2.imshow("Skeleton", skeleton)
            cv2.imshow("Path Mask", path_masks[i])
            cv2.waitKey(0)
    end_points = []
    path_cnts = []
    path_cnts_approx = []

    return