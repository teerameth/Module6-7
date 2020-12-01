import cv2
from skimage.measure import compare_ssim
import argparse
import imutils
import numpy as np

warped_list = np.load("X:/warped.npy", mmap_mode='r')
valid_mask_list = np.load("X:/mask.npy", mmap_mode='r')

# for i in range(len(warped_list)):
#     warped, valid_mask = warped_list[i], valid_mask_list[i]
#     cv2.imshow("Warped", warped)
#     cv2.imshow("Valid Mask", valid_mask)
#     cv2.waitKey(1)
def findMedian(images): return np.asarray(np.median(np.dstack(images), axis=2), dtype=np.uint8)
def apply_pair(imageA, maskA, imageB, maskB):
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    print("SSIM: {}".format(score))
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    ## Apply only valid area
    mask = cv2.bitwise_and(cv2.bitwise_and(thresh, maskA), maskB)
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5),np.uint8))
    return closed_mask
def apply_three(imageA, maskA, imageB, maskB, imageC, maskC):
    AB = apply_pair(imageA, maskA, imageB, maskB)
    AC = apply_pair(imageA, maskA, imageC, maskC)
    BC = apply_pair(imageB, maskB, imageC, maskC)
    A = cv2.bitwise_or(cv2.bitwise_and(AB, cv2.bitwise_not(BC)),  # A = AB & !BC
                       cv2.bitwise_and(AC, cv2.bitwise_not(BC)))  # A = AC & !BC
    B = cv2.bitwise_or(cv2.bitwise_and(AB, cv2.bitwise_not(AC)),  # B = AB & !AC
                       cv2.bitwise_and(AC, cv2.bitwise_not(AC)))  # B = BC & !AC
    C = cv2.bitwise_or(cv2.bitwise_and(AB, cv2.bitwise_not(AB)),  # C = AC & !AB
                       cv2.bitwise_and(AC, cv2.bitwise_not(AB)))  # C = BC & !AB
    return A, B, C


cv2.imshow("A", apply_pair(warped_list[0], valid_mask_list[0], warped_list[100], valid_mask_list[100]))
cv2.waitKey(0)

original = imutils.resize(np.hstack([warped_list[0], warped_list[100], warped_list[200]]), width=1920)
rail_mask = imutils.resize(np.hstack(apply_three(warped_list[0], valid_mask_list[0], warped_list[100], valid_mask_list[100], warped_list[200], valid_mask_list[200])), width=1920)
cv2.imshow("A", original)
cv2.imshow("B", rail_mask)
cv2.waitKey(0)

circular_round = 3
N = 3
step_round = int(len(warped_list)/N/circular_round)
for i in range(N):
    img_list = []
    for j in range(circular_round):
        img_list.append(warped_list[int(j*step_round + i*step_round/N)])
    cv2.imshow("Preview", np.hstack(img_list))
    cv2.waitKey(0)