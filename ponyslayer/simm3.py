from skimage.measure import compare_ssim
import argparse
import imutils
import cv2
from transform import aruco_crop

def apply_pair(imageA, imageB):
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
    # compute the Structural Similarity Index (SSIM) between the two
    # images, ensuring that the difference image is returned
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    print("SSIM: {}".format(score))
    # threshold the difference image, followed by finding contours to
    # obtain the regions of the two input images that differ
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    # cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    cv2.imshow("Original", imageA)
    cv2.imshow("Modified", imageB)
    cv2.imshow("Diff", diff)
    cv2.imshow("Thresh", thresh)
    cv2.waitKey(0)
def apply(imageA, imageB, imageC):
    pass

cap = cv2.VideoCapture("../K.mp4")
total_frame = cap.get(cv2.CAP_PROP_FRAME_COUNT)
print(total_frame)

cap.set(1,0)
imageA = cap.read()[1]
warpedA, valid_maskA = aruco_crop(imageA)
cv2.imshow("A", warpedA)

cap.set(1,int(total_frame/6))
imageB = cap.read()[1]
warpedB, valid_maskB = aruco_crop(imageB)
cv2.imshow("B", warpedB)

cap.set(1,int(total_frame/3-1))
imageC = cap.read()[1]
warpedC, valid_maskC = aruco_crop(imageC)
cv2.imshow("C", warpedC)

apply_pair(warpedA, warpedC)
cv2.waitKey(0)
cap.release()

